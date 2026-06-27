from __future__ import annotations

import csv
import ctypes
import math
import os
import random
import time
from dataclasses import replace
from collections.abc import Callable, Iterator
from ctypes import c_double, c_int
from typing import Any

from Utils.robot_spec import RobotSpec
from Utils.simulation_config import SimulationConfig, derive_runtime_params
from Utils.simulation_state import SimulationResult, SimulationState
from Utils.robot_runtime import robot_local_to_world
from Utils.track_spec import TrackSpec
from sim.native_linesim import CPoint, get_linesim
from sim.physics.factory import create_physics_model
from sim.track_runtime import (
    build_markers,
    ensure_track_raster,
    point_in_obb,
    rot,
    segments_from_track,
    segments_polyline,
    start_finish_lines,
)
from Utils.track_geometry import Pt, Pose, advance_straight


class SimLogger:
    """Records per-step telemetry/events and writes CSV + JSON logs under Logs/."""

    # Stable, human-friendly order for the most important scalar telemetry.
    _preferred_cols = [
        "t_ms", "dt_s", "physics_profile",
        "x_mm", "y_mm", "heading_deg",
        "v_mm_s", "omega_rad_s", "a_lin_mm_s2", "alpha_rad_s2",
        "v_left_mm_s", "v_right_mm_s",
        "wheel_left_surface_speed_mm_s", "wheel_right_surface_speed_mm_s",
        "ground_left_speed_mm_s", "ground_right_speed_mm_s",
        "omega_wheel_left_rad_s", "omega_wheel_right_rad_s",
        "alpha_wheel_left_rad_s2", "alpha_wheel_right_rad_s2",
        "J_eq_left_kgm2", "J_eq_right_kgm2",
        "pwm_left", "pwm_right", "duty_left", "duty_right", "pwm_min", "pwm_max",
        "battery_voltage_v", "battery_soc", "battery_current_a",
        "current_left_a", "current_right_a", "current_total_a", "battery_power_w",
        "motor_left_current_a", "motor_right_current_a",
        "motor_left_current_signed_a", "motor_right_current_signed_a",
        "motor_left_voltage_v", "motor_right_voltage_v",
        "motor_left_back_emf_v", "motor_right_back_emf_v",
        "motor_left_torque_nm", "motor_right_torque_nm",
        "wheel_left_torque_nm", "wheel_right_torque_nm",
        "tau_motor_em_left_nm", "tau_motor_em_right_nm",
        "tau_motor_viscous_left_nm", "tau_motor_viscous_right_nm",
        "tau_motor_coulomb_left_nm", "tau_motor_coulomb_right_nm",
        "tau_motor_net_left_nm", "tau_motor_net_right_nm",
        "tau_wheel_drive_left_nm", "tau_wheel_drive_right_nm",
        "tau_ground_left_nm", "tau_ground_right_nm", "tau_slip_loss_left_nm", "tau_slip_loss_right_nm",
        "mechanical_power_left_w", "mechanical_power_right_w", "brake_dissipated_power_w",
        "enc_left_ticks", "enc_right_ticks",
        "enc_left_delta_ticks", "enc_right_delta_ticks",
        "enc_left_rad_s", "enc_right_rad_s",
        "imu_omega_rad_s", "imu_alpha_rad_s2",
        "imu_accel_x_mm_s2", "imu_accel_y_mm_s2",
        "slip_ratio_left", "slip_ratio_right", "lateral_slip_left", "lateral_slip_right",
        "force_longitudinal_command_left_n", "force_longitudinal_command_right_n",
        "force_longitudinal_ground_left_n", "force_longitudinal_ground_right_n",
        "force_longitudinal_max_left_n", "force_longitudinal_max_right_n",
        "lateral_force_left_n", "lateral_force_right_n", "friction_usage_left", "friction_usage_right",
        "traction_force_left_n", "traction_force_right_n",
        "max_static_force_left_n", "max_static_force_right_n",
        "copper_loss_left_w", "copper_loss_right_w", "driver_loss_left_w", "driver_loss_right_w",
        "rolling_resistance_loss_w", "tire_slip_loss_left_w", "tire_slip_loss_right_w",
        "battery_energy_j", "total_kinetic_energy_j", "total_loss_energy_j",
        "energy_balance_error_j", "energy_balance_error_percent",
        "physics_backend", "linesim_abi_version", "c_backend_loaded", "c_backend_path",
        "c_backend_error", "c_modular_step_available", "using_python_fallback",
        "c_step_call_count", "last_step_executed_in_c",
        "hit", "finished",
    ]

    def __init__(self, base_dir: str):
        self.base_dir = base_dir
        os.makedirs(os.path.join(base_dir, "Logs"), exist_ok=True)
        run_id = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(base_dir, "Logs", f"sim_log_{run_id}.csv")
        self.json_path = os.path.join(base_dir, "Logs", f"sim_log_{run_id}.json")
        self.steps: list[dict[str, Any]] = []
        self.events: list[dict[str, Any]] = []
        self._max_sensors = 0

    def log_step(
        self,
        t_ms: int | None = None,
        x: float | None = None,
        y: float | None = None,
        h: float | None = None,
        v: float | None = None,
        w: float | None = None,
        pwm_l: int | None = None,
        pwm_r: int | None = None,
        sensors: list[int] | None = None,
        step_data: dict[str, Any] | None = None,
    ) -> None:
        """Append one step.

        The old positional arguments are kept for compatibility, but Phase 4.2
        uses ``step_data`` so every physical channel present in SimulationState
        is persisted to CSV and JSON.
        """
        vals = list(sensors) if sensors is not None else []
        record: dict[str, Any] = dict(step_data or {})
        if t_ms is not None:
            record["t_ms"] = t_ms
        if x is not None:
            record["x_mm"] = x
        if y is not None:
            record["y_mm"] = y
        if h is not None:
            record["heading_deg"] = h
        if v is not None:
            record["v_mm_s"] = v
        if w is not None:
            record["omega_rad_s"] = w
        if pwm_l is not None:
            record["pwm_left"] = pwm_l
        if pwm_r is not None:
            record["pwm_right"] = pwm_r
        if sensors is not None:
            record["sensors"] = vals
        else:
            vals = list(record.get("sensors", []))
        self._max_sensors = max(self._max_sensors, len(vals))
        self.steps.append(record)

    def log_event(self, kind: str, t_ms: int, x: float, y: float, h: float, extra: dict | None = None) -> None:
        ev = {"event": kind, "t_ms": t_ms, "x_mm": x, "y_mm": y, "heading_deg": h}
        if extra:
            ev.update(extra)
        self.events.append(ev)

    def _csv_columns(self) -> list[str]:
        seen = set()
        cols: list[str] = []
        for col in self._preferred_cols:
            if any(col in step for step in self.steps):
                cols.append(col)
                seen.add(col)
        dynamic = sorted({k for step in self.steps for k in step.keys()} - seen - {"sensors"})
        cols.extend(dynamic)
        cols.extend([f"s{i}" for i in range(self._max_sensors)])
        return cols

    def flush(self) -> None:
        try:
            with open(self.csv_path, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                cols = self._csv_columns()
                w.writerow(cols)
                for s in self.steps:
                    vals = list(s.get("sensors", []))
                    row: list[Any] = []
                    for col in cols:
                        if col.startswith("s") and col[1:].isdigit():
                            i = int(col[1:])
                            row.append(vals[i] if i < len(vals) else "")
                        else:
                            row.append(s.get(col, ""))
                    w.writerow(row)
        except Exception as e:
            print("CSV log error:", e)

        try:
            import json
            with open(self.json_path, "w", encoding="utf-8") as f:
                json.dump({"steps": self.steps, "events": self.events}, f, indent=2, ensure_ascii=False)
        except Exception as e:
            print("JSON log error:", e)


class NoopLogger:
    csv_path = ""
    json_path = ""

    def log_step(self, *args, **kwargs):
        return

    def log_event(self, *args, **kwargs):
        return

    def flush(self):
        return


class FinishZoneChecker:
    """Detects valid Start→Finish crossing using a rectangular zone pair."""

    __slots__ = (
        "s_mid", "f_mid", "ux", "uy", "nx", "ny", "L", "HALF_W", "EPS",
        "started_inside", "last_inside", "exited_once", "entered_once", "armed", "last_event",
    )

    def __init__(self, sa: Pt, sb: Pt, fa: Pt, fb: Pt, half_width: float = 250.0, eps: float = 3.0):
        self.s_mid = Pt((sa.x + sb.x) / 2.0, (sa.y + sb.y) / 2.0)
        self.f_mid = Pt((fa.x + fb.x) / 2.0, (fa.y + fb.y) / 2.0)
        ux = self.s_mid.x - self.f_mid.x
        uy = self.s_mid.y - self.f_mid.y
        L = math.hypot(ux, uy) or 1.0
        self.ux, self.uy = (ux / L, uy / L)
        self.nx, self.ny = (-self.uy, self.ux)
        self.L = L
        self.HALF_W = float(half_width)
        self.EPS = float(eps)
        self.started_inside = False
        self.last_inside = False
        self.exited_once = False
        self.entered_once = False
        self.armed = False
        self.last_event: str | None = None

    def prime(self, cx: float, cy: float) -> None:
        self.started_inside = self._point_inside(cx, cy)
        self.last_inside = self.started_inside
        self.exited_once = not self.started_inside
        self.entered_once = self.started_inside
        self.armed = False
        self.last_event = "init_inside" if self.started_inside else "init_outside"

    def _proj_t(self, x: float, y: float) -> float:
        vx, vy = x - self.f_mid.x, y - self.f_mid.y
        return vx * self.ux + vy * self.uy

    def _proj_w(self, x: float, y: float) -> float:
        vx, vy = x - self.f_mid.x, y - self.f_mid.y
        return vx * self.nx + vy * self.ny

    def _point_inside(self, x: float, y: float) -> bool:
        t = self._proj_t(x, y)
        w = abs(self._proj_w(x, y))
        return (-self.EPS <= t <= self.L + self.EPS) and (w <= self.HALF_W + self.EPS)

    def update(self, prev_pose, curr_pose, env_w, env_h, t_ms: int | None = None) -> bool:
        self.last_event = None
        px1, py1, _h1 = curr_pose
        prev_inside = self.last_inside
        inside_now = self._point_inside(px1, py1)

        if (not prev_inside) and inside_now:
            self.entered_once = True
            self.last_event = "entered_zone"
        elif prev_inside and (not inside_now):
            self.exited_once = True
            self.last_event = "exited_zone"

        if (not self.armed) and self.exited_once and self.entered_once:
            self.armed = True
            if self.last_event is None:
                self.last_event = "armed"

        if self.armed and (not prev_inside) and inside_now:
            self.last_event = "finish"
            self.last_inside = inside_now
            return True

        self.last_inside = inside_now
        return False


def sensor_value_from_coverage(cov: float, sensor_mode: str, sensor_bits: int, value_of_line: int,
                               value_of_background: int, analog_noise_line: int, analog_noise_background: int) -> int:
    cov = max(0.0, min(1.0, float(cov)))
    is_line = cov >= 0.5
    mode = "digital" if str(sensor_mode).lower().startswith("d") else "analog"
    nbits = int(max(1, min(16, int(sensor_bits))))
    maxv = (1 << nbits) - 1
    base_line = int(max(0, min(maxv, int(value_of_line))))
    base_bg = int(max(0, min(maxv, int(value_of_background))))
    noise_line = int(max(0, min(maxv, int(analog_noise_line))))
    noise_bg = int(max(0, min(maxv, int(analog_noise_background))))

    if mode == "digital":
        return base_line if is_line else base_bg

    if is_line:
        lo = max(0, base_line - noise_line)
        hi = min(maxv, base_line + noise_line)
    else:
        lo = max(0, base_bg - noise_bg)
        hi = min(maxv, base_bg + noise_bg)
    if hi < lo:
        lo, hi = hi, lo
    return random.randint(lo, hi)


class SimulationEngine:
    """Pure simulation engine. No PySide6 dependency and no UI responsibilities."""

    def __init__(
        self,
        track: TrackSpec,
        robot: RobotSpec,
        config: SimulationConfig,
        controller_fn: Callable[[dict], dict],
        track_path: str | None = None,
        save_logs: bool | None = None,
        base_dir: str | None = None,
    ):
        self.track = track
        self.robot = robot
        self.config = config
        self.controller_fn = controller_fn
        self.track_path = track_path
        self.cancelled = False
        self.params = derive_runtime_params(robot, config)
        if config.random_seed is not None:
            random.seed(config.random_seed)
        self._phase4_noise_rng = random.Random(int(config.custom_sensor_noise_seed))
        self._sensor_runtime: list[dict[str, Any]] = []
        self._last_sensor_debug: dict[str, Any] = {}

        self.dt_s = float(config.dt_s)

        self.sensor_mode = self.params.get("sensor_mode", "analog")
        self.sensor_bits = int(self.params.get("sensor_bits", 8))
        self.value_of_line = int(self.params.get("value_of_line", 255))
        self.value_of_background = int(self.params.get("value_of_background", 0))
        self.analog_noise_line = int(self.params.get("analog_noise_line", 50))
        self.analog_noise_background = int(self.params.get("analog_noise_background", 50))

        root_dir = base_dir or os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        do_logs = config.save_logs if save_logs is None else bool(save_logs)
        self.logger = SimLogger(root_dir) if do_logs else NoopLogger()
        self._linesim = None

        self.physics_model = create_physics_model(config, robot=robot, params=self.params)

    def cancel(self) -> None:
        self.cancelled = True

    def _native(self):
        if self._linesim is None:
            self._linesim = get_linesim()
        return self._linesim

    def _native_or_policy(self):
        try:
            return self._native() if self.physics_model.requires_native else None
        except RuntimeError:
            cfg = self._effective_physics_config()
            allow = bool(getattr(cfg, "custom_allow_python_fallback", False))
            if str(self.config.physics_profile).strip().lower() == "realistic":
                allow = bool(getattr(self.config, "allow_python_fallback_for_realistic", False))
            if allow:
                return None
            raise

    def _initial_pose(self) -> Pose:
        segs = getattr(self, "_segs", None)
        origin = getattr(self, "_origin", None)
        gates = getattr(self, "_gates", None)
        if (segs is None) or (origin is None):
            segs, origin, _tape_w = segments_from_track(self.track)
        if gates:
            (sa, sb, shdg_run, _shdg_base), _ = gates
            back = (self.robot.envelope.height_mm / 2.0) + 250.0
            pose_gate = Pose(Pt((sa.x + sb.x) * 0.5, (sa.y + sb.y) * 0.5), shdg_run)
            pos = advance_straight(pose_gate, -back)
        else:
            pos = origin
        return pos

    def _sensors_world_xy(self, x: float, y: float, h_deg: float) -> tuple[list[float], list[float]]:
        sx: list[float] = []
        sy: list[float] = []
        imperfection_offset = 0.0
        if self._track_imperfections_enabled():
            cfg = self._effective_physics_config()
            amp = float(cfg.custom_track_imperfection_amplitude_mm)
            wave = max(1e-9, float(cfg.custom_track_imperfection_wavelength_mm))
            # Runtime procedural offset: it changes where sensors sample the cached
            # raster, but never mutates the track JSON or the .rmap cache.
            s_approx = math.hypot(float(x), float(y))
            imperfection_offset = amp * math.sin((2.0 * math.pi * s_approx) / wave)
            if cfg.custom_track_imperfection_noise_std > 0.0:
                imperfection_offset += self._phase4_noise_rng.gauss(0.0, cfg.custom_track_imperfection_noise_std)
        h_rad = math.radians(float(h_deg))
        nx = -math.sin(h_rad)
        ny = math.cos(h_rad)
        for sensor in self.robot.sensors:
            wx, wy = robot_local_to_world(self.robot, x, y, h_deg, sensor.x_mm, sensor.y_mm)
            if imperfection_offset:
                wx += nx * imperfection_offset
                wy += ny * imperfection_offset
            sx.append(wx)
            sy.append(wy)
        return sx, sy

    def _effective_physics_config(self) -> SimulationConfig:
        if str(self.config.physics_profile).strip().lower() == "realistic":
            return self.config.as_realistic_preset()
        return self.config

    def _sensor_noise_enabled(self) -> bool:
        cfg = self._effective_physics_config()
        return bool(cfg.custom_use_sensor_noise and cfg.custom_sensor_noise_std > 0.0)

    def _track_imperfections_enabled(self) -> bool:
        cfg = self._effective_physics_config()
        return bool(cfg.custom_use_track_imperfections and cfg.custom_track_imperfection_amplitude_mm > 0.0)

    def _apply_phase4_sensor_noise(self, values: list[int]) -> list[int]:
        # Backward-compatible wrapper: Phase 4.4 calls _process_sensor_values.
        return self._process_sensor_values(values, self.dt_s)

    def _sensor_param(self, sensor: Any, attr: str, default: float) -> float:
        value = getattr(sensor, attr, None)
        return float(default if value is None else value)

    def _ensure_sensor_runtime(self, n: int, initial_values: list[int]) -> None:
        if len(self._sensor_runtime) == n:
            return
        self._sensor_runtime = []
        cfg = self._effective_physics_config()
        for i in range(n):
            init = float(initial_values[i]) if i < len(initial_values) else 0.0
            self._sensor_runtime.append({
                "filtered": init,
                "held": init,
                "elapsed": 0.0,
                "queue": [],
            })
        self._last_sensor_debug = {}

    def _process_sensor_values(self, values: list[int], dt_s: float) -> list[int]:
        cfg = self._effective_physics_config()
        self._ensure_sensor_runtime(len(values), values)
        maxv = (1 << int(max(1, min(16, int(self.sensor_bits))))) - 1
        self._last_sensor_debug = {}
        if not cfg.custom_use_sensor_noise:
            return values

        sigma_common = cfg.sensor_common_noise_std or cfg.custom_sensor_noise_std
        sigma_common_counts = sigma_common * maxv if sigma_common <= 1.0 else sigma_common
        common_noise = self._phase4_noise_rng.gauss(0.0, sigma_common_counts) if sigma_common_counts > 0.0 else 0.0
        out: list[int] = []
        for i, base in enumerate(values):
            sensor = self.robot.sensors[i] if i < len(self.robot.sensors) else None
            rt = self._sensor_runtime[i]
            gain = self._sensor_param(sensor, "gain", cfg.sensor_gain_default)
            offset = self._sensor_param(sensor, "offset", cfg.sensor_offset_default)
            indiv_std = self._sensor_param(sensor, "noise_std", cfg.sensor_individual_noise_std)
            indiv_counts = indiv_std * maxv if indiv_std <= 1.0 else indiv_std
            tau_ms = self._sensor_param(sensor, "filter_tau_ms", cfg.sensor_filter_tau_ms)
            latency_ms = self._sensor_param(sensor, "latency_ms", cfg.sensor_latency_ms)
            rate_hz = self._sensor_param(sensor, "update_rate_Hz", cfg.sensor_update_rate_Hz)

            raw = float(base)
            calibrated = gain * raw + offset
            noisy = calibrated + common_noise
            if indiv_counts > 0.0:
                noisy += self._phase4_noise_rng.gauss(0.0, indiv_counts)

            due = True
            if rate_hz > 0.0:
                rt["elapsed"] = float(rt.get("elapsed", 0.0)) + max(0.0, float(dt_s))
                period = 1.0 / rate_hz
                due = rt["elapsed"] + 1e-12 >= period
                if due:
                    rt["elapsed"] = math.fmod(rt["elapsed"], period)
            if due:
                if tau_ms > 0.0:
                    tau_s = tau_ms * 0.001
                    alpha = float(dt_s) / (tau_s + float(dt_s))
                    rt["filtered"] = float(rt["filtered"]) + alpha * (noisy - float(rt["filtered"]))
                else:
                    rt["filtered"] = noisy
                rt["held"] = rt["filtered"]

            produced = float(rt["held"])
            if latency_ms > 0.0:
                q = rt["queue"]
                q.append((0.0, produced))
                for j, (age, val) in enumerate(q):
                    q[j] = (age + float(dt_s) * 1000.0, val)
                delayed = q[0][1]
                while q and q[0][0] >= latency_ms:
                    delayed = q.pop(0)[1]
                produced = delayed
            quant = int(max(0, min(maxv, round(produced))))
            out.append(quant)
            if cfg.verbose_sensor_log:
                self._last_sensor_debug[f"s{i}_raw"] = raw
                self._last_sensor_debug[f"s{i}_calibrated"] = calibrated
                self._last_sensor_debug[f"s{i}_filtered"] = float(rt["filtered"])
                self._last_sensor_debug[f"s{i}_latency_output"] = produced
        return out

    def _coverage_from_raster_batch(self, sx: list[float], sy: list[float], size_mm: float, grid_n: int = 3) -> list[float]:
        meta = getattr(self, "_rmap_meta", None)
        data = getattr(self, "_rmap_data", None)
        if not meta or not data:
            return [0.0 for _ in sx]
        W = int(meta["W"]); H = int(meta["H"])
        origin_x = float(meta["origin_x"]); origin_y = float(meta["origin_y"])
        pix = float(meta.get("pixel_mm", 1.0)) or 1.0
        mv = memoryview(data)
        half = float(size_mm) * 0.5
        if grid_n <= 1:
            offs = [(0.0, 0.0)]
        else:
            step = (2.0 * half) / (grid_n - 1)
            offs = [(i * step - half, j * step - half) for j in range(grid_n) for i in range(grid_n)]
        denom = float(len(offs))
        out: list[float] = []
        for cx, cy in zip(sx, sy):
            k = 0
            for dx, dy in offs:
                wx = cx + dx; wy = cy + dy
                px = int((wx - origin_x) // pix)
                py = int((wy - origin_y) // pix)
                if 0 <= px < W and 0 <= py < H and mv[py * W + px] != 0:
                    k += 1
            out.append(k / denom if denom > 0 else 0.0)
        return out

    def _apply_marker_overrides(self, sx: list[float], sy: list[float], base_cov: list[float]) -> list[float]:
        if not getattr(self, "_markers_obb", None):
            return base_cov
        out = list(base_cov)
        for i in range(len(sx)):
            px, py = sx[i], sy[i]
            for (mcx, mcy, mux, muy, mvx, mvy, mhalf_l, mhalf_w) in self._markers_obb:
                if point_in_obb(px, py, mcx, mcy, mux, muy, mhalf_l, mvx, mvy, mhalf_w):
                    out[i] = 1.0
                    break
        return out

    def _prepare_track_geometry(self):
        segs, origin, tape_w = segments_from_track(self.track)
        pts = segments_polyline(segs, step=1.0)
        n = len(pts)
        poly_arr = (CPoint * n)(*[(CPoint(p.x, p.y)) for p in pts])
        self._poly_ptr, self._poly_n = poly_arr, n
        self._segs, self._origin, self._tapeW = segs, origin, tape_w
        self._gates = start_finish_lines(self.track)
        self._markers_obb = build_markers(segs, tape_w, self._gates)

        if self.track_path:
            info = ensure_track_raster(self.track_path, self.track, segs, tape_w, self._gates)
            self._rmap_meta = info["meta"]
            self._rmap_data = info["data"]
            self._rmap_arr = (ctypes.c_ubyte * len(self._rmap_data)).from_buffer_copy(self._rmap_data)
            self._rmap_ptr = ctypes.cast(self._rmap_arr, ctypes.POINTER(ctypes.c_ubyte))
        else:
            self._rmap_meta = None
            self._rmap_data = b""
            self._rmap_ptr = None

        return segs, origin, tape_w

    def iter_chunks(self, chunk_size: int = 200) -> Iterator[list[dict[str, Any]]]:
        self._prepare_track_geometry()

        start_pose = self._initial_pose()
        dt = float(self.dt_s)
        t_ms = 0

        current_state = SimulationState(
            t_ms=t_ms,
            x_mm=start_pose.p.x,
            y_mm=start_pose.p.y,
            heading_deg=float(start_pose.headingDeg),
            v_mm_s=0.0,
            omega_rad_s=0.0,
            a_lin_mm_s2=0.0,
            alpha_rad_s2=0.0,
            v_left_mm_s=0.0,
            v_right_mm_s=0.0,
            sensors=[],
        )
        self.physics_model.reset(current_state)

        native = self._native_or_policy()

        zone = None
        if self._gates:
            (sa, sb, _shdg_run, _shdg_base), (fa, fb, *_rest) = self._gates
            zone = FinishZoneChecker(sa, sb, fa, fb)
            zone.prime(current_state.x_mm, current_state.y_mm)

        chunk_buf: list[dict[str, Any]] = []

        env_w = float(self.robot.envelope.width_mm)
        env_h = float(self.robot.envelope.height_mm)
        sensor_half = float(self.robot.sensors[0].size_mm) * 0.5 if self.robot.sensors else 2.5
        max_time_ms = int(float(self.params.get("max_time_s", 100.0)) * 1000.0)

        while not self.cancelled:
            sx, sy = self._sensors_world_xy(current_state.x_mm, current_state.y_mm, current_state.heading_deg)
            cov = self._coverage_from_raster_batch(sx, sy, sensor_half * 2.0, grid_n=3)
            sn_vals = [sensor_value_from_coverage(
                cov[i], self.sensor_mode, self.sensor_bits,
                self.value_of_line, self.value_of_background,
                self.analog_noise_line, self.analog_noise_background,
            ) for i in range(len(cov))]
            sn_vals = self._process_sensor_values(sn_vals, dt)

            controller_state = replace(current_state, t_ms=t_ms, sensors=sn_vals)

            try:
                out = self.controller_fn(controller_state.to_controller_state(dt))
                pwm_l = int(out.get("pwm_left", 1500)) if isinstance(out, dict) else 1500
                pwm_r = int(out.get("pwm_right", 1500)) if isinstance(out, dict) else 1500
            except Exception as e:
                print(f"[Controller Error] {e}")
                pwm_l, pwm_r = 1500, 1500

            px_prev, py_prev, h_prev = current_state.x_mm, current_state.y_mm, current_state.heading_deg
            next_state = self.physics_model.step(
                controller_state,
                pwm_l,
                pwm_r,
                dt,
                self.robot,
                self.config,
                native=native,
            )
            current_state = next_state

            try:
                if self._rmap_ptr and self._rmap_meta:
                    collision_native = native if native is not None else self._native()
                    cx, cy = robot_local_to_world(
                        self.robot,
                        current_state.x_mm,
                        current_state.y_mm,
                        current_state.heading_deg,
                        0.0,
                        0.0,
                    )
                    hit = collision_native.envelope_contacts_raster_C(
                        c_double(cx), c_double(cy), c_double(math.radians(current_state.heading_deg)),
                        c_double(env_w), c_double(env_h),
                        self._rmap_ptr, c_int(self._rmap_meta["W"]), c_int(self._rmap_meta["H"]),
                        c_double(self._rmap_meta["origin_x"]), c_double(self._rmap_meta["origin_y"]), c_double(self._rmap_meta["pixel_mm"]),
                    )
                else:
                    hit = 1
            except Exception:
                hit = 1

            finished = False
            if zone is not None:
                finished = zone.update(
                    (px_prev, py_prev, h_prev),
                    (current_state.x_mm, current_state.y_mm, current_state.heading_deg),
                    env_w,
                    env_h,
                    t_ms,
                )

            step = current_state.to_step_dict()
            log_record = dict(step)
            log_record.update({
                "t_ms": t_ms,
                "dt_s": dt,
                "physics_profile": str(self.config.physics_profile),
                "pwm_left": int(pwm_l),
                "pwm_right": int(pwm_r),
                "sensors": list(sn_vals),
                "hit": int(hit),
                "finished": bool(finished),
            })
            if self._last_sensor_debug:
                log_record.update(self._last_sensor_debug)
            self.logger.log_step(step_data=log_record)
            chunk_buf.append(step)
            if len(chunk_buf) >= chunk_size:
                yield chunk_buf
                chunk_buf = []

            t_ms += int(round(dt * 1000.0))
            current_state.t_ms = t_ms
            if finished or (self._rmap_ptr and not hit):
                break
            if t_ms > max_time_ms:
                break

        if chunk_buf:
            yield chunk_buf
        self.logger.flush()

    def iter_steps(self) -> Iterator[dict[str, Any]]:
        for chunk in self.iter_chunks(chunk_size=1):
            for step in chunk:
                yield step

    def run(self) -> SimulationResult:
        steps: list[SimulationState] = []
        for step in self.iter_steps():
            steps.append(SimulationState.from_step_dict(step))
        return SimulationResult(steps=steps, summary={"dt_s": self.dt_s})
