from __future__ import annotations

import csv
import ctypes
import math
import os
import random
import time
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
    """Records steps/events and writes CSV + JSON logs under Logs/."""

    def __init__(self, base_dir: str):
        self.base_dir = base_dir
        os.makedirs(os.path.join(base_dir, "Logs"), exist_ok=True)
        run_id = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(base_dir, "Logs", f"sim_log_{run_id}.csv")
        self.json_path = os.path.join(base_dir, "Logs", f"sim_log_{run_id}.json")
        self.steps: list[dict[str, Any]] = []
        self.events: list[dict[str, Any]] = []
        self._max_sensors = 0

    def log_step(self, t_ms: int, x: float, y: float, h: float, v: float, w: float, pwm_l: int, pwm_r: int,
                 sensors: list[int] | None = None) -> None:
        vals = list(sensors) if sensors is not None else []
        self._max_sensors = max(self._max_sensors, len(vals))
        self.steps.append({
            "t_ms": t_ms, "x_mm": x, "y_mm": y, "heading_deg": h,
            "v_mm_s": v, "omega_rad_s": w, "pwm_left": pwm_l, "pwm_right": pwm_r,
            "sensors": vals,
        })

    def log_event(self, kind: str, t_ms: int, x: float, y: float, h: float, extra: dict | None = None) -> None:
        ev = {"event": kind, "t_ms": t_ms, "x_mm": x, "y_mm": y, "heading_deg": h}
        if extra:
            ev.update(extra)
        self.events.append(ev)

    def flush(self) -> None:
        try:
            with open(self.csv_path, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                base_cols = ["t_ms", "x_mm", "y_mm", "heading_deg", "v_mm_s", "omega_rad_s", "pwm_left", "pwm_right"]
                sn_cols = [f"s{i}" for i in range(self._max_sensors)]
                w.writerow(base_cols + sn_cols)
                for s in self.steps:
                    row = [s["t_ms"], s["x_mm"], s["y_mm"], s["heading_deg"], s["v_mm_s"], s["omega_rad_s"], s["pwm_left"], s["pwm_right"]]
                    vals = s.get("sensors", [])
                    row.extend([vals[i] if i < len(vals) else "" for i in range(self._max_sensors)])
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
        for sensor in self.robot.sensors:
            wx, wy = robot_local_to_world(self.robot, x, y, h_deg, sensor.x_mm, sensor.y_mm)
            sx.append(wx)
            sy.append(wy)
        return sx, sy

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

        native = self._native() if self.physics_model.requires_native else None

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

            controller_state = SimulationState(
                t_ms=t_ms,
                x_mm=current_state.x_mm,
                y_mm=current_state.y_mm,
                heading_deg=current_state.heading_deg,
                v_mm_s=current_state.v_mm_s,
                omega_rad_s=current_state.omega_rad_s,
                a_lin_mm_s2=current_state.a_lin_mm_s2,
                alpha_rad_s2=current_state.alpha_rad_s2,
                v_left_mm_s=current_state.v_left_mm_s,
                v_right_mm_s=current_state.v_right_mm_s,
                pwm_left=current_state.pwm_left,
                pwm_right=current_state.pwm_right,
                sensors=sn_vals,
            )

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
            self.logger.log_step(
                t_ms,
                current_state.x_mm,
                current_state.y_mm,
                current_state.heading_deg,
                current_state.v_mm_s,
                current_state.omega_rad_s,
                pwm_l,
                pwm_r,
                sensors=sn_vals,
            )
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
            steps.append(SimulationState(
                t_ms=int(step["t_ms"]),
                x_mm=float(step["x_mm"]),
                y_mm=float(step["y_mm"]),
                heading_deg=float(step["heading_deg"]),
                v_mm_s=float(step["v_mm_s"]),
                omega_rad_s=float(step["omega_rad_s"]),
                a_lin_mm_s2=float(step["a_lin_mm_s2"]),
                alpha_rad_s2=float(step["alpha_rad_s2"]),
                v_left_mm_s=float(step["v_left_mm_s"]),
                v_right_mm_s=float(step["v_right_mm_s"]),
                sensors=list(step.get("sensors", [])),
            ))
        return SimulationResult(steps=steps, summary={"dt_s": self.dt_s})
