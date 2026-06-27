from __future__ import annotations

import ctypes
import math
import random
from typing import Any

from Utils.robot_runtime import derive_wheel_track_mm
from Utils.robot_spec import RobotSpec
from Utils.simulation_config import SimulationConfig, derive_runtime_params
from Utils.simulation_state import SimulationState

from sim.native_linesim import EXPECTED_LINESIM_ABI_VERSION, PhysicsConfigC, PhysicsInputC, PhysicsStateC, PhysicsTelemetryC, backend_info, has_modular_physics
from sim.physics.base import (
    PhysicsModel,
    auto_max_wheel_speed_mm_s,
    auto_wheel_accel_mm_s2,
    clamp,
    integrate_differential_pose,
)
from sim.physics.dc_motor import DCMotorPhysicsModel
from sim.physics.kinematic import BasicKinematicPhysicsModel


_G = 9.81


def _sign(value: float) -> float:
    if value > 0.0:
        return 1.0
    if value < 0.0:
        return -1.0
    return 0.0


def _round_half_away_from_zero(value: float) -> int:
    if value >= 0.0:
        return int(math.floor(value + 0.5))
    return int(math.ceil(value - 0.5))


class CustomPhysicsModel(PhysicsModel):
    """Configurable Phase 4.3 physics pipeline.

    The model still accepts the modular C backend when available, but all user
    facing physical channels are finalized in Python so the same invariants are
    enforced in C, Python fallback, custom and realistic profiles:

    * exposed current is consumption and is never negative;
    * signed motor current is kept separately for torque direction;
    * slip is calculated from demanded wheel force versus tire friction unless
      custom manual slip is explicitly requested;
    * encoder ticks are integers and use wheel surface speed before slip;
    * IMU acceleration is reported in the robot body frame.
    """

    def __init__(self, config: SimulationConfig, robot: RobotSpec | None = None, params: dict[str, Any] | None = None):
        self.config = config
        self.params = dict(params or {})
        self.requires_native = True
        self.is_realistic_preset = False
        self._dc = DCMotorPhysicsModel(robot=robot, params=params) if bool(config.custom_use_dc_motor_model) else None
        self._kinematic = BasicKinematicPhysicsModel(
            use_acceleration_limit=config.custom_use_acceleration_limit,
            params=params,
        )
        self._battery_soc = float(config.custom_battery_soc_initial)
        self._battery_voltage_v = float(config.custom_battery_initial_voltage_v)
        self._battery_current_a_prev = 0.0
        self._enc_left_ticks_float = 0.0
        self._enc_right_ticks_float = 0.0
        self._enc_left_ticks_int = 0
        self._enc_right_ticks_int = 0
        self._enc_elapsed_s = 0.0
        self._imu_elapsed_s = 0.0
        self._last_imu = (0.0, 0.0, 0.0, 0.0)
        self._motor_current_left_signed_a = 0.0
        self._motor_current_right_signed_a = 0.0
        # Wheel-peripheral speed before slip.  This must feed back into back-EMF
        # and encoder; ground speed is derived later after slip.
        self._model_v_left_mm_s = 0.0
        self._model_v_right_mm_s = 0.0
        self._model_v_mm_s = 0.0
        self._model_omega_rad_s = 0.0
        seed = config.custom_sensor_noise_seed if config.custom_sensor_noise_seed is not None else 12345
        self._rng = random.Random(int(seed))
        self.c_step_call_count = 0
        self.last_c_backend_error = ""
        self.last_c_backend_path = ""
        self.last_c_backend_loaded = False
        self.last_c_modular_available = False
        self.last_using_python_fallback = False
        if robot is not None and not self.params:
            self.params = derive_runtime_params(robot, config)

    def reset(self, initial_state: SimulationState) -> None:
        self._battery_soc = clamp(float(self.config.custom_battery_soc_initial), 0.0, 1.0)
        self._battery_voltage_v = max(0.0, float(self.config.custom_battery_initial_voltage_v))
        self._battery_current_a_prev = 0.0
        self._enc_left_ticks_float = float(initial_state.enc_left_ticks)
        self._enc_right_ticks_float = float(initial_state.enc_right_ticks)
        self._enc_left_ticks_int = int(round(float(initial_state.enc_left_ticks)))
        self._enc_right_ticks_int = int(round(float(initial_state.enc_right_ticks)))
        self._enc_elapsed_s = 0.0
        self._imu_elapsed_s = 0.0
        self._last_imu = (
            float(initial_state.imu_omega_rad_s),
            float(initial_state.imu_alpha_rad_s2),
            float(initial_state.imu_accel_x_mm_s2),
            float(initial_state.imu_accel_y_mm_s2),
        )
        self._motor_current_left_signed_a = float(initial_state.motor_left_current_signed_a)
        self._motor_current_right_signed_a = float(initial_state.motor_right_current_signed_a)
        self._model_v_left_mm_s = float(initial_state.wheel_left_surface_speed_mm_s or initial_state.v_left_mm_s)
        self._model_v_right_mm_s = float(initial_state.wheel_right_surface_speed_mm_s or initial_state.v_right_mm_s)
        self._model_v_mm_s = 0.5 * (self._model_v_left_mm_s + self._model_v_right_mm_s)
        track_mm = max(1e-9, float(self.params.get("track_m", 0.07)) * 1000.0)
        self._model_omega_rad_s = (self._model_v_right_mm_s - self._model_v_left_mm_s) / track_mm
        self.c_step_call_count = 0
        self.last_using_python_fallback = False
        if self._dc is not None:
            self._dc.reset(initial_state)
        self._kinematic.reset(initial_state)

    def _max_accel(self, config: SimulationConfig) -> float:
        if config.custom_use_auto_acceleration_limit:
            return auto_wheel_accel_mm_s2(config.basic_max_wheel_accel_mm_s2, self.params)
        return max(1e-9, float(config.custom_max_wheel_accel_mm_s2))

    def _pwm_limits(self, robot: RobotSpec, config: SimulationConfig) -> tuple[float, float, float]:
        ctrl = robot.controller
        pwm_min = float(self.params.get("pwm_min", ctrl.pwm_min))
        pwm_max = float(config.custom_max_pwm if config.custom_max_pwm is not None else self.params.get("pwm_max", ctrl.pwm_max))
        if config.custom_max_pwm is not None and pwm_min < 0.0:
            pwm_min = -abs(float(config.custom_max_pwm))
        if pwm_max <= pwm_min:
            pwm_min, pwm_max = -4095.0, 4095.0
        neutral_raw = self.params.get("pwm_neutral", ctrl.pwm_neutral)
        pwm_center = float(neutral_raw) if neutral_raw is not None else 0.5 * (pwm_min + pwm_max)
        pwm_center = clamp(pwm_center, pwm_min, pwm_max)
        return pwm_min, pwm_max, pwm_center

    def _duty(self, pwm: int, robot: RobotSpec, config: SimulationConfig) -> float:
        pwm_min, pwm_max, center = self._pwm_limits(robot, config)
        raw = clamp(float(pwm), pwm_min, pwm_max)
        if raw >= center:
            denom = max(1e-12, pwm_max - center)
            duty = (raw - center) / denom
        else:
            denom = max(1e-12, center - pwm_min)
            duty = (raw - center) / denom
        deadband = clamp(float(robot.controller.deadband_percent) * 0.01, 0.0, 0.95)
        if config.custom_motor_deadzone_pwm > 0.0:
            span = max(abs(pwm_max - center), abs(center - pwm_min), 1.0)
            deadband = max(deadband, clamp(float(config.custom_motor_deadzone_pwm) / span, 0.0, 0.95))
        if abs(duty) <= deadband:
            return 0.0
        if deadband > 0.0:
            duty = (abs(duty) - deadband) / max(1e-12, 1.0 - deadband) * _sign(duty)
        return clamp(duty, -1.0, 1.0)

    def _ocv_from_soc(self, robot: RobotSpec, soc: float, config: SimulationConfig) -> float:
        soc = clamp(float(soc), 0.0, 1.0)
        table = list(getattr(robot.electrical, "battery_ocv_table", []) or [])
        if table:
            pts = sorted((float(a), float(b)) for a, b in table)
            if soc <= pts[0][0]:
                return pts[0][1]
            if soc >= pts[-1][0]:
                return pts[-1][1]
            for (s0, v0), (s1, v1) in zip(pts, pts[1:]):
                if s0 <= soc <= s1:
                    if abs(s1 - s0) < 1e-12:
                        return v1
                    t = (soc - s0) / (s1 - s0)
                    return v0 + t * (v1 - v0)
        return float(config.custom_battery_min_voltage_v) + soc * max(0.0, float(config.custom_battery_initial_voltage_v) - float(config.custom_battery_min_voltage_v))

    def _current_limit(self, robot: RobotSpec, config: SimulationConfig) -> float:
        if config.custom_current_limit_a > 0.0:
            return float(config.custom_current_limit_a)
        motor = robot.motor_transmission
        if motor.driver_current_limit_A > 0.0:
            return float(motor.driver_current_limit_A)
        if motor.stall_current_A > 0.0:
            return float(motor.stall_current_A)
        return float(self.params.get("I_max_A", 3.0))

    def _motor_channel(
        self,
        *,
        pwm: int,
        wheel_surface_speed_mm_s: float,
        prev_current_signed_a: float,
        robot: RobotSpec,
        config: SimulationConfig,
        dt_s: float,
    ) -> dict[str, float]:
        motor = robot.motor_transmission
        elec = robot.electrical
        wheel_r_m = max(1e-12, float(robot.geometric_mechanical.wheel_radius_mm) * 0.001)
        gear = max(1e-12, float(motor.gear_ratio))
        eta = clamp(float(motor.eta), 1e-9, 1.0)
        duty = self._duty(pwm, robot, config)
        wheel_omega = (float(wheel_surface_speed_mm_s) * 0.001) / wheel_r_m
        motor_omega = wheel_omega * gear
        kv_rad_per_v = float(getattr(motor, "Kv_rad_per_V", 0.0) or 0.0)
        if kv_rad_per_v <= 1e-12 and motor.Kv_rpm_per_V > 0.0:
            kv_rad_per_v = motor.Kv_rpm_per_V * 2.0 * math.pi / 60.0
        ke = 1.0 / kv_rad_per_v if kv_rad_per_v > 1e-12 else float(self.params.get("Ke_V_per_rad", 1.0 / 500.0))
        ocv = self._ocv_from_soc(robot, self._battery_soc, config)
        bus_sag = self._battery_current_a_prev * max(0.0, elec.r_batt_ohm + elec.wiring_r_ohm)
        terminal_v_est = max(0.0, ocv - bus_sag)
        driver_available_v = max(0.0, terminal_v_est - max(0.0, elec.driver_drop_v))
        v_applied = duty * driver_available_v
        back_emf = ke * motor_omega
        rm = max(1e-12, float(motor.Rm_ohm))
        target_i = (v_applied - back_emf) / rm
        lm = max(0.0, float(motor.Lm_H))
        if lm > 0.0:
            alpha = 1.0 - math.exp(-max(0.0, dt_s) * rm / max(1e-12, lm))
            current_signed = float(prev_current_signed_a) + alpha * (target_i - float(prev_current_signed_a))
        else:
            current_signed = target_i
        limit = self._current_limit(robot, config)
        if limit > 0.0:
            current_signed = clamp(current_signed, -limit, limit)
        current_abs = abs(current_signed)
        if abs(duty) > 0.0 and motor.I0_A > 0.0:
            current_abs = max(current_abs, float(motor.I0_A))
        tau_motor_em = float(motor.Kt_Nm_per_A) * current_signed
        tau_visc = float(motor.b_visc_Nm_per_radps) * motor_omega
        if abs(motor_omega) > 1e-9:
            tau_coul = float(motor.tau_coulomb_Nm) * _sign(motor_omega)
        else:
            # Static Coulomb friction opposes the requested electromagnetic torque.
            tau_coul = min(abs(tau_motor_em), float(motor.tau_coulomb_Nm)) * _sign(tau_motor_em)
        tau_motor_net = tau_motor_em - tau_visc - tau_coul
        tau_wheel = tau_motor_net * gear * eta
        mech_power = tau_wheel * wheel_omega
        return {
            "duty": duty,
            "current_signed_a": current_signed,
            "current_a": current_abs,
            "voltage_v": v_applied,
            "back_emf_v": back_emf,
            "motor_torque_nm": tau_motor_net,
            "wheel_torque_nm": tau_wheel,
            "mechanical_power_w": mech_power,
        }

    def _manual_slip_enabled(self, config: SimulationConfig) -> bool:
        if getattr(config, "custom_use_manual_slip", False):
            return True
        # Backward compatibility: existing custom configurations that set
        # custom_slip_ratio_* still mean manual slip.  Realistic never forces
        # slip this way; it remains torque/friction based.
        if not self.is_realistic_preset:
            return abs(config.custom_slip_ratio_left) > 0.0 or abs(config.custom_slip_ratio_right) > 0.0
        return False

    def _slip_for_side(
        self,
        *,
        side: str,
        wheel_torque_nm: float,
        ground_speed_mm_s: float,
        robot: RobotSpec,
        config: SimulationConfig,
    ) -> tuple[float, float, float]:
        gm = robot.geometric_mechanical
        wheel_r_m = max(1e-12, float(gm.wheel_radius_mm) * 0.001)
        n_per_wheel = max(1e-12, float(gm.mass_kg) * _G * 0.5)
        f_static_max = max(0.0, float(gm.mu_static)) * n_per_wheel
        f_kinetic_max = max(0.0, float(gm.mu_kinetic)) * n_per_wheel
        f_command = float(wheel_torque_nm) / wheel_r_m
        sign_ref = _sign(float(ground_speed_mm_s)) or _sign(f_command)
        f_rr = max(0.0, float(gm.Crr)) * n_per_wheel * sign_ref
        f_drive = f_command - f_rr
        if abs(f_drive) <= f_static_max or abs(f_drive) < 1e-12:
            return 0.0, f_drive, f_static_max
        f_ground = _sign(f_drive) * f_kinetic_max
        slip = clamp(1.0 - abs(f_ground) / max(abs(f_drive), 1e-12), 0.0, 0.95)
        return slip, f_ground, f_static_max

    def _encoder_enabled(self, robot: RobotSpec, config: SimulationConfig) -> bool:
        if not config.custom_use_encoder_model:
            return False
        enc = getattr(robot, "encoder", {}) or {}
        if (not self.is_realistic_preset) and enc.get("enable") is False:
            return False
        return True

    def _encoder_ticks_per_rev(self, robot: RobotSpec, config: SimulationConfig) -> int:
        enc = getattr(robot, "encoder", {}) or {}
        for key in ("ppr", "ticks_per_rev", "cpr"):
            try:
                value = int(enc.get(key, 0))
            except Exception:
                value = 0
            if value > 0:
                return value
        return max(1, int(config.custom_encoder_ticks_per_rev))

    def _encoder_noise_std(self, robot: RobotSpec, config: SimulationConfig) -> float:
        enc = getattr(robot, "encoder", {}) or {}
        return max(float(config.custom_encoder_noise_std_ticks), float(enc.get("noise_std_pulses", 0.0) or 0.0))

    def _encoder_update_due(self, robot: RobotSpec, dt_s: float) -> bool:
        enc = getattr(robot, "encoder", {}) or {}
        rate = float(enc.get("update_rate_Hz", 0.0) or 0.0)
        if rate <= 0.0:
            return True
        self._enc_elapsed_s += max(0.0, float(dt_s))
        period = 1.0 / rate
        if self._enc_elapsed_s + 1e-12 < period:
            return False
        self._enc_elapsed_s = math.fmod(self._enc_elapsed_s, period)
        return True

    def _imu_enabled(self, robot: RobotSpec, config: SimulationConfig) -> bool:
        if not config.custom_use_imu_model:
            return False
        imu = getattr(robot, "imu", {}) or {}
        if (not self.is_realistic_preset) and imu.get("enable") is False:
            return False
        return True

    def _imu_update_due(self, robot: RobotSpec, dt_s: float) -> bool:
        imu = getattr(robot, "imu", {}) or {}
        rate = float(imu.get("update_rate_Hz", 0.0) or 0.0)
        if rate <= 0.0:
            return True
        self._imu_elapsed_s += max(0.0, float(dt_s))
        period = 1.0 / rate
        if self._imu_elapsed_s + 1e-12 < period:
            return False
        self._imu_elapsed_s = math.fmod(self._imu_elapsed_s, period)
        return True

    def _native_abi_version(self, native: Any | None) -> int:
        if native is None:
            return 0
        try:
            return int(native.linesim_abi_version_C())
        except Exception:
            return 0

    def _build_c_config(self, robot: RobotSpec, config: SimulationConfig, dt_s: float) -> PhysicsConfigC:
        p = self.params or derive_runtime_params(robot, config)
        pwm_min, pwm_max, pwm_center = self._pwm_limits(robot, config)
        gm = robot.geometric_mechanical
        motor = robot.motor_transmission
        elec = robot.electrical
        return PhysicsConfigC(
            int(config.custom_use_dc_motor_model),
            int(config.custom_use_kinematic_model),
            int(config.custom_use_acceleration_limit),
            int(config.custom_use_battery_model),
            int(config.custom_use_wheel_slip),
            int(self._encoder_enabled(robot, config)),
            int(self._imu_enabled(robot, config)),
            float(dt_s),
            float(self._max_accel(config)),
            float(auto_max_wheel_speed_mm_s(config.basic_max_wheel_speed_mm_s, p)),
            float(self._ocv_from_soc(robot, self._battery_soc, config)),
            float(config.custom_battery_nominal_voltage_v),
            float(config.custom_battery_min_voltage_v),
            float(elec.battery_capacity_mah or config.custom_battery_capacity_mah),
            float(elec.r_batt_ohm + elec.wiring_r_ohm),
            float(self._battery_soc),
            float(config.custom_slip_ratio_left),
            float(config.custom_slip_ratio_right),
            int(self._encoder_ticks_per_rev(robot, config)),
            int(config.custom_encoder_quantization),
            float(p.get("track_m", derive_wheel_track_mm(robot) / 1000.0)),
            float(p.get("wheel_r_m", gm.wheel_radius_mm / 1000.0)),
            pwm_min,
            pwm_max,
            pwm_center,
            float(config.custom_motor_deadzone_pwm),
            float(self._current_limit(robot, config)),
            float(config.custom_drivetrain_efficiency or p.get("eta_drive", 0.9)),
            float(config.custom_viscous_friction if config.custom_viscous_friction > 0 else p.get("b_visc_Nm_per_radps", motor.b_visc_Nm_per_radps)),
            float(config.custom_coulomb_friction if config.custom_coulomb_friction > 0 else p.get("tau_coulomb_Nm", motor.tau_coulomb_Nm)),
            float(elec.driver_drop_v),
            float(p.get("Rm_ohm", motor.Rm_ohm)),
            float(p.get("Lm_H", motor.Lm_H)),
            float(p.get("Kt_Nm_per_A", motor.Kt_Nm_per_A)),
            float(p.get("Ke_V_per_rad", 1.0 / 500.0)),
            float(p.get("gear_ratio", motor.gear_ratio)),
            float(p.get("mass_kg", gm.mass_kg)),
            float(p.get("Jz_kgm2", gm.J_body_kgm2 if gm.J_body_kgm2 > 0 else 1e-4)),
            float(p.get("Crr", gm.Crr)),
            float(p.get("rho_air", 1.225)),
            float(p.get("CdA", 0.02)),
            float(gm.mu_static),
            float(gm.mu_kinetic),
            int(True),  # use_wheel_dynamics
            int(config.custom_use_wheel_slip),  # continuous slip travels with slip switch
            int(config.custom_use_lateral_slip),
            int(config.custom_use_combined_slip_limit),
            int(True),
            float(p.get("Jm_kgm2", motor.Jm_kgm2)),
            float(p.get("Jload_kgm2", motor.Jload_reflected_kgm2)),
            float(max(0.001, 0.03 * gm.mass_kg)),
            float(elec.r_batt_ohm),
            float(elec.wiring_r_ohm),
            float(config.custom_slip_stiffness_factor),
            float(config.custom_slip_at_limit),
            float(config.custom_slip_max_ratio),
            float(config.custom_mu_static_left),
            float(config.custom_mu_static_right),
            float(config.custom_mu_kinetic_left),
            float(config.custom_mu_kinetic_right),
        )

    def _build_c_state(self, state: SimulationState) -> PhysicsStateC:
        return PhysicsStateC(
            float(state.x_mm),
            float(state.y_mm),
            float(state.heading_deg),
            float(state.v_left_mm_s),
            float(state.v_right_mm_s),
            float(state.v_mm_s),
            float(state.omega_rad_s),
            float(state.a_lin_mm_s2),
            float(state.alpha_rad_s2),
            float(self._battery_voltage_v or state.battery_voltage_v),
            float(self._battery_soc),
            float(self._enc_left_ticks_int),
            float(self._enc_right_ticks_int),
            0.0,
            0.0,
            float(state.imu_omega_rad_s),
            float(state.imu_alpha_rad_s2),
            float(state.imu_accel_x_mm_s2),
            float(state.imu_accel_y_mm_s2),
            float(self._motor_current_left_signed_a),
            float(self._motor_current_right_signed_a),
            float(state.omega_wheel_left_rad_s or (self._model_v_left_mm_s * 0.001 / max(1e-12, self.params.get("wheel_r_m", 0.011)))),
            float(state.omega_wheel_right_rad_s or (self._model_v_right_mm_s * 0.001 / max(1e-12, self.params.get("wheel_r_m", 0.011)))),
            float(state.alpha_wheel_left_rad_s2),
            float(state.alpha_wheel_right_rad_s2),
            float(state.battery_energy_j),
            float(state.copper_loss_energy_j),
            float(state.driver_loss_energy_j),
            float(state.battery_internal_loss_energy_j),
            float(state.wiring_loss_energy_j),
            float(state.mechanical_friction_loss_energy_j),
            float(state.rolling_resistance_energy_j),
            float(state.tire_slip_loss_energy_j),
            float(state.brake_dissipated_energy_j),
        )

    def _from_c_state(
        self,
        c_state: PhysicsStateC,
        c_telem: PhysicsTelemetryC,
        state: SimulationState,
        pwm_left: int,
        pwm_right: int,
        dt_s: float,
        robot: RobotSpec,
        config: SimulationConfig,
        native: Any | None,
        backend_name: str,
    ) -> SimulationState:
        self._model_v_left_mm_s = float(c_telem.wheel_left_surface_speed_mm_s)
        self._model_v_right_mm_s = float(c_telem.wheel_right_surface_speed_mm_s)
        self._model_v_mm_s = 0.5 * (self._model_v_left_mm_s + self._model_v_right_mm_s)
        self._model_omega_rad_s = (self._model_v_right_mm_s - self._model_v_left_mm_s) / max(1e-9, derive_wheel_track_mm(robot))
        self._motor_current_left_signed_a = float(c_state.current_left_a)
        self._motor_current_right_signed_a = float(c_state.current_right_a)
        self._battery_soc = clamp(float(c_state.battery_soc), 0.0, 1.0)
        self._battery_voltage_v = max(0.0, float(c_state.battery_voltage_v))
        self._enc_left_ticks_int = int(round(float(c_state.enc_left_ticks)))
        self._enc_right_ticks_int = int(round(float(c_state.enc_right_ticks)))
        self._enc_left_ticks_float = float(self._enc_left_ticks_int)
        self._enc_right_ticks_float = float(self._enc_right_ticks_int)
        pwm_min, pwm_max, _center = self._pwm_limits(robot, config)
        heading = float(c_state.heading_deg)
        heading_wrapped = ((heading + 180.0) % 360.0) - 180.0
        abi = self._native_abi_version(native)
        return SimulationState(
            t_ms=state.t_ms,
            x_mm=float(c_state.x_mm),
            y_mm=float(c_state.y_mm),
            heading_deg=heading,
            heading_wrapped_deg=heading_wrapped,
            v_mm_s=float(c_state.v_mm_s),
            omega_rad_s=float(c_state.omega_rad_s),
            a_lin_mm_s2=float(c_state.a_lin_mm_s2),
            alpha_rad_s2=float(c_state.alpha_rad_s2),
            v_left_mm_s=float(c_state.v_left_mm_s),
            v_right_mm_s=float(c_state.v_right_mm_s),
            pwm_left=int(pwm_left),
            pwm_right=int(pwm_right),
            sensors=list(state.sensors),
            duty_left=float(c_telem.duty_left),
            duty_right=float(c_telem.duty_right),
            pwm_min=pwm_min,
            pwm_max=pwm_max,
            battery_voltage_v=float(c_state.battery_voltage_v),
            battery_soc=float(c_state.battery_soc),
            battery_current_a=float(c_telem.battery_current_a),
            current_left_a=max(0.0, float(c_telem.current_left_a)),
            current_right_a=max(0.0, float(c_telem.current_right_a)),
            current_total_a=max(0.0, float(c_telem.battery_current_a)),
            motor_left_current_a=max(0.0, float(c_telem.current_left_a)),
            motor_right_current_a=max(0.0, float(c_telem.current_right_a)),
            motor_left_current_signed_a=float(c_state.current_left_a),
            motor_right_current_signed_a=float(c_state.current_right_a),
            battery_power_w=float(c_telem.battery_power_w),
            motor_left_torque_nm=float(c_telem.tau_motor_net_left_nm),
            motor_right_torque_nm=float(c_telem.tau_motor_net_right_nm),
            wheel_left_torque_nm=float(c_telem.tau_wheel_drive_left_nm),
            wheel_right_torque_nm=float(c_telem.tau_wheel_drive_right_nm),
            tau_motor_em_left_nm=float(c_telem.tau_motor_em_left_nm),
            tau_motor_em_right_nm=float(c_telem.tau_motor_em_right_nm),
            tau_motor_viscous_left_nm=float(c_telem.tau_motor_viscous_left_nm),
            tau_motor_viscous_right_nm=float(c_telem.tau_motor_viscous_right_nm),
            tau_motor_coulomb_left_nm=float(c_telem.tau_motor_coulomb_left_nm),
            tau_motor_coulomb_right_nm=float(c_telem.tau_motor_coulomb_right_nm),
            tau_motor_net_left_nm=float(c_telem.tau_motor_net_left_nm),
            tau_motor_net_right_nm=float(c_telem.tau_motor_net_right_nm),
            tau_wheel_drive_left_nm=float(c_telem.tau_wheel_drive_left_nm),
            tau_wheel_drive_right_nm=float(c_telem.tau_wheel_drive_right_nm),
            tau_rolling_left_nm=float(c_telem.tau_rolling_left_nm),
            tau_rolling_right_nm=float(c_telem.tau_rolling_right_nm),
            tau_bearing_left_nm=float(c_telem.tau_bearing_left_nm),
            tau_bearing_right_nm=float(c_telem.tau_bearing_right_nm),
            tau_ground_left_nm=float(c_telem.tau_ground_left_nm),
            tau_ground_right_nm=float(c_telem.tau_ground_right_nm),
            tau_slip_loss_left_nm=float(c_telem.tau_slip_loss_left_nm),
            tau_slip_loss_right_nm=float(c_telem.tau_slip_loss_right_nm),
            brake_dissipated_power_w=float(c_telem.brake_dissipated_power_w),
            mechanical_power_left_w=float(c_telem.tau_wheel_drive_left_nm * c_state.omega_wheel_left_rad_s),
            mechanical_power_right_w=float(c_telem.tau_wheel_drive_right_nm * c_state.omega_wheel_right_rad_s),
            wheel_left_surface_speed_mm_s=float(c_telem.wheel_left_surface_speed_mm_s),
            wheel_right_surface_speed_mm_s=float(c_telem.wheel_right_surface_speed_mm_s),
            ground_left_speed_mm_s=float(c_telem.ground_left_speed_mm_s),
            ground_right_speed_mm_s=float(c_telem.ground_right_speed_mm_s),
            omega_wheel_left_rad_s=float(c_state.omega_wheel_left_rad_s),
            omega_wheel_right_rad_s=float(c_state.omega_wheel_right_rad_s),
            alpha_wheel_left_rad_s2=float(c_state.alpha_wheel_left_rad_s2),
            alpha_wheel_right_rad_s2=float(c_state.alpha_wheel_right_rad_s2),
            J_eq_left_kgm2=float(c_telem.j_eq_left_kgm2),
            J_eq_right_kgm2=float(c_telem.j_eq_right_kgm2),
            enc_left_ticks=int(round(float(c_state.enc_left_ticks))),
            enc_right_ticks=int(round(float(c_state.enc_right_ticks))),
            enc_left_delta_ticks=int(round(float(c_state.enc_left_delta_ticks))),
            enc_right_delta_ticks=int(round(float(c_state.enc_right_delta_ticks))),
            enc_left_rad_s=float(c_state.omega_wheel_left_rad_s) if self._encoder_enabled(robot, config) else 0.0,
            enc_right_rad_s=float(c_state.omega_wheel_right_rad_s) if self._encoder_enabled(robot, config) else 0.0,
            imu_omega_rad_s=float(c_state.imu_omega_rad_s),
            imu_alpha_rad_s2=float(c_state.imu_alpha_rad_s2),
            imu_accel_x_mm_s2=float(c_state.imu_accel_x_mm_s2),
            imu_accel_y_mm_s2=float(c_state.imu_accel_y_mm_s2),
            slip_ratio_left=float(c_telem.slip_ratio_left),
            slip_ratio_right=float(c_telem.slip_ratio_right),
            longitudinal_slip_left=float(c_telem.slip_ratio_left),
            longitudinal_slip_right=float(c_telem.slip_ratio_right),
            lateral_slip_left=float(c_telem.lateral_slip_left),
            lateral_slip_right=float(c_telem.lateral_slip_right),
            traction_force_left_n=float(c_telem.force_longitudinal_ground_left_n),
            traction_force_right_n=float(c_telem.force_longitudinal_ground_right_n),
            max_static_force_left_n=float(c_telem.force_longitudinal_max_left_n),
            max_static_force_right_n=float(c_telem.force_longitudinal_max_right_n),
            force_longitudinal_command_left_n=float(c_telem.force_longitudinal_command_left_n),
            force_longitudinal_command_right_n=float(c_telem.force_longitudinal_command_right_n),
            force_longitudinal_ground_left_n=float(c_telem.force_longitudinal_ground_left_n),
            force_longitudinal_ground_right_n=float(c_telem.force_longitudinal_ground_right_n),
            force_longitudinal_max_left_n=float(c_telem.force_longitudinal_max_left_n),
            force_longitudinal_max_right_n=float(c_telem.force_longitudinal_max_right_n),
            force_longitudinal_saturation_left=float(c_telem.force_longitudinal_saturation_left),
            force_longitudinal_saturation_right=float(c_telem.force_longitudinal_saturation_right),
            lambda_long_left=float(c_telem.lambda_long_left),
            lambda_long_right=float(c_telem.lambda_long_right),
            lateral_accel_mm_s2=float(c_telem.lateral_accel_mm_s2),
            lateral_force_total_n=float(c_telem.lateral_force_total_n),
            lateral_force_left_n=float(c_telem.lateral_force_left_n),
            lateral_force_right_n=float(c_telem.lateral_force_right_n),
            friction_usage_left=float(c_telem.friction_usage_left),
            friction_usage_right=float(c_telem.friction_usage_right),
            combined_friction_limit_left_n=float(c_telem.combined_friction_limit_left_n),
            combined_friction_limit_right_n=float(c_telem.combined_friction_limit_right_n),
            copper_loss_left_w=float(c_telem.copper_loss_left_w),
            copper_loss_right_w=float(c_telem.copper_loss_right_w),
            driver_loss_left_w=float(c_telem.driver_loss_left_w),
            driver_loss_right_w=float(c_telem.driver_loss_right_w),
            battery_internal_loss_w=float(c_telem.battery_internal_loss_w),
            wiring_loss_w=float(c_telem.wiring_loss_w),
            mechanical_friction_loss_left_w=float(c_telem.mechanical_friction_loss_left_w),
            mechanical_friction_loss_right_w=float(c_telem.mechanical_friction_loss_right_w),
            rolling_resistance_loss_w=float(c_telem.rolling_resistance_loss_w),
            tire_slip_loss_left_w=float(c_telem.tire_slip_loss_left_w),
            tire_slip_loss_right_w=float(c_telem.tire_slip_loss_right_w),
            kinetic_power_delta_w=float(c_telem.kinetic_power_delta_w),
            battery_energy_j=float(c_state.battery_energy_j),
            copper_loss_energy_j=float(c_state.copper_loss_energy_j),
            driver_loss_energy_j=float(c_state.driver_loss_energy_j),
            battery_internal_loss_energy_j=float(c_state.battery_internal_loss_energy_j),
            wiring_loss_energy_j=float(c_state.wiring_loss_energy_j),
            mechanical_friction_loss_energy_j=float(c_state.mechanical_friction_loss_energy_j),
            rolling_resistance_energy_j=float(c_state.rolling_resistance_energy_j),
            tire_slip_loss_energy_j=float(c_state.tire_slip_loss_energy_j),
            brake_dissipated_energy_j=float(c_state.brake_dissipated_energy_j),
            kinetic_energy_linear_j=float(c_telem.kinetic_energy_linear_j),
            kinetic_energy_angular_j=float(c_telem.kinetic_energy_angular_j),
            kinetic_energy_wheels_j=float(c_telem.kinetic_energy_wheels_j),
            total_kinetic_energy_j=float(c_telem.total_kinetic_energy_j),
            total_loss_energy_j=float(c_telem.total_loss_energy_j),
            energy_balance_error_j=float(c_telem.energy_balance_error_j),
            energy_balance_error_percent=float(c_telem.energy_balance_error_percent),
            physics_backend=backend_name,
            linesim_abi_version=abi,
            c_backend_loaded=True,
            c_backend_path=self.last_c_backend_path,
            c_backend_error="",
            c_modular_step_available=True,
            using_python_fallback=False,
            c_step_call_count=int(self.c_step_call_count),
            last_step_executed_in_c=bool(c_telem.step_executed_in_c),
        )

    def _step_modular_c(
        self,
        state: SimulationState,
        pwm_left: int,
        pwm_right: int,
        dt_s: float,
        robot: RobotSpec,
        config: SimulationConfig,
        native: Any,
    ) -> SimulationState | None:
        if native is None:
            self.last_c_backend_loaded = False
            self.last_c_modular_available = False
            return None
        info = backend_info(native)
        self.last_c_backend_loaded = info.loaded
        self.last_c_backend_path = info.path
        self.last_c_backend_error = info.error
        self.last_c_modular_available = info.modular_step_available
        if not has_modular_physics(native):
            if info.abi_version == 0:
                self.last_c_backend_error = "Backend C carregado não exporta linesim_abi_version_C. Recompile a DLL/SO."
            elif info.abi_version < EXPECTED_LINESIM_ABI_VERSION:
                self.last_c_backend_error = f"Backend C ABI {info.abi_version} incompatível; esperado >= {EXPECTED_LINESIM_ABI_VERSION}. Recompile a DLL/SO."
            elif not info.modular_step_available:
                self.last_c_backend_error = "Backend C não exporta step_physics_modular_C. Recompile a DLL/SO."
            return None
        c_input = PhysicsInputC(
            float(pwm_left),
            float(pwm_right),
            float(robot.controller.pwm_max),
            float(self._ocv_from_soc(robot, self._battery_soc, config)),
        )
        c_config = self._build_c_config(robot, config, dt_s)
        c_state = self._build_c_state(state)
        c_telem = PhysicsTelemetryC()
        rc = native.step_physics_modular_C(ctypes.byref(c_input), ctypes.byref(c_config), ctypes.byref(c_state), ctypes.byref(c_telem))
        if int(rc) != 0:
            raise RuntimeError(f"Backend C modular retornou erro {rc}.")
        self.c_step_call_count += 1
        backend_name = "realistic_c_modular" if self.is_realistic_preset else "custom_c_modular"
        return self._from_c_state(c_state, c_telem, state, pwm_left, pwm_right, dt_s, robot, config, native, backend_name)

    def _annotate_python_fallback(self, out: SimulationState, native: Any | None = None) -> SimulationState:
        info = backend_info(native) if native is not None else None
        out.physics_backend = "python_fallback_explicit" if self.last_using_python_fallback else out.physics_backend
        out.linesim_abi_version = info.abi_version if info else 0
        out.c_backend_loaded = bool(info.loaded) if info else False
        out.c_backend_path = info.path if info else self.last_c_backend_path
        out.c_backend_error = self.last_c_backend_error
        out.c_modular_step_available = bool(info.modular_step_available) if info else False
        out.using_python_fallback = bool(self.last_using_python_fallback)
        out.c_step_call_count = int(self.c_step_call_count)
        out.last_step_executed_in_c = False
        return out

    def _step_python_motor(
        self,
        prev: SimulationState,
        pwm_left: int,
        pwm_right: int,
        dt_s: float,
        robot: RobotSpec,
        config: SimulationConfig,
    ) -> SimulationState:
        dt = max(1e-9, float(dt_s))
        left_ch = self._motor_channel(
            pwm=pwm_left,
            wheel_surface_speed_mm_s=self._model_v_left_mm_s,
            prev_current_signed_a=self._motor_current_left_signed_a,
            robot=robot,
            config=config,
            dt_s=dt,
        )
        right_ch = self._motor_channel(
            pwm=pwm_right,
            wheel_surface_speed_mm_s=self._model_v_right_mm_s,
            prev_current_signed_a=self._motor_current_right_signed_a,
            robot=robot,
            config=config,
            dt_s=dt,
        )
        self._motor_current_left_signed_a = left_ch["current_signed_a"]
        self._motor_current_right_signed_a = right_ch["current_signed_a"]

        wheel_r_m = max(1e-12, float(robot.geometric_mechanical.wheel_radius_mm) * 0.001)
        motor = robot.motor_transmission
        gear = max(1e-12, float(motor.gear_ratio))
        j_eq = max(1e-9, float(motor.Jm_kgm2) * gear * gear + float(motor.Jload_reflected_kgm2) + 0.5 * float(robot.geometric_mechanical.mass_kg) * wheel_r_m * wheel_r_m)
        alpha_l = left_ch["wheel_torque_nm"] / j_eq
        alpha_r = right_ch["wheel_torque_nm"] / j_eq
        next_left = self._model_v_left_mm_s + alpha_l * wheel_r_m * 1000.0 * dt
        next_right = self._model_v_right_mm_s + alpha_r * wheel_r_m * 1000.0 * dt
        if config.custom_use_acceleration_limit:
            max_delta = self._max_accel(config) * dt
            next_left = self._model_v_left_mm_s + clamp(next_left - self._model_v_left_mm_s, -max_delta, max_delta)
            next_right = self._model_v_right_mm_s + clamp(next_right - self._model_v_right_mm_s, -max_delta, max_delta)
        self._model_v_left_mm_s = next_left
        self._model_v_right_mm_s = next_right
        self._model_v_mm_s = 0.5 * (next_left + next_right)
        self._model_omega_rad_s = (next_right - next_left) / max(1e-9, derive_wheel_track_mm(robot))
        base = SimulationState(
            t_ms=prev.t_ms,
            x_mm=prev.x_mm,
            y_mm=prev.y_mm,
            heading_deg=prev.heading_deg,
            v_mm_s=self._model_v_mm_s,
            omega_rad_s=self._model_omega_rad_s,
            a_lin_mm_s2=(self._model_v_mm_s - prev.v_mm_s) / dt,
            alpha_rad_s2=(self._model_omega_rad_s - prev.omega_rad_s) / dt,
            v_left_mm_s=next_left,
            v_right_mm_s=next_right,
            pwm_left=int(pwm_left),
            pwm_right=int(pwm_right),
            sensors=list(prev.sensors),
            physics_backend="python_dc",
        )
        # Recalculate channel telemetry at the new speed without advancing current.
        left_ch = self._motor_channel(pwm=pwm_left, wheel_surface_speed_mm_s=next_left, prev_current_signed_a=self._motor_current_left_signed_a, robot=robot, config=config, dt_s=0.0)
        right_ch = self._motor_channel(pwm=pwm_right, wheel_surface_speed_mm_s=next_right, prev_current_signed_a=self._motor_current_right_signed_a, robot=robot, config=config, dt_s=0.0)
        left_ch["current_signed_a"] = self._motor_current_left_signed_a
        right_ch["current_signed_a"] = self._motor_current_right_signed_a
        left_ch["current_a"] = abs(self._motor_current_left_signed_a)
        right_ch["current_a"] = abs(self._motor_current_right_signed_a)
        return self._finalize_surface_state(base, prev, dt, robot, config, pwm_left, pwm_right, left_ch, right_ch)

    def _finalize_surface_state(
        self,
        surface: SimulationState,
        prev: SimulationState,
        dt_s: float,
        robot: RobotSpec,
        config: SimulationConfig,
        pwm_left: int,
        pwm_right: int,
        left_ch: dict[str, float] | None = None,
        right_ch: dict[str, float] | None = None,
        *,
        prefer_existing_encoder: bool = False,
    ) -> SimulationState:
        dt = max(1e-9, float(dt_s))
        track = max(1e-9, float(derive_wheel_track_mm(robot)))
        base_left = float(surface.v_left_mm_s)
        base_right = float(surface.v_right_mm_s)
        if left_ch is None:
            left_ch = self._motor_channel(pwm=pwm_left, wheel_surface_speed_mm_s=base_left, prev_current_signed_a=self._motor_current_left_signed_a, robot=robot, config=config, dt_s=0.0)
        if right_ch is None:
            right_ch = self._motor_channel(pwm=pwm_right, wheel_surface_speed_mm_s=base_right, prev_current_signed_a=self._motor_current_right_signed_a, robot=robot, config=config, dt_s=0.0)

        slip_l = slip_r = 0.0
        traction_l = traction_r = 0.0
        static_l = static_r = 0.0
        if config.custom_use_wheel_slip:
            if self._manual_slip_enabled(config):
                slip_l = clamp(float(config.custom_slip_ratio_left) + self._rng.gauss(0.0, config.custom_slip_noise_std), 0.0, 0.95)
                slip_r = clamp(float(config.custom_slip_ratio_right) + self._rng.gauss(0.0, config.custom_slip_noise_std), 0.0, 0.95)
                _, traction_l, static_l = self._slip_for_side(side="left", wheel_torque_nm=left_ch["wheel_torque_nm"], ground_speed_mm_s=float(prev.v_left_mm_s), robot=robot, config=config)
                _, traction_r, static_r = self._slip_for_side(side="right", wheel_torque_nm=right_ch["wheel_torque_nm"], ground_speed_mm_s=float(prev.v_right_mm_s), robot=robot, config=config)
            else:
                slip_l, traction_l, static_l = self._slip_for_side(side="left", wheel_torque_nm=left_ch["wheel_torque_nm"], ground_speed_mm_s=float(prev.v_left_mm_s), robot=robot, config=config)
                slip_r, traction_r, static_r = self._slip_for_side(side="right", wheel_torque_nm=right_ch["wheel_torque_nm"], ground_speed_mm_s=float(prev.v_right_mm_s), robot=robot, config=config)
                if config.custom_slip_noise_std > 0.0:
                    slip_l = clamp(slip_l + self._rng.gauss(0.0, config.custom_slip_noise_std), 0.0, 0.95)
                    slip_r = clamp(slip_r + self._rng.gauss(0.0, config.custom_slip_noise_std), 0.0, 0.95)

        ground_left = base_left * (1.0 - slip_l)
        ground_right = base_right * (1.0 - slip_r)
        x, y, heading, v, omega = integrate_differential_pose(prev.x_mm, prev.y_mm, prev.heading_deg, ground_left, ground_right, track, dt)
        a = (v - prev.v_mm_s) / dt
        alpha = (omega - prev.omega_rad_s) / dt

        battery_current = max(0.0, float(left_ch["current_a"]) + float(right_ch["current_a"]))
        soc = self._battery_soc
        voltage = self._battery_voltage_v
        if config.custom_use_battery_model:
            cap_as = max(1e-9, float(robot.electrical.battery_capacity_mah or config.custom_battery_capacity_mah) * 3.6)
            soc = clamp(soc - battery_current * dt / cap_as, 0.0, 1.0)
            ocv = self._ocv_from_soc(robot, soc, config)
            voltage = max(float(config.custom_battery_min_voltage_v), ocv - battery_current * max(0.0, robot.electrical.r_batt_ohm + robot.electrical.wiring_r_ohm))
            self._battery_soc = soc
            self._battery_voltage_v = voltage
        else:
            voltage = self._ocv_from_soc(robot, soc, config)
        self._battery_current_a_prev = battery_current
        battery_power = max(0.0, voltage * battery_current)

        enc_l = self._enc_left_ticks_int
        enc_r = self._enc_right_ticks_int
        enc_ld = enc_rd = 0
        wheel_r_mm = max(1e-9, float(robot.geometric_mechanical.wheel_radius_mm))
        enc_l_rad_s = base_left / wheel_r_mm
        enc_r_rad_s = base_right / wheel_r_mm
        if self._encoder_enabled(robot, config):
            if prefer_existing_encoder and (surface.enc_left_delta_ticks != 0 or surface.enc_right_delta_ticks != 0 or surface.enc_left_ticks != 0 or surface.enc_right_ticks != 0):
                enc_ld = int(round(float(surface.enc_left_delta_ticks)))
                enc_rd = int(round(float(surface.enc_right_delta_ticks)))
                enc_l = int(round(float(surface.enc_left_ticks)))
                enc_r = int(round(float(surface.enc_right_ticks)))
                self._enc_left_ticks_int = enc_l
                self._enc_right_ticks_int = enc_r
                self._enc_left_ticks_float = float(enc_l)
                self._enc_right_ticks_float = float(enc_r)
            elif self._encoder_update_due(robot, dt):
                ticks_per_rad = self._encoder_ticks_per_rev(robot, config) / (2.0 * math.pi)
                d_l = (base_left * dt / wheel_r_mm) * ticks_per_rad
                d_r = (base_right * dt / wheel_r_mm) * ticks_per_rad
                noise = self._encoder_noise_std(robot, config)
                if noise > 0.0:
                    d_l += self._rng.gauss(0.0, noise)
                    d_r += self._rng.gauss(0.0, noise)
                self._enc_left_ticks_float += d_l
                self._enc_right_ticks_float += d_r
                new_l = _round_half_away_from_zero(self._enc_left_ticks_float) if config.custom_encoder_quantization else int(self._enc_left_ticks_float)
                new_r = _round_half_away_from_zero(self._enc_right_ticks_float) if config.custom_encoder_quantization else int(self._enc_right_ticks_float)
                enc_ld = new_l - self._enc_left_ticks_int
                enc_rd = new_r - self._enc_right_ticks_int
                enc_l = new_l
                enc_r = new_r
                self._enc_left_ticks_int = enc_l
                self._enc_right_ticks_int = enc_r
        else:
            enc_l = enc_r = enc_ld = enc_rd = 0
            enc_l_rad_s = enc_r_rad_s = 0.0

        imu_omega, imu_alpha, imu_ax, imu_ay = self._last_imu
        if self._imu_enabled(robot, config) and self._imu_update_due(robot, dt):
            imu = getattr(robot, "imu", {}) or {}
            gyro_std = math.radians(float(imu.get("std_deg", 0.0) or 0.0))
            gyro_std = max(gyro_std, float(config.custom_imu_gyro_noise_std_rad_s))
            gyro_bias = math.radians(float(imu.get("bias_deg_s", 0.0) or 0.0))
            accel_std = max(0.0, float(config.custom_imu_accel_noise_std_mm_s2))
            imu_omega = omega + gyro_bias + (self._rng.gauss(0.0, gyro_std) if gyro_std > 0.0 else 0.0)
            imu_alpha = alpha + (self._rng.gauss(0.0, gyro_std) if gyro_std > 0.0 else 0.0)
            # Body-frame IMU: X is longitudinal, Y is lateral/centripetal.  Do
            # not rotate by world heading here.
            imu_ax = a + (self._rng.gauss(0.0, accel_std) if accel_std > 0.0 else 0.0)
            imu_ay = v * omega + (self._rng.gauss(0.0, accel_std) if accel_std > 0.0 else 0.0)
            self._last_imu = (imu_omega, imu_alpha, imu_ax, imu_ay)
        elif not self._imu_enabled(robot, config):
            imu_omega = imu_alpha = imu_ax = imu_ay = 0.0
            self._last_imu = (0.0, 0.0, 0.0, 0.0)

        left_mech = float(left_ch["mechanical_power_w"])
        right_mech = float(right_ch["mechanical_power_w"])
        brake_diss = max(0.0, -left_mech) + max(0.0, -right_mech)
        pwm_min, pwm_max, _center = self._pwm_limits(robot, config)
        heading_wrapped = ((heading + 180.0) % 360.0) - 180.0

        return SimulationState(
            t_ms=prev.t_ms,
            x_mm=x,
            y_mm=y,
            heading_deg=heading,
            heading_wrapped_deg=heading_wrapped,
            v_mm_s=v,
            omega_rad_s=omega,
            a_lin_mm_s2=a,
            alpha_rad_s2=alpha,
            v_left_mm_s=ground_left,
            v_right_mm_s=ground_right,
            pwm_left=int(pwm_left),
            pwm_right=int(pwm_right),
            sensors=list(prev.sensors),
            duty_left=float(left_ch["duty"]),
            duty_right=float(right_ch["duty"]),
            pwm_min=pwm_min,
            pwm_max=pwm_max,
            battery_voltage_v=voltage,
            battery_soc=soc,
            battery_current_a=battery_current,
            current_left_a=max(0.0, float(left_ch["current_a"])),
            current_right_a=max(0.0, float(right_ch["current_a"])),
            current_total_a=battery_current,
            motor_left_current_a=max(0.0, float(left_ch["current_a"])),
            motor_right_current_a=max(0.0, float(right_ch["current_a"])),
            motor_left_current_signed_a=float(left_ch["current_signed_a"]),
            motor_right_current_signed_a=float(right_ch["current_signed_a"]),
            battery_power_w=battery_power,
            motor_left_voltage_v=float(left_ch["voltage_v"]),
            motor_right_voltage_v=float(right_ch["voltage_v"]),
            motor_left_back_emf_v=float(left_ch["back_emf_v"]),
            motor_right_back_emf_v=float(right_ch["back_emf_v"]),
            motor_left_torque_nm=float(left_ch["motor_torque_nm"]),
            motor_right_torque_nm=float(right_ch["motor_torque_nm"]),
            wheel_left_torque_nm=float(left_ch["wheel_torque_nm"]),
            wheel_right_torque_nm=float(right_ch["wheel_torque_nm"]),
            brake_dissipated_power_w=brake_diss,
            mechanical_power_left_w=left_mech,
            mechanical_power_right_w=right_mech,
            wheel_left_surface_speed_mm_s=base_left,
            wheel_right_surface_speed_mm_s=base_right,
            ground_left_speed_mm_s=ground_left,
            ground_right_speed_mm_s=ground_right,
            enc_left_ticks=enc_l,
            enc_right_ticks=enc_r,
            enc_left_delta_ticks=enc_ld,
            enc_right_delta_ticks=enc_rd,
            enc_left_rad_s=enc_l_rad_s,
            enc_right_rad_s=enc_r_rad_s,
            imu_omega_rad_s=imu_omega,
            imu_alpha_rad_s2=imu_alpha,
            imu_accel_x_mm_s2=imu_ax,
            imu_accel_y_mm_s2=imu_ay,
            slip_ratio_left=slip_l,
            slip_ratio_right=slip_r,
            traction_force_left_n=traction_l,
            traction_force_right_n=traction_r,
            max_static_force_left_n=static_l,
            max_static_force_right_n=static_r,
            physics_backend=surface.physics_backend or "python",
            linesim_abi_version=int(surface.linesim_abi_version),
        )

    def _apply_python_effects(
        self,
        out: SimulationState,
        prev: SimulationState,
        dt_s: float,
        robot: RobotSpec,
        config: SimulationConfig,
        pwm_left: int,
        pwm_right: int,
    ) -> SimulationState:
        # Treat the incoming model output as wheel-surface speed.  This keeps
        # encoder/back-EMF pre-slip and applies slip only to ground motion.
        self._model_v_left_mm_s = float(out.v_left_mm_s)
        self._model_v_right_mm_s = float(out.v_right_mm_s)
        self._model_v_mm_s = 0.5 * (self._model_v_left_mm_s + self._model_v_right_mm_s)
        self._model_omega_rad_s = (self._model_v_right_mm_s - self._model_v_left_mm_s) / max(1e-9, derive_wheel_track_mm(robot))
        out.physics_backend = out.physics_backend or "python_kinematic"
        return self._finalize_surface_state(out, prev, dt_s, robot, config, pwm_left, pwm_right)

    def step(
        self,
        state: SimulationState,
        pwm_left: int,
        pwm_right: int,
        dt_s: float,
        robot: RobotSpec,
        config: SimulationConfig,
        native: Any | None = None,
    ) -> SimulationState:
        effective = config
        if not self.params:
            self.params = derive_runtime_params(robot, config)

        want_c = bool(effective.custom_use_c_backend and effective.custom_use_dc_motor_model)
        fallback_allowed = bool(effective.custom_allow_python_fallback)
        if self.is_realistic_preset:
            want_c = bool(self.original_config.require_c_backend_for_realistic)
            fallback_allowed = bool(self.original_config.allow_python_fallback_for_realistic)

        if effective.custom_use_dc_motor_model:
            if want_c:
                modular = self._step_modular_c(state, pwm_left, pwm_right, dt_s, robot, effective, native)
                if modular is not None:
                    self.last_using_python_fallback = False
                    return modular
                if not fallback_allowed:
                    detail = self.last_c_backend_error or "backend C modular não carregado."
                    raise RuntimeError(
                        "Perfil realistic/custom exige backend C modular, mas ele não está disponível: "
                        f"{detail}"
                    )
                self.last_using_python_fallback = True
                return self._annotate_python_fallback(self._step_python_motor(state, pwm_left, pwm_right, dt_s, robot, effective), native)

            if native is not None:
                modular = self._step_modular_c(state, pwm_left, pwm_right, dt_s, robot, effective, native)
                if modular is not None:
                    return modular
            self.last_using_python_fallback = bool(fallback_allowed)
            out = self._step_python_motor(state, pwm_left, pwm_right, dt_s, robot, effective)
            if self.last_using_python_fallback:
                return self._annotate_python_fallback(out, native)
            return out
        else:
            self._kinematic.use_acceleration_limit = bool(effective.custom_use_acceleration_limit)
            base = self._kinematic.step(state, pwm_left, pwm_right, dt_s, robot, effective, native=None)
            base.physics_backend = "basic_python" if self.is_realistic_preset else "python_kinematic"
        return self._apply_python_effects(base, state, dt_s, robot, effective, pwm_left, pwm_right)


class RealisticPhysicsModel(CustomPhysicsModel):
    """Preset that enables the complete realistic/custom physical model."""

    def __init__(self, config: SimulationConfig, robot: RobotSpec | None = None, params: dict[str, Any] | None = None):
        self.original_config = config
        super().__init__(config.as_realistic_preset(), robot=robot, params=params)
        self.is_realistic_preset = True
        # Python fallback is now explicit and logged; the simulation no longer
        # needs to fail just because a stale DLL/SO is absent.
        self.requires_native = True

    def step(
        self,
        state: SimulationState,
        pwm_left: int,
        pwm_right: int,
        dt_s: float,
        robot: RobotSpec,
        config: SimulationConfig,
        native: Any | None = None,
    ) -> SimulationState:
        return super().step(state, pwm_left, pwm_right, dt_s, robot, self.config, native=native)
