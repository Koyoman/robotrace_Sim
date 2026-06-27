from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from typing import Any

from Utils.robot_spec import RobotSpec
from Utils.robot_runtime import derive_wheel_track_mm
from Utils.validation import ValidationError, as_bool, as_float, as_int, raise_if_errors


VALID_PHYSICS_PROFILES = {"ideal", "basic", "realistic", "custom"}


def _maybe_float(obj: dict[str, Any], key: str, path: str, errors: list[str]) -> float | None:
    if obj.get(key, None) is None:
        return None
    return as_float(obj.get(key), path, None, errors)


@dataclass(slots=True)
class SimulationConfig:
    final_linear_speed_mps: float = 2.0
    motor_time_constant_s: float = 0.010
    simulation_step_dt_ms: float = 1.0
    max_time_s: float = 100.0
    save_logs: bool = False
    random_seed: int | None = None

    physics_profile: str = "realistic"
    # None means "auto": derive a full-PWM wheel speed from the robot motor/battery data.
    # Explicit numeric values are still accepted for deterministic experiments.
    ideal_max_wheel_speed_mm_s: float | None = None
    basic_max_wheel_speed_mm_s: float | None = None
    basic_max_wheel_accel_mm_s2: float | None = None

    # Custom profile switches. Every flag is consumed by CustomPhysicsModel or SimulationEngine.
    custom_use_dc_motor_model: bool = True
    custom_use_kinematic_model: bool = True
    custom_use_acceleration_limit: bool = True
    custom_use_battery_model: bool = False
    custom_use_sensor_noise: bool = False
    custom_use_encoder_model: bool = False
    custom_use_imu_model: bool = False
    custom_use_wheel_slip: bool = False
    custom_use_track_imperfections: bool = False
    custom_use_manual_slip: bool = False

    # Phase 4.4 backend policy. Realistic now requires the ABI-compatible
    # modular C backend by default; Python fallback must be an explicit choice.
    require_c_backend_for_realistic: bool = True
    allow_python_fallback_for_realistic: bool = False
    custom_use_c_backend: bool = True
    custom_allow_python_fallback: bool = False

    custom_use_auto_acceleration_limit: bool = True
    custom_max_wheel_accel_mm_s2: float = 9810.0

    custom_battery_initial_voltage_v: float = 7.4
    custom_battery_nominal_voltage_v: float = 7.4
    custom_battery_min_voltage_v: float = 6.0
    custom_battery_capacity_mah: float = 1000.0
    custom_battery_internal_resistance_ohm: float = 0.15
    custom_battery_soc_initial: float = 1.0

    custom_sensor_noise_std: float = 0.02
    custom_sensor_noise_seed: int = 12345
    sensor_common_noise_std: float = 0.0
    sensor_individual_noise_std: float = 0.0
    sensor_filter_tau_ms: float = 0.0
    sensor_latency_ms: float = 0.0
    sensor_update_rate_Hz: float = 0.0
    sensor_gain_default: float = 1.0
    sensor_offset_default: float = 0.0
    verbose_sensor_log: bool = False

    custom_encoder_ticks_per_rev: int = 1024
    custom_encoder_noise_std_ticks: float = 0.0
    custom_encoder_quantization: bool = True

    custom_imu_gyro_noise_std_rad_s: float = 0.0
    custom_imu_accel_noise_std_mm_s2: float = 0.0

    custom_slip_ratio_left: float = 0.0
    custom_slip_ratio_right: float = 0.0
    custom_slip_noise_std: float = 0.0
    custom_slip_stiffness_factor: float = 0.03
    custom_slip_at_limit: float = 0.05
    custom_slip_max_ratio: float = 0.95
    custom_use_combined_slip_limit: bool = True
    custom_use_lateral_slip: bool = True
    custom_mu_static_left: float = 0.0
    custom_mu_static_right: float = 0.0
    custom_mu_kinetic_left: float = 0.0
    custom_mu_kinetic_right: float = 0.0

    custom_track_imperfection_amplitude_mm: float = 0.0
    custom_track_imperfection_wavelength_mm: float = 500.0
    custom_track_imperfection_noise_std: float = 0.0

    custom_motor_deadzone_pwm: float = 0.0
    custom_current_limit_a: float = 0.0
    custom_drivetrain_efficiency: float = 0.9
    custom_viscous_friction: float = 0.0
    custom_coulomb_friction: float = 0.0
    custom_pwm_saturation_enabled: bool = True
    custom_max_pwm: float | None = None

    sensor_mode: str = "analog"
    sensor_bits: int = 8
    value_of_line: int = 0
    value_of_background: int = 255
    analog_noise_line: int = 50
    analog_noise_background: int = 50

    @property
    def dt_s(self) -> float:
        return self.simulation_step_dt_ms / 1000.0

    @classmethod
    def from_dict(cls, obj: dict[str, Any]) -> "SimulationConfig":
        if not isinstance(obj, dict):
            raise ValidationError("Configuração de simulação inválida.", ["Raiz deve ser objeto JSON."])
        errors: list[str] = []
        seed_raw = obj.get("random_seed", None)
        cfg = cls(
            final_linear_speed_mps=as_float(obj.get("final_linear_speed_mps", 2.0), "$.final_linear_speed_mps", 2.0, errors),
            motor_time_constant_s=as_float(obj.get("motor_time_constant_s", 0.010), "$.motor_time_constant_s", 0.010, errors),
            simulation_step_dt_ms=as_float(obj.get("simulation_step_dt_ms", 1.0), "$.simulation_step_dt_ms", 1.0, errors),
            max_time_s=as_float(obj.get("max_time_s", 100.0), "$.max_time_s", 100.0, errors),
            save_logs=as_bool(obj.get("save_logs", False), "$.save_logs", False, errors),
            random_seed=None if seed_raw is None else as_int(seed_raw, "$.random_seed", 0, errors),
            physics_profile=str(obj.get("physics_profile", "realistic")).strip().lower(),
            ideal_max_wheel_speed_mm_s=_maybe_float(obj, "ideal_max_wheel_speed_mm_s", "$.ideal_max_wheel_speed_mm_s", errors),
            basic_max_wheel_speed_mm_s=_maybe_float(obj, "basic_max_wheel_speed_mm_s", "$.basic_max_wheel_speed_mm_s", errors),
            basic_max_wheel_accel_mm_s2=_maybe_float(obj, "basic_max_wheel_accel_mm_s2", "$.basic_max_wheel_accel_mm_s2", errors),
            custom_use_dc_motor_model=as_bool(obj.get("custom_use_dc_motor_model", True), "$.custom_use_dc_motor_model", True, errors),
            custom_use_kinematic_model=as_bool(obj.get("custom_use_kinematic_model", True), "$.custom_use_kinematic_model", True, errors),
            custom_use_acceleration_limit=as_bool(obj.get("custom_use_acceleration_limit", True), "$.custom_use_acceleration_limit", True, errors),
            custom_use_battery_model=as_bool(obj.get("custom_use_battery_model", False), "$.custom_use_battery_model", False, errors),
            custom_use_sensor_noise=as_bool(obj.get("custom_use_sensor_noise", False), "$.custom_use_sensor_noise", False, errors),
            custom_use_encoder_model=as_bool(obj.get("custom_use_encoder_model", False), "$.custom_use_encoder_model", False, errors),
            custom_use_imu_model=as_bool(obj.get("custom_use_imu_model", False), "$.custom_use_imu_model", False, errors),
            custom_use_wheel_slip=as_bool(obj.get("custom_use_wheel_slip", False), "$.custom_use_wheel_slip", False, errors),
            custom_use_track_imperfections=as_bool(obj.get("custom_use_track_imperfections", False), "$.custom_use_track_imperfections", False, errors),
            custom_use_manual_slip=as_bool(obj.get("custom_use_manual_slip", False), "$.custom_use_manual_slip", False, errors),
            require_c_backend_for_realistic=as_bool(obj.get("require_c_backend_for_realistic", True), "$.require_c_backend_for_realistic", True, errors),
            allow_python_fallback_for_realistic=as_bool(obj.get("allow_python_fallback_for_realistic", False), "$.allow_python_fallback_for_realistic", False, errors),
            custom_use_c_backend=as_bool(obj.get("custom_use_c_backend", True), "$.custom_use_c_backend", True, errors),
            custom_allow_python_fallback=as_bool(obj.get("custom_allow_python_fallback", False), "$.custom_allow_python_fallback", False, errors),
            custom_use_auto_acceleration_limit=as_bool(obj.get("custom_use_auto_acceleration_limit", True), "$.custom_use_auto_acceleration_limit", True, errors),
            custom_max_wheel_accel_mm_s2=as_float(obj.get("custom_max_wheel_accel_mm_s2", 9810.0), "$.custom_max_wheel_accel_mm_s2", 9810.0, errors),
            custom_battery_initial_voltage_v=as_float(obj.get("custom_battery_initial_voltage_v", 7.4), "$.custom_battery_initial_voltage_v", 7.4, errors),
            custom_battery_nominal_voltage_v=as_float(obj.get("custom_battery_nominal_voltage_v", 7.4), "$.custom_battery_nominal_voltage_v", 7.4, errors),
            custom_battery_min_voltage_v=as_float(obj.get("custom_battery_min_voltage_v", 6.0), "$.custom_battery_min_voltage_v", 6.0, errors),
            custom_battery_capacity_mah=as_float(obj.get("custom_battery_capacity_mah", 1000.0), "$.custom_battery_capacity_mah", 1000.0, errors),
            custom_battery_internal_resistance_ohm=as_float(obj.get("custom_battery_internal_resistance_ohm", 0.15), "$.custom_battery_internal_resistance_ohm", 0.15, errors),
            custom_battery_soc_initial=as_float(obj.get("custom_battery_soc_initial", 1.0), "$.custom_battery_soc_initial", 1.0, errors),
            custom_sensor_noise_std=as_float(obj.get("custom_sensor_noise_std", 0.02), "$.custom_sensor_noise_std", 0.02, errors),
            custom_sensor_noise_seed=as_int(obj.get("custom_sensor_noise_seed", 12345), "$.custom_sensor_noise_seed", 12345, errors),
            sensor_common_noise_std=as_float(obj.get("sensor_common_noise_std", 0.0), "$.sensor_common_noise_std", 0.0, errors),
            sensor_individual_noise_std=as_float(obj.get("sensor_individual_noise_std", 0.0), "$.sensor_individual_noise_std", 0.0, errors),
            sensor_filter_tau_ms=as_float(obj.get("sensor_filter_tau_ms", 0.0), "$.sensor_filter_tau_ms", 0.0, errors),
            sensor_latency_ms=as_float(obj.get("sensor_latency_ms", 0.0), "$.sensor_latency_ms", 0.0, errors),
            sensor_update_rate_Hz=as_float(obj.get("sensor_update_rate_Hz", 0.0), "$.sensor_update_rate_Hz", 0.0, errors),
            sensor_gain_default=as_float(obj.get("sensor_gain_default", 1.0), "$.sensor_gain_default", 1.0, errors),
            sensor_offset_default=as_float(obj.get("sensor_offset_default", 0.0), "$.sensor_offset_default", 0.0, errors),
            verbose_sensor_log=as_bool(obj.get("verbose_sensor_log", False), "$.verbose_sensor_log", False, errors),
            custom_encoder_ticks_per_rev=as_int(obj.get("custom_encoder_ticks_per_rev", 1024), "$.custom_encoder_ticks_per_rev", 1024, errors),
            custom_encoder_noise_std_ticks=as_float(obj.get("custom_encoder_noise_std_ticks", 0.0), "$.custom_encoder_noise_std_ticks", 0.0, errors),
            custom_encoder_quantization=as_bool(obj.get("custom_encoder_quantization", True), "$.custom_encoder_quantization", True, errors),
            custom_imu_gyro_noise_std_rad_s=as_float(obj.get("custom_imu_gyro_noise_std_rad_s", 0.0), "$.custom_imu_gyro_noise_std_rad_s", 0.0, errors),
            custom_imu_accel_noise_std_mm_s2=as_float(obj.get("custom_imu_accel_noise_std_mm_s2", 0.0), "$.custom_imu_accel_noise_std_mm_s2", 0.0, errors),
            custom_slip_ratio_left=as_float(obj.get("custom_slip_ratio_left", 0.0), "$.custom_slip_ratio_left", 0.0, errors),
            custom_slip_ratio_right=as_float(obj.get("custom_slip_ratio_right", 0.0), "$.custom_slip_ratio_right", 0.0, errors),
            custom_slip_noise_std=as_float(obj.get("custom_slip_noise_std", 0.0), "$.custom_slip_noise_std", 0.0, errors),
            custom_slip_stiffness_factor=as_float(obj.get("custom_slip_stiffness_factor", 0.03), "$.custom_slip_stiffness_factor", 0.03, errors),
            custom_slip_at_limit=as_float(obj.get("custom_slip_at_limit", 0.05), "$.custom_slip_at_limit", 0.05, errors),
            custom_slip_max_ratio=as_float(obj.get("custom_slip_max_ratio", 0.95), "$.custom_slip_max_ratio", 0.95, errors),
            custom_use_combined_slip_limit=as_bool(obj.get("custom_use_combined_slip_limit", True), "$.custom_use_combined_slip_limit", True, errors),
            custom_use_lateral_slip=as_bool(obj.get("custom_use_lateral_slip", True), "$.custom_use_lateral_slip", True, errors),
            custom_mu_static_left=as_float(obj.get("custom_mu_static_left", 0.0), "$.custom_mu_static_left", 0.0, errors),
            custom_mu_static_right=as_float(obj.get("custom_mu_static_right", 0.0), "$.custom_mu_static_right", 0.0, errors),
            custom_mu_kinetic_left=as_float(obj.get("custom_mu_kinetic_left", 0.0), "$.custom_mu_kinetic_left", 0.0, errors),
            custom_mu_kinetic_right=as_float(obj.get("custom_mu_kinetic_right", 0.0), "$.custom_mu_kinetic_right", 0.0, errors),
            custom_track_imperfection_amplitude_mm=as_float(obj.get("custom_track_imperfection_amplitude_mm", 0.0), "$.custom_track_imperfection_amplitude_mm", 0.0, errors),
            custom_track_imperfection_wavelength_mm=as_float(obj.get("custom_track_imperfection_wavelength_mm", 500.0), "$.custom_track_imperfection_wavelength_mm", 500.0, errors),
            custom_track_imperfection_noise_std=as_float(obj.get("custom_track_imperfection_noise_std", 0.0), "$.custom_track_imperfection_noise_std", 0.0, errors),
            custom_motor_deadzone_pwm=as_float(obj.get("custom_motor_deadzone_pwm", 0.0), "$.custom_motor_deadzone_pwm", 0.0, errors),
            custom_current_limit_a=as_float(obj.get("custom_current_limit_a", 0.0), "$.custom_current_limit_a", 0.0, errors),
            custom_drivetrain_efficiency=as_float(obj.get("custom_drivetrain_efficiency", 0.9), "$.custom_drivetrain_efficiency", 0.9, errors),
            custom_viscous_friction=as_float(obj.get("custom_viscous_friction", 0.0), "$.custom_viscous_friction", 0.0, errors),
            custom_coulomb_friction=as_float(obj.get("custom_coulomb_friction", 0.0), "$.custom_coulomb_friction", 0.0, errors),
            custom_pwm_saturation_enabled=as_bool(obj.get("custom_pwm_saturation_enabled", True), "$.custom_pwm_saturation_enabled", True, errors),
            custom_max_pwm=_maybe_float(obj, "custom_max_pwm", "$.custom_max_pwm", errors),
            sensor_mode=str(obj.get("sensor_mode", "analog")).lower(),
            sensor_bits=as_int(obj.get("sensor_bits", 8), "$.sensor_bits", 8, errors),
            value_of_line=as_int(obj.get("value_of_line", 0), "$.value_of_line", 0, errors),
            value_of_background=as_int(obj.get("value_of_background", 255), "$.value_of_background", 255, errors),
            analog_noise_line=as_int(obj.get("analog_noise_line", 50), "$.analog_noise_line", 50, errors),
            analog_noise_background=as_int(obj.get("analog_noise_background", 50), "$.analog_noise_background", 50, errors),
        )
        errors.extend(cfg.validate())
        raise_if_errors("Configuração de simulação inválida.", errors)
        return cfg

    @classmethod
    def from_json_file(cls, path: str) -> "SimulationConfig":
        try:
            with open(path, "r", encoding="utf-8") as f:
                return cls.from_dict(json.load(f))
        except ValidationError:
            raise
        except json.JSONDecodeError as e:
            raise ValidationError("Configuração de simulação inválida.", [f"Erro de sintaxe em {path}: {e}"])
        except OSError as e:
            raise ValidationError("Não foi possível abrir a configuração de simulação.", [str(e)])

    @classmethod
    def from_robot_spec(cls, robot: RobotSpec) -> "SimulationConfig":
        sens = robot.sensor_model
        elec = robot.electrical
        return cls(
            simulation_step_dt_ms=robot.controller.simulation_step_dt_ms,
            sensor_mode=sens.sensor_mode,
            sensor_bits=sens.sensor_bits,
            value_of_line=sens.value_of_line,
            value_of_background=sens.value_of_background,
            analog_noise_line=sens.analog_noise_line,
            analog_noise_background=sens.analog_noise_background,
            custom_battery_initial_voltage_v=elec.battery_voltage_v,
            custom_battery_nominal_voltage_v=elec.battery_voltage_v,
            custom_battery_min_voltage_v=max(0.0, min(elec.battery_voltage_v, 6.0)),
            custom_battery_capacity_mah=elec.battery_capacity_mah,
            custom_battery_internal_resistance_ohm=elec.r_batt_ohm,
        )

    def as_realistic_preset(self) -> "SimulationConfig":
        data = self.to_dict()
        data.update({
            "physics_profile": "custom",
            "custom_use_dc_motor_model": True,
            "custom_use_kinematic_model": False,
            "custom_use_acceleration_limit": True,
            "custom_use_battery_model": True,
            "custom_use_sensor_noise": True,
            "custom_use_encoder_model": True,
            "custom_use_imu_model": True,
            "custom_use_wheel_slip": True,
            "custom_use_track_imperfections": True,
            "custom_use_manual_slip": False,
            "custom_use_c_backend": True,
            "custom_allow_python_fallback": self.allow_python_fallback_for_realistic,
            "custom_use_auto_acceleration_limit": True,
            "custom_use_combined_slip_limit": True,
            "custom_use_lateral_slip": True,
            # Noise/imperfection defaults make optional effects observable, while
            # slip is now calculated from torque/friction rather than forced.
            "custom_sensor_noise_std": self.custom_sensor_noise_std if self.custom_sensor_noise_std > 0 else 0.005,
            "custom_slip_ratio_left": self.custom_slip_ratio_left,
            "custom_slip_ratio_right": self.custom_slip_ratio_right,
            "custom_track_imperfection_amplitude_mm": max(self.custom_track_imperfection_amplitude_mm, 0.5),
        })
        return SimulationConfig.from_dict(data)

    def validate(self) -> list[str]:
        errors: list[str] = []
        if self.final_linear_speed_mps <= 0:
            errors.append("final_linear_speed_mps deve ser > 0.")
        if self.motor_time_constant_s <= 0:
            errors.append("motor_time_constant_s deve ser > 0.")
        if self.simulation_step_dt_ms <= 0:
            errors.append("simulation_step_dt_ms deve ser > 0.")
        if self.max_time_s <= 0:
            errors.append("max_time_s deve ser > 0.")
        if self.physics_profile not in VALID_PHYSICS_PROFILES:
            expected = ", ".join(sorted(VALID_PHYSICS_PROFILES))
            errors.append(f"Invalid physics_profile {self.physics_profile!r}. Expected one of: {expected}.")
        if self.ideal_max_wheel_speed_mm_s is not None and self.ideal_max_wheel_speed_mm_s <= 0:
            errors.append("ideal_max_wheel_speed_mm_s deve ser > 0 quando informado.")
        if self.basic_max_wheel_speed_mm_s is not None and self.basic_max_wheel_speed_mm_s <= 0:
            errors.append("basic_max_wheel_speed_mm_s deve ser > 0 quando informado.")
        if self.basic_max_wheel_accel_mm_s2 is not None and self.basic_max_wheel_accel_mm_s2 <= 0:
            errors.append("basic_max_wheel_accel_mm_s2 deve ser > 0 quando informado.")
        if self.custom_max_wheel_accel_mm_s2 <= 0:
            errors.append("custom_max_wheel_accel_mm_s2 deve ser > 0.")
        if self.custom_battery_initial_voltage_v <= 0 or self.custom_battery_nominal_voltage_v <= 0:
            errors.append("tensões inicial e nominal da bateria devem ser > 0.")
        if self.custom_battery_min_voltage_v < 0:
            errors.append("custom_battery_min_voltage_v deve ser >= 0.")
        if self.custom_battery_min_voltage_v > self.custom_battery_initial_voltage_v:
            errors.append("custom_battery_min_voltage_v não pode ser maior que a tensão inicial.")
        if self.custom_battery_capacity_mah <= 0:
            errors.append("custom_battery_capacity_mah deve ser > 0.")
        if self.custom_battery_internal_resistance_ohm < 0:
            errors.append("custom_battery_internal_resistance_ohm deve ser >= 0.")
        if not (0.0 <= self.custom_battery_soc_initial <= 1.0):
            errors.append("custom_battery_soc_initial deve estar entre 0.0 e 1.0.")
        if self.custom_sensor_noise_std < 0:
            errors.append("custom_sensor_noise_std deve ser >= 0.")
        for name in ("sensor_common_noise_std", "sensor_individual_noise_std", "sensor_filter_tau_ms", "sensor_latency_ms", "sensor_update_rate_Hz"):
            if getattr(self, name) < 0:
                errors.append(f"{name} deve ser >= 0.")
        if self.sensor_gain_default <= 0:
            errors.append("sensor_gain_default deve ser > 0.")
        if self.custom_encoder_ticks_per_rev <= 0:
            errors.append("custom_encoder_ticks_per_rev deve ser > 0.")
        if self.custom_encoder_noise_std_ticks < 0:
            errors.append("custom_encoder_noise_std_ticks deve ser >= 0.")
        if self.custom_imu_gyro_noise_std_rad_s < 0 or self.custom_imu_accel_noise_std_mm_s2 < 0:
            errors.append("ruídos de IMU devem ser >= 0.")
        if not (0.0 <= self.custom_slip_ratio_left <= 0.95):
            errors.append("custom_slip_ratio_left deve estar entre 0.0 e 0.95.")
        if not (0.0 <= self.custom_slip_ratio_right <= 0.95):
            errors.append("custom_slip_ratio_right deve estar entre 0.0 e 0.95.")
        if self.custom_slip_noise_std < 0:
            errors.append("custom_slip_noise_std deve ser >= 0.")
        if self.custom_slip_stiffness_factor < 0:
            errors.append("custom_slip_stiffness_factor deve ser >= 0.")
        if not (0.0 <= self.custom_slip_at_limit <= 1.0):
            errors.append("custom_slip_at_limit deve estar entre 0.0 e 1.0.")
        if not (0.0 <= self.custom_slip_max_ratio <= 0.99):
            errors.append("custom_slip_max_ratio deve estar entre 0.0 e 0.99.")
        for name in ("custom_mu_static_left", "custom_mu_static_right", "custom_mu_kinetic_left", "custom_mu_kinetic_right"):
            if getattr(self, name) < 0:
                errors.append(f"{name} deve ser >= 0.")
        if self.custom_track_imperfection_amplitude_mm < 0:
            errors.append("custom_track_imperfection_amplitude_mm deve ser >= 0.")
        if self.custom_track_imperfection_wavelength_mm <= 0:
            errors.append("custom_track_imperfection_wavelength_mm deve ser > 0.")
        if self.custom_track_imperfection_noise_std < 0:
            errors.append("custom_track_imperfection_noise_std deve ser >= 0.")
        if self.custom_motor_deadzone_pwm < 0:
            errors.append("custom_motor_deadzone_pwm deve ser >= 0.")
        if self.custom_current_limit_a < 0:
            errors.append("custom_current_limit_a deve ser >= 0.")
        if not (0.0 < self.custom_drivetrain_efficiency <= 1.0):
            errors.append("custom_drivetrain_efficiency deve estar em (0, 1].")
        if self.custom_viscous_friction < 0 or self.custom_coulomb_friction < 0:
            errors.append("atritos configuráveis devem ser >= 0.")
        if self.custom_max_pwm is not None and self.custom_max_pwm <= 0:
            errors.append("custom_max_pwm deve ser > 0 quando informado.")
        if self.sensor_mode not in {"analog", "digital"}:
            errors.append("sensor_mode deve ser 'analog' ou 'digital'.")
        if not (1 <= self.sensor_bits <= 16):
            errors.append("sensor_bits deve estar entre 1 e 16.")
        return errors

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def derive_runtime_params(robot: RobotSpec, config: SimulationConfig | None = None) -> dict[str, Any]:
    """Deriva escalares físicos usados pelos perfis e pelo backend C."""
    cfg = config or SimulationConfig.from_robot_spec(robot)
    gm = robot.geometric_mechanical
    elec = robot.electrical
    motor = robot.motor_transmission
    ctrl = robot.controller

    wheel_r_m = gm.wheel_radius_mm / 1000.0
    track_m = derive_wheel_track_mm(robot) / 1000.0
    mass_kg = gm.mass_kg
    jz = gm.J_body_kgm2
    if jz <= 0.0:
        jz = max(1e-6, mass_kg * (track_m**2) / 12.0)

    gear = motor.gear_ratio or 1.0
    eta = motor.eta or 0.9
    kt = motor.Kt_Nm_per_A
    kv_rad_per_v = float(getattr(motor, "Kv_rad_per_V", 0.0) or 0.0)
    if kv_rad_per_v <= 1e-12 and motor.Kv_rpm_per_V > 0:
        kv_rad_per_v = (motor.Kv_rpm_per_V * 2.0 * math.pi) / 60.0
    ke = 1.0 / kv_rad_per_v if kv_rad_per_v > 1e-12 else 1.0 / 500.0

    # Estimate the full-duty no-load wheel speed using the same motor constants
    # used by the DC model. This keeps the ideal/basic profiles on the same
    # velocity scale as the realistic profile instead of relying on an arbitrary
    # fixed 500 mm/s fallback.
    available_voltage = max(0.0, float(elec.battery_voltage_v) - float(elec.driver_drop_v))
    if ke > 1e-12 and gear > 1e-12:
        motor_no_load_rad_s = available_voltage / ke
        v_mps = (motor_no_load_rad_s / gear) * wheel_r_m
    else:
        rpm_no_load = 10000.0
        wheel_rps = (rpm_no_load / max(1.0, gear)) / 60.0
        v_mps = wheel_rps * (2.0 * math.pi * wheel_r_m)
    v_mps = max(0.1, min(20.0, float(v_mps)))

    # A safe acceleration estimate for the basic kinematic profile.  It is not
    # a detailed tire model; it only prevents the default from being so sluggish
    # that the robot cannot turn at realistic speeds.
    max_wheel_accel_mps2 = max(5.0, min(30.0, float(gm.mu_static) * 9.81))

    tau_s = cfg.motor_time_constant_s
    try:
        j_eq = motor.Jm_kgm2 + (gear**2) * motor.Jload_reflected_kgm2
        if motor.Rm_ohm > 0 and kt > 0 and ke > 0 and j_eq > 0:
            tau_s = max(0.002, min(0.100, (j_eq * motor.Rm_ohm) / (kt * ke)))
    except Exception:
        pass

    i_max = max(0.0, motor.driver_current_limit_A, motor.stall_current_A)
    if i_max <= 0.0:
        i_max = 3.0

    return {
        "final_linear_speed_mps": v_mps or cfg.final_linear_speed_mps,
        "motor_no_load_wheel_speed_mps": v_mps,
        "basic_max_wheel_accel_mps2": max_wheel_accel_mps2,
        "motor_time_constant_s": tau_s,
        "simulation_step_dt_ms": cfg.simulation_step_dt_ms,
        "max_time_s": cfg.max_time_s,
        "pwm_max": ctrl.pwm_max,
        "pwm_min": ctrl.pwm_min,
        "pwm_neutral": ctrl.pwm_neutral,
        "deadband_percent": ctrl.deadband_percent,
        "sensor_mode": cfg.sensor_mode,
        "sensor_bits": cfg.sensor_bits,
        "value_of_line": cfg.value_of_line,
        "value_of_background": cfg.value_of_background,
        "analog_noise_line": cfg.analog_noise_line,
        "analog_noise_background": cfg.analog_noise_background,
        "V_batt_nom_V": elec.battery_voltage_v,
        "R_batt_ohm": elec.r_batt_ohm,
        "R_wiring_ohm": elec.wiring_r_ohm,
        "driver_drop_V": elec.driver_drop_v,
        "Rm_ohm": motor.Rm_ohm,
        "Lm_H": motor.Lm_H,
        "Kt_Nm_per_A": kt,
        "Kv_rad_per_V": kv_rad_per_v,
        "Ke_V_per_rad": ke,
        "gear_ratio": gear,
        "eta_drive": eta,
        "Jm_kgm2": motor.Jm_kgm2,
        "Jload_kgm2": motor.Jload_reflected_kgm2,
        "b_visc_Nm_per_radps": motor.b_visc_Nm_per_radps,
        "tau_coulomb_Nm": motor.tau_coulomb_Nm,
        "I_max_A": i_max,
        "I0_noLoad_A": motor.I0_A,
        "mass_kg": mass_kg,
        "track_m": track_m,
        "wheel_r_m": wheel_r_m,
        "Jz_kgm2": jz,
        "Crr": gm.Crr,
        "rho_air": 1.225,
        "CdA": 0.02,
        # Explicit defaults replacing formerly implicit/hardcoded C values.
        "battery_initial_voltage_v": cfg.custom_battery_initial_voltage_v or elec.battery_voltage_v,
        "battery_nominal_voltage_v": cfg.custom_battery_nominal_voltage_v or elec.battery_voltage_v,
        "battery_min_voltage_v": cfg.custom_battery_min_voltage_v,
        "battery_capacity_mah": cfg.custom_battery_capacity_mah or elec.battery_capacity_mah,
        "battery_internal_resistance_ohm": cfg.custom_battery_internal_resistance_ohm or elec.r_batt_ohm,
        "battery_ocv_table": list(elec.battery_ocv_table),
        "wiring_R_ohm": elec.wiring_r_ohm,
        "battery_soc_initial": cfg.custom_battery_soc_initial,
        "motor_deadzone_pwm": cfg.custom_motor_deadzone_pwm,
        "current_limit_a": cfg.custom_current_limit_a if cfg.custom_current_limit_a > 0 else i_max,
        "drivetrain_efficiency": cfg.custom_drivetrain_efficiency if cfg.custom_drivetrain_efficiency > 0 else eta,
        "viscous_friction": cfg.custom_viscous_friction if cfg.custom_viscous_friction >= 0 else motor.b_visc_Nm_per_radps,
        "coulomb_friction": cfg.custom_coulomb_friction if cfg.custom_coulomb_friction >= 0 else motor.tau_coulomb_Nm,
    }
