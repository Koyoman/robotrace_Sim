from __future__ import annotations

from dataclasses import MISSING, asdict, dataclass, field, fields
from typing import Any


@dataclass(slots=True)
class SimulationState:
    t_ms: int
    x_mm: float
    y_mm: float
    heading_deg: float
    v_mm_s: float
    omega_rad_s: float
    a_lin_mm_s2: float
    alpha_rad_s2: float
    v_left_mm_s: float
    v_right_mm_s: float
    pwm_left: int = 0
    pwm_right: int = 0
    sensors: list[int] = field(default_factory=list)

    # Phase 4.3: extra physical channels.  Defaults keep old controllers and
    # old replay/log readers working; new controllers can opt in field by field.
    heading_wrapped_deg: float = 0.0
    duty_left: float = 0.0
    duty_right: float = 0.0
    pwm_min: float = -4095.0
    pwm_max: float = 4095.0

    battery_voltage_v: float = 0.0
    battery_soc: float = 1.0
    battery_current_a: float = 0.0
    current_left_a: float = 0.0
    current_right_a: float = 0.0
    current_total_a: float = 0.0
    motor_left_current_a: float = 0.0
    motor_right_current_a: float = 0.0
    motor_left_current_signed_a: float = 0.0
    motor_right_current_signed_a: float = 0.0
    battery_power_w: float = 0.0
    motor_left_voltage_v: float = 0.0
    motor_right_voltage_v: float = 0.0
    motor_left_back_emf_v: float = 0.0
    motor_right_back_emf_v: float = 0.0
    motor_left_torque_nm: float = 0.0
    motor_right_torque_nm: float = 0.0
    wheel_left_torque_nm: float = 0.0
    wheel_right_torque_nm: float = 0.0
    tau_motor_em_left_nm: float = 0.0
    tau_motor_em_right_nm: float = 0.0
    tau_motor_viscous_left_nm: float = 0.0
    tau_motor_viscous_right_nm: float = 0.0
    tau_motor_coulomb_left_nm: float = 0.0
    tau_motor_coulomb_right_nm: float = 0.0
    tau_motor_net_left_nm: float = 0.0
    tau_motor_net_right_nm: float = 0.0
    tau_wheel_drive_left_nm: float = 0.0
    tau_wheel_drive_right_nm: float = 0.0
    tau_rolling_left_nm: float = 0.0
    tau_rolling_right_nm: float = 0.0
    tau_bearing_left_nm: float = 0.0
    tau_bearing_right_nm: float = 0.0
    tau_ground_left_nm: float = 0.0
    tau_ground_right_nm: float = 0.0
    tau_slip_loss_left_nm: float = 0.0
    tau_slip_loss_right_nm: float = 0.0
    brake_dissipated_power_w: float = 0.0
    mechanical_power_left_w: float = 0.0
    mechanical_power_right_w: float = 0.0

    wheel_left_surface_speed_mm_s: float = 0.0
    wheel_right_surface_speed_mm_s: float = 0.0
    ground_left_speed_mm_s: float = 0.0
    ground_right_speed_mm_s: float = 0.0
    omega_wheel_left_rad_s: float = 0.0
    omega_wheel_right_rad_s: float = 0.0
    alpha_wheel_left_rad_s2: float = 0.0
    alpha_wheel_right_rad_s2: float = 0.0
    J_eq_left_kgm2: float = 0.0
    J_eq_right_kgm2: float = 0.0

    enc_left_ticks: int = 0
    enc_right_ticks: int = 0
    enc_left_delta_ticks: int = 0
    enc_right_delta_ticks: int = 0
    enc_left_rad_s: float = 0.0
    enc_right_rad_s: float = 0.0

    imu_omega_rad_s: float = 0.0
    imu_alpha_rad_s2: float = 0.0
    imu_accel_x_mm_s2: float = 0.0
    imu_accel_y_mm_s2: float = 0.0

    slip_ratio_left: float = 0.0
    slip_ratio_right: float = 0.0
    longitudinal_slip_left: float = 0.0
    longitudinal_slip_right: float = 0.0
    lateral_slip_left: float = 0.0
    lateral_slip_right: float = 0.0
    traction_force_left_n: float = 0.0
    traction_force_right_n: float = 0.0
    max_static_force_left_n: float = 0.0
    max_static_force_right_n: float = 0.0
    force_longitudinal_command_left_n: float = 0.0
    force_longitudinal_command_right_n: float = 0.0
    force_longitudinal_ground_left_n: float = 0.0
    force_longitudinal_ground_right_n: float = 0.0
    force_longitudinal_max_left_n: float = 0.0
    force_longitudinal_max_right_n: float = 0.0
    force_longitudinal_saturation_left: float = 0.0
    force_longitudinal_saturation_right: float = 0.0
    lambda_long_left: float = 0.0
    lambda_long_right: float = 0.0
    lateral_accel_mm_s2: float = 0.0
    lateral_force_total_n: float = 0.0
    lateral_force_left_n: float = 0.0
    lateral_force_right_n: float = 0.0
    friction_usage_left: float = 0.0
    friction_usage_right: float = 0.0
    combined_friction_limit_left_n: float = 0.0
    combined_friction_limit_right_n: float = 0.0

    copper_loss_left_w: float = 0.0
    copper_loss_right_w: float = 0.0
    driver_loss_left_w: float = 0.0
    driver_loss_right_w: float = 0.0
    battery_internal_loss_w: float = 0.0
    wiring_loss_w: float = 0.0
    mechanical_friction_loss_left_w: float = 0.0
    mechanical_friction_loss_right_w: float = 0.0
    rolling_resistance_loss_w: float = 0.0
    tire_slip_loss_left_w: float = 0.0
    tire_slip_loss_right_w: float = 0.0
    kinetic_power_delta_w: float = 0.0
    battery_energy_j: float = 0.0
    copper_loss_energy_j: float = 0.0
    driver_loss_energy_j: float = 0.0
    battery_internal_loss_energy_j: float = 0.0
    wiring_loss_energy_j: float = 0.0
    mechanical_friction_loss_energy_j: float = 0.0
    rolling_resistance_energy_j: float = 0.0
    tire_slip_loss_energy_j: float = 0.0
    brake_dissipated_energy_j: float = 0.0
    kinetic_energy_linear_j: float = 0.0
    kinetic_energy_angular_j: float = 0.0
    kinetic_energy_wheels_j: float = 0.0
    total_kinetic_energy_j: float = 0.0
    total_loss_energy_j: float = 0.0
    energy_balance_error_j: float = 0.0
    energy_balance_error_percent: float = 0.0

    physics_backend: str = "python"
    linesim_abi_version: int = 0
    c_backend_loaded: bool = False
    c_backend_path: str = ""
    c_backend_error: str = ""
    c_modular_step_available: bool = False
    using_python_fallback: bool = False
    c_step_call_count: int = 0
    last_step_executed_in_c: bool = False

    def to_controller_state(self, dt_s: float) -> dict[str, Any]:
        d = asdict(self)
        d["dt_s"] = dt_s
        d["sensors"] = list(self.sensors)
        return d

    def to_step_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_step_dict(cls, step: dict[str, Any]) -> "SimulationState":
        kwargs: dict[str, Any] = {}
        for f in fields(cls):
            if f.name in step:
                kwargs[f.name] = step[f.name]
            elif f.default is not MISSING:
                kwargs[f.name] = f.default
            elif f.default_factory is not MISSING:  # type: ignore[attr-defined]
                kwargs[f.name] = f.default_factory()  # type: ignore[misc]
        # Keep encoder channels integer even when loading older JSON logs with floats.
        for key in ("enc_left_ticks", "enc_right_ticks", "enc_left_delta_ticks", "enc_right_delta_ticks"):
            if key in kwargs:
                kwargs[key] = int(round(float(kwargs[key])))
        return cls(**kwargs)


@dataclass(slots=True)
class SimulationResult:
    steps: list[SimulationState]
    summary: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {"steps": [asdict(s) for s in self.steps], "summary": dict(self.summary)}
