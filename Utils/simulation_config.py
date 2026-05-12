from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from typing import Any

from Utils.robot_spec import RobotSpec
from Utils.robot_runtime import derive_wheel_track_mm
from Utils.validation import ValidationError, as_bool, as_float, as_int, raise_if_errors


@dataclass(slots=True)
class SimulationConfig:
    final_linear_speed_mps: float = 2.0
    motor_time_constant_s: float = 0.010
    simulation_step_dt_ms: float = 1.0
    max_time_s: float = 100.0
    save_logs: bool = False
    random_seed: int | None = None

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
        return cls(
            simulation_step_dt_ms=robot.controller.simulation_step_dt_ms,
            sensor_mode=sens.sensor_mode,
            sensor_bits=sens.sensor_bits,
            value_of_line=sens.value_of_line,
            value_of_background=sens.value_of_background,
            analog_noise_line=sens.analog_noise_line,
            analog_noise_background=sens.analog_noise_background,
        )

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
        if self.sensor_mode not in {"analog", "digital"}:
            errors.append("sensor_mode deve ser 'analog' ou 'digital'.")
        if not (1 <= self.sensor_bits <= 16):
            errors.append("sensor_bits deve estar entre 1 e 16.")
        return errors

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def derive_runtime_params(robot: RobotSpec, config: SimulationConfig | None = None) -> dict[str, Any]:
    """Deriva escalares usados pelo loop atual, preservando a física existente."""
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
    kv_rad_per_v = (motor.Kv_rpm_per_V * 2.0 * math.pi) / 60.0 if motor.Kv_rpm_per_V > 0 else 0.0
    ke = 1.0 / kv_rad_per_v if kv_rad_per_v > 1e-12 else 1.0 / 500.0

    rpm_no_load = 10000.0
    wheel_rps = (rpm_no_load / max(1.0, gear)) / 60.0
    v_mps = wheel_rps * (2.0 * math.pi * wheel_r_m) * max(0.1, min(1.0, eta))
    v_mps = max(0.1, min(20.0, float(v_mps)))

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
    }
