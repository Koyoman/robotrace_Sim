from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from typing import Any

from Utils.robot_geometry import Envelope as EditorEnvelope
from Utils.robot_model import RobotModel, Wheel as EditorWheel, Sensor as EditorSensor
from Utils.validation import ValidationError, as_float, as_int, raise_if_errors


@dataclass(slots=True)
class WheelSpec:
    id: str
    x_mm: float
    y_mm: float
    width_mm: float = 22.0
    height_mm: float = 15.0


@dataclass(slots=True)
class SensorSpec:
    id: str
    x_mm: float
    y_mm: float
    size_mm: float = 5.0
    gain: float | None = None
    offset: float | None = None
    noise_std: float | None = None
    latency_ms: float | None = None
    update_rate_Hz: float | None = None
    filter_tau_ms: float | None = None


@dataclass(slots=True)
class EnvelopeSpec:
    width_mm: float = 160.0
    height_mm: float = 140.0


@dataclass(slots=True)
class GeometryMechanicalSpec:
    body_length_mm: float = 160.0
    body_width_mm: float = 140.0
    wheel_radius_mm: float = 11.0
    track_mm: float = 70.0
    wheelbase_mm: float = 0.0
    wheelbase_offset_mm: float = 0.0
    mass_kg: float = 0.2
    J_body_kgm2: float = 0.0
    mu_static: float = 1.0
    mu_kinetic: float = 0.8
    Crr: float = 0.005
    cg_origin_xy_mm: tuple[float, float] = (-10.0, 0.0)
    rot_origin_xy_mm: tuple[float, float] = (-10.0, 0.0)


@dataclass(slots=True)
class ElectricalSpec:
    battery_voltage_v: float = 8.0
    battery_capacity_mah: float = 400.0
    r_batt_ohm: float = 0.08
    wiring_r_ohm: float = 0.03
    driver_drop_v: float = 0.2
    battery_ocv_table: list[tuple[float, float]] = field(default_factory=lambda: [
        (0.0, 6.0), (0.1, 7.0), (0.2, 7.4), (0.5, 7.7), (0.8, 8.0), (0.9, 8.2), (1.0, 8.4)
    ])


@dataclass(slots=True)
class MotorTransmissionSpec:
    gear_ratio: float = 19.92
    eta: float = 0.9
    Rm_ohm: float = 4.0
    Lm_H: float = 0.00015
    Kv_rpm_per_V: float = 5478.655528120322
    Kv_rad_per_V: float = 0.0
    Kt_Nm_per_A: float = 0.001743
    I0_A: float = 0.15
    b_visc_Nm_per_radps: float = 7e-08
    tau_coulomb_Nm: float = 0.000261
    Jm_kgm2: float = 1e-08
    Jload_reflected_kgm2: float = 0.0
    stall_current_A: float = 1.5
    driver_current_limit_A: float = 0.0


@dataclass(slots=True)
class ControllerSpec:
    pwm_min: int = -4095
    pwm_max: int = 4095
    pwm_neutral: int | None = None
    deadband_percent: float = 0.0
    pwm_resolution_bits: int = 12
    pwm_frequency_Hz: float = 20000.0
    simulation_step_dt_ms: float = 1.0


@dataclass(slots=True)
class SensorModelSpec:
    sensor_mode: str = "analog"
    sensor_bits: int = 8
    value_of_line: int = 0
    value_of_background: int = 255
    analog_noise_line: int = 50
    analog_noise_background: int = 50


@dataclass(slots=True)
class RobotSpec:
    version: str = "robot-v1"
    envelope: EnvelopeSpec = field(default_factory=EnvelopeSpec)
    origin_x_mm: float = 0.0
    origin_y_mm: float = 0.0
    wheels: list[WheelSpec] = field(default_factory=list)
    sensors: list[SensorSpec] = field(default_factory=list)
    geometric_mechanical: GeometryMechanicalSpec = field(default_factory=GeometryMechanicalSpec)
    electrical: ElectricalSpec = field(default_factory=ElectricalSpec)
    motor_transmission: MotorTransmissionSpec = field(default_factory=MotorTransmissionSpec)
    controller: ControllerSpec = field(default_factory=ControllerSpec)
    sensor_model: SensorModelSpec = field(default_factory=SensorModelSpec)
    encoder: dict[str, Any] = field(default_factory=dict)
    imu: dict[str, Any] = field(default_factory=dict)
    odometry: dict[str, Any] = field(default_factory=dict)

    @staticmethod
    def _xy_pair(obj: Any, path: str, default: tuple[float, float], errors: list[str]) -> tuple[float, float]:
        if obj is None:
            return default
        if isinstance(obj, (list, tuple)) and len(obj) == 2:
            return (as_float(obj[0], f"{path}[0]", default[0], errors), as_float(obj[1], f"{path}[1]", default[1], errors))
        errors.append(f"{path} deve ser uma lista [x, y].")
        return default

    @classmethod
    def from_dict(cls, obj: dict[str, Any]) -> "RobotSpec":
        if not isinstance(obj, dict):
            raise ValidationError("Robot JSON inválido.", ["Raiz do arquivo deve ser um objeto JSON."])
        errors: list[str] = []

        env_raw = obj.get("envelope") or {}
        width = as_float(env_raw.get("widthMM", obj.get("widthMM", 160.0)), "$.envelope.widthMM", 160.0, errors)
        height = as_float(env_raw.get("heightMM", obj.get("heightMM", 140.0)), "$.envelope.heightMM", 140.0, errors)
        if width <= 0:
            errors.append("$.envelope.widthMM deve ser > 0.")
        if height <= 0:
            errors.append("$.envelope.heightMM deve ser > 0.")

        origin = obj.get("origin") or {}
        origin_x = as_float(obj.get("originXMM", origin.get("xMM", 0.0)), "$.origin.xMM", 0.0, errors)
        origin_y = as_float(obj.get("originYMM", origin.get("yMM", 0.0)), "$.origin.yMM", 0.0, errors)

        wheels_raw = obj.get("wheels") or []
        if not isinstance(wheels_raw, list) or not wheels_raw:
            errors.append("$.wheels deve conter pelo menos uma roda.")
            wheels_raw = []
        wheels: list[WheelSpec] = []
        for i, w in enumerate(wheels_raw):
            if not isinstance(w, dict):
                errors.append(f"$.wheels[{i}] deve ser objeto.")
                continue
            wheels.append(WheelSpec(
                id=str(w.get("id", f"wheel{i+1}")),
                x_mm=as_float(w.get("xMM", 0.0), f"$.wheels[{i}].xMM", 0.0, errors),
                y_mm=as_float(w.get("yMM", 0.0), f"$.wheels[{i}].yMM", 0.0, errors),
                width_mm=as_float(w.get("widthMM", 22.0), f"$.wheels[{i}].widthMM", 22.0, errors),
                height_mm=as_float(w.get("heightMM", 15.0), f"$.wheels[{i}].heightMM", 15.0, errors),
            ))

        sensors_raw = obj.get("sensors") or []
        if not isinstance(sensors_raw, list) or not sensors_raw:
            errors.append("$.sensors deve conter pelo menos um sensor.")
            sensors_raw = []
        sensors: list[SensorSpec] = []
        for i, s in enumerate(sensors_raw):
            if not isinstance(s, dict):
                errors.append(f"$.sensors[{i}] deve ser objeto.")
                continue
            size = as_float(s.get("sizeMM", 5.0), f"$.sensors[{i}].sizeMM", 5.0, errors)
            if size <= 0:
                errors.append(f"$.sensors[{i}].sizeMM deve ser > 0.")
            sensors.append(SensorSpec(
                id=str(s.get("id", f"S{i+1}")),
                x_mm=as_float(s.get("xMM", 0.0), f"$.sensors[{i}].xMM", 0.0, errors),
                y_mm=as_float(s.get("yMM", 0.0), f"$.sensors[{i}].yMM", 0.0, errors),
                size_mm=size,
                gain=None if s.get("gain") is None else as_float(s.get("gain"), f"$.sensors[{i}].gain", 1.0, errors),
                offset=None if s.get("offset") is None else as_float(s.get("offset"), f"$.sensors[{i}].offset", 0.0, errors),
                noise_std=None if s.get("noise_std") is None else as_float(s.get("noise_std"), f"$.sensors[{i}].noise_std", 0.0, errors),
                latency_ms=None if s.get("latency_ms") is None else as_float(s.get("latency_ms"), f"$.sensors[{i}].latency_ms", 0.0, errors),
                update_rate_Hz=None if s.get("update_rate_Hz") is None else as_float(s.get("update_rate_Hz"), f"$.sensors[{i}].update_rate_Hz", 0.0, errors),
                filter_tau_ms=None if s.get("filter_tau_ms") is None else as_float(s.get("filter_tau_ms"), f"$.sensors[{i}].filter_tau_ms", 0.0, errors),
            ))

        gm_raw = obj.get("geometric_mechanical") or {}
        gm = GeometryMechanicalSpec(
            body_length_mm=as_float(gm_raw.get("body_length_mm", width), "$.geometric_mechanical.body_length_mm", width, errors),
            body_width_mm=as_float(gm_raw.get("body_width_mm", height), "$.geometric_mechanical.body_width_mm", height, errors),
            wheel_radius_mm=as_float(gm_raw.get("wheel_radius_mm", 11.0), "$.geometric_mechanical.wheel_radius_mm", 11.0, errors),
            track_mm=as_float(gm_raw.get("track_mm", 70.0), "$.geometric_mechanical.track_mm", 70.0, errors),
            wheelbase_mm=as_float(gm_raw.get("wheelbase_mm", 0.0), "$.geometric_mechanical.wheelbase_mm", 0.0, errors),
            wheelbase_offset_mm=as_float(gm_raw.get("wheelbase_offset_mm", 0.0), "$.geometric_mechanical.wheelbase_offset_mm", 0.0, errors),
            mass_kg=as_float(gm_raw.get("mass_kg", 0.2), "$.geometric_mechanical.mass_kg", 0.2, errors),
            J_body_kgm2=as_float(gm_raw.get("J_body_kgm2", 0.0), "$.geometric_mechanical.J_body_kgm2", 0.0, errors),
            mu_static=as_float(gm_raw.get("mu_static", 1.0), "$.geometric_mechanical.mu_static", 1.0, errors),
            mu_kinetic=as_float(gm_raw.get("mu_kinetic", 0.8), "$.geometric_mechanical.mu_kinetic", 0.8, errors),
            Crr=as_float(gm_raw.get("Crr", 0.005), "$.geometric_mechanical.Crr", 0.005, errors),
            cg_origin_xy_mm=cls._xy_pair(gm_raw.get("cg_origin_xy_mm"), "$.geometric_mechanical.cg_origin_xy_mm", (origin_x, origin_y), errors),
            rot_origin_xy_mm=cls._xy_pair(gm_raw.get("rot_origin_xy_mm"), "$.geometric_mechanical.rot_origin_xy_mm", (origin_x, origin_y), errors),
        )
        if gm.wheel_radius_mm <= 0:
            errors.append("$.geometric_mechanical.wheel_radius_mm deve ser > 0.")
        if gm.mass_kg <= 0:
            errors.append("$.geometric_mechanical.mass_kg deve ser > 0.")

        elec_raw = obj.get("electrical") or {}
        table_raw = elec_raw.get("battery_OCV_table", elec_raw.get("battery_ocv_table"))
        default_table = ElectricalSpec().battery_ocv_table
        table: list[tuple[float, float]] = []
        if table_raw is None:
            table = list(default_table)
        elif isinstance(table_raw, list):
            for i, row in enumerate(table_raw):
                if isinstance(row, (list, tuple)) and len(row) == 2:
                    table.append((as_float(row[0], f"$.electrical.battery_OCV_table[{i}][0]", 0.0, errors),
                                  as_float(row[1], f"$.electrical.battery_OCV_table[{i}][1]", 0.0, errors)))
                else:
                    errors.append(f"$.electrical.battery_OCV_table[{i}] deve ser [soc, ocv].")
        else:
            errors.append("$.electrical.battery_OCV_table deve ser lista de pares.")
            table = list(default_table)
        elec = ElectricalSpec(
            battery_voltage_v=as_float(elec_raw.get("batteryVoltageV", elec_raw.get("battery_voltage_v", 8.0)), "$.electrical.batteryVoltageV", 8.0, errors),
            battery_capacity_mah=as_float(elec_raw.get("batteryCapacitymAh", elec_raw.get("battery_capacity_mah", 400.0)), "$.electrical.batteryCapacitymAh", 400.0, errors),
            r_batt_ohm=as_float(elec_raw.get("R_batt_ohm", elec_raw.get("r_batt_ohm", 0.08)), "$.electrical.R_batt_ohm", 0.08, errors),
            wiring_r_ohm=as_float(elec_raw.get("wiring_R_ohm", elec_raw.get("wiring_r_ohm", 0.03)), "$.electrical.wiring_R_ohm", 0.03, errors),
            driver_drop_v=as_float(elec_raw.get("driver_drop_V", elec_raw.get("driver_drop_v", 0.2)), "$.electrical.driver_drop_V", 0.2, errors),
            battery_ocv_table=table,
        )

        motor_raw = obj.get("motor_transmission") or {}
        motor = MotorTransmissionSpec(
            gear_ratio=as_float(motor_raw.get("gear_ratio", 19.92), "$.motor_transmission.gear_ratio", 19.92, errors),
            eta=as_float(motor_raw.get("eta", 0.9), "$.motor_transmission.eta", 0.9, errors),
            Rm_ohm=as_float(motor_raw.get("R_motor_ohm", motor_raw.get("Rm_ohm", 4.0)), "$.motor_transmission.R_motor_ohm", 4.0, errors),
            Lm_H=as_float(motor_raw.get("L_motor_H", motor_raw.get("Lm_H", 0.00015)), "$.motor_transmission.L_motor_H", 0.00015, errors),
            Kv_rpm_per_V=as_float(motor_raw.get("Kv_rpm_per_V", 5478.655528120322), "$.motor_transmission.Kv_rpm_per_V", 5478.655528120322, errors),
            Kv_rad_per_V=as_float(motor_raw.get("Kv_rad_per_V", 0.0), "$.motor_transmission.Kv_rad_per_V", 0.0, errors),
            Kt_Nm_per_A=as_float(motor_raw.get("Kt_Nm_per_A", 0.001743), "$.motor_transmission.Kt_Nm_per_A", 0.001743, errors),
            I0_A=as_float(motor_raw.get("I0_noLoad_A", motor_raw.get("I0_A", 0.15)), "$.motor_transmission.I0_noLoad_A", 0.15, errors),
            b_visc_Nm_per_radps=as_float(motor_raw.get("b_visc_Nm_per_radps", 7e-08), "$.motor_transmission.b_visc_Nm_per_radps", 7e-08, errors),
            tau_coulomb_Nm=as_float(motor_raw.get("tau_coulomb_Nm", 0.000261), "$.motor_transmission.tau_coulomb_Nm", 0.000261, errors),
            Jm_kgm2=as_float(motor_raw.get("J_motor_kgm2", motor_raw.get("Jm_kgm2", 1e-08)), "$.motor_transmission.J_motor_kgm2", 1e-08, errors),
            Jload_reflected_kgm2=as_float(motor_raw.get("J_load_kgm2", motor_raw.get("Jload_reflected_kgm2", 0.0)), "$.motor_transmission.J_load_kgm2", 0.0, errors),
            stall_current_A=as_float(motor_raw.get("stallCurrent_A", motor_raw.get("stall_current_A", 1.5)), "$.motor_transmission.stallCurrent_A", 1.5, errors),
            driver_current_limit_A=as_float(motor_raw.get("driver_current_limit_A", 0.0), "$.motor_transmission.driver_current_limit_A", 0.0, errors),
        )

        ctrl_raw = obj.get("controller") or {}
        ctrl = ControllerSpec(
            pwm_min=as_int(ctrl_raw.get("pwm_min", -4095), "$.controller.pwm_min", -4095, errors),
            pwm_max=as_int(ctrl_raw.get("pwm_max", 4095), "$.controller.pwm_max", 4095, errors),
            pwm_neutral=None if ctrl_raw.get("pwm_neutral") is None else as_int(ctrl_raw.get("pwm_neutral"), "$.controller.pwm_neutral", 0, errors),
            deadband_percent=as_float(ctrl_raw.get("deadband_percent", 0.0), "$.controller.deadband_percent", 0.0, errors),
            pwm_resolution_bits=as_int(ctrl_raw.get("pwm_resolution_bits", 12), "$.controller.pwm_resolution_bits", 12, errors),
            pwm_frequency_Hz=as_float(ctrl_raw.get("pwm_frequency_Hz", 20000.0), "$.controller.pwm_frequency_Hz", 20000.0, errors),
            simulation_step_dt_ms=as_float(ctrl_raw.get("simulation_step_dt_ms", 1.0), "$.controller.simulation_step_dt_ms", 1.0, errors),
        )

        sens_raw = obj.get("sensor_model") or obj.get("sensorsConfig") or {}
        sens_model = SensorModelSpec(
            sensor_mode=str(sens_raw.get("sensor_mode", "analog")).lower(),
            sensor_bits=as_int(sens_raw.get("sensor_bits", 8), "$.sensorsConfig.sensor_bits", 8, errors),
            value_of_line=as_int(sens_raw.get("value_of_line", 0), "$.sensorsConfig.value_of_line", 0, errors),
            value_of_background=as_int(sens_raw.get("value_of_background", 255), "$.sensorsConfig.value_of_background", 255, errors),
            analog_noise_line=as_int(sens_raw.get("analog_noise_line", 50), "$.sensorsConfig.analog_noise_line", 50, errors),
            analog_noise_background=as_int(sens_raw.get("analog_noise_background", 50), "$.sensorsConfig.analog_noise_background", 50, errors),
        )
        if sens_model.sensor_mode not in {"analog", "digital"}:
            errors.append("$.sensorsConfig.sensor_mode deve ser 'analog' ou 'digital'.")

        spec = cls(
            version=str(obj.get("version", "robot-v1")),
            envelope=EnvelopeSpec(width, height),
            origin_x_mm=origin_x,
            origin_y_mm=origin_y,
            wheels=wheels,
            sensors=sensors,
            geometric_mechanical=gm,
            electrical=elec,
            motor_transmission=motor,
            controller=ctrl,
            sensor_model=sens_model,
            encoder=dict(obj.get("encoder") or obj.get("encoders") or {}),
            imu=dict(obj.get("imu") or {}),
            odometry=dict(obj.get("odometry") or {}),
        )
        errors.extend(spec.validate())
        raise_if_errors("Robot JSON inválido.", errors)
        return spec

    @classmethod
    def from_json_file(cls, path: str) -> "RobotSpec":
        try:
            with open(path, "r", encoding="utf-8") as f:
                return cls.from_dict(json.load(f))
        except ValidationError:
            raise
        except json.JSONDecodeError as e:
            raise ValidationError("Robot JSON inválido.", [f"Erro de sintaxe em {path}: {e}"])
        except OSError as e:
            raise ValidationError("Não foi possível abrir o arquivo de robô.", [str(e)])

    def validate(self) -> list[str]:
        errors: list[str] = []
        if self.envelope.width_mm <= 0:
            errors.append("Envelope widthMM deve ser > 0.")
        if self.envelope.height_mm <= 0:
            errors.append("Envelope heightMM deve ser > 0.")
        if not self.wheels:
            errors.append("Robô deve possuir pelo menos uma roda.")
        if not self.sensors:
            errors.append("Robô deve possuir pelo menos um sensor.")
        if self.controller.pwm_max <= self.controller.pwm_min:
            errors.append("controller.pwm_max deve ser maior que controller.pwm_min.")
        if self.controller.simulation_step_dt_ms <= 0:
            errors.append("controller.simulation_step_dt_ms deve ser > 0.")
        return errors

    def to_dict(self) -> dict[str, Any]:
        return {
            "version": self.version,
            "envelope": {"widthMM": self.envelope.width_mm, "heightMM": self.envelope.height_mm},
            "origin": {"xMM": self.origin_x_mm, "yMM": self.origin_y_mm},
            "wheels": [
                {"id": w.id, "xMM": w.x_mm, "yMM": w.y_mm, "widthMM": w.width_mm, "heightMM": w.height_mm}
                for w in self.wheels
            ],
            "sensors": [
                {
                    "id": s.id, "xMM": s.x_mm, "yMM": s.y_mm, "sizeMM": s.size_mm,
                    **({"gain": s.gain} if s.gain is not None else {}),
                    **({"offset": s.offset} if s.offset is not None else {}),
                    **({"noise_std": s.noise_std} if s.noise_std is not None else {}),
                    **({"latency_ms": s.latency_ms} if s.latency_ms is not None else {}),
                    **({"update_rate_Hz": s.update_rate_Hz} if s.update_rate_Hz is not None else {}),
                    **({"filter_tau_ms": s.filter_tau_ms} if s.filter_tau_ms is not None else {}),
                }
                for s in self.sensors
            ],
            "geometric_mechanical": {
                **{k: v for k, v in asdict(self.geometric_mechanical).items() if k not in {"cg_origin_xy_mm", "rot_origin_xy_mm"}},
                "cg_origin_xy_mm": list(self.geometric_mechanical.cg_origin_xy_mm),
                "rot_origin_xy_mm": list(self.geometric_mechanical.rot_origin_xy_mm),
            },
            "electrical": {
                "batteryVoltageV": self.electrical.battery_voltage_v,
                "batteryCapacitymAh": self.electrical.battery_capacity_mah,
                "R_batt_ohm": self.electrical.r_batt_ohm,
                "wiring_R_ohm": self.electrical.wiring_r_ohm,
                "driver_drop_V": self.electrical.driver_drop_v,
                "battery_OCV_table": [list(x) for x in self.electrical.battery_ocv_table],
            },
            "motor_transmission": {
                "gear_ratio": self.motor_transmission.gear_ratio,
                "eta": self.motor_transmission.eta,
                "R_motor_ohm": self.motor_transmission.Rm_ohm,
                "L_motor_H": self.motor_transmission.Lm_H,
                "Kv_rpm_per_V": self.motor_transmission.Kv_rpm_per_V,
                "Kv_rad_per_V": self.motor_transmission.Kv_rad_per_V,
                "Kt_Nm_per_A": self.motor_transmission.Kt_Nm_per_A,
                "I0_noLoad_A": self.motor_transmission.I0_A,
                "b_visc_Nm_per_radps": self.motor_transmission.b_visc_Nm_per_radps,
                "tau_coulomb_Nm": self.motor_transmission.tau_coulomb_Nm,
                "J_motor_kgm2": self.motor_transmission.Jm_kgm2,
                "J_load_kgm2": self.motor_transmission.Jload_reflected_kgm2,
                "stallCurrent_A": self.motor_transmission.stall_current_A,
                "driver_current_limit_A": self.motor_transmission.driver_current_limit_A,
            },
            "controller": {
                "pwm_resolution_bits": self.controller.pwm_resolution_bits,
                "pwm_frequency_Hz": self.controller.pwm_frequency_Hz,
                "deadband_percent": self.controller.deadband_percent,
                "pwm_max": self.controller.pwm_max,
                "pwm_min": self.controller.pwm_min,
                "simulation_step_dt_ms": self.controller.simulation_step_dt_ms,
                **({"pwm_neutral": self.controller.pwm_neutral} if self.controller.pwm_neutral is not None else {}),
            },
            "sensorsConfig": asdict(self.sensor_model),
            "encoders": self.encoder,
            "imu": self.imu,
            "odometry": self.odometry,
        }

    @classmethod
    def from_robot_model(cls, model: RobotModel) -> "RobotSpec":
        obj = model.to_json()
        return cls.from_dict(obj)

    def to_robot_model(self) -> RobotModel:
        return RobotModel(
            envelope=EditorEnvelope(self.envelope.width_mm, self.envelope.height_mm),
            originXMM=self.origin_x_mm,
            originYMM=self.origin_y_mm,
            wheels=[EditorWheel(w.id, w.x_mm, w.y_mm, w.width_mm, w.height_mm) for w in self.wheels],
            sensors=[EditorSensor(s.id, s.x_mm, s.y_mm, s.size_mm) for s in self.sensors],
        )
