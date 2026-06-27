from __future__ import annotations

import ctypes
import os
import sys
from ctypes import POINTER, c_char_p, c_double, c_int
from dataclasses import dataclass

EXPECTED_LINESIM_ABI_VERSION = 4


class CPoint(ctypes.Structure):
    """ctypes struct matching the C backend Pt type."""

    _fields_ = [("x", c_double), ("y", c_double)]


class PhysicsInputC(ctypes.Structure):
    _fields_ = [
        ("pwm_left", c_double),
        ("pwm_right", c_double),
        ("max_pwm", c_double),
        ("ocv_voltage_v", c_double),
    ]


class PhysicsConfigC(ctypes.Structure):
    _fields_ = [
        ("use_dc_motor_model", c_int),
        ("use_kinematic_model", c_int),
        ("use_acceleration_limit", c_int),
        ("use_battery_model", c_int),
        ("use_wheel_slip", c_int),
        ("use_encoder_model", c_int),
        ("use_imu_model", c_int),
        ("dt_s", c_double),
        ("max_wheel_accel_mm_s2", c_double),
        ("max_wheel_speed_mm_s", c_double),
        ("battery_voltage_v", c_double),
        ("battery_nominal_voltage_v", c_double),
        ("battery_min_voltage_v", c_double),
        ("battery_capacity_mah", c_double),
        ("battery_internal_resistance_ohm", c_double),
        ("battery_soc", c_double),
        ("slip_ratio_left", c_double),
        ("slip_ratio_right", c_double),
        ("encoder_ticks_per_rev", c_int),
        ("encoder_quantization", c_int),
        ("track_m", c_double),
        ("wheel_radius_m", c_double),
        ("pwm_min", c_double),
        ("pwm_max", c_double),
        ("pwm_center", c_double),
        ("motor_deadzone_pwm", c_double),
        ("current_limit_a", c_double),
        ("drivetrain_efficiency", c_double),
        ("viscous_friction", c_double),
        ("coulomb_friction", c_double),
        ("driver_drop_v", c_double),
        ("rm_ohm", c_double),
        ("lm_h", c_double),
        ("kt_nm_per_a", c_double),
        ("ke_v_per_rad", c_double),
        ("gear_ratio", c_double),
        ("mass_kg", c_double),
        ("jz_kgm2", c_double),
        ("crr", c_double),
        ("rho_air", c_double),
        ("cda", c_double),
        ("mu_static", c_double),
        ("mu_kinetic", c_double),
        # ABI 4 additions.
        ("use_wheel_dynamics", c_int),
        ("use_continuous_slip", c_int),
        ("use_lateral_slip", c_int),
        ("use_combined_friction_limit", c_int),
        ("use_energy_balance", c_int),
        ("j_motor_kgm2", c_double),
        ("j_load_kgm2", c_double),
        ("wheel_mass_kg", c_double),
        ("r_batt_ohm", c_double),
        ("wiring_r_ohm", c_double),
        ("slip_stiffness_factor", c_double),
        ("slip_at_limit", c_double),
        ("slip_max_ratio", c_double),
        ("mu_static_left", c_double),
        ("mu_static_right", c_double),
        ("mu_kinetic_left", c_double),
        ("mu_kinetic_right", c_double),
    ]


class PhysicsStateC(ctypes.Structure):
    _fields_ = [
        ("x_mm", c_double),
        ("y_mm", c_double),
        ("heading_deg", c_double),
        ("v_left_mm_s", c_double),
        ("v_right_mm_s", c_double),
        ("v_mm_s", c_double),
        ("omega_rad_s", c_double),
        ("a_lin_mm_s2", c_double),
        ("alpha_rad_s2", c_double),
        ("battery_voltage_v", c_double),
        ("battery_soc", c_double),
        ("enc_left_ticks", c_double),
        ("enc_right_ticks", c_double),
        ("enc_left_delta_ticks", c_double),
        ("enc_right_delta_ticks", c_double),
        ("imu_omega_rad_s", c_double),
        ("imu_alpha_rad_s2", c_double),
        ("imu_accel_x_mm_s2", c_double),
        ("imu_accel_y_mm_s2", c_double),
        ("current_left_a", c_double),
        ("current_right_a", c_double),
        # ABI 4 persistent state.
        ("omega_wheel_left_rad_s", c_double),
        ("omega_wheel_right_rad_s", c_double),
        ("alpha_wheel_left_rad_s2", c_double),
        ("alpha_wheel_right_rad_s2", c_double),
        ("battery_energy_j", c_double),
        ("copper_loss_energy_j", c_double),
        ("driver_loss_energy_j", c_double),
        ("battery_internal_loss_energy_j", c_double),
        ("wiring_loss_energy_j", c_double),
        ("mechanical_friction_loss_energy_j", c_double),
        ("rolling_resistance_energy_j", c_double),
        ("tire_slip_loss_energy_j", c_double),
        ("brake_dissipated_energy_j", c_double),
    ]


class PhysicsTelemetryC(ctypes.Structure):
    _fields_ = [
        ("duty_left", c_double), ("duty_right", c_double),
        ("current_left_a", c_double), ("current_right_a", c_double), ("battery_current_a", c_double),
        ("tau_motor_em_left_nm", c_double), ("tau_motor_em_right_nm", c_double),
        ("tau_motor_viscous_left_nm", c_double), ("tau_motor_viscous_right_nm", c_double),
        ("tau_motor_coulomb_left_nm", c_double), ("tau_motor_coulomb_right_nm", c_double),
        ("tau_motor_net_left_nm", c_double), ("tau_motor_net_right_nm", c_double),
        ("tau_wheel_drive_left_nm", c_double), ("tau_wheel_drive_right_nm", c_double),
        ("tau_rolling_left_nm", c_double), ("tau_rolling_right_nm", c_double),
        ("tau_bearing_left_nm", c_double), ("tau_bearing_right_nm", c_double),
        ("tau_ground_left_nm", c_double), ("tau_ground_right_nm", c_double),
        ("tau_slip_loss_left_nm", c_double), ("tau_slip_loss_right_nm", c_double),
        ("force_longitudinal_command_left_n", c_double), ("force_longitudinal_command_right_n", c_double),
        ("force_longitudinal_ground_left_n", c_double), ("force_longitudinal_ground_right_n", c_double),
        ("force_longitudinal_max_left_n", c_double), ("force_longitudinal_max_right_n", c_double),
        ("force_longitudinal_saturation_left", c_double), ("force_longitudinal_saturation_right", c_double),
        ("lambda_long_left", c_double), ("lambda_long_right", c_double),
        ("lateral_accel_mm_s2", c_double),
        ("lateral_force_total_n", c_double), ("lateral_force_left_n", c_double), ("lateral_force_right_n", c_double),
        ("lateral_slip_left", c_double), ("lateral_slip_right", c_double),
        ("friction_usage_left", c_double), ("friction_usage_right", c_double),
        ("combined_friction_limit_left_n", c_double), ("combined_friction_limit_right_n", c_double),
        ("slip_ratio_left", c_double), ("slip_ratio_right", c_double),
        ("wheel_left_surface_speed_mm_s", c_double), ("wheel_right_surface_speed_mm_s", c_double),
        ("ground_left_speed_mm_s", c_double), ("ground_right_speed_mm_s", c_double),
        ("j_eq_left_kgm2", c_double), ("j_eq_right_kgm2", c_double),
        ("battery_power_w", c_double),
        ("copper_loss_left_w", c_double), ("copper_loss_right_w", c_double),
        ("driver_loss_left_w", c_double), ("driver_loss_right_w", c_double),
        ("battery_internal_loss_w", c_double), ("wiring_loss_w", c_double),
        ("mechanical_friction_loss_left_w", c_double), ("mechanical_friction_loss_right_w", c_double),
        ("rolling_resistance_loss_w", c_double),
        ("tire_slip_loss_left_w", c_double), ("tire_slip_loss_right_w", c_double),
        ("brake_dissipated_power_w", c_double),
        ("kinetic_power_delta_w", c_double),
        ("kinetic_energy_linear_j", c_double), ("kinetic_energy_angular_j", c_double),
        ("kinetic_energy_wheels_j", c_double), ("total_kinetic_energy_j", c_double),
        ("total_loss_energy_j", c_double),
        ("energy_balance_error_j", c_double), ("energy_balance_error_percent", c_double),
        ("step_executed_in_c", c_int),
    ]


@dataclass(slots=True)
class NativeBackendInfo:
    loaded: bool
    path: str
    error: str
    abi_version: int
    modular_step_available: bool
    compatible_modular: bool


_linesim = None
_load_error: Exception | None = None
_loaded_path = ""


def linesim_abi_version(lib) -> int:
    try:
        fn = getattr(lib, "linesim_abi_version_C")
        try:
            fn.restype = c_int
        except Exception:
            pass
        return int(fn())
    except Exception:
        return 0


def has_modular_physics(lib) -> bool:
    return bool(lib is not None and hasattr(lib, "step_physics_modular_C") and linesim_abi_version(lib) >= EXPECTED_LINESIM_ABI_VERSION)


def backend_info(lib=None) -> NativeBackendInfo:
    if lib is None:
        try:
            lib = get_linesim()
        except Exception as exc:
            return NativeBackendInfo(False, _loaded_path, str(exc), 0, False, False)
    abi = linesim_abi_version(lib)
    modular = hasattr(lib, "step_physics_modular_C")
    return NativeBackendInfo(True, _loaded_path, "", abi, modular, modular and abi >= EXPECTED_LINESIM_ABI_VERSION)


def _candidate_library_paths() -> list[str]:
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    lib_dir = os.path.join(root, "utills_c")
    names = ["linesim.dll"] if sys.platform.startswith("win") else ["liblinesim.so", "linesim.so", "linesim.dll"]
    return [os.path.join(lib_dir, name) for name in names]


def _dll_path() -> str:
    return _candidate_library_paths()[0]


def get_linesim():
    """Load and configure the native backend lazily."""
    global _linesim, _load_error, _loaded_path
    if _linesim is not None:
        return _linesim
    if _load_error is not None:
        raise RuntimeError(f"Não foi possível carregar o backend nativo linesim: {_load_error}")

    errors: list[Exception] = []
    try:
        if sys.platform.startswith("win") and hasattr(os, "add_dll_directory"):
            os.add_dll_directory(os.path.dirname(_dll_path()))
        lib = None
        for path in _candidate_library_paths():
            if not os.path.exists(path):
                continue
            try:
                lib = ctypes.CDLL(path)
                _loaded_path = path
                break
            except Exception as exc:
                errors.append(exc)
        if lib is None:
            detail = "; ".join(str(e) for e in errors) or "biblioteca nativa não encontrada"
            raise RuntimeError(detail)

        lib.envelope_contacts_tape_C.argtypes = [
            c_double, c_double, c_double,
            c_double, c_double,
            POINTER(CPoint), c_int,
            c_double, c_int,
        ]
        lib.envelope_contacts_tape_C.restype = c_int

        lib.estimate_sensor_coverage_C.argtypes = [
            c_double, c_double,
            POINTER(CPoint), c_int,
            c_double, c_double, c_int,
        ]
        lib.estimate_sensor_coverage_C.restype = c_double

        lib.estimate_sensors_coverage_batch_C = getattr(lib, "estimate_sensors_coverage_batch_C")
        lib.estimate_sensors_coverage_batch_C.argtypes = [
            POINTER(c_double), POINTER(c_double), c_int,
            POINTER(CPoint), c_int,
            c_double,
            POINTER(c_double), c_double,
            c_int,
            POINTER(c_double),
        ]
        lib.estimate_sensors_coverage_batch_C.restype = None

        lib.crossed_finish_C.argtypes = [
            c_double, c_double, c_double, c_double,
            c_double, c_double, c_double, c_double,
        ]
        lib.crossed_finish_C.restype = c_int

        lib.step_motor_drivetrain_C = getattr(lib, "step_motor_drivetrain_C")
        lib.step_motor_drivetrain_C.argtypes = [
            c_double, c_double, c_double,
            c_double, c_double, c_double, c_double,
            c_int, c_int,
            c_double, c_double, c_double, c_double,
            c_double, c_double, c_double, c_double,
            c_double, c_double, c_double, c_double,
            c_double, c_double,
            c_double, c_double,
            c_double, c_double, c_double, c_double,
            c_double, c_double, c_double,
            c_double, c_double,
            c_double,
            c_double,
            POINTER(c_double), POINTER(c_double), POINTER(c_double),
            POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_double),
        ]
        lib.step_motor_drivetrain_C.restype = None

        try:
            lib.linesim_abi_version_C = getattr(lib, "linesim_abi_version_C")
            lib.linesim_abi_version_C.argtypes = []
            lib.linesim_abi_version_C.restype = c_int
        except AttributeError:
            pass

        try:
            lib.linesim_backend_name_C = getattr(lib, "linesim_backend_name_C")
            lib.linesim_backend_name_C.argtypes = []
            lib.linesim_backend_name_C.restype = c_char_p
        except AttributeError:
            pass

        try:
            lib.step_physics_modular_C = getattr(lib, "step_physics_modular_C")
            lib.step_physics_modular_C.argtypes = [
                ctypes.POINTER(PhysicsInputC),
                ctypes.POINTER(PhysicsConfigC),
                ctypes.POINTER(PhysicsStateC),
                ctypes.POINTER(PhysicsTelemetryC),
            ]
            lib.step_physics_modular_C.restype = c_int
        except AttributeError:
            pass

        try:
            lib.envelope_contacts_raster_C.argtypes = [
                c_double, c_double, c_double,
                c_double, c_double,
                ctypes.POINTER(ctypes.c_ubyte), c_int, c_int,
                c_double, c_double, c_double,
            ]
            lib.envelope_contacts_raster_C.restype = c_int
        except Exception:
            pass

        _linesim = lib
        return lib
    except Exception as exc:
        _load_error = exc
        raise RuntimeError(f"Não foi possível carregar o backend nativo linesim: {exc}") from exc
