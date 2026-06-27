import pytest

from Utils.robot_spec import RobotSpec
from Utils.simulation_config import SimulationConfig
from Utils.simulation_state import SimulationState
from sim.native_linesim import EXPECTED_LINESIM_ABI_VERSION, get_linesim, has_modular_physics, linesim_abi_version
from sim.physics.custom import CustomPhysicsModel, RealisticPhysicsModel


def _robot(**overrides):
    gm = {
        "track_mm": 100.0,
        "wheel_radius_mm": 10.0,
        "mass_kg": 0.2,
        "mu_static": 0.8,
        "mu_kinetic": 0.6,
        "Crr": 0.005,
    }
    gm.update(overrides.pop("geometric_mechanical", {}))
    motor = {
        "gear_ratio": 10.0,
        "eta": 0.9,
        "R_motor_ohm": 4.0,
        "L_motor_H": 0.0001,
        "Kv_rad_per_V": 100.0,
        "Kt_Nm_per_A": 0.01,
        "I0_noLoad_A": 0.0,
        "b_visc_Nm_per_radps": 0.0,
        "tau_coulomb_Nm": 0.0,
        "J_motor_kgm2": 1e-8,
        "J_load_kgm2": 0.0,
        "stallCurrent_A": 5.0,
        "driver_current_limit_A": 5.0,
    }
    motor.update(overrides.pop("motor_transmission", {}))
    return RobotSpec.from_dict({
        "version": "robot-v1",
        "envelope": {"widthMM": 160, "heightMM": 140},
        "origin": {"xMM": 0, "yMM": 0},
        "wheels": [{"id": "left", "xMM": 0, "yMM": 50}, {"id": "right", "xMM": 0, "yMM": -50}],
        "sensors": [{"id": "S1", "xMM": 60, "yMM": 0, "sizeMM": 5, "gain": 1.1, "offset": 3.0}],
        "controller": {"pwm_min": -100, "pwm_max": 100, "simulation_step_dt_ms": 1.0},
        "geometric_mechanical": gm,
        "motor_transmission": motor,
        "electrical": {
            "batteryVoltageV": 8.0,
            "batteryCapacitymAh": 400.0,
            "R_batt_ohm": 0.08,
            "wiring_R_ohm": 0.03,
            "driver_drop_V": 0.2,
            "battery_OCV_table": [[0.0, 6.0], [1.0, 8.0]],
        },
        "encoders": {"enable": True, "ppr": 1000},
        "imu": {"enable": True, "bias_deg_s": 0.0},
        **overrides,
    })


def _state(**kw):
    data = dict(
        t_ms=0,
        x_mm=0.0,
        y_mm=0.0,
        heading_deg=0.0,
        v_mm_s=0.0,
        omega_rad_s=0.0,
        a_lin_mm_s2=0.0,
        alpha_rad_s2=0.0,
        v_left_mm_s=0.0,
        v_right_mm_s=0.0,
        sensors=[1],
    )
    data.update(kw)
    return SimulationState(**data)


def _cfg(**kw):
    data = dict(
        physics_profile="custom",
        custom_use_dc_motor_model=True,
        custom_use_c_backend=True,
        custom_allow_python_fallback=False,
        custom_use_battery_model=True,
        custom_use_encoder_model=True,
        custom_use_imu_model=True,
        custom_use_wheel_slip=True,
        custom_use_acceleration_limit=False,
        custom_current_limit_a=5.0,
    )
    data.update(kw)
    return SimulationConfig(**data)


def test_phase44_native_abi_and_modular_step_are_available():
    lib = get_linesim()
    assert linesim_abi_version(lib) >= EXPECTED_LINESIM_ABI_VERSION
    assert has_modular_physics(lib)


def test_phase44_realistic_uses_c_and_logs_backend_channels():
    robot = _robot()
    cfg = SimulationConfig(physics_profile="realistic")
    model = RealisticPhysicsModel(cfg, robot=robot)
    s0 = _state()
    model.reset(s0)
    out = model.step(s0, 100, 100, 0.01, robot, cfg, native=get_linesim())
    assert out.physics_backend == "realistic_c_modular"
    assert out.linesim_abi_version >= EXPECTED_LINESIM_ABI_VERSION
    assert out.c_step_call_count > 0
    assert out.last_step_executed_in_c is True
    assert out.using_python_fallback is False


def test_phase44_realistic_does_not_fallback_silently_when_c_missing():
    robot = _robot()
    cfg = SimulationConfig(physics_profile="realistic")
    model = RealisticPhysicsModel(cfg, robot=robot)
    with pytest.raises(RuntimeError, match="exige backend C modular"):
        model.step(_state(), 100, 100, 0.01, robot, cfg, native=None)


def test_phase44_explicit_python_fallback_is_logged():
    robot = _robot()
    cfg = SimulationConfig(physics_profile="realistic", allow_python_fallback_for_realistic=True)
    model = RealisticPhysicsModel(cfg, robot=robot)
    model.reset(_state())
    out = model.step(_state(), 100, 100, 0.01, robot, cfg, native=None)
    assert out.physics_backend == "python_fallback_explicit"
    assert out.using_python_fallback is True
    assert out.last_step_executed_in_c is False


def test_phase44_low_friction_scenario_has_continuous_slip_and_force_logs():
    robot = _robot(geometric_mechanical={"mu_static": 0.05, "mu_kinetic": 0.04})
    cfg = _cfg()
    model = CustomPhysicsModel(cfg, robot=robot)
    s = _state()
    model.reset(s)
    out = model.step(s, 100, 100, 0.02, robot, cfg, native=get_linesim())
    assert 0.0 < out.slip_ratio_left <= cfg.custom_slip_max_ratio
    assert abs(out.force_longitudinal_command_left_n) >= abs(out.force_longitudinal_ground_left_n)
    assert out.wheel_left_surface_speed_mm_s > out.ground_left_speed_mm_s
    assert out.friction_usage_left > 0.0


def test_phase44_larger_reflected_inertia_reduces_wheel_acceleration():
    fast_robot = _robot(motor_transmission={"J_load_kgm2": 0.0})
    slow_robot = _robot(motor_transmission={"J_load_kgm2": 1e-3})
    cfg = _cfg(custom_use_wheel_slip=False)
    s = _state()
    mf = CustomPhysicsModel(cfg, robot=fast_robot)
    ms = CustomPhysicsModel(cfg, robot=slow_robot)
    mf.reset(s)
    ms.reset(s)
    fast = mf.step(s, 100, 100, 0.01, fast_robot, cfg, native=get_linesim())
    slow = ms.step(s, 100, 100, 0.01, slow_robot, cfg, native=get_linesim())
    assert fast.J_eq_left_kgm2 < slow.J_eq_left_kgm2
    assert abs(fast.alpha_wheel_left_rad_s2) > abs(slow.alpha_wheel_left_rad_s2)


def test_phase44_energy_and_torque_channels_are_finite_and_named():
    robot = _robot()
    cfg = _cfg()
    model = CustomPhysicsModel(cfg, robot=robot)
    s = _state()
    model.reset(s)
    out = model.step(s, 100, 80, 0.01, robot, cfg, native=get_linesim())
    assert out.battery_energy_j >= 0.0
    assert out.copper_loss_left_w >= 0.0
    assert out.driver_loss_left_w >= 0.0
    assert out.total_kinetic_energy_j >= 0.0
    assert out.energy_balance_error_percent == pytest.approx(out.energy_balance_error_percent)
    assert out.tau_motor_em_left_nm != 0.0
    assert out.tau_wheel_drive_left_nm != 0.0
    assert out.tau_ground_left_nm == pytest.approx(out.force_longitudinal_ground_left_n * robot.geometric_mechanical.wheel_radius_mm * 0.001)


def test_phase44_one_wheel_slippery_scenario_diverges_between_wheels():
    robot = _robot()
    cfg = _cfg(
        custom_mu_static_left=0.10,
        custom_mu_kinetic_left=0.08,
        custom_mu_static_right=1.00,
        custom_mu_kinetic_right=0.80,
    )
    model = CustomPhysicsModel(cfg, robot=robot)
    s = _state()
    model.reset(s)
    out = model.step(s, 10, 10, 0.02, robot, cfg, native=get_linesim())
    assert out.slip_ratio_left > out.slip_ratio_right
    assert abs(out.heading_deg) > 0.0
    assert abs(out.wheel_left_surface_speed_mm_s - out.ground_left_speed_mm_s) > abs(out.wheel_right_surface_speed_mm_s - out.ground_right_speed_mm_s)


def test_phase44_weak_battery_limits_voltage_torque_and_speed():
    robot = _robot()
    high_cfg = _cfg(custom_battery_soc_initial=1.0)
    low_cfg = _cfg(custom_battery_soc_initial=0.0)
    high_model = CustomPhysicsModel(high_cfg, robot=robot)
    low_model = CustomPhysicsModel(low_cfg, robot=robot)
    s = _state()
    high_model.reset(s)
    low_model.reset(s)
    high = high_model.step(s, 100, 100, 0.02, robot, high_cfg, native=get_linesim())
    low = low_model.step(s, 100, 100, 0.02, robot, low_cfg, native=get_linesim())
    assert low.battery_voltage_v < high.battery_voltage_v
    assert abs(low.tau_wheel_drive_left_nm) < abs(high.tau_wheel_drive_left_nm)
    assert abs(low.v_mm_s) < abs(high.v_mm_s)


def test_phase44_low_resolution_encoder_reports_integer_quantized_ticks():
    robot = _robot(encoders={"enable": True, "ppr": 4})
    cfg = _cfg()
    model = CustomPhysicsModel(cfg, robot=robot)
    s = _state()
    model.reset(s)
    out = model.step(s, 60, 60, 0.02, robot, cfg, native=get_linesim())
    assert isinstance(out.enc_left_ticks, int)
    assert isinstance(out.enc_left_delta_ticks, int)
    assert out.enc_left_ticks == out.enc_left_delta_ticks
