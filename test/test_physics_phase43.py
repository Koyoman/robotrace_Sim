import math

import pytest

from Utils.robot_spec import RobotSpec
from Utils.simulation_config import SimulationConfig
from Utils.simulation_state import SimulationState
from sim.physics.custom import CustomPhysicsModel, RealisticPhysicsModel


def _robot(**overrides):
    gm = {
        "track_mm": 100.0,
        "wheel_radius_mm": 10.0,
        "mass_kg": 0.2,
        "mu_static": 1.0,
        "mu_kinetic": 0.8,
        "Crr": 0.005,
    }
    gm.update(overrides.pop("geometric_mechanical", {}))
    motor = {
        "gear_ratio": 10.0,
        "eta": 0.9,
        "R_motor_ohm": 4.0,
        "L_motor_H": 0.0,
        "Kv_rad_per_V": 100.0,
        "Kt_Nm_per_A": 0.01,
        "I0_noLoad_A": 0.0,
        "b_visc_Nm_per_radps": 0.0,
        "tau_coulomb_Nm": 0.0,
        "J_motor_kgm2": 1e-8,
        "J_load_kgm2": 0.0,
        "stallCurrent_A": 100.0,
        "driver_current_limit_A": 0.0,
    }
    motor.update(overrides.pop("motor_transmission", {}))
    electrical = {
        "batteryVoltageV": 8.0,
        "batteryCapacitymAh": 400.0,
        "R_batt_ohm": 0.08,
        "wiring_R_ohm": 0.03,
        "driver_drop_V": 0.2,
        "battery_OCV_table": [[0.0, 6.0], [0.5, 7.0], [1.0, 8.0]],
    }
    electrical.update(overrides.pop("electrical", {}))
    enc = overrides.pop("encoders", {"enable": True, "ppr": 1000, "noise_std_pulses": 0.0, "update_rate_Hz": 0.0})
    imu = overrides.pop("imu", {"enable": True, "std_deg": 0.0, "bias_deg_s": 0.0, "update_rate_Hz": 0.0})
    return RobotSpec.from_dict({
        "version": "robot-v1",
        "envelope": {"widthMM": 160, "heightMM": 140},
        "origin": {"xMM": 0, "yMM": 0},
        "wheels": [{"id": "left", "xMM": 0, "yMM": 50}, {"id": "right", "xMM": 0, "yMM": -50}],
        "sensors": [{"id": "S1", "xMM": 60, "yMM": 0, "sizeMM": 5}],
        "controller": {"pwm_min": -100, "pwm_max": 100, "simulation_step_dt_ms": 1.0},
        "geometric_mechanical": gm,
        "motor_transmission": motor,
        "electrical": electrical,
        "encoders": enc,
        "imu": imu,
        "sensorsConfig": {"sensor_mode": "analog", "sensor_bits": 10, "value_of_line": 900, "value_of_background": 100, "analog_noise_line": 7, "analog_noise_background": 9},
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
        custom_use_c_backend=False,
        custom_use_battery_model=True,
        custom_use_encoder_model=True,
        custom_use_imu_model=True,
        custom_use_wheel_slip=True,
        custom_use_acceleration_limit=False,
        custom_current_limit_a=100.0,
        custom_sensor_noise_seed=123,
    )
    data.update(kw)
    return SimulationConfig(**data)


def test_phase43_consumption_current_never_negative_and_reverse_pwm_increases_current():
    robot = _robot()
    cfg = _cfg()
    moving = _state(
        v_mm_s=500.0,
        v_left_mm_s=500.0,
        v_right_mm_s=500.0,
        wheel_left_surface_speed_mm_s=500.0,
        wheel_right_surface_speed_mm_s=500.0,
    )

    fwd_model = CustomPhysicsModel(cfg, robot=robot)
    fwd_model.reset(moving)
    fwd = fwd_model.step(moving, 100, 100, 0.001, robot, cfg)

    rev_model = CustomPhysicsModel(cfg, robot=robot)
    rev_model.reset(moving)
    rev = rev_model.step(moving, -100, -100, 0.001, robot, cfg)

    assert fwd.current_left_a >= 0.0
    assert rev.current_left_a >= 0.0
    assert rev.current_total_a >= 0.0
    assert rev.battery_current_a >= 0.0
    assert rev.battery_power_w >= 0.0
    assert rev.motor_left_current_signed_a < 0.0
    assert rev.current_left_a > fwd.current_left_a


def test_phase43_encoder_is_integer_and_measures_wheel_not_ground_when_slipping():
    robot = _robot(geometric_mechanical={"mu_static": 0.01, "mu_kinetic": 0.008})
    cfg = _cfg()
    state = _state()
    model = CustomPhysicsModel(cfg, robot=robot)
    model.reset(state)
    out = model.step(state, 100, 100, 0.01, robot, cfg)

    assert isinstance(out.enc_left_ticks, int)
    assert isinstance(out.enc_left_delta_ticks, int)
    assert out.slip_ratio_left > 0.0
    encoder_surface_speed = out.enc_left_rad_s * robot.geometric_mechanical.wheel_radius_mm
    assert encoder_surface_speed == pytest.approx(out.wheel_left_surface_speed_mm_s)
    assert out.ground_left_speed_mm_s < encoder_surface_speed


def test_phase43_zero_slip_encoder_matches_ground_speed():
    robot = _robot(geometric_mechanical={"mu_static": 100.0, "mu_kinetic": 80.0})
    cfg = _cfg(custom_current_limit_a=0.05)
    state = _state()
    model = CustomPhysicsModel(cfg, robot=robot)
    model.reset(state)
    out = model.step(state, 10, 10, 0.01, robot, cfg)

    assert out.slip_ratio_left == pytest.approx(0.0)
    assert out.enc_left_rad_s * robot.geometric_mechanical.wheel_radius_mm == pytest.approx(out.ground_left_speed_mm_s)


def test_phase43_imu_uses_body_frame_not_world_heading_and_includes_bias():
    robot = _robot(imu={"enable": True, "std_deg": 0.0, "bias_deg_s": 5.0, "update_rate_Hz": 0.0})
    cfg = _cfg(custom_use_dc_motor_model=False, custom_use_acceleration_limit=False, custom_use_wheel_slip=False, basic_max_wheel_speed_mm_s=1000.0)
    s0 = _state(heading_deg=0.0)
    s90 = _state(heading_deg=90.0)

    m0 = CustomPhysicsModel(cfg, robot=robot)
    m0.reset(s0)
    out0 = m0.step(s0, 100, 50, 0.01, robot, cfg)

    m90 = CustomPhysicsModel(cfg, robot=robot)
    m90.reset(s90)
    out90 = m90.step(s90, 100, 50, 0.01, robot, cfg)

    assert out0.imu_accel_x_mm_s2 == pytest.approx(out90.imu_accel_x_mm_s2)
    assert out0.imu_accel_y_mm_s2 == pytest.approx(out0.v_mm_s * out0.omega_rad_s)
    assert out0.imu_omega_rad_s == pytest.approx(out0.omega_rad_s + math.radians(5.0))


def test_phase43_robot_json_parameters_affect_physics_channels():
    r_small = _robot(encoders={"enable": True, "ppr": 1000})
    r_big = _robot(geometric_mechanical={"wheel_radius_mm": 20.0}, encoders={"enable": True, "ppr": 1000})
    cfg = _cfg(custom_use_dc_motor_model=False, custom_use_acceleration_limit=False, custom_use_wheel_slip=False, basic_max_wheel_speed_mm_s=1000.0)
    s = _state()

    m_small = CustomPhysicsModel(cfg, robot=r_small)
    m_small.reset(s)
    out_small = m_small.step(s, 100, 100, 0.01, r_small, cfg)

    m_big = CustomPhysicsModel(cfg, robot=r_big)
    m_big.reset(s)
    out_big = m_big.step(s, 100, 100, 0.01, r_big, cfg)

    assert out_small.enc_left_delta_ticks != out_big.enc_left_delta_ticks
    assert r_big.sensor_model.sensor_bits == 10
    assert r_big.electrical.battery_ocv_table[-1][1] == pytest.approx(8.0)


def test_phase43_realistic_preset_does_not_force_fixed_slip_ratios():
    cfg = SimulationConfig(physics_profile="realistic", custom_slip_ratio_left=0.0, custom_slip_ratio_right=0.0)
    model = RealisticPhysicsModel(cfg, robot=_robot())
    assert model.config.custom_use_wheel_slip
    assert model.config.custom_use_manual_slip is False
    assert model.config.custom_slip_ratio_left == pytest.approx(0.0)
    assert model.config.custom_slip_ratio_right == pytest.approx(0.0)
