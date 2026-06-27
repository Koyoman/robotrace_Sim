import ctypes

import pytest

from Utils.robot_spec import RobotSpec
from Utils.simulation_config import SimulationConfig
from Utils.simulation_state import SimulationState
from sim.native_linesim import PhysicsConfigC, PhysicsInputC, PhysicsStateC
from sim.physics.custom import CustomPhysicsModel, RealisticPhysicsModel
from sim.physics.factory import create_physics_model


def _robot():
    return RobotSpec.from_dict({
        "version": "robot-v1",
        "envelope": {"widthMM": 160, "heightMM": 140},
        "origin": {"xMM": 0, "yMM": 0},
        "wheels": [
            {"id": "left", "xMM": 0, "yMM": 35},
            {"id": "right", "xMM": 0, "yMM": -35},
        ],
        "sensors": [{"id": "S1", "xMM": 60, "yMM": 0, "sizeMM": 5}],
        "controller": {"pwm_min": -100, "pwm_max": 100, "simulation_step_dt_ms": 10},
        "geometric_mechanical": {"track_mm": 100, "wheel_radius_mm": 10, "mass_kg": 0.2},
    })


def _state(**overrides):
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
    data.update(overrides)
    return SimulationState(**data)


class FakeModularNative:
    def __init__(self):
        self.calls = 0
        self.last_config = None

    def linesim_abi_version_C(self):
        return 4

    def step_physics_modular_C(self, input_ptr, config_ptr, state_ptr, telemetry_ptr):
        self.calls += 1
        cfg = ctypes.cast(config_ptr, ctypes.POINTER(PhysicsConfigC)).contents
        st = ctypes.cast(state_ptr, ctypes.POINTER(PhysicsStateC)).contents
        telem = ctypes.cast(telemetry_ptr, ctypes.POINTER(__import__('sim.native_linesim', fromlist=['PhysicsTelemetryC']).PhysicsTelemetryC)).contents
        self.last_config = cfg
        telem.step_executed_in_c = 1
        telem.wheel_left_surface_speed_mm_s = st.v_left_mm_s
        telem.wheel_right_surface_speed_mm_s = st.v_right_mm_s
        telem.ground_left_speed_mm_s = st.v_left_mm_s
        telem.ground_right_speed_mm_s = st.v_right_mm_s
        st.v_left_mm_s = cfg.max_wheel_accel_mm_s2 * cfg.dt_s
        st.v_right_mm_s = cfg.max_wheel_accel_mm_s2 * cfg.dt_s
        st.v_mm_s = st.v_left_mm_s
        st.x_mm += st.v_mm_s * cfg.dt_s
        st.battery_soc = 0.99 if cfg.use_battery_model else 1.0
        st.battery_voltage_v = cfg.battery_voltage_v
        st.enc_left_delta_ticks = 1.0 if cfg.use_encoder_model else 0.0
        st.enc_right_delta_ticks = 1.0 if cfg.use_encoder_model else 0.0
        st.enc_left_ticks += st.enc_left_delta_ticks
        st.enc_right_ticks += st.enc_right_delta_ticks
        st.imu_omega_rad_s = st.omega_rad_s if cfg.use_imu_model else 0.0
        return 0


def test_phase4_config_flags_defaults_and_validation():
    cfg = SimulationConfig()
    assert cfg.physics_profile == "realistic"
    assert cfg.custom_use_battery_model is False
    assert cfg.custom_encoder_ticks_per_rev == 1024
    assert cfg.custom_track_imperfection_wavelength_mm == pytest.approx(500.0)
    with pytest.raises(Exception):
        SimulationConfig.from_dict({"custom_encoder_ticks_per_rev": 0})
    with pytest.raises(Exception):
        SimulationConfig.from_dict({"custom_slip_ratio_left": 1.1})
    with pytest.raises(Exception):
        SimulationConfig.from_dict({"custom_sensor_noise_std": -1})


def test_realistic_preset_enables_complete_model():
    model = create_physics_model(SimulationConfig(physics_profile="realistic"), _robot())
    assert isinstance(model, RealisticPhysicsModel)
    cfg = model.config
    assert cfg.custom_use_dc_motor_model
    assert cfg.custom_use_battery_model
    assert cfg.custom_use_sensor_noise
    assert cfg.custom_use_encoder_model
    assert cfg.custom_use_imu_model
    assert cfg.custom_use_wheel_slip
    assert cfg.custom_use_track_imperfections


def test_custom_dc_uses_modular_c_and_passes_flags():
    robot = _robot()
    cfg = SimulationConfig(
        physics_profile="custom",
        custom_use_dc_motor_model=True,
        custom_use_battery_model=True,
        custom_use_encoder_model=True,
        custom_use_imu_model=True,
        custom_max_wheel_accel_mm_s2=1234.0,
        custom_use_auto_acceleration_limit=False,
    )
    model = CustomPhysicsModel(cfg, robot=robot)
    fake = FakeModularNative()
    out = model.step(_state(), 100, 100, 0.01, robot, cfg, native=fake)
    assert fake.calls == 1
    assert fake.last_config.use_battery_model == 1
    assert fake.last_config.use_encoder_model == 1
    assert fake.last_config.max_wheel_accel_mm_s2 == pytest.approx(1234.0)
    assert out.battery_soc == pytest.approx(0.99)
    assert out.enc_left_delta_ticks == pytest.approx(1.0)


def test_custom_kinematic_slip_encoder_and_imu_have_effect():
    robot = _robot()
    cfg = SimulationConfig(
        physics_profile="custom",
        custom_use_dc_motor_model=False,
        custom_use_acceleration_limit=False,
        custom_use_encoder_model=True,
        custom_use_imu_model=True,
        custom_use_wheel_slip=True,
        custom_slip_ratio_left=0.0,
        custom_slip_ratio_right=0.5,
        basic_max_wheel_speed_mm_s=1000.0,
    )
    model = CustomPhysicsModel(cfg, robot=robot)
    state = _state()
    model.reset(state)
    out = model.step(state, 100, 100, 0.01, robot, cfg)
    assert out.v_left_mm_s > out.v_right_mm_s
    assert out.omega_rad_s < 0.0
    assert out.enc_left_delta_ticks != 0.0
    assert "imu_omega_rad_s" in out.to_controller_state(cfg.dt_s)



def test_modular_battery_voltage_does_not_collapse_recursively():
    robot = _robot()
    cfg = SimulationConfig(
        physics_profile="custom",
        custom_use_dc_motor_model=True,
        custom_use_battery_model=True,
        custom_use_wheel_slip=False,
        custom_use_acceleration_limit=True,
        custom_battery_initial_voltage_v=8.0,
        custom_battery_min_voltage_v=6.0,
        custom_battery_internal_resistance_ohm=0.08,
        custom_battery_capacity_mah=400.0,
    )
    model = CustomPhysicsModel(cfg, robot=robot)
    state = _state()
    model.reset(state)

    class Native(FakeModularNative):
        def step_physics_modular_C(self, input_ptr, config_ptr, state_ptr, telemetry_ptr):
            cfg_c = ctypes.cast(config_ptr, ctypes.POINTER(PhysicsConfigC)).contents
            st = ctypes.cast(state_ptr, ctypes.POINTER(PhysicsStateC)).contents
            telem = ctypes.cast(telemetry_ptr, ctypes.POINTER(__import__('sim.native_linesim', fromlist=['PhysicsTelemetryC']).PhysicsTelemetryC)).contents
            telem.step_executed_in_c = 1
            self.calls += 1
            self.last_config = cfg_c
            # Emulate a fixed-current backend: terminal voltage should stay near
            # source voltage minus one sag term, not compound every call.
            current = 1.0
            soc = max(0.0, min(1.0, st.battery_soc - current * cfg_c.dt_s / (cfg_c.battery_capacity_mah * 3.6)))
            voc = cfg_c.battery_min_voltage_v + soc * (cfg_c.battery_voltage_v - cfg_c.battery_min_voltage_v)
            st.battery_soc = soc
            st.battery_voltage_v = max(cfg_c.battery_min_voltage_v, voc - current * cfg_c.battery_internal_resistance_ohm)
            st.v_left_mm_s = 1000.0
            st.v_right_mm_s = 1000.0
            st.v_mm_s = 1000.0
            telem.wheel_left_surface_speed_mm_s = st.v_left_mm_s
            telem.wheel_right_surface_speed_mm_s = st.v_right_mm_s
            telem.ground_left_speed_mm_s = st.v_left_mm_s
            telem.ground_right_speed_mm_s = st.v_right_mm_s
            return 0

    native = Native()
    for _ in range(100):
        state = model.step(state, 100, 100, 0.001, robot, cfg, native=native)
    assert state.battery_voltage_v > 7.8


def test_modular_slip_is_not_reapplied_as_motor_speed_damping():
    robot = _robot()
    cfg = SimulationConfig(
        physics_profile="custom",
        custom_use_dc_motor_model=True,
        custom_use_battery_model=False,
        custom_use_wheel_slip=True,
        custom_slip_ratio_left=0.01,
        custom_slip_ratio_right=0.01,
        custom_use_acceleration_limit=False,
    )
    model = CustomPhysicsModel(cfg, robot=robot)
    state = _state()
    model.reset(state)

    class Native(FakeModularNative):
        def step_physics_modular_C(self, input_ptr, config_ptr, state_ptr, telemetry_ptr):
            cfg_c = ctypes.cast(config_ptr, ctypes.POINTER(PhysicsConfigC)).contents
            st = ctypes.cast(state_ptr, ctypes.POINTER(PhysicsStateC)).contents
            telem = ctypes.cast(telemetry_ptr, ctypes.POINTER(__import__('sim.native_linesim', fromlist=['PhysicsTelemetryC']).PhysicsTelemetryC)).contents
            telem.step_executed_in_c = 1
            self.calls += 1
            self.last_config = cfg_c
            # ABI 4 makes C authoritative for slip; Python no longer reapplies it.
            assert cfg_c.use_wheel_slip == 1
            st.v_left_mm_s = 1000.0
            st.v_right_mm_s = 1000.0
            st.v_mm_s = 1000.0
            telem.wheel_left_surface_speed_mm_s = st.v_left_mm_s
            telem.wheel_right_surface_speed_mm_s = st.v_right_mm_s
            telem.ground_left_speed_mm_s = st.v_left_mm_s
            telem.ground_right_speed_mm_s = st.v_right_mm_s
            return 0

    native = Native()
    for _ in range(10):
        state = model.step(state, 100, 100, 0.01, robot, cfg, native=native)
    assert state.v_mm_s == pytest.approx(1000.0)
    assert model._model_v_mm_s == pytest.approx(1000.0)
    assert state.physics_backend == "custom_c_modular"
