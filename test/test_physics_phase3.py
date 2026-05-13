import math

import pytest

from Utils.robot_spec import RobotSpec
from Utils.simulation_config import SimulationConfig
from Utils.simulation_state import SimulationState
from Utils.track_spec import TrackSpec
from Utils.validation import ValidationError
from sim.engine import SimulationEngine
from sim.physics.dc_motor import DCMotorPhysicsModel
from sim.physics.factory import create_physics_model
from sim.physics.ideal import IdealPhysicsModel
from sim.physics.kinematic import BasicKinematicPhysicsModel


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
        "geometric_mechanical": {"track_mm": 100, "wheel_radius_mm": 11, "mass_kg": 0.2},
    })


def _track():
    return TrackSpec.from_dict({
        "area": {"widthMM": 1000, "heightMM": 1000},
        "origin": {"p": {"x": 100, "y": 100}, "headingDeg": 0},
        "tapeWidthMM": 20,
        "segments": [{"kind": "straight", "id": "s1", "lengthMM": 500}],
        "startFinish": None,
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


class FakeNative:
    def __init__(self):
        self.calls = 0

    def step_motor_drivetrain_C(self, *args):
        self.calls += 1
        x, y, heading = args[0].value, args[1].value, args[2].value
        dt = args[-8].value
        ox, oy, oh, ov, ow, oil, oir = args[-7:]
        ox._obj.value = x + 0.1 * dt
        oy._obj.value = y
        oh._obj.value = heading
        ov._obj.value = 0.1
        ow._obj.value = 0.0
        oil._obj.value = 0.01
        oir._obj.value = 0.01


def test_simulation_config_default_and_valid_profiles():
    assert SimulationConfig().physics_profile == "realistic"
    for profile in ["ideal", "basic", "realistic", "custom"]:
        cfg = SimulationConfig.from_dict({"physics_profile": profile})
        assert cfg.physics_profile == profile


def test_simulation_config_rejects_invalid_physics_profile():
    with pytest.raises(ValidationError) as exc:
        SimulationConfig.from_dict({"physics_profile": "arcade"})
    assert "Invalid physics_profile" in str(exc.value)


def test_physics_factory_selects_models():
    robot = _robot()
    assert isinstance(create_physics_model(SimulationConfig(physics_profile="ideal"), robot), IdealPhysicsModel)
    assert isinstance(create_physics_model(SimulationConfig(physics_profile="basic"), robot), BasicKinematicPhysicsModel)
    assert isinstance(create_physics_model(SimulationConfig(physics_profile="realistic"), robot), DCMotorPhysicsModel)
    assert isinstance(create_physics_model(SimulationConfig(physics_profile="custom", custom_use_dc_motor_model=True), robot), DCMotorPhysicsModel)
    custom_basic = create_physics_model(SimulationConfig(physics_profile="custom", custom_use_dc_motor_model=False), robot)
    assert isinstance(custom_basic, BasicKinematicPhysicsModel)


def test_physics_factory_rejects_invalid_profile_without_raw_traceback():
    with pytest.raises(ValidationError):
        create_physics_model(SimulationConfig(physics_profile="invalid"), _robot())




def test_ideal_auto_speed_uses_robot_runtime_params():
    model = IdealPhysicsModel(params={"motor_no_load_wheel_speed_mps": 2.0})
    robot = _robot()
    cfg = SimulationConfig(physics_profile="ideal")
    out = model.step(_state(), 100, 100, 0.01, robot, cfg)
    assert out.v_left_mm_s == pytest.approx(2000.0)
    assert out.x_mm == pytest.approx(20.0)


def test_basic_auto_acceleration_uses_robot_runtime_params():
    model = BasicKinematicPhysicsModel(use_acceleration_limit=True, params={
        "motor_no_load_wheel_speed_mps": 2.0,
        "basic_max_wheel_accel_mps2": 5.0,
    })
    robot = _robot()
    cfg = SimulationConfig(physics_profile="basic")
    state = _state()
    model.reset(state)
    out = model.step(state, 100, 100, 0.01, robot, cfg)
    assert out.v_left_mm_s == pytest.approx(50.0)
    assert out.a_lin_mm_s2 == pytest.approx(5000.0)

def test_ideal_pwm_zero_keeps_robot_stopped():
    model = IdealPhysicsModel()
    robot = _robot()
    cfg = SimulationConfig(physics_profile="ideal", ideal_max_wheel_speed_mm_s=500.0)
    out = model.step(_state(), 0, 0, 0.01, robot, cfg)
    assert out.x_mm == pytest.approx(0.0)
    assert out.y_mm == pytest.approx(0.0)
    assert out.v_left_mm_s == pytest.approx(0.0)
    assert out.v_right_mm_s == pytest.approx(0.0)


def test_ideal_equal_pwm_moves_straight_and_scales_with_dt():
    model = IdealPhysicsModel()
    robot = _robot()
    cfg = SimulationConfig(physics_profile="ideal", ideal_max_wheel_speed_mm_s=500.0)
    out_10ms = model.step(_state(), 100, 100, 0.01, robot, cfg)
    out_20ms = model.step(_state(), 100, 100, 0.02, robot, cfg)
    assert out_10ms.x_mm == pytest.approx(5.0)
    assert out_10ms.y_mm == pytest.approx(0.0)
    assert out_10ms.heading_deg == pytest.approx(0.0)
    assert out_20ms.x_mm == pytest.approx(10.0)
    assert out_20ms.v_left_mm_s == pytest.approx(500.0)
    assert out_20ms.v_right_mm_s == pytest.approx(500.0)


def test_ideal_opposite_pwm_rotates():
    model = IdealPhysicsModel()
    robot = _robot()
    cfg = SimulationConfig(physics_profile="ideal", ideal_max_wheel_speed_mm_s=500.0)
    out = model.step(_state(), -100, 100, 0.01, robot, cfg)
    assert out.x_mm == pytest.approx(0.0)
    assert out.omega_rad_s == pytest.approx(10.0)
    assert out.heading_deg == pytest.approx(math.degrees(0.1))


def test_basic_limits_acceleration_and_converges():
    model = BasicKinematicPhysicsModel(use_acceleration_limit=True)
    robot = _robot()
    cfg = SimulationConfig(
        physics_profile="basic",
        basic_max_wheel_speed_mm_s=500.0,
        basic_max_wheel_accel_mm_s2=1000.0,
    )
    state = _state()
    model.reset(state)
    first = model.step(state, 100, 100, 0.01, robot, cfg)
    assert first.v_left_mm_s == pytest.approx(10.0)
    assert first.v_mm_s == pytest.approx(10.0)
    assert first.a_lin_mm_s2 == pytest.approx(1000.0)

    state = first
    for _ in range(100):
        state = model.step(state, 100, 100, 0.01, robot, cfg)
    assert state.v_left_mm_s == pytest.approx(500.0)
    assert state.v_right_mm_s == pytest.approx(500.0)


def test_basic_custom_can_disable_acceleration_limit():
    model = create_physics_model(SimulationConfig(
        physics_profile="custom",
        custom_use_dc_motor_model=False,
        custom_use_acceleration_limit=False,
        basic_max_wheel_speed_mm_s=500.0,
        basic_max_wheel_accel_mm_s2=1.0,
    ), _robot())
    state = _state()
    model.reset(state)
    out = model.step(state, 100, 100, 0.01, _robot(), SimulationConfig(
        physics_profile="custom",
        custom_use_dc_motor_model=False,
        custom_use_acceleration_limit=False,
        basic_max_wheel_speed_mm_s=500.0,
        basic_max_wheel_accel_mm_s2=1.0,
    ))
    assert out.v_left_mm_s == pytest.approx(500.0)


def test_dc_model_calls_native_backend_and_updates_state():
    robot = _robot()
    cfg = SimulationConfig(physics_profile="realistic")
    model = DCMotorPhysicsModel(robot=robot)
    fake = FakeNative()
    out = model.step(_state(), 100, 100, 0.01, robot, cfg, native=fake)
    assert fake.calls == 1
    assert out.x_mm == pytest.approx(1.0)
    assert out.v_mm_s == pytest.approx(100.0)
    assert out.v_left_mm_s == pytest.approx(100.0)
    assert out.v_right_mm_s == pytest.approx(100.0)


def test_engine_uses_selected_ideal_model_and_keeps_controller_payload():
    captured = []

    def controller(state):
        captured.append(state)
        return {"pwm_left": 100, "pwm_right": 100}

    cfg = SimulationConfig(
        physics_profile="ideal",
        simulation_step_dt_ms=10.0,
        max_time_s=0.01,
        save_logs=False,
        ideal_max_wheel_speed_mm_s=500.0,
    )
    engine = SimulationEngine(_track(), _robot(), cfg, controller, track_path=None, save_logs=False)
    steps = list(engine.iter_steps())
    assert isinstance(engine.physics_model, IdealPhysicsModel)
    assert captured
    for key in ["t_ms", "dt_s", "x_mm", "y_mm", "heading_deg", "v_mm_s", "omega_rad_s", "a_lin_mm_s2", "alpha_rad_s2", "sensors", "v_left_mm_s", "v_right_mm_s"]:
        assert key in captured[0]
    assert captured[0]["dt_s"] == pytest.approx(cfg.dt_s)
    assert steps[0]["x_mm"] > 100.0


def test_custom_kinematic_uses_manual_speed_and_acceleration_overrides():
    robot = _robot()
    cfg = SimulationConfig(
        physics_profile="custom",
        custom_use_dc_motor_model=False,
        custom_use_acceleration_limit=True,
        basic_max_wheel_speed_mm_s=800.0,
        basic_max_wheel_accel_mm_s2=2000.0,
    )
    model = create_physics_model(cfg, robot)
    assert isinstance(model, BasicKinematicPhysicsModel)
    state = _state()
    model.reset(state)
    out = model.step(state, 100, 100, 0.01, robot, cfg)
    assert out.v_left_mm_s == pytest.approx(20.0)
    assert out.v_right_mm_s == pytest.approx(20.0)
    assert out.a_lin_mm_s2 == pytest.approx(2000.0)
