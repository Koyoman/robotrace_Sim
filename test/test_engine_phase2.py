import math

from Utils.robot_spec import RobotSpec
from Utils.simulation_config import SimulationConfig
from Utils.track_spec import TrackSpec
from sim.engine import SimulationEngine


def _track():
    return TrackSpec.from_dict({
        "area": {"widthMM": 1000, "heightMM": 1000},
        "origin": {"p": {"x": 100, "y": 100}, "headingDeg": 0},
        "tapeWidthMM": 20,
        "segments": [{"kind": "straight", "id": "s1", "lengthMM": 500}],
        "startFinish": None,
    })


def _robot():
    return RobotSpec.from_dict({
        "version": "robot-v1",
        "envelope": {"widthMM": 160, "heightMM": 140},
        "origin": {"xMM": -10, "yMM": 0},
        "wheels": [
            {"id": "left", "xMM": -10, "yMM": 35},
            {"id": "right", "xMM": -10, "yMM": -35},
        ],
        "sensors": [{"id": "S1", "xMM": 60, "yMM": 0, "sizeMM": 5}],
        "geometric_mechanical": {"track_mm": 70, "wheel_radius_mm": 11, "mass_kg": 0.2},
    })


class FakeNative:
    def step_motor_drivetrain_C(self, *args):
        x, y, heading = args[0].value, args[1].value, args[2].value
        ox, oy, oh, ov, ow, oil, oir = args[-7:]
        ox._obj.value = x
        oy._obj.value = y
        oh._obj.value = heading
        ov._obj.value = 0.0
        ow._obj.value = 0.0
        oil._obj.value = 0.0
        oir._obj.value = 0.0


def test_engine_controller_receives_config_dt_s(monkeypatch):
    import sim.engine as engine_module

    monkeypatch.setattr(engine_module, "get_linesim", lambda: FakeNative())
    captured = []

    def controller(state):
        captured.append(state)
        return {"pwm_left": 0, "pwm_right": 0}

    cfg = SimulationConfig(simulation_step_dt_ms=2.5, max_time_s=0.001, save_logs=False)
    engine = SimulationEngine(_track(), _robot(), cfg, controller, track_path=None, save_logs=False)
    list(engine.iter_steps())

    assert captured
    assert captured[0]["dt_s"] == cfg.dt_s == 0.0025


def test_engine_sensor_world_positions_apply_origin():
    engine = SimulationEngine(_track(), _robot(), SimulationConfig(save_logs=False), lambda s: {})
    sx, sy = engine._sensors_world_xy(100.0, 200.0, 90.0)
    assert len(sx) == len(sy) == 1
    assert math.isclose(sx[0], 100.0, abs_tol=1e-9)
    assert math.isclose(sy[0], 270.0, abs_tol=1e-9)
