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

    cfg = SimulationConfig(simulation_step_dt_ms=2.5, max_time_s=0.001, save_logs=False, allow_python_fallback_for_realistic=True)
    engine = SimulationEngine(_track(), _robot(), cfg, controller, track_path=None, save_logs=False)
    list(engine.iter_steps())

    assert captured
    assert captured[0]["dt_s"] == cfg.dt_s == 0.0025


def test_engine_sensor_world_positions_apply_origin():
    engine = SimulationEngine(_track(), _robot(), SimulationConfig(save_logs=False, physics_profile="ideal"), lambda s: {})
    sx, sy = engine._sensors_world_xy(100.0, 200.0, 90.0)
    assert len(sx) == len(sy) == 1
    assert math.isclose(sx[0], 100.0, abs_tol=1e-9)
    assert math.isclose(sy[0], 270.0, abs_tol=1e-9)


def test_logs_include_phase4_telemetry_columns(tmp_path):
    def controller(state):
        return {"pwm_left": 0, "pwm_right": 0}

    cfg = SimulationConfig(
        physics_profile="ideal",
        simulation_step_dt_ms=1.0,
        max_time_s=0.001,
        save_logs=True,
    )
    engine = SimulationEngine(_track(), _robot(), cfg, controller, track_path=None, save_logs=True, base_dir=str(tmp_path))
    list(engine.iter_steps())

    log_dir = tmp_path / "Logs"
    csv_files = list(log_dir.glob("sim_log_*.csv"))
    json_files = list(log_dir.glob("sim_log_*.json"))
    assert len(csv_files) == 1
    assert len(json_files) == 1

    header = csv_files[0].read_text(encoding="utf-8").splitlines()[0].split(",")
    for col in [
        "battery_voltage_v",
        "battery_soc",
        "current_left_a",
        "current_right_a",
        "current_total_a",
        "battery_power_w",
        "enc_left_ticks",
        "enc_right_ticks",
        "imu_omega_rad_s",
        "imu_accel_x_mm_s2",
        "slip_ratio_left",
        "dt_s",
        "physics_profile",
    ]:
        assert col in header
