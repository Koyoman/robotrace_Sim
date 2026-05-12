import math

from Utils.robot_runtime import derive_wheel_track_mm, robot_local_to_runtime, robot_local_to_world
from Utils.robot_spec import RobotSpec


def _robot(origin_x=-10.0, origin_y=0.0, track_mm=0.0):
    return RobotSpec.from_dict({
        "version": "robot-v1",
        "envelope": {"widthMM": 160, "heightMM": 140},
        "origin": {"xMM": origin_x, "yMM": origin_y},
        "wheels": [
            {"id": "left", "xMM": -10, "yMM": 35, "widthMM": 22, "heightMM": 15},
            {"id": "right", "xMM": -10, "yMM": -35, "widthMM": 22, "heightMM": 15},
        ],
        "sensors": [{"id": "S1", "xMM": 60, "yMM": 10, "sizeMM": 5}],
        "geometric_mechanical": {"track_mm": track_mm, "wheel_radius_mm": 11, "mass_kg": 0.2},
    })


def test_robot_local_to_runtime_subtracts_origin():
    robot = _robot(origin_x=-10.0, origin_y=5.0)
    assert robot_local_to_runtime(robot, 60.0, 10.0) == (70.0, 5.0)


def test_robot_local_to_world_uses_pose_origin():
    robot = _robot(origin_x=-10.0, origin_y=0.0)
    wx, wy = robot_local_to_world(robot, 100.0, 200.0, 90.0, 60.0, 0.0)
    assert math.isclose(wx, 100.0, abs_tol=1e-9)
    assert math.isclose(wy, 270.0, abs_tol=1e-9)


def test_derive_wheel_track_prefers_explicit_track_mm():
    robot = _robot(origin_x=-10.0, track_mm=80.0)
    assert derive_wheel_track_mm(robot) == 80.0


def test_derive_wheel_track_fallback_is_not_distorted_by_origin():
    robot = _robot(origin_x=-10.0, track_mm=0.0)
    assert derive_wheel_track_mm(robot) == 70.0
