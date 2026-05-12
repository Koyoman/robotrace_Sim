from __future__ import annotations

import math
from typing import Protocol

from Utils.robot_spec import RobotSpec


class _PoseLike(Protocol):
    x_mm: float
    y_mm: float
    heading_deg: float


def robot_local_to_runtime(robot: RobotSpec, x_mm: float, y_mm: float) -> tuple[float, float]:
    """Convert robot JSON drawing coordinates to pose-relative runtime coordinates.

    The simulation pose represents ``robot.origin``. Points in the robot JSON are
    authored in the editor/drawing frame, so the configured origin must be
    subtracted before applying the pose transform.
    """
    return (float(x_mm) - float(robot.origin_x_mm), float(y_mm) - float(robot.origin_y_mm))


def robot_local_to_world(
    robot: RobotSpec,
    pose_x_mm: float,
    pose_y_mm: float,
    heading_deg: float,
    x_mm: float,
    y_mm: float,
) -> tuple[float, float]:
    """Convert a robot JSON local point to world coordinates for the current pose."""
    lx, ly = robot_local_to_runtime(robot, x_mm, y_mm)
    a = math.radians(float(heading_deg))
    c, s = math.cos(a), math.sin(a)
    return (
        float(pose_x_mm) + c * lx - s * ly,
        float(pose_y_mm) + s * lx + c * ly,
    )


def derive_wheel_track_mm(robot: RobotSpec) -> float:
    """Return wheel track in mm without being affected by robot.origin.

    Prefer the explicit geometric/mechanical value. If it is missing or invalid,
    derive the center-to-center distance from the left/right wheel coordinates.
    Subtracting the same origin from both wheels does not change their distance.
    """
    explicit = float(getattr(robot.geometric_mechanical, "track_mm", 0.0) or 0.0)
    if explicit > 0.0:
        return explicit

    wheels = list(getattr(robot, "wheels", []) or [])
    left = next((w for w in wheels if str(w.id).lower() == "left"), None)
    right = next((w for w in wheels if str(w.id).lower() == "right"), None)

    if left is not None and right is not None:
        lx, ly = robot_local_to_runtime(robot, left.x_mm, left.y_mm)
        rx, ry = robot_local_to_runtime(robot, right.x_mm, right.y_mm)
        dist = math.hypot(rx - lx, ry - ly)
        if dist > 0.0:
            return dist

    if len(wheels) >= 2:
        ax, ay = robot_local_to_runtime(robot, wheels[0].x_mm, wheels[0].y_mm)
        bx, by = robot_local_to_runtime(robot, wheels[1].x_mm, wheels[1].y_mm)
        dist = math.hypot(bx - ax, by - ay)
        if dist > 0.0:
            return dist

    return 70.0
