from __future__ import annotations

import math
from abc import ABC, abstractmethod
from typing import Any

from Utils.robot_runtime import derive_wheel_track_mm
from Utils.robot_spec import RobotSpec
from Utils.simulation_config import SimulationConfig
from Utils.simulation_state import SimulationState


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def pwm_to_normalized(pwm: int, robot: RobotSpec) -> float:
    """Map a raw PWM command to [-1, 1] using the robot controller limits."""
    ctrl = robot.controller
    pwm_min = float(ctrl.pwm_min)
    pwm_max = float(ctrl.pwm_max)
    if pwm_max <= pwm_min:
        return 0.0

    neutral = 0.5 * (pwm_min + pwm_max) if ctrl.pwm_neutral is None else float(ctrl.pwm_neutral)
    neutral = clamp(neutral, pwm_min, pwm_max)
    raw = float(pwm)

    if raw >= neutral:
        denom = max(1e-9, pwm_max - neutral)
        norm = (raw - neutral) / denom
    else:
        denom = max(1e-9, neutral - pwm_min)
        norm = (raw - neutral) / denom

    norm = clamp(norm, -1.0, 1.0)
    deadband = clamp(float(ctrl.deadband_percent) * 0.01, 0.0, 0.99)
    if abs(norm) <= deadband:
        return 0.0
    return norm


def integrate_differential_pose(
    x_mm: float,
    y_mm: float,
    heading_deg: float,
    v_left_mm_s: float,
    v_right_mm_s: float,
    track_mm: float,
    dt_s: float,
) -> tuple[float, float, float, float, float]:
    """Integrate differential-drive kinematics for one timestep."""
    track = max(1e-9, float(track_mm))
    dt = max(0.0, float(dt_s))
    v_mm_s = 0.5 * (float(v_left_mm_s) + float(v_right_mm_s))
    omega_rad_s = (float(v_right_mm_s) - float(v_left_mm_s)) / track

    h0 = math.radians(float(heading_deg))
    h_mid = h0 + 0.5 * omega_rad_s * dt
    x_new = float(x_mm) + v_mm_s * math.cos(h_mid) * dt
    y_new = float(y_mm) + v_mm_s * math.sin(h_mid) * dt
    h_new = h0 + omega_rad_s * dt
    return x_new, y_new, math.degrees(h_new), v_mm_s, omega_rad_s


def wheel_track_mm(robot: RobotSpec) -> float:
    return max(1e-9, float(derive_wheel_track_mm(robot)))


def auto_max_wheel_speed_mm_s(config_value: float | None, params: dict[str, Any] | None = None, fallback_mm_s: float = 500.0) -> float:
    """Resolve full-PWM wheel speed for ideal/basic profiles.

    When the config value is None, derive the scale from the robot motor/battery
    data prepared by derive_runtime_params. This keeps ideal/basic comparable to
    the DC model for the same PWM command.
    """
    if config_value is not None:
        return max(1e-9, float(config_value))
    params = params or {}
    speed_mps = params.get("motor_no_load_wheel_speed_mps", params.get("final_linear_speed_mps", None))
    if speed_mps is None:
        return max(1e-9, float(fallback_mm_s))
    return max(1e-9, float(speed_mps) * 1000.0)


def auto_wheel_accel_mm_s2(config_value: float | None, params: dict[str, Any] | None = None, fallback_mm_s2: float = 9810.0) -> float:
    """Resolve wheel acceleration limit for the basic profile."""
    if config_value is not None:
        return max(1e-9, float(config_value))
    params = params or {}
    accel_mps2 = params.get("basic_max_wheel_accel_mps2", None)
    if accel_mps2 is None:
        return max(1e-9, float(fallback_mm_s2))
    return max(1e-9, float(accel_mps2) * 1000.0)


class PhysicsModel(ABC):
    """Small interface implemented by all physics profiles."""

    requires_native: bool = False

    def reset(self, initial_state: SimulationState) -> None:
        """Reset model-internal state before a new run."""
        return None

    @abstractmethod
    def step(
        self,
        state: SimulationState,
        pwm_left: int,
        pwm_right: int,
        dt_s: float,
        robot: RobotSpec,
        config: SimulationConfig,
        native: Any | None = None,
    ) -> SimulationState:
        """Advance the physical state by one timestep."""
        raise NotImplementedError
