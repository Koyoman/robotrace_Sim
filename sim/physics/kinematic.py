from __future__ import annotations

from Utils.robot_spec import RobotSpec
from Utils.simulation_config import SimulationConfig
from Utils.simulation_state import SimulationState

from sim.physics.base import PhysicsModel, auto_max_wheel_speed_mm_s, auto_wheel_accel_mm_s2, clamp, integrate_differential_pose, pwm_to_normalized, wheel_track_mm


class BasicKinematicPhysicsModel(PhysicsModel):
    """Kinematic differential-drive model with optional wheel acceleration limit."""

    requires_native = False

    def __init__(self, *, use_acceleration_limit: bool = True, params: dict | None = None):
        self.use_acceleration_limit = bool(use_acceleration_limit)
        self.params = dict(params or {})
        self._v_left_mm_s = 0.0
        self._v_right_mm_s = 0.0

    def reset(self, initial_state: SimulationState) -> None:
        self._v_left_mm_s = float(initial_state.v_left_mm_s)
        self._v_right_mm_s = float(initial_state.v_right_mm_s)

    def _advance_wheel_speed(self, current: float, target: float, dt_s: float, accel_limit: float) -> float:
        if not self.use_acceleration_limit:
            return target
        max_delta = max(0.0, float(accel_limit)) * max(0.0, float(dt_s))
        return current + clamp(target - current, -max_delta, max_delta)

    def step(
        self,
        state: SimulationState,
        pwm_left: int,
        pwm_right: int,
        dt_s: float,
        robot: RobotSpec,
        config: SimulationConfig,
        native=None,
    ) -> SimulationState:
        max_speed = auto_max_wheel_speed_mm_s(config.basic_max_wheel_speed_mm_s, self.params)
        accel_limit = auto_wheel_accel_mm_s2(config.basic_max_wheel_accel_mm_s2, self.params)
        target_left = pwm_to_normalized(pwm_left, robot) * max_speed
        target_right = pwm_to_normalized(pwm_right, robot) * max_speed

        v_left = self._advance_wheel_speed(self._v_left_mm_s, target_left, dt_s, accel_limit)
        v_right = self._advance_wheel_speed(self._v_right_mm_s, target_right, dt_s, accel_limit)
        self._v_left_mm_s = v_left
        self._v_right_mm_s = v_right

        x, y, heading, v, omega = integrate_differential_pose(
            state.x_mm,
            state.y_mm,
            state.heading_deg,
            v_left,
            v_right,
            wheel_track_mm(robot),
            dt_s,
        )
        dt = max(1e-9, float(dt_s))
        return SimulationState(
            t_ms=state.t_ms,
            x_mm=x,
            y_mm=y,
            heading_deg=heading,
            v_mm_s=v,
            omega_rad_s=omega,
            a_lin_mm_s2=(v - state.v_mm_s) / dt,
            alpha_rad_s2=(omega - state.omega_rad_s) / dt,
            v_left_mm_s=v_left,
            v_right_mm_s=v_right,
            pwm_left=int(pwm_left),
            pwm_right=int(pwm_right),
            sensors=list(state.sensors),
        )
