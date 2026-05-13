from __future__ import annotations

from Utils.robot_spec import RobotSpec
from Utils.simulation_config import SimulationConfig
from Utils.simulation_state import SimulationState

from sim.physics.base import PhysicsModel, auto_max_wheel_speed_mm_s, integrate_differential_pose, pwm_to_normalized, wheel_track_mm


class IdealPhysicsModel(PhysicsModel):
    """Instantaneous PWM-to-wheel-speed differential-drive model."""

    requires_native = False

    def __init__(self, params: dict | None = None):
        self.params = dict(params or {})

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
        params = getattr(self, "params", {})
        max_speed = auto_max_wheel_speed_mm_s(config.ideal_max_wheel_speed_mm_s, params)
        v_left = pwm_to_normalized(pwm_left, robot) * max_speed
        v_right = pwm_to_normalized(pwm_right, robot) * max_speed

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
