from __future__ import annotations

from typing import Any

from Utils.robot_spec import RobotSpec
from Utils.simulation_config import SimulationConfig, VALID_PHYSICS_PROFILES
from Utils.validation import ValidationError

from sim.physics.base import PhysicsModel
from sim.physics.dc_motor import DCMotorPhysicsModel
from sim.physics.ideal import IdealPhysicsModel
from sim.physics.kinematic import BasicKinematicPhysicsModel


def create_physics_model(
    config: SimulationConfig,
    robot: RobotSpec | None = None,
    params: dict[str, Any] | None = None,
) -> PhysicsModel:
    profile = str(config.physics_profile).strip().lower()
    if profile not in VALID_PHYSICS_PROFILES:
        expected = ", ".join(sorted(VALID_PHYSICS_PROFILES))
        raise ValidationError("Configuração de simulação inválida.", [
            f"Invalid physics_profile {config.physics_profile!r}. Expected one of: {expected}."
        ])

    if profile == "ideal":
        return IdealPhysicsModel(params=params)
    if profile == "basic":
        return BasicKinematicPhysicsModel(use_acceleration_limit=True, params=params)
    if profile == "realistic":
        return DCMotorPhysicsModel(robot=robot, params=params)

    # custom: only expose flags that currently have a real effect.
    if config.custom_use_dc_motor_model:
        return DCMotorPhysicsModel(robot=robot, params=params)
    return BasicKinematicPhysicsModel(use_acceleration_limit=config.custom_use_acceleration_limit, params=params)
