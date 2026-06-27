from __future__ import annotations

from typing import Any

from Utils.robot_spec import RobotSpec
from Utils.simulation_config import SimulationConfig, VALID_PHYSICS_PROFILES
from Utils.validation import ValidationError

from sim.physics.base import PhysicsModel
from sim.physics.custom import CustomPhysicsModel, RealisticPhysicsModel
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
        return RealisticPhysicsModel(config=config, robot=robot, params=params)

    return CustomPhysicsModel(config=config, robot=robot, params=params)
