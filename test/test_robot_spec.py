import copy
import json
from pathlib import Path

import pytest

from Utils.robot_spec import RobotSpec
from Utils.validation import ValidationError

ROOT = Path(__file__).resolve().parents[1]


def test_robot_spec_loads_current_json():
    spec = RobotSpec.from_json_file(str(ROOT / "Example/Robot/robot-spec.json"))
    assert spec.version == "robot-v1"
    assert len(spec.sensors) == 8
    assert spec.controller.pwm_max == 4095


def test_robot_spec_rejects_robot_without_sensors():
    data = json.loads((ROOT / "Example/Robot/robot-spec.json").read_text(encoding="utf-8"))
    data["sensors"] = []
    with pytest.raises(ValidationError):
        RobotSpec.from_dict(data)
