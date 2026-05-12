import json
from pathlib import Path

import pytest

from Utils.track_spec import TrackSpec
from Utils.validation import ValidationError

ROOT = Path(__file__).resolve().parents[1]


def test_track_spec_loads_current_json():
    spec = TrackSpec.from_json_file(str(ROOT / "Example/Track/track_1_cw.json"))
    assert spec.area_width_mm == 3000.0
    assert len(spec.segments) == 9
    assert spec.start_finish.enabled is True


def test_track_spec_rejects_invalid_straight_length():
    data = json.loads((ROOT / "Example/Track/track_1_cw.json").read_text(encoding="utf-8"))
    data["segments"][0]["lengthMM"] = 0
    with pytest.raises(ValidationError):
        TrackSpec.from_dict(data)


def test_track_spec_rejects_missing_start_finish_segment():
    data = json.loads((ROOT / "Example/Track/track_1_cw.json").read_text(encoding="utf-8"))
    data["startFinish"]["onSegmentId"] = "MISSING"
    with pytest.raises(ValidationError):
        TrackSpec.from_dict(data)
