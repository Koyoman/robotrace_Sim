import json
from pathlib import Path

from Utils.track_spec import TrackSpec
from sim.track_runtime import RMAP_FORMAT, RMAP_VERSION, is_rmap_cache_valid, track_cache_fingerprint

ROOT = Path(__file__).resolve().parents[1]


def _track():
    return TrackSpec.from_json_file(str(ROOT / "Example/Track/track_1_cw.json"))


def _valid_meta(fp):
    return {
        "format": RMAP_FORMAT,
        "version": RMAP_VERSION,
        "fingerprint": fp,
        "origin_x": 0.0,
        "origin_y": 0.0,
        "W": 2,
        "H": 3,
        "pixel_mm": 1.0,
    }


def test_track_fingerprint_changes_when_tape_width_changes():
    track = _track()
    data = track.to_dict()
    data["tapeWidthMM"] = data["tapeWidthMM"] + 1.0
    changed = TrackSpec.from_dict(data)
    assert track_cache_fingerprint(track) != track_cache_fingerprint(changed)


def test_track_fingerprint_changes_when_segments_change():
    track = _track()
    data = track.to_dict()
    data["segments"][0]["lengthMM"] = data["segments"][0]["lengthMM"] + 1.0
    changed = TrackSpec.from_dict(data)
    assert track_cache_fingerprint(track) != track_cache_fingerprint(changed)


def test_cache_without_hash_is_invalid():
    track = _track()
    fp = track_cache_fingerprint(track)
    meta = {"origin_x": 0.0, "origin_y": 0.0, "W": 2, "H": 3, "pixel_mm": 1.0}
    assert not is_rmap_cache_valid(meta, fp, expected_data_len=6)


def test_cache_with_different_hash_is_invalid():
    track = _track()
    fp = track_cache_fingerprint(track)
    meta = _valid_meta("not-" + fp)
    assert not is_rmap_cache_valid(meta, fp, expected_data_len=6)


def test_cache_with_matching_hash_is_valid():
    track = _track()
    fp = track_cache_fingerprint(track)
    meta = _valid_meta(fp)
    assert is_rmap_cache_valid(meta, fp, expected_data_len=6)
