from pathlib import Path

from sim.controller_loader import load_controller

ROOT = Path(__file__).resolve().parents[1]


def test_controller_loader_loads_p_basic():
    fn = load_controller(str(ROOT / "Example/Controller/P_basic.py"))
    assert callable(fn)
    out = fn({"sensors": [0, 1, 2], "dt_s": 0.001})
    assert "pwm_left" in out and "pwm_right" in out
