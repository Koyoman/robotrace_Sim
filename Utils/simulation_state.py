from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


@dataclass(slots=True)
class SimulationState:
    t_ms: int
    x_mm: float
    y_mm: float
    heading_deg: float
    v_mm_s: float
    omega_rad_s: float
    a_lin_mm_s2: float
    alpha_rad_s2: float
    v_left_mm_s: float
    v_right_mm_s: float
    pwm_left: int = 0
    pwm_right: int = 0
    sensors: list[int] = field(default_factory=list)

    def to_controller_state(self, dt_s: float) -> dict[str, Any]:
        return {
            "t_ms": self.t_ms,
            "dt_s": dt_s,
            "x_mm": self.x_mm,
            "y_mm": self.y_mm,
            "heading_deg": self.heading_deg,
            "v_mm_s": self.v_mm_s,
            "omega_rad_s": self.omega_rad_s,
            "a_lin_mm_s2": self.a_lin_mm_s2,
            "alpha_rad_s2": self.alpha_rad_s2,
            "sensors": list(self.sensors),
            "v_left_mm_s": self.v_left_mm_s,
            "v_right_mm_s": self.v_right_mm_s,
        }

    def to_step_dict(self) -> dict[str, Any]:
        d = self.to_controller_state(dt_s=0.0)
        d.pop("dt_s", None)
        return d


@dataclass(slots=True)
class SimulationResult:
    steps: list[SimulationState]
    summary: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {"steps": [asdict(s) for s in self.steps], "summary": dict(self.summary)}
