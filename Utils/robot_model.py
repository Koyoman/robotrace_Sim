
from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional
from Utils.robot_geometry import (
    Envelope,
    DEFAULT_WHEEL_W, DEFAULT_WHEEL_H, DEFAULT_SENSOR_SIZE,
    clamp_inside_envelope, enforce_envelope_rules
)

@dataclass
class Wheel:
    id: str                  # "left" | "right"
    xMM: float
    yMM: float
    widthMM: float = DEFAULT_WHEEL_W
    heightMM: float = DEFAULT_WHEEL_H

@dataclass
class Sensor:
    id: str
    xMM: float
    yMM: float
    sizeMM: float = DEFAULT_SENSOR_SIZE

@dataclass
class RobotModel:
    envelope: Envelope = field(default_factory=lambda: Envelope(160.0, 140.0))
    originXMM: float = 0.0
    originYMM: float = 0.0
    wheels: List[Wheel] = field(default_factory=lambda: [
        Wheel(id="left",  xMM=-10.0, yMM=+35.0),
        Wheel(id="right", xMM=-10.0, yMM=-35.0),
    ])
    sensors: List[Sensor] = field(default_factory=lambda: [
        Sensor(id="S1", xMM=+60.0, yMM=-10.0),
        Sensor(id="S2", xMM=+60.0, yMM=+10.0),
    ])
    gridStepMM: float = 20.0

    def clamp_all_inside(self) -> None:
        halfW = self.envelope.widthMM / 2.0
        halfH = self.envelope.heightMM / 2.0
        for w in self.wheels:
            w.widthMM  = DEFAULT_WHEEL_W
            w.heightMM = DEFAULT_WHEEL_H
            w.xMM, w.yMM = clamp_inside_envelope(
                w.xMM, w.yMM, halfW, halfH, w.widthMM/2.0, w.heightMM/2.0
            )
        for s in self.sensors:
            s.sizeMM = DEFAULT_SENSOR_SIZE
            s.xMM, s.yMM = clamp_inside_envelope(
                s.xMM, s.yMM, halfW, halfH, s.sizeMM/2.0, s.sizeMM/2.0
            )
        self.originXMM, self.originYMM = clamp_inside_envelope(
            self.originXMM, self.originYMM, halfW, halfH, 0.0, 0.0
        )

    def set_envelope(self, widthMM: float, heightMM: float) -> None:
        w, h = enforce_envelope_rules(widthMM, heightMM)
        self.envelope.widthMM = w
        self.envelope.heightMM = h
        self.clamp_all_inside()

    def to_json(self) -> Dict[str, Any]:
        return {
            "version": "robot-v1",
            "envelope": {"widthMM": self.envelope.widthMM, "heightMM": self.envelope.heightMM},
            "origin": {"xMM": self.originXMM, "yMM": self.originYMM},
            "wheels": [
                {"id": w.id, "xMM": w.xMM, "yMM": w.yMM, "widthMM": w.widthMM, "heightMM": w.heightMM}
                for w in self.wheels
            ],
            "sensors": [
                {"id": s.id, "xMM": s.xMM, "yMM": s.yMM, "sizeMM": s.sizeMM}
                for s in self.sensors
            ]
        }

    @staticmethod
    def from_json(obj: Dict[str, Any]) -> "RobotModel":
        env = obj.get("envelope") or {}
        width  = float(env.get("widthMM", 160.0))
        height = float(env.get("heightMM", 140.0))
        width, height = enforce_envelope_rules(width, height)
        model = RobotModel(envelope=Envelope(width, height))

        origin = obj.get("origin") or {}
        model.originXMM = float(origin.get("xMM", 0.0))
        model.originYMM = float(origin.get("yMM", 0.0))

        wheels_raw = obj.get("wheels") or []
        if len(wheels_raw) != 2 or not all(w.get("id") in ("left", "right") for w in wheels_raw):
            raise ValueError("O JSON deve conter exatamente duas rodas com ids 'left' e 'right'.")
        ws: List[Wheel] = []
        for w in wheels_raw:
            ws.append(Wheel(
                id=str(w["id"]),
                xMM=float(w.get("xMM", 0.0)),
                yMM=float(w.get("yMM", 0.0)),
                widthMM=DEFAULT_WHEEL_W,
                heightMM=DEFAULT_WHEEL_H
            ))
        model.wheels = sorted(ws, key=lambda w: 0 if w.id == "left" else 1)

        sens_raw = obj.get("sensors") or []
        sensors: List[Sensor] = []
        for s in sens_raw:
            sensors.append(Sensor(
                id=str(s.get("id", f"S{len(sensors)+1}")),
                xMM=float(s.get("xMM", 0.0)),
                yMM=float(s.get("yMM", 0.0)),
                sizeMM=DEFAULT_SENSOR_SIZE
            ))
        model.sensors = sensors
        model.clamp_all_inside()
        return model

    def add_sensor(self, xMM: Optional[float] = None, yMM: Optional[float] = None) -> Sensor:
        i = 1
        existing = {s.id for s in self.sensors}
        while f"S{i}" in existing:
            i += 1
        sx = 0.0 if xMM is None else float(xMM)
        sy = 0.0 if yMM is None else float(yMM)
        s = Sensor(id=f"S{i}", xMM=sx, yMM=sy)
        self.sensors.append(s)
        self.clamp_all_inside()
        return s

    def remove_sensor(self, sid: str) -> None:
        self.sensors = [s for s in self.sensors if s.id != sid]

    def find_wheel(self, wid: str) -> Optional[Wheel]:
        for w in self.wheels:
            if w.id == wid:
                return w
        return None

    def find_sensor(self, sid: str) -> Optional[Sensor]:
        for s in self.sensors:
            if s.id == sid:
                return s
        return None
