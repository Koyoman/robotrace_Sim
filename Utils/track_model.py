from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any
from Utils.track_geometry import Pt, Pose, SegStraight, SegArc, AnySeg, advance_straight, advance_arc

@dataclass
class StartFinishCfg:
    enabled: bool = False
    onSegmentId: Optional[str] = None
    sParamMM: float = 0.0
    # Compat: 'invert' mantido para arquivos antigos, mas o campo canônico é startIsForward
    invert: bool = False
    startIsForward: bool = True  # True: Partida no gate +s; False: Partida no gate -gap

@dataclass
class TrackModel:
    area_widthMM: float = 3000.0
    area_heightMM: float = 2000.0
    tapeWidthMM: float = 20.0
    gridStepMM: float = 100.0
    origin: Pose = field(default_factory=lambda: Pose(Pt(500.0, 500.0), 0.0))
    segments: List[AnySeg] = field(default_factory=list)
    startFinish: StartFinishCfg = field(default_factory=StartFinishCfg)

    def to_json(self) -> Dict[str, Any]:
        segs: List[Dict[str, Any]] = []
        for s in self.segments:
            if isinstance(s, SegStraight):
                segs.append({"kind": "straight", "id": s.id, "lengthMM": s.lengthMM})
            else:
                segs.append({"kind": "arc", "id": s.id, "radiusMM": s.radiusMM, "sweepDeg": s.sweepDeg})
        sf = None
        if self.startFinish.enabled and self.startFinish.onSegmentId:
            # mantém 'invert' para compat e grava o canônico 'startIsForward'
            invert = not self.startFinish.startIsForward
            sf = {
                "enabled": True,
                "onSegmentId": self.startFinish.onSegmentId,
                "sParamMM": self.startFinish.sParamMM,
                "invert": invert,
                "startIsForward": self.startFinish.startIsForward,
            }
        return {
            "area": {"widthMM": self.area_widthMM, "heightMM": self.area_heightMM},
            "origin": {"p": {"x": self.origin.p.x, "y": self.origin.p.y}, "headingDeg": self.origin.headingDeg},
            "tapeWidthMM": self.tapeWidthMM,
            "segments": segs,
            "startFinish": sf
        }

    @staticmethod
    def from_json(obj: Dict[str, Any]) -> "TrackModel":
        area = obj.get("area") or {}
        width = float(area.get("widthMM", 3000.0))
        height = float(area.get("heightMM", 2000.0))
        tape = float(obj.get("tapeWidthMM", obj.get("lineWidthMM", 20.0)))
        origin_obj = obj.get("origin") or {"p": {"x": width/2, "y": height/2}, "headingDeg": 0.0}
        origin = Pose(Pt(float(origin_obj["p"]["x"]), float(origin_obj["p"]["y"])),
                      float(origin_obj.get("headingDeg", 0.0)))

        segs_raw = obj.get("segments", [])
        segs: List[AnySeg] = []
        cur = origin
        for s in segs_raw:
            kind = s.get("kind")
            sid = s.get("id") or f"seg{len(segs)+1}"
            if kind == "straight":
                length = float(s.get("lengthMM", 0.0))
                seg = SegStraight(kind="straight", id=sid, from_pose=cur, lengthMM=length)
                cur = advance_straight(seg.from_pose, seg.lengthMM)
                segs.append(seg)
            elif kind == "arc":
                radius = float(s.get("radiusMM", 0.0))
                sweep = float(s.get("sweepDeg", 0.0))
                seg = SegArc(kind="arc", id=sid, from_pose=cur, radiusMM=radius, sweepDeg=sweep)
                cur = advance_arc(seg.from_pose, seg.radiusMM, seg.sweepDeg)
                segs.append(seg)

        sf_obj = obj.get("startFinish") or None
        # defaults
        sf = StartFinishCfg(False, None, 0.0, False, True)
        if sf_obj and sf_obj.get("enabled") and sf_obj.get("onSegmentId"):
            sf.enabled = True
            sf.onSegmentId = str(sf_obj.get("onSegmentId"))
            sf.sParamMM = float(sf_obj.get("sParamMM", 0.0))
            # leitura canônica
            if "startIsForward" in sf_obj:
                sf.startIsForward = bool(sf_obj["startIsForward"])
                sf.invert = not sf.startIsForward
            else:
                # retrocompat: se só tiver 'invert', converte
                inv = bool(sf_obj.get("invert", False))
                sf.invert = inv
                sf.startIsForward = not inv

        return TrackModel(width, height, tape, 100.0, origin, segs, sf)
