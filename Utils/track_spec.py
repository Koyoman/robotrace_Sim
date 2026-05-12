from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any

from Utils.track_geometry import Pt, Pose, SegArc, SegStraight, AnySeg, advance_arc, advance_straight
from Utils.track_model import StartFinishCfg, TrackModel
from Utils.validation import ValidationError, as_bool, as_float, raise_if_errors


@dataclass(slots=True)
class StartFinishSpec:
    enabled: bool = False
    on_segment_id: str | None = None
    s_param_mm: float = 0.0
    invert: bool = False
    start_is_forward: bool = True


@dataclass(slots=True)
class TrackSpec:
    area_width_mm: float = 3000.0
    area_height_mm: float = 2000.0
    tape_width_mm: float = 20.0
    origin: Pose = field(default_factory=lambda: Pose(Pt(500.0, 500.0), 0.0))
    segments: list[AnySeg] = field(default_factory=list)
    start_finish: StartFinishSpec = field(default_factory=StartFinishSpec)

    @classmethod
    def from_dict(cls, obj: dict[str, Any]) -> "TrackSpec":
        if not isinstance(obj, dict):
            raise ValidationError("Track JSON inválido.", ["Raiz do arquivo deve ser um objeto JSON."])
        errors: list[str] = []
        area = obj.get("area") or {}
        area_w = as_float(area.get("widthMM", 3000.0), "$.area.widthMM", 3000.0, errors)
        area_h = as_float(area.get("heightMM", 2000.0), "$.area.heightMM", 2000.0, errors)
        tape = as_float(obj.get("tapeWidthMM", obj.get("lineWidthMM", 20.0)), "$.tapeWidthMM", 20.0, errors)

        origin_obj = obj.get("origin") or {"p": {"x": area_w / 2.0, "y": area_h / 2.0}, "headingDeg": 0.0}
        p_obj = origin_obj.get("p") if isinstance(origin_obj, dict) else None
        if not isinstance(p_obj, dict):
            errors.append("$.origin.p deve existir com campos x e y.")
            p_obj = {"x": area_w / 2.0, "y": area_h / 2.0}
        origin = Pose(
            Pt(as_float(p_obj.get("x", area_w / 2.0), "$.origin.p.x", area_w / 2.0, errors),
               as_float(p_obj.get("y", area_h / 2.0), "$.origin.p.y", area_h / 2.0, errors)),
            as_float(origin_obj.get("headingDeg", 0.0) if isinstance(origin_obj, dict) else 0.0, "$.origin.headingDeg", 0.0, errors),
        )

        segs_raw = obj.get("segments") or []
        if not isinstance(segs_raw, list) or not segs_raw:
            errors.append("$.segments deve conter pelo menos um segmento para simulação.")
            segs_raw = []
        segments: list[AnySeg] = []
        ids: set[str] = set()
        cur = origin
        for i, s in enumerate(segs_raw):
            if not isinstance(s, dict):
                errors.append(f"$.segments[{i}] deve ser objeto.")
                continue
            kind = str(s.get("kind", "")).lower()
            sid = str(s.get("id", f"seg{i+1}"))
            if sid in ids:
                errors.append(f"ID de segmento duplicado: {sid!r}.")
            ids.add(sid)
            if kind == "straight":
                length = as_float(s.get("lengthMM", 0.0), f"$.segments[{i}].lengthMM", 0.0, errors)
                if length <= 0:
                    errors.append(f"$.segments[{i}].lengthMM deve ser > 0.")
                seg = SegStraight(kind="straight", id=sid, from_pose=cur, lengthMM=length)
                cur = advance_straight(cur, length)
                segments.append(seg)
            elif kind == "arc":
                radius = as_float(s.get("radiusMM", 0.0), f"$.segments[{i}].radiusMM", 0.0, errors)
                sweep = as_float(s.get("sweepDeg", 0.0), f"$.segments[{i}].sweepDeg", 0.0, errors)
                if radius <= 0:
                    errors.append(f"$.segments[{i}].radiusMM deve ser > 0.")
                if sweep == 0:
                    errors.append(f"$.segments[{i}].sweepDeg deve ser diferente de 0.")
                seg = SegArc(kind="arc", id=sid, from_pose=cur, radiusMM=radius, sweepDeg=sweep)
                cur = advance_arc(cur, radius, sweep)
                segments.append(seg)
            else:
                errors.append(f"$.segments[{i}].kind deve ser 'straight' ou 'arc'.")

        sf_raw = obj.get("startFinish") or {}
        sf = StartFinishSpec()
        if isinstance(sf_raw, dict) and sf_raw.get("enabled", False):
            on_id = sf_raw.get("onSegmentId")
            sf.enabled = as_bool(sf_raw.get("enabled", False), "$.startFinish.enabled", False, errors)
            sf.on_segment_id = str(on_id) if on_id is not None else None
            sf.s_param_mm = as_float(sf_raw.get("sParamMM", 0.0), "$.startFinish.sParamMM", 0.0, errors)
            sf.invert = as_bool(sf_raw.get("invert", False), "$.startFinish.invert", False, errors)
            if "startIsForward" in sf_raw:
                sf.start_is_forward = as_bool(sf_raw.get("startIsForward"), "$.startFinish.startIsForward", True, errors)
            else:
                sf.start_is_forward = not sf.invert
        elif sf_raw in ({}, None):
            sf = StartFinishSpec()
        else:
            errors.append("$.startFinish deve ser objeto ou null.")

        spec = cls(area_w, area_h, tape, origin, segments, sf)
        errors.extend(spec.validate())
        raise_if_errors("Track JSON inválido.", errors)
        return spec

    @classmethod
    def from_json_file(cls, path: str) -> "TrackSpec":
        try:
            with open(path, "r", encoding="utf-8") as f:
                return cls.from_dict(json.load(f))
        except ValidationError:
            raise
        except json.JSONDecodeError as e:
            raise ValidationError("Track JSON inválido.", [f"Erro de sintaxe em {path}: {e}"])
        except OSError as e:
            raise ValidationError("Não foi possível abrir o arquivo de pista.", [str(e)])

    @classmethod
    def from_track_model(cls, model: TrackModel) -> "TrackSpec":
        return cls.from_dict(model.to_json())

    def to_track_model(self) -> TrackModel:
        sf = StartFinishCfg(
            enabled=self.start_finish.enabled,
            onSegmentId=self.start_finish.on_segment_id,
            sParamMM=self.start_finish.s_param_mm,
            invert=self.start_finish.invert,
            startIsForward=self.start_finish.start_is_forward,
        )
        return TrackModel(
            area_widthMM=self.area_width_mm,
            area_heightMM=self.area_height_mm,
            tapeWidthMM=self.tape_width_mm,
            gridStepMM=100.0,
            origin=self.origin,
            segments=list(self.segments),
            startFinish=sf,
        )

    def validate(self) -> list[str]:
        errors: list[str] = []
        if self.area_width_mm <= 0:
            errors.append("area.widthMM deve ser > 0.")
        if self.area_height_mm <= 0:
            errors.append("area.heightMM deve ser > 0.")
        if self.tape_width_mm <= 0:
            errors.append("tapeWidthMM deve ser > 0.")
        if not self.segments:
            errors.append("segments não pode ser vazio para simulação.")
        ids = {s.id for s in self.segments}
        if len(ids) != len(self.segments):
            errors.append("IDs de segmentos devem ser únicos.")
        for i, seg in enumerate(self.segments):
            if isinstance(seg, SegStraight) and seg.lengthMM <= 0:
                errors.append(f"segments[{i}].lengthMM deve ser > 0.")
            if isinstance(seg, SegArc):
                if seg.radiusMM <= 0:
                    errors.append(f"segments[{i}].radiusMM deve ser > 0.")
                if seg.sweepDeg == 0:
                    errors.append(f"segments[{i}].sweepDeg deve ser diferente de 0.")
        sf = self.start_finish
        if sf.enabled:
            if not sf.on_segment_id:
                errors.append("startFinish.onSegmentId é obrigatório quando startFinish.enabled=true.")
            elif sf.on_segment_id not in ids:
                errors.append(f"startFinish.onSegmentId aponta para segmento inexistente: {sf.on_segment_id!r}.")
            else:
                seg = next(s for s in self.segments if s.id == sf.on_segment_id)
                if not isinstance(seg, SegStraight):
                    errors.append("startFinish.onSegmentId deve apontar para segmento straight.")
        return errors

    def to_dict(self) -> dict[str, Any]:
        segs: list[dict[str, Any]] = []
        for s in self.segments:
            if isinstance(s, SegStraight):
                segs.append({"kind": "straight", "id": s.id, "lengthMM": s.lengthMM})
            else:
                segs.append({"kind": "arc", "id": s.id, "radiusMM": s.radiusMM, "sweepDeg": s.sweepDeg})
        sf = None
        if self.start_finish.enabled and self.start_finish.on_segment_id:
            sf = {
                "enabled": True,
                "onSegmentId": self.start_finish.on_segment_id,
                "sParamMM": self.start_finish.s_param_mm,
                "invert": self.start_finish.invert,
                "startIsForward": self.start_finish.start_is_forward,
            }
        return {
            "area": {"widthMM": self.area_width_mm, "heightMM": self.area_height_mm},
            "origin": {"p": {"x": self.origin.p.x, "y": self.origin.p.y}, "headingDeg": self.origin.headingDeg},
            "tapeWidthMM": self.tape_width_mm,
            "segments": segs,
            "startFinish": sf,
        }
