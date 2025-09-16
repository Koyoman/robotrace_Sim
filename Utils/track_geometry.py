from __future__ import annotations
from dataclasses import dataclass
from math import sin, cos, atan2, radians, degrees, pi, hypot
from typing import List, Tuple, Union

def rad(d: float) -> float:
    return radians(d)

def deg(r: float) -> float:
    return degrees(r)

@dataclass
class Pt:
    x: float
    y: float

@dataclass
class Pose:
    p: Pt
    headingDeg: float = 0.0

@dataclass
class SegStraight:
    kind: str
    id: str
    from_pose: Pose
    lengthMM: float

@dataclass
class SegArc:
    kind: str
    id: str
    from_pose: Pose
    radiusMM: float
    sweepDeg: float

AnySeg = Union[SegStraight, SegArc]

# Constantes (iguais ao editor TS)
START_FINISH_GAP_MM = 1000.0
STRAIGHT_NEAR_XING_MM = 250.0
MARKER_OFFSET_MM = 40.0
MARKER_LENGTH_MM = 40.0
MARKER_THICKNESS_MM = 20.0

def advance_straight(from_pose: Pose, length: float) -> Pose:
    h = rad(from_pose.headingDeg)
    return Pose(Pt(from_pose.p.x + cos(h)*length, from_pose.p.y + sin(h)*length), from_pose.headingDeg)

def advance_arc(from_pose: Pose, radius: float, sweep_deg: float) -> Pose:
    left = 1 if sweep_deg >= 0 else -1
    h = rad(from_pose.headingDeg)
    cx = from_pose.p.x - left * sin(h) * radius
    cy = from_pose.p.y + left * cos(h) * radius
    a0 = atan2(from_pose.p.y - cy, from_pose.p.x - cx)
    a1 = a0 + rad(sweep_deg)
    ex = cx + cos(a1) * radius
    ey = cy + sin(a1) * radius
    return Pose(Pt(ex, ey), from_pose.headingDeg + sweep_deg)

def sample_straight(from_pose: Pose, L: float, stepMM: float=20.0) -> List[Pt]:
    n = max(2, int(abs(L)/stepMM + 0.5))
    pts: List[Pt] = []
    for i in range(n+1):
        s = L * (i/n)
        p = advance_straight(from_pose, s).p
        pts.append(p)
    return pts

def sample_arc(from_pose: Pose, R: float, sweepDeg: float, stepChordMM: float=10.0) -> List[Pt]:
    left = 1 if sweepDeg >= 0 else -1
    h = rad(from_pose.headingDeg)
    cx = from_pose.p.x - left * sin(h) * R
    cy = from_pose.p.y + left * cos(h) * R
    a0 = atan2(from_pose.p.y - cy, from_pose.p.x - cx)
    a1 = a0 + rad(sweepDeg)
    arcLen = abs(R * rad(sweepDeg))
    n = max(2, int(arcLen/stepChordMM + 0.5))
    pts: List[Pt] = []
    for i in range(n+1):
        a = a0 + (i/n)*(a1 - a0)
        pts.append(Pt(cx + R * cos(a), cy + R * sin(a)))
    return pts

def segments_to_polyline(segs: List[AnySeg]) -> List[Pt]:
    out: List[Pt] = []
    for s in segs:
        pts = sample_straight(s.from_pose, s.lengthMM, 20.0) if isinstance(s, SegStraight) \
              else sample_arc(s.from_pose, s.radiusMM, s.sweepDeg, 10.0)
        if out:
            out.extend(pts[1:])
        else:
            out.extend(pts)
    return out

def curvature_change_markers(segs: List[AnySeg]) -> List[Tuple[Pt, float]]:
    """Retorna lista de (ponto, headingDeg) nos pontos onde a curvatura muda."""
    def kappa(s: AnySeg) -> float:
        if isinstance(s, SegArc):
            return (1.0 if s.sweepDeg >= 0 else -1.0)/max(1e-9, s.radiusMM)
        return 0.0
    def end_pose(s: AnySeg) -> Pose:
        return advance_straight(s.from_pose, s.lengthMM) if isinstance(s, SegStraight) \
               else advance_arc(s.from_pose, s.radiusMM, s.sweepDeg)

    if not segs:
        return []
    out: List[Tuple[Pt, float]] = []
    for i in range(len(segs)-1):
        a, b = segs[i], segs[i+1]
        if abs(kappa(a) - kappa(b)) > 1e-6:
            ep = end_pose(a)
            out.append((ep.p, ep.headingDeg))

    # se for fechada e muda na emenda
    first, last = segs[0], segs[-1]
    ep_last = end_pose(last)
    closed = hypot(ep_last.p.x - first.from_pose.p.x, ep_last.p.y - first.from_pose.p.y) <= 1.0 and \
             abs(((ep_last.headingDeg - first.from_pose.headingDeg + 540) % 360) - 180) <= 2.0
    if closed and abs(kappa(last) - kappa(first)) > 1e-6:
        out.append((ep_last.p, ep_last.headingDeg))
    return out

def clamp_start_on_seg(seg: SegStraight, t: float) -> float:
    """Limita a posição da linha de largada para evitar muito próximo das emendas"""
    mn = START_FINISH_GAP_MM + STRAIGHT_NEAR_XING_MM
    mx = seg.lengthMM - STRAIGHT_NEAR_XING_MM
    return max(mn, min(mx, t))
