from __future__ import annotations

import hashlib
import json
import math
import os
import zlib
from typing import Any

from Utils.track_geometry import (
    MARKER_LENGTH_MM,
    MARKER_OFFSET_MM,
    MARKER_THICKNESS_MM,
    START_FINISH_GAP_MM,
    STRAIGHT_NEAR_XING_MM,
    Pt,
    Pose,
    SegArc,
    SegStraight,
    advance_arc,
    advance_straight,
    rad,
)
from Utils.track_spec import TrackSpec

RMAP_FORMAT = "RobotraceSim.rmap"
RMAP_VERSION = 2
DEFAULT_RASTER_PARAMS: dict[str, float] = {
    "polyline_step_mm": 1.0,
    "margin_mm": 80.0,
    "pixel_mm": 1.0,
    "marker_offset_mm": float(MARKER_OFFSET_MM),
    "marker_length_mm": float(MARKER_LENGTH_MM),
    "marker_thickness_mm": float(MARKER_THICKNESS_MM),
    "start_finish_gap_mm": float(START_FINISH_GAP_MM),
    "straight_near_xing_mm": float(STRAIGHT_NEAR_XING_MM),
}


def _canonical_track_payload(track: TrackSpec) -> dict[str, Any]:
    """Return only the track content that changes generated raster output."""
    return track.to_dict()


def track_cache_fingerprint(track: TrackSpec, raster_params: dict[str, Any] | None = None) -> str:
    """Hash relevant track content plus rasterization parameters for .rmap cache validation."""
    payload = {
        "track": _canonical_track_payload(track),
        "raster_params": dict(raster_params or DEFAULT_RASTER_PARAMS),
        "rmap_format": RMAP_FORMAT,
        "rmap_version": RMAP_VERSION,
    }
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def is_rmap_cache_valid(meta: dict[str, Any], expected_fingerprint: str, expected_data_len: int | None = None) -> bool:
    """Validate .rmap metadata against the expected fingerprint and basic dimensions."""
    if not isinstance(meta, dict):
        return False
    if meta.get("format") != RMAP_FORMAT:
        return False
    try:
        version = int(meta.get("version", -1))
    except (TypeError, ValueError):
        return False
    if version != RMAP_VERSION:
        return False
    if meta.get("fingerprint") != expected_fingerprint:
        return False
    try:
        width = int(meta["W"])
        height = int(meta["H"])
        pixel = float(meta["pixel_mm"])
        float(meta["origin_x"])
        float(meta["origin_y"])
    except (KeyError, TypeError, ValueError):
        return False
    if width <= 0 or height <= 0 or pixel <= 0:
        return False
    if expected_data_len is not None and expected_data_len != width * height:
        return False
    return True


def rot(x: float, y: float, a: float) -> tuple[float, float]:
    c, s = math.cos(a), math.sin(a)
    return (c * x - s * y, s * x + c * y)


def segments_from_track(track: TrackSpec) -> tuple[list[object], Pose, float]:
    return list(track.segments), track.origin, float(track.tape_width_mm)


def segments_polyline(segs: list[object], step: float = 1.0) -> list[Pt]:
    pts: list[Pt] = []
    for s in segs:
        if isinstance(s, SegStraight):
            n = max(2, int(math.ceil(s.lengthMM / step)))
            for i in range(n):
                t = i / (n - 1)
                p = advance_straight(s.from_pose, s.lengthMM * t).p
                pts.append(Pt(p.x, p.y))
        else:
            L = abs(rad(s.sweepDeg)) * s.radiusMM
            n = max(6, int(math.ceil(L / step)))
            for i in range(n):
                t = i / (n - 1)
                p = advance_arc(s.from_pose, s.radiusMM, s.sweepDeg * t).p
                pts.append(Pt(p.x, p.y))
    return pts


def curvature_change_markers(segs: list[object]) -> list[tuple[Pt, float]]:
    def kappa(s: object) -> float:
        if isinstance(s, SegArc):
            return (1.0 if s.sweepDeg >= 0.0 else -1.0) / max(1e-9, s.radiusMM)
        return 0.0

    def end_pose(s: object) -> Pose:
        return advance_straight(s.from_pose, s.lengthMM) if isinstance(s, SegStraight) else advance_arc(s.from_pose, s.radiusMM, s.sweepDeg)

    out: list[tuple[Pt, float]] = []
    if not segs:
        return out
    for i in range(len(segs) - 1):
        a, b = segs[i], segs[i + 1]
        if abs(kappa(a) - kappa(b)) > 1e-6:
            ep = end_pose(a)
            out.append((ep.p, ep.headingDeg))
    first, last = segs[0], segs[-1]
    ep_last = end_pose(last)
    is_closed = math.hypot(ep_last.p.x - first.from_pose.p.x, ep_last.p.y - first.from_pose.p.y) <= 1.0
    if is_closed and abs(kappa(last) - kappa(first)) > 1e-6:
        out.append((ep_last.p, ep_last.headingDeg))
    return out


def start_finish_lines(track: TrackSpec):
    sf = track.start_finish
    if not sf.enabled:
        return None

    seg_id = sf.on_segment_id
    start_is_fwd = bool(sf.start_is_forward)
    invert = bool(sf.invert)
    s_param = float(sf.s_param_mm)

    straight = next((s for s in track.segments if isinstance(s, SegStraight) and s.id == seg_id), None)
    if not straight:
        return None

    def clamp_start_on_seg(seg: SegStraight, t: float) -> float:
        mn = START_FINISH_GAP_MM + STRAIGHT_NEAR_XING_MM
        mx = seg.lengthMM - STRAIGHT_NEAR_XING_MM
        return max(mn, min(mx, t))

    s_param = clamp_start_on_seg(straight, s_param)

    def gate_at(d: float) -> tuple[Pt, Pt, float, float]:
        pose = advance_straight(straight.from_pose, max(0.0, min(straight.lengthMM, d)))
        base_hdg = pose.headingDeg
        run_hdg = (base_hdg + 180.0) if invert else base_hdg
        a = math.radians(base_hdg)
        nx, ny = -math.sin(a), math.cos(a)
        half = (track.tape_width_mm * 1.2) * 0.5
        ax, ay = pose.p.x + nx * half, pose.p.y + ny * half
        bx, by = pose.p.x - nx * half, pose.p.y - ny * half
        return (Pt(ax, ay), Pt(bx, by), run_hdg, base_hdg)

    start_pose = gate_at(s_param)
    finish_pose = gate_at(max(0.0, s_param - START_FINISH_GAP_MM))
    return (start_pose, finish_pose) if start_is_fwd else (finish_pose, start_pose)


def build_markers(segs: list[object], tape_w: float, gates) -> list[tuple[float, float, float, float, float, float, float, float]]:
    rects: list[tuple[float, float, float, float, float, float, float, float]] = []
    half_l = MARKER_LENGTH_MM * 0.5
    half_w = MARKER_THICKNESS_MM * 0.5

    for (pp, hdg) in curvature_change_markers(segs):
        a = math.radians(hdg)
        tx, ty = math.cos(a), math.sin(a)
        nx, ny = math.sin(a), -math.cos(a)
        base = (tape_w * 0.5) + MARKER_OFFSET_MM
        cx, cy = pp.x + nx * base, pp.y + ny * base
        rects.append((cx, cy, nx, ny, tx, ty, half_l, half_w))

    if gates:
        (sa, sb, _shdg_run, shdg_base), (fa, fb, _fhdg_run, fhdg_base) = gates

        def add_right_rect(pa: Pt, pb: Pt, base_hdg: float) -> None:
            a = math.radians(base_hdg)
            tx, ty = math.cos(a), math.sin(a)
            nx, ny = -math.sin(a), math.cos(a)
            mx, my = (pa.x + pb.x) * 0.5, (pa.y + pb.y) * 0.5
            base = (tape_w * 0.5) + MARKER_OFFSET_MM
            cx, cy = mx + nx * base, my + ny * base
            rects.append((cx, cy, nx, ny, tx, ty, half_l, half_w))

        add_right_rect(sa, sb, shdg_base)
        add_right_rect(fa, fb, fhdg_base)

    return rects


def oriented_rect(cx, cy, ux, uy, half_l, vx, vy, half_w):
    return [
        (cx - ux * half_l - vx * half_w, cy - uy * half_l - vy * half_w),
        (cx + ux * half_l - vx * half_w, cy + uy * half_l - vy * half_w),
        (cx + ux * half_l + vx * half_w, cy + uy * half_l + vy * half_w),
        (cx - ux * half_l + vx * half_w, cy - uy * half_l + vy * half_w),
    ]


def point_in_obb(px: float, py: float, cx: float, cy: float, ux: float, uy: float, half_l: float, vx: float, vy: float, half_w: float) -> bool:
    dx = px - cx
    dy = py - cy
    t = dx * ux + dy * uy
    w = dx * vx + dy * vy
    return (abs(t) <= half_l) and (abs(w) <= half_w)


def _raster_paths_for_track(track_path: str) -> str:
    base, _ = os.path.splitext(track_path)
    return base + ".rmap"


def _rmap_save(path: str, meta: dict, mask_bytes: bytes) -> None:
    blob = zlib.compress(mask_bytes, level=6)
    with open(path, "wb") as f:
        header = json.dumps(meta, separators=(",", ":")).encode("utf-8") + b"\n"
        f.write(header)
        f.write(blob)


def _rmap_load(path: str) -> tuple[dict, bytes] | None:
    try:
        with open(path, "rb") as f:
            header = f.readline()
            meta = json.loads(header.decode("utf-8"))
            blob = f.read()
            data = zlib.decompress(blob)
            return meta, data
    except Exception:
        return None


def ensure_track_raster(track_path: str, track: TrackSpec, segs: list, tape_w: float, gates) -> dict[str, Any]:
    rpath = _raster_paths_for_track(track_path)
    raster_params = dict(DEFAULT_RASTER_PARAMS)
    expected_fingerprint = track_cache_fingerprint(track, raster_params)
    cached = _rmap_load(rpath)
    if cached is not None:
        meta, data = cached
        if is_rmap_cache_valid(meta, expected_fingerprint, expected_data_len=len(data)):
            return {"path": rpath, "meta": meta, "data": data}

    pts = segments_polyline(segs, step=float(raster_params["polyline_step_mm"]))
    markers = build_markers(segs, tape_w, gates)

    xs = [p.x for p in pts] + [m[0] for m in markers]
    ys = [p.y for p in pts] + [m[1] for m in markers]
    if not xs or not ys:
        raise RuntimeError("Invalid track geometry for rasterization.")
    margin_mm = float(raster_params["margin_mm"])
    minx = math.floor(min(xs) - margin_mm)
    miny = math.floor(min(ys) - margin_mm)
    maxx = math.ceil(max(xs) + margin_mm)
    maxy = math.ceil(max(ys) + margin_mm)

    W = int(maxx - minx)
    H = int(maxy - miny)
    origin_x = float(minx)
    origin_y = float(miny)
    pixel_mm = float(raster_params["pixel_mm"])

    buf = bytearray(W * H)
    half = tape_w * 0.5
    if len(pts) >= 2:
        for i in range(len(pts) - 1):
            x1, y1 = pts[i].x, pts[i].y
            x2, y2 = pts[i + 1].x, pts[i + 1].y
            mnx = math.floor(min(x1, x2) - half); mxx = math.ceil(max(x1, x2) + half)
            mny = math.floor(min(y1, y2) - half); mxy = math.ceil(max(y1, y2) + half)
            ix0 = max(0, int(mnx - origin_x)); iy0 = max(0, int(mny - origin_y))
            ix1 = min(W - 1, int(mxx - origin_x)); iy1 = min(H - 1, int(mxy - origin_y))
            dx = x2 - x1; dy = y2 - y1
            seg_l2 = dx * dx + dy * dy or 1e-9
            for py in range(iy0, iy1 + 1):
                wy = origin_y + (py + 0.5) * pixel_mm
                for px in range(ix0, ix1 + 1):
                    wx = origin_x + (px + 0.5) * pixel_mm
                    t = ((wx - x1) * dx + (wy - y1) * dy) / seg_l2
                    if t < 0.0:
                        qx, qy = x1, y1
                    elif t > 1.0:
                        qx, qy = x2, y2
                    else:
                        qx, qy = x1 + t * dx, y1 + t * dy
                    ddx = wx - qx; ddy = wy - qy
                    if (ddx * ddx + ddy * ddy) <= (half * half):
                        buf[py * W + px] = 255

    for (cx, cy, ux, uy, vx, vy, half_l, half_w) in markers:
        corners = [
            (cx - ux * half_l - vx * half_w, cy - uy * half_l - vy * half_w),
            (cx + ux * half_l - vx * half_w, cy + uy * half_l - vy * half_w),
            (cx + ux * half_l + vx * half_w, cy + uy * half_l + vy * half_w),
            (cx - ux * half_l + vx * half_w, cy - uy * half_l + vy * half_w),
        ]
        mnx = math.floor(min(p[0] for p in corners)); mxx = math.ceil(max(p[0] for p in corners))
        mny = math.floor(min(p[1] for p in corners)); mxy = math.ceil(max(p[1] for p in corners))
        ix0 = max(0, int(mnx - origin_x)); iy0 = max(0, int(mny - origin_y))
        ix1 = min(W - 1, int(mxx - origin_x)); iy1 = min(H - 1, int(mxy - origin_y))
        for py in range(iy0, iy1 + 1):
            wy = origin_y + (py + 0.5) * pixel_mm
            for px in range(ix0, ix1 + 1):
                wx = origin_x + (px + 0.5) * pixel_mm
                dx = wx - cx; dy = wy - cy
                t = dx * ux + dy * uy
                w = dx * vx + dy * vy
                if (abs(t) <= half_l) and (abs(w) <= half_w):
                    buf[py * W + px] = 255

    meta = {
        "format": RMAP_FORMAT,
        "version": RMAP_VERSION,
        "fingerprint": expected_fingerprint,
        "raster_params": raster_params,
        "origin_x": origin_x,
        "origin_y": origin_y,
        "W": W,
        "H": H,
        "pixel_mm": pixel_mm,
    }
    _rmap_save(rpath, meta, bytes(buf))
    return {"path": rpath, "meta": meta, "data": bytes(buf)}
