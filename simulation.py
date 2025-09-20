from __future__ import annotations
import os, sys, math, json, csv, ctypes, importlib.util, random, time
from dataclasses import dataclass
from typing import List, Tuple, Dict, Any, Optional
from datetime import datetime

from PySide6.QtCore import Qt, QPointF, QThread, Signal, QTimer
from PySide6.QtGui import QPen, QColor, QPainterPath, QPainter
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QFormLayout, QPushButton,
    QFileDialog, QDoubleSpinBox, QLabel, QSplitter, QGraphicsView,
    QGraphicsScene, QProgressBar, QMessageBox, QComboBox
)

# ---------- Native bindings (linesim) ----------
from ctypes import c_double, c_int, POINTER

class CPoint(ctypes.Structure):
    _fields_ = [("x", c_double), ("y", c_double)]

_here = os.path.dirname(__file__)
_dlldir = os.path.join(_here, "utills_c")
_dllpath = os.path.join(_dlldir, "linesim.dll")
if sys.platform.startswith("win") and hasattr(os, "add_dll_directory"):
    os.add_dll_directory(_dlldir)
_linesim = ctypes.CDLL(_dllpath)

_linesim.envelope_contacts_tape_C.argtypes = [
    c_double, c_double, c_double,
    c_double, c_double,
    POINTER(CPoint), c_int,
    c_double, c_int
]
_linesim.envelope_contacts_tape_C.restype = c_int

_linesim.estimate_sensor_coverage_C.argtypes = [
    c_double, c_double,
    POINTER(CPoint), c_int,
    c_double, c_double, c_int
]
_linesim.estimate_sensor_coverage_C.restype = c_double

# Batch sensor coverage
_linesim.estimate_sensors_coverage_batch_C = getattr(_linesim, "estimate_sensors_coverage_batch_C")
_linesim.estimate_sensors_coverage_batch_C.argtypes = [
    POINTER(c_double), POINTER(c_double), c_int,
    POINTER(CPoint), c_int,
    c_double,
    POINTER(c_double), c_double,
    c_int,
    POINTER(c_double)
]
_linesim.estimate_sensors_coverage_batch_C.restype = None

_linesim.crossed_finish_C.argtypes = [
    c_double, c_double, c_double, c_double,
    c_double, c_double, c_double, c_double
]
_linesim.crossed_finish_C.restype = c_int

# Wheel visuals to match robot_editor
WHEEL_W_MM = 22.0
WHEEL_H_MM = 15.0
WHEEL_PEN   = QPen(QColor("#000000"), 1)
WHEEL_BRUSH = QColor("#dddddd")

# ---------- Geometry / track helpers ----------
@dataclass
class Pt:
    x: float
    y: float

@dataclass
class Pose:
    p: Pt
    headingDeg: float

def rad(deg: float) -> float:
    return math.radians(deg)

def rot(x: float, y: float, a: float) -> Tuple[float, float]:
    c, s = math.cos(a), math.sin(a)
    return (c*x - s*y, s*x + c*y)

def advance_straight(pose: Pose, d: float) -> Pose:
    a = rad(pose.headingDeg)
    return Pose(Pt(pose.p.x + d*math.cos(a), pose.p.y + d*math.sin(a)), pose.headingDeg)

def advance_arc(pose: Pose, R: float, sweepDeg: float) -> Pose:
    a0 = rad(pose.headingDeg)
    s = 1.0 if sweepDeg >= 0.0 else -1.0
    Lx, Ly = -math.sin(a0), math.cos(a0)
    cx = pose.p.x + s * R * Lx
    cy = pose.p.y + s * R * Ly
    phi0 = math.atan2(pose.p.y - cy, pose.p.x - cx)
    phi1 = phi0 + rad(sweepDeg)
    x = cx + R * math.cos(phi1)
    y = cy + R * math.sin(phi1)
    return Pose(Pt(x, y), pose.headingDeg + sweepDeg)

@dataclass
class SegStraight:
    kind: str
    id: str
    lengthMM: float
    from_pose: Pose

@dataclass
class SegArc:
    kind: str
    id: str
    radiusMM: float
    sweepDeg: float
    from_pose: Pose

def segments_from_json(track: Dict[str, Any]) -> Tuple[List[object], Pose, float]:
    origin = Pose(Pt(track["origin"]["p"]["x"], track["origin"]["p"]["y"]),
                  float(track["origin"]["headingDeg"]))
    tapeW = float(track.get("tapeWidthMM", 20.0))
    cur = origin
    segs: List[object] = []
    for s in track["segments"]:
        if s["kind"] == "straight":
            seg = SegStraight("straight", s["id"], float(s["lengthMM"]), cur)
            cur = advance_straight(cur, seg.lengthMM)
        else:
            seg = SegArc("arc", s["id"], float(s["radiusMM"]), float(s["sweepDeg"]), cur)
            cur = advance_arc(cur, seg.radiusMM, seg.sweepDeg)
        segs.append(seg)
    return segs, origin, tapeW

def segments_polyline(segs: List[object], step: float = 1.0) -> List[Pt]:
    pts: List[Pt] = []
    for s in segs:
        if isinstance(s, SegStraight):
            n = max(2, int(math.ceil(s.lengthMM/step)))
            for i in range(n):
                t = i/(n-1)
                p = advance_straight(s.from_pose, s.lengthMM*t).p
                pts.append(Pt(p.x, p.y))
        else:
            L = abs(rad(s.sweepDeg))*s.radiusMM
            n = max(6, int(math.ceil(L/step)))
            for i in range(n):
                t = i/(n-1)
                p = advance_arc(s.from_pose, s.radiusMM, s.sweepDeg*t).p
                pts.append(Pt(p.x, p.y))
    return pts

# Drawing constants
START_FINISH_GAP_MM = 1000.0
STRAIGHT_NEAR_XING_MM = 250.0
MARKER_OFFSET_MM = 40.0
MARKER_LENGTH_MM = 40.0
MARKER_THICKNESS_MM = 20.0

def curvature_change_markers(segs: List[object]) -> List[Tuple[Pt, float]]:
    def kappa(s: object) -> float:
        if isinstance(s, SegArc): return (1.0 if s.sweepDeg >= 0.0 else -1.0)/max(1e-9, s.radiusMM)
        return 0.0
    def end_pose(s: object) -> Pose:
        return advance_straight(s.from_pose, s.lengthMM) if isinstance(s, SegStraight) \
               else advance_arc(s.from_pose, s.radiusMM, s.sweepDeg)
    out: List[Tuple[Pt, float]] = []
    if not segs: return out
    for i in range(len(segs) - 1):
        a, b = segs[i], segs[i+1]
        if abs(kappa(a) - kappa(b)) > 1e-6:
            ep = end_pose(a); out.append((ep.p, ep.headingDeg))
    first, last = segs[0], segs[-1]
    ep_last = end_pose(last)
    is_closed = (math.hypot(ep_last.p.x - first.from_pose.p.x,
                            ep_last.p.y - first.from_pose.p.y) <= 1.0)
    if is_closed and abs(kappa(last) - kappa(first)) > 1e-6:
        out.append((ep_last.p, ep_last.headingDeg))
    return out

def start_finish_lines(track: Dict[str, Any], segs: List[object], tapeW: float):
    sf = track.get("startFinish") or {}
    if not sf.get("enabled", False):
        return None

    segId = sf.get("onSegmentId")
    startIsFwd = bool(sf.get("startIsForward", True))
    invert = bool(sf.get("invert", False))
    sParam = float(sf.get("sParamMM", 0.0))

    straight = next((s for s in segs if isinstance(s, SegStraight) and s.id == segId), None)
    if not straight:
        return None

    def clamp_start_on_seg(seg: SegStraight, t: float) -> float:
        mn = START_FINISH_GAP_MM + STRAIGHT_NEAR_XING_MM
        mx = seg.lengthMM - STRAIGHT_NEAR_XING_MM
        return max(mn, min(mx, t))

    sParam = clamp_start_on_seg(straight, sParam)

    def gate_at(d: float) -> Tuple[Pt, Pt, float, float]:
        pose = advance_straight(straight.from_pose, max(0.0, min(straight.lengthMM, d)))

        base_hdg = pose.headingDeg
        run_hdg  = (base_hdg + 180.0) if invert else base_hdg

        a = math.radians(base_hdg)
        nx, ny = -math.sin(a), math.cos(a)
        half = (tapeW * 1.2) * 0.5

        ax, ay = pose.p.x + nx*half, pose.p.y + ny*half
        bx, by = pose.p.x - nx*half, pose.p.y - ny*half
        return (Pt(ax, ay), Pt(bx, by), run_hdg, base_hdg)

    start_pose  = gate_at(sParam)
    finish_pose = gate_at(max(0.0, sParam - START_FINISH_GAP_MM))
    return (start_pose, finish_pose) if startIsFwd else (finish_pose, start_pose)

# ---------- Robot ----------
@dataclass
class Envelope:
    widthMM: float
    heightMM: float

@dataclass
class Wheel:
    id: str
    xMM: float
    yMM: float

@dataclass
class Sensor:
    id: str
    xMM: float
    yMM: float
    sizeMM: float = 5.0

@dataclass
class Robot:
    envelope: Envelope
    wheels: List[Wheel]
    sensors: List[Sensor]
    gridStepMM: float = 5.0
    originXMM: float = 0.0
    originYMM: float = 0.0

def robot_from_json(obj: Dict[str, Any]) -> Robot:
    env = obj.get("envelope")
    if not env:
        width  = obj.get("widthMM")  or (obj.get("body") or {}).get("widthMM")
        height = obj.get("heightMM") or (obj.get("body") or {}).get("heightMM")
        if width is None or height is None:
            width, height = 160.0, 140.0
        env = {"widthMM": float(width), "heightMM": float(height)}
    ox = float(obj.get("originXMM", (obj.get("origin") or {}).get("xMM", 0.0)))
    oy = float(obj.get("originYMM", (obj.get("origin") or {}).get("yMM", 0.0)))
    if not obj.get("wheels"):  raise ValueError("Invalid robot file: missing 'wheels'.")
    if not obj.get("sensors"): raise ValueError("Invalid robot file: missing 'sensors'.")
    wheels = [Wheel(w["id"], float(w["xMM"]), float(w["yMM"])) for w in obj["wheels"]]
    sensors = [Sensor(s["id"], float(s["xMM"]), float(s["yMM"]), float(s.get("sizeMM", 5.0)))
               for s in obj["sensors"]]
    return Robot(
        Envelope(float(env["widthMM"]), float(env["heightMM"])),
        wheels,
        sensors,
        float(obj.get("gridStepMM", 5.0)),
        ox, oy
    )

# ---------- Sensor utility (8-bit two-range with noise) ----------
def sensor_value_from_coverage_random(cov: float) -> int:
    cov = max(0.0, min(1.0, cov))
    if cov >= 0.5:
        return random.randint(0, 100)   # darker = lower value
    else:
        return random.randint(200, 255) # lighter = higher value

# ---------- Controller loading ----------
def load_python_controller(path: str):
    spec = importlib.util.spec_from_file_location("controller_mod", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)  # type: ignore
    if not hasattr(mod, "control_step"):
        raise ValueError("Python controller must define control_step(state)->{'pwm_left','pwm_right'}.")
    return mod.control_step

def import_controller(path: str):
    ext = os.path.splitext(path)[1].lower()
    if ext == ".py": return load_python_controller(path)
    raise ValueError("Only .py is supported.")

# ---------- Simulation logger (CSV + JSON) ----------
class SimLogger:
    """Stores steps and events under Logs/sim_log_*.csv and Logs/sim_log_*.json."""
    def __init__(self, base_dir: str):
        self.base_dir = base_dir
        os.makedirs(os.path.join(base_dir, "Logs"), exist_ok=True)
        run_id = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(base_dir, "Logs", f"sim_log_{run_id}.csv")
        self.json_path = os.path.join(base_dir, "Logs", f"sim_log_{run_id}.json")
        self.steps: list[dict] = []
        self.events: list[dict] = []
        self._max_sensors = 0

    def log_step(self, t_ms: int, x: float, y: float, h: float,
                 v: float, w: float, pwmL: int, pwmR: int,
                 sensors: Optional[List[int]] = None) -> None:
        self.steps.append({
            "t_ms": t_ms, "x_mm": x, "y_mm": y, "heading_deg": h,
            "v_mm_s": v, "omega_rad_s": w, "pwm_left": pwmL, "pwm_right": pwmR,
            "sensors": list(sensors) if sensors is not None else []
        })

    def log_event(self, kind: str, t_ms: int, x: float, y: float, h: float, extra: dict | None = None) -> None:
        ev = {"event": kind, "t_ms": t_ms, "x_mm": x, "y_mm": y, "heading_deg": h}
        if extra: ev.update(extra)
        self.events.append(ev)

    def flush(self) -> None:
        # CSV (steps only, with sensor columns)
        try:
            with open(self.csv_path, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                base_cols = ["t_ms","x_mm","y_mm","heading_deg","v_mm_s","omega_rad_s","pwm_left","pwm_right"]
                sn_cols = [f"s{i}" for i in range(self._max_sensors)]
                w.writerow(base_cols + sn_cols)

                for s in self.steps:
                    row = [
                        s["t_ms"], s["x_mm"], s["y_mm"], s["heading_deg"],
                        s["v_mm_s"], s["omega_rad_s"], s["pwm_left"], s["pwm_right"]
                    ]
                    vals = s.get("sensors", [])
                    row.extend([vals[i] if i < len(vals) else "" for i in range(self._max_sensors)])
                    w.writerow(row)
        except Exception as e:
            print("CSV log error:", e)

        # JSON (steps + events)
        try:
            with open(self.json_path, "w", encoding="utf-8") as f:
                json.dump({"steps": self.steps, "events": self.events}, f, indent=2, ensure_ascii=False)
        except Exception as e:
            print("JSON log error:", e)

# ---------- Finish zone checker ----------
class FinishZoneChecker:
    """
    Robust FSM to end the run:
      • If started INSIDE: must exit once, then arm on START crossing.
      • If started OUTSIDE: must enter once, then arm on START crossing.
      • Finish when re-entering the zone (armed).
    """
    def __init__(self, sa: Pt, sb: Pt, fa: Pt, fb: Pt, half_width=250.0, eps=3.0):
        self.s_mid = Pt((sa.x + sb.x)/2.0, (sa.y + sb.y)/2.0)
        self.f_mid = Pt((fa.x + fb.x)/2.0, (fa.y + fb.y)/2.0)
        ux = self.s_mid.x - self.f_mid.x
        uy = self.s_mid.y - self.f_mid.y
        L = math.hypot(ux, uy) or 1.0
        self.ux, self.uy = (ux/L, uy/L)
        self.nx, self.ny = (-self.uy, self.ux)
        self.L = L
        self.HALF_W = float(half_width)
        self.EPS = float(eps)

        self.started_inside = False
        self.last_inside = False
        self.exited_once = False
        self.entered_once = False
        self.armed = False
        self.last_event: Optional[str] = None

        self._cross = _linesim.crossed_finish_C
        self._sa, self._sb = sa, sb
        self._fa, self._fb = fa, fb

        self._finish_cross_t_ms: Optional[int] = None
        self._require_envelope_ms = 0

    def prime(self, cx: float, cy: float) -> None:
        self.started_inside = self._point_inside(cx, cy)
        self.last_inside = self.started_inside
        self.exited_once = (not self.started_inside)
        self.entered_once = self.started_inside
        self.armed = False
        self.last_event = "init_inside" if self.started_inside else "init_outside"

    def _proj_t(self, x: float, y: float) -> float:
        vx, vy = x - self.f_mid.x, y - self.f_mid.y
        return vx*self.ux + vy*self.uy

    def _proj_w(self, x: float, y: float) -> float:
        vx, vy = x - self.f_mid.x, y - self.f_mid.y
        return vx*self.nx + vy*self.ny

    def _point_inside(self, x: float, y: float) -> bool:
        t = self._proj_t(x, y); w = abs(self._proj_w(x, y))
        return (-self.EPS <= t <= self.L + self.EPS) and (w <= self.HALF_W + self.EPS)

    def _envelope_inside(self, cx, cy, heading_deg, env_w, env_h) -> bool:
        hw, hh = env_w*0.5, env_h*0.5
        ang = math.radians(heading_deg)
        for (lx, ly) in [(-hw,-hh),(hw,-hh),(hw,hh),(-hw,hh)]:
            rx, ry = rot(lx, ly, ang); px, py = cx+rx, cy+ry
            if not self._point_inside(px, py): return False
        return True

    def update(self, prev_pose, curr_pose, env_w, env_h, t_ms: Optional[int] = None) -> bool:
        self.last_event = None
        px0, py0, _ = prev_pose
        px1, py1, h1 = curr_pose

        prev_inside = self.last_inside
        inside_now = self._point_inside(px1, py1)

        if (not prev_inside) and inside_now:
            self.entered_once = True
            self.last_event = "entered_zone"
        elif prev_inside and (not inside_now):
            self.exited_once = True
            self.last_event = "exited_zone"

        if (not self.armed) and self.exited_once and self.entered_once:
            self.armed = True
            if self.last_event is None:
                self.last_event = "armed"

        if self.armed and (not prev_inside) and inside_now:
            self.last_event = "finish"
            self.last_inside = inside_now
            return True

        self.last_inside = inside_now
        return False

# ---------- Convex polygon utils (rect/rect overlap) ----------
def poly_area(poly):
    if len(poly) < 3: return 0.0
    a = 0.0
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i+1) % len(poly)]
        a += x1*y2 - x2*y1
    return abs(a) * 0.5

def suth_hodg_clip(subject, clip):
    """Convex polygon intersection (Sutherland–Hodgman)."""
    def inside(p, a, b):
        return (b[0]-a[0])*(p[1]-a[1]) - (b[1]-a[1])*(p[0]-a[0]) >= 0.0
    def intersect(p1, p2, a, b):
        x1,y1 = p1; x2,y2 = p2; x3,y3 = a; x4,y4 = b
        den = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4) or 1e-12
        px = ((x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4 - y3*x4))/den
        py = ((x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4 - y3*x4))/den
        return (px, py)

    output = subject[:]
    for i in range(len(clip)):
        a = clip[i]
        b = clip[(i+1) % len(clip)]
        input_list = output
        output = []
        if not input_list:
            break
        s = input_list[-1]
        for e in input_list:
            if inside(e, a, b):
                if not inside(s, a, b):
                    output.append(intersect(s, e, a, b))
                output.append(e)
            elif inside(s, a, b):
                output.append(intersect(s, e, a, b))
            s = e
    return output

def oriented_rect(cx, cy, ux, uy, halfL, vx, vy, halfW):
    """Axis-aligned rectangle in local (u,v) axes, returned as 4 points (clockwise)."""
    return [
        (cx - ux*halfL - vx*halfW, cy - uy*halfL - vy*halfW),
        (cx + ux*halfL - vx*halfW, cy + uy*halfL - vy*halfW),
        (cx + ux*halfL + vx*halfW, cy + uy*halfL + vy*halfW),
        (cx - ux*halfL + vx*halfW, cy - uy*halfL + vy*halfW),
    ]

def rect_rect_overlap_area(R1, R2):
    poly = suth_hodg_clip(R1, R2)
    return poly_area(poly)

# ---------- Simulation worker thread ----------
class SimWorker(QThread):
    sig_chunk = Signal(list)
    sig_done  = Signal(dict)
    sig_fail  = Signal(str)

    def __init__(self, track: Dict[str, Any], robot: Robot, controller_fn,
                 v_final_mps: float, tau_s: float, parent=None):
        super().__init__(parent)
        self.track = track
        self.robot = robot
        self.controller_fn = controller_fn
        self.v_final = float(v_final_mps)*1000.0  # mm/s @ PWM=4095
        self.tau = max(1e-6, float(tau_s))
        self.cancelled = False

        self.logger = SimLogger(base_dir=_here)
        self._marker_logged = False

    def envelope_contacts_tape(self, x, y, h_deg, tape_half_with_margin) -> bool:
        # Convert robot ORIGIN pose to envelope CENTER pose expected by C library
        cx = x - self.robot.originXMM
        cy = y - self.robot.originYMM
        hit = _linesim.envelope_contacts_tape_C(
            cx, cy,
            h_deg,
            self.robot.envelope.widthMM,
            self.robot.envelope.heightMM,
            self._poly_ptr, self._poly_n,
            tape_half_with_margin,
            0
        )
        return bool(hit)

    def build_markers(self, segs, tapeW, gates):
        """Return a list of rectangles (each as 4 points) for curvature and start/finish markers."""
        rects = []
        halfL = MARKER_LENGTH_MM * 0.5
        halfW = MARKER_THICKNESS_MM * 0.5

        # Curvature-change markers on the LEFT
        for (pp, hdg) in curvature_change_markers(segs):
            a = math.radians(hdg)
            tx, ty = math.cos(a), math.sin(a)      # tangent
            nx, ny = math.sin(a), -math.cos(a)     # left normal
            base = (tapeW*0.5) + MARKER_OFFSET_MM
            cx, cy = pp.x + nx*base, pp.y + ny*base
            rects.append(oriented_rect(cx, cy, nx, ny, halfL, tx, ty, halfW))

        if gates:
            (sa, sb, shdg_run, shdg_base), (fa, fb, fhdg_run, fhdg_base) = gates

            def add_right_rect(pa, pb, base_hdg):
                a = math.radians(base_hdg)
                tx, ty = math.cos(a), math.sin(a)
                nx, ny = -math.sin(a), math.cos(a)
                mx, my = (pa.x + pb.x)*0.5, (pa.y + pb.y)*0.5
                base = (tapeW*0.5) + MARKER_OFFSET_MM
                cx, cy = mx + nx*base, my + ny*base
                rects.append(oriented_rect(cx, cy, nx, ny, halfL, tx, ty, halfW))

            add_right_rect(sa, sb, shdg_base)  # START
            add_right_rect(fa, fb, fhdg_base)  # FINISH

        return rects

    def run(self):
        try:
            # Track
            segs, origin, tapeW = segments_from_json(self.track)
            poly = segments_polyline(segs, step=0.5)
            _poly_arr = (CPoint * len(poly))(*(CPoint(p.x, p.y) for p in poly))
            _poly_ptr = ctypes.cast(_poly_arr, POINTER(CPoint))
            _poly_n   = len(poly)
            self._poly_ptr = _poly_ptr
            self._poly_n   = _poly_n

            # Start/Finish
            gates = start_finish_lines(self.track, segs, tapeW)
            start_gate, finish_gate = (None, None) if (gates is None) else gates

            # Initial pose (behind START if available)
            if start_gate is None:
                x, y, h = origin.p.x, origin.p.y, origin.headingDeg
                sa = sb = None
            else:
                (sa, sb, shdg, _) = start_gate
                back = (self.robot.envelope.heightMM/2.0) + 10.0
                pose = Pose(Pt((sa.x+sb.x)/2.0, (sa.y+sb.y)/2.0), shdg)
                start_pose = advance_straight(pose, -back)
                x, y, h = start_pose.p.x, start_pose.p.y, start_pose.headingDeg

            # Finish zone checker
            zone_checker: Optional[FinishZoneChecker] = None
            if start_gate is not None and finish_gate is not None:
                (fa, fb, _, _) = finish_gate
                zone_checker = FinishZoneChecker(sa, sb, fa, fb, half_width=400.0, eps=5.0)
                zone_checker.prime(x, y)

            # States
            dt = 0.001
            vL = vR = v = w = 0.0
            prev_v = prev_w = 0.0

            wl = next((ww for ww in self.robot.wheels if ww.id.lower() == "left"), None)
            wr = next((ww for ww in self.robot.wheels if ww.id.lower() == "right"), None)
            trackW = abs((wr.yMM if wr else 35.0) - (wl.yMM if wl else -35.0))

            def pwm_to_wheel_v(pwm: int) -> float:
                pwm = max(-4095, min(4095, int(pwm)))
                return (pwm/4095.0) * self.v_final

            # Sensor buffers (batch)
            nS = len(self.robot.sensors)
            sens_px = (c_double * nS)()
            sens_py = (c_double * nS)()
            sens_sz = (c_double * nS)(*([s.sizeMM for s in self.robot.sensors]))
            cov_out = (c_double * nS)()

            steps_out: List[Dict[str, Any]] = []
            CHUNK_STEPS = 100
            MAX_MS = 300000
            reason = "timeout"

            random.seed(datetime.now().timestamp())

            # Log start
            self.logger.log_event("init", 0, x, y, h, {"note": "simulation started"})

            # Markers (curvature + start/finish small ticks)
            markers = self.build_markers(segs, tapeW, gates)

            enter_t_ms = None

            for t_ms in range(MAX_MS):
                if self.cancelled:
                    reason = "user_stop"
                    self.logger.log_event("user_stop", t_ms, x, y, h, {})
                    break

                # Sensor world positions
                a = math.radians(h)
                ca, sa_ = math.cos(a), math.sin(a)
                for i, s in enumerate(self.robot.sensors):
                    rx = ca*s.xMM - sa_*s.yMM
                    ry = sa_*s.xMM + ca*s.yMM
                    sens_px[i] = x + rx
                    sens_py[i] = y + ry

                # Line coverage from DLL (tape only)
                _linesim.estimate_sensors_coverage_batch_C(
                    sens_px, sens_py, nS,
                    _poly_ptr, _poly_n,
                    tapeW*0.5,
                    sens_sz, 5.0, 3,
                    cov_out
                )

                # Combine tape coverage with marker overlap (treated as "light")
                sn_vals = []
                ang = math.radians(h)
                tx, ty = math.cos(ang), math.sin(ang)   # local X (forward)
                nx, ny = -math.sin(ang), math.cos(ang)  # local Y (left)
                marker_hit_this_step = False

                for i, s in enumerate(self.robot.sensors):
                    px, py = sens_px[i], sens_py[i]

                    # Oriented square (sensor area)
                    halfS = 0.5 * float(s.sizeMM)
                    sensor_rect = oriented_rect(px, py, tx, ty, halfS, nx, ny, halfS)
                    area_sensor = (2*halfS)*(2*halfS)

                    # Max overlap with any marker rectangle
                    cov_marker = 0.0
                    for Rm in markers:
                        interA = rect_rect_overlap_area(sensor_rect, Rm)
                        if interA <= 0.0:
                            continue
                        cov_marker = max(cov_marker, interA / area_sensor)

                    cov = max(float(cov_out[i]), cov_marker)

                    if cov_marker > 0.0:
                        marker_hit_this_step = True

                    sn_vals.append(sensor_value_from_coverage_random(cov))

                if (not self._marker_logged) and marker_hit_this_step:
                    self._marker_logged = True
                    self.logger.log_event("marker_touch", t_ms, x, y, h, {})

                # Controller
                a_lin = (v - prev_v) / dt
                a_ang = (w - prev_w) / dt
                state = {
                    "t_ms": t_ms,
                    "pose": {"x_mm": x, "y_mm": y, "heading_deg": h},
                    "vel": {"v_mm_s": v, "omega_rad_s": w},
                    "accel": {"a_lin_mm_s2": a_lin, "alpha_rad_s2": a_ang},
                    "sensors": {"values": sn_vals},
                    "wheels": {"v_left_mm_s": vL, "v_right_mm_s": vR}
                }
                ctrl = self.controller_fn(state)
                pwmL = int(ctrl.get("pwm_left", 0))
                pwmR = int(ctrl.get("pwm_right", 0))

                # First-order wheel dynamics
                vL_cmd = pwm_to_wheel_v(pwmL)
                vR_cmd = pwm_to_wheel_v(pwmR)
                alpha = 1.0 - math.exp(-dt/self.tau)
                vL += (vL_cmd - vL)*alpha
                vR += (vR_cmd - vR)*alpha

                prev_v, prev_w = v, w
                v = 0.5*(vL + vR)
                w = (vR - vL)/max(1e-6, trackW)

                # Integrate pose (x,y are the robot ORIGIN)
                prev_x, prev_y, prev_h = x, y, h
                h += math.degrees(w*dt)
                a = math.radians(h)
                x += v*dt*math.cos(a)
                y += v*dt*math.sin(a)

                # Log step
                self.logger.log_step(t_ms, x, y, h, v, w, pwmL, pwmR, sensors=sn_vals)

                # Start/Finish zone
                if zone_checker is not None:
                    finished = zone_checker.update(
                        (prev_x, prev_y, prev_h),
                        (x, y, h),
                        self.robot.envelope.widthMM,
                        self.robot.envelope.heightMM
                    )
                    if zone_checker.last_event:
                        self.logger.log_event(zone_checker.last_event, t_ms, x, y, h, {})
                    if finished:
                        enter_t_ms = t_ms
                    # Stop shortly after first re-entry (debounced)
                    if (enter_t_ms is not None) and (t_ms - enter_t_ms >= 100):
                        reason = "finished"
                        self.logger.log_event("finished", t_ms, x, y, h, {"enter_t_ms": enter_t_ms})
                        break

                # Off-track stop: no envelope corner touching the tape
                tape_half_with_margin = (tapeW * 0.5) + 25.0
                if not self.envelope_contacts_tape(x, y, h, tape_half_with_margin):
                    reason = "offtrack"
                    self.logger.log_event("offtrack", t_ms, x, y, h, {"criterion": "envelope-not-touching"})
                    break

                # Stream chunk to UI
                steps_out.append({
                    "t_ms": t_ms, "x_mm": x, "y_mm": y, "heading_deg": h,
                    "v_mm_s": v, "omega_rad_s": w, "pwmL": pwmL, "pwmR": pwmR
                })
                if len(steps_out) >= CHUNK_STEPS:
                    self.sig_chunk.emit(steps_out); steps_out = []

            if steps_out:
                self.sig_chunk.emit(steps_out)
            if reason == "timeout":
                self.logger.log_event("timeout", t_ms, x, y, h, {})

            self.logger.flush()
            self.sig_done.emit({"ok": True, "reason": reason,
                                "csv": self.logger.csv_path, "json": self.logger.json_path})
        except Exception as e:
            self.sig_fail.emit(str(e))

# ---------- Scene / View ----------
class SimScene(QGraphicsScene):
    def __init__(self):
        super().__init__()
        self.setBackgroundBrush(QColor("#0c0c0c"))

    def drawBackground(self, painter: QPainter, rect):
        super().drawBackground(painter, rect)
        # Subtle grid
        step = 25.0
        pen = QPen(QColor(30, 30, 30), 1)
        painter.setPen(pen)
        x = math.floor(rect.left()/step)*step
        while x <= rect.right():
            painter.drawLine(x, rect.top(), x, rect.bottom()); x += step
        y = math.floor(rect.top()/step)*step
        while y <= rect.bottom():
            painter.drawLine(rect.left(), y, rect.right(), y); y += step

class SimView(QGraphicsView):
    def __init__(self, scene: SimScene):
        super().__init__(scene)
        self.setRenderHints(self.renderHints() | QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self.setDragMode(QGraphicsView.ScrollHandDrag)

    def wheelEvent(self, e):
        s = 1.15 if e.angleDelta().y() > 0 else 1/1.15
        self.scale(s, s)

# ---------- Main window (English UI) ----------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Line-Follower Simulator (dt = 1 ms)")

        # Data
        self.track: Optional[Dict[str, Any]] = None
        self.robot: Optional[Robot] = None
        self.controller_fn = lambda state: {"pwm_left": 2000, "pwm_right": 2000}

        # Scene / View
        self.scene = SimScene()
        self.view = SimView(self.scene)
        self.view.setSceneRect(0, 0, 4000, 3000)

        # Left panel
        left = QWidget(); form = QFormLayout(left)

        self.btn_track = QPushButton("Load track (.json)")
        self.btn_robot = QPushButton("Load robot (.json)")
        self.btn_ctrl  = QPushButton("Load controller (.py)")
        self.spin_vf   = QDoubleSpinBox(); self.spin_vf.setRange(0.1, 20.0); self.spin_vf.setValue(2.0); self.spin_vf.setSingleStep(0.1)
        self.spin_tau  = QDoubleSpinBox(); self.spin_tau.setRange(0.001, 5.0); self.spin_tau.setValue(0.05); self.spin_tau.setSingleStep(0.01)
        self.btn_sim   = QPushButton("Start simulation")
        self.btn_stop  = QPushButton("Stop")
        self.btn_replay= QPushButton("Replay")
        self.progress  = QProgressBar(); self.progress.setValue(0)
        self.step_count = 0
        self.lbl_progress = QLabel("Executed steps: 0"); form.addRow(self.lbl_progress)
        self.lbl_time = QLabel("Sim time: 0.00 s | Anim speed: 1.0×"); form.addRow(self.lbl_time)

        self.combo_speed = QComboBox()
        self.combo_speed.addItems(["0.1×", "0.5×", "1×", "2×", "4×"])
        self.combo_speed.setCurrentIndex(2)
        form.addRow(QLabel("Animation speed"), self.combo_speed)

        self.anim_speed = 1.0
        self.anim_spf = 17
        self.combo_speed.currentIndexChanged.connect(self.on_speed_change)

        form.addRow(self.btn_track)
        form.addRow(self.btn_robot)
        form.addRow(self.btn_ctrl)
        form.addRow(QLabel("Final linear speed (m/s)"), self.spin_vf)
        form.addRow(QLabel("Motor time constant τ (s)"), self.spin_tau)
        form.addRow(self.btn_sim)
        form.addRow(self.btn_stop)
        form.addRow(self.btn_replay)
        form.addRow(self.progress)

        self.btn_stop.setEnabled(False)
        self.btn_replay.setEnabled(False)

        splitter = QSplitter()
        splitter.addWidget(left)
        splitter.addWidget(self.view)
        splitter.setSizes([320, 1200])
        self.setCentralWidget(splitter)
        self.resize(1280, 800)

        # Signals
        self.btn_track.clicked.connect(self.on_load_track)
        self.btn_robot.clicked.connect(self.on_load_robot)
        self.btn_ctrl.clicked.connect(self.on_load_controller)
        self.btn_sim.clicked.connect(self.on_simulate)
        self.btn_stop.clicked.connect(self.on_stop)
        self.btn_replay.clicked.connect(self.on_replay)

        # Replay/animation
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.anim_idx = 0
        self.anim_steps: List[Dict[str, Any]] = []
        self.anim_items: Dict[str, Any] = {}

        # Worker
        self.worker: Optional[SimWorker] = None
        self.v_max_mm_s = None

    def _draw_robot_at_initial_pose(self):
        if not (self.track and self.robot):
            return
        segs, origin, tapeW = segments_from_json(self.track)
        gates = start_finish_lines(self.track, segs, tapeW)

        if gates:
            (sa, sb, shdg_run, shdg_base), _ = gates
            back = (self.robot.envelope.heightMM/2.0) + 250.0
            pose_gate = Pose(Pt((sa.x + sb.x)*0.5, (sa.y + sb.y)*0.5), shdg_run)
            pos = advance_straight(pose_gate, -back)
        else:
            pos = origin

        # Body
        hw = self.robot.envelope.widthMM * 0.5
        hh = self.robot.envelope.heightMM * 0.5
        ang = rad(pos.headingDeg)
        ox, oy = self.robot.originXMM, self.robot.originYMM

        corners = [(-hw,-hh),(hw,-hh),(hw,hh),(-hw,hh)]
        poly = QPainterPath()
        for k,(cx,cy) in enumerate(corners + [corners[0]]):
            lx, ly = cx - ox, cy - oy
            rx, ry = rot(lx, ly, ang)
            px, py = pos.p.x + rx, pos.p.y + ry
            poly.moveTo(px, py) if k == 0 else poly.lineTo(px, py)
        self.anim_items["robot"].setPath(poly)

        # Sensors
        for k, s in enumerate(self.robot.sensors):
            px, py = s.xMM - ox, s.yMM - oy
            rx, ry = rot(px, py, ang)
            cx, cy = pos.p.x + rx, pos.p.y + ry
            sp = QPainterPath()
            sp.addRect(cx - 2, cy - 2, 4, 4)
            if k < len(self.anim_items["sensors"]):
                self.anim_items["sensors"][k].setPath(sp)

        # Wheels
        half_w = WHEEL_W_MM * 0.5
        half_h = WHEEL_H_MM * 0.5
        for k, wdef in enumerate(self.robot.wheels):
            px, py = wdef.xMM - ox, wdef.yMM - oy
            rx, ry = rot(px, py, ang)
            cx, cy = pos.p.x + rx, pos.p.y + ry
            wp = QPainterPath()
            for i,(lx,ly) in enumerate([(-half_w,-half_h),(half_w,-half_h),(half_w,half_h),(-half_w,half_h),(-half_w,-half_h)]):
                rlx, rly = rot(lx, ly, ang)
                vx, vy = cx + rlx, cy + rly
                wp.moveTo(vx, vy) if i == 0 else wp.lineTo(vx, vy)
            if k < len(self.anim_items["wheels"]):
                self.anim_items["wheels"][k].setPath(wp)

    # ---------- UI helpers ----------
    def on_speed_change(self, idx: int):
        mapping = {0: 0.1, 1: 0.5, 2: 1.0, 3: 2.0, 4: 4.0}
        self.anim_speed = mapping.get(idx, 1.0)
        base_spf = int(round(1000.0 / 60.0))
        self.anim_spf = max(1, int(round(base_spf * self.anim_speed)))
        i = self.anim_idx if self.anim_steps else 0
        self.lbl_time.setText("Sim time: {:.2f} s | Anim speed: {:.1f}×".format(i*0.001, self.anim_speed))

    # ---------- Loaders ----------
    def on_load_track(self):
        path, _ = QFileDialog.getOpenFileName(self, "Track file", "", "JSON (*.json)")
        if not path: return
        with open(path, "r", encoding="utf-8") as f:
            self.track = json.load(f)
        self.draw_static_track()
        if self.robot: self.draw_robot_outline_preview()
        self.statusBar().showMessage(f"Track: {os.path.basename(path)}")

    def on_load_robot(self):
        path, _ = QFileDialog.getOpenFileName(self, "Robot file", "", "JSON (*.json)")
        if not path: return
        try:
            with open(path, "r", encoding="utf-8") as f:
                self.robot = robot_from_json(json.load(f))
            self.statusBar().showMessage(f"Robot: {os.path.basename(path)}", 5000)
            if self.track: self.draw_robot_outline_preview()
        except Exception as e:
            self.robot = None
            QMessageBox.critical(self, "Robot load error",
                                 f"Failed to read '{os.path.basename(path)}':\n{e}")
            if self.worker and self.worker.isRunning():
                try:
                    self.worker.cancelled = True
                    self.worker.wait(500)
                except Exception:
                    pass

    def on_load_controller(self):
        path, _ = QFileDialog.getOpenFileName(self, "Controller", "", "Python (*.py)")
        if not path: return
        self.controller_fn = import_controller(path)
        QMessageBox.information(self, "Controller", "Controller loaded successfully.")

    # ---------- Static drawing ----------
    def clear_static(self):
        self.scene.clear()
        self.anim_items.clear()

    def draw_static_track(self):
        if not self.track: return
        self.clear_static()
        segs, origin, tapeW = segments_from_json(self.track)
        pts = segments_polyline(segs, step=1.0)
        if len(pts) >= 2:
            path = QPainterPath(QPointF(pts[0].x, pts[0].y))
            for p in pts[1:]:
                path.lineTo(p.x, p.y)
            self.scene.addPath(path, QPen(QColor("#f5f5f5"), tapeW, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            self.scene.addPath(path, QPen(QColor("#444444"), 1, Qt.DashLine))

        for (pp, hdg) in curvature_change_markers(segs):
            a = rad(hdg)
            tx, ty = math.cos(a), math.sin(a)
            nx, ny = math.sin(a), -math.cos(a)
            base = (tapeW*0.5) + MARKER_OFFSET_MM
            cx, cy = pp.x + nx*base, pp.y + ny*base
            halfL = MARKER_LENGTH_MM * 0.5
            halfW = MARKER_THICKNESS_MM * 0.5
            R = oriented_rect(cx, cy, nx, ny, halfL, tx, ty, halfW)

            path = QPainterPath(QPointF(R[0][0], R[0][1]))
            for k in range(1,4):
                path.lineTo(R[k][0], R[k][1])
            path.closeSubpath()
            self.scene.addPath(path, QPen(QColor("#FFFFFF"), 1), QColor("#FFFFFF"))

        gates = start_finish_lines(self.track, segs, tapeW)
        if gates:
            (sa, sb, shdg_run, shdg_base), (fa, fb, fhdg_run, fhdg_base) = gates
            # Build finish->start zone polygon for visualization
            s_mid_x = (sa.x + sb.x) * 0.5; s_mid_y = (sa.y + sb.y) * 0.5
            f_mid_x = (fa.x + fb.x) * 0.5; f_mid_y = (fa.y + fb.y) * 0.5
            ux = s_mid_x - f_mid_x; uy = s_mid_y - f_mid_y
            L = math.hypot(ux, uy) or 1.0
            nx, ny = -uy / L, ux / L
            half = 250.0

            p1 = QPointF(f_mid_x + nx*half, f_mid_y + ny*half)
            p2 = QPointF(s_mid_x + nx*half, s_mid_y + ny*half)
            p3 = QPointF(s_mid_x - nx*half, s_mid_y - ny*half)
            p4 = QPointF(f_mid_x - nx*half, f_mid_y - ny*half)

            zone = QPainterPath(p1); zone.lineTo(p2); zone.lineTo(p3); zone.lineTo(p4); zone.closeSubpath()

            pen = QPen(QColor(0, 188, 212, 200), 2)
            brush = QColor(0, 188, 212, 60)
            item = self.scene.addPath(zone, pen)
            item.setBrush(brush)

            # Small right-side ticks for START and FINISH
            def draw_right_rect(pa, pb, base_hdg):
                a = rad(base_hdg)
                tx, ty = math.cos(a), math.sin(a)
                nx, ny = -math.sin(a), math.cos(a)
                mx, my = (pa.x + pb.x)*0.5, (pa.y + pb.y)*0.5
                base = (tapeW*0.5) + MARKER_OFFSET_MM
                cx, cy = mx + nx*base, my + ny*base
                halfL = MARKER_LENGTH_MM * 0.5
                halfW = MARKER_THICKNESS_MM * 0.5
                R = oriented_rect(cx, cy, nx, ny, halfL, tx, ty, halfW)
                path = QPainterPath(QPointF(R[0][0], R[0][1]))
                for k in range(1,4):
                    path.lineTo(R[k][0], R[k][1])
                path.closeSubpath()
                self.scene.addPath(path, QPen(QColor("#FFFFFF"), 1), QColor("#FFFFFF"))

            draw_right_rect(sa, sb, shdg_base)
            draw_right_rect(fa, fb, fhdg_base)

        # Fit view
        self.view.fitInView(self.scene.itemsBoundingRect().adjusted(-100, -100, +100, +100), Qt.KeepAspectRatio)

    def draw_robot_outline_preview(self):
        """Draw envelope & sensors at initial pose (behind START or at origin)."""
        if not (self.track and self.robot): return
        self.draw_static_track()
        self.reset_anim_items()

        segs, origin, tapeW = segments_from_json(self.track)
        gates = start_finish_lines(self.track, segs, tapeW)
        if gates:
            (a, b, hdg_run, hdg_base), _ = gates
            back = (self.robot.envelope.heightMM / 2.0) + 250.0
            pose_gate = Pose(Pt((a.x + b.x) / 2.0, (a.y + b.y) / 2.0), hdg_run)
            pos = advance_straight(pose_gate, -back)
        else:
            pos = origin

        hw = self.robot.envelope.widthMM / 2.0
        hh = self.robot.envelope.heightMM / 2.0
        ang = rad(pos.headingDeg)

        ox, oy = self.robot.originXMM, self.robot.originYMM
        corners = [(-hw,-hh),(hw,-hh),(hw,hh),(-hw,hh)]
        poly = QPainterPath()
        for i,(cx,cy) in enumerate(corners + [corners[0]]):
            lx, ly = cx - ox, cy - oy
            rx, ry = rot(lx, ly, ang)
            px, py = pos.p.x + rx, pos.p.y + ry
            poly.moveTo(px, py) if i == 0 else poly.lineTo(px, py)
        self.anim_items["robot"].setPath(poly)

        self.anim_items["sensors"] = []
        for s in self.robot.sensors:
            rx, ry = rot(s.xMM - self.robot.originXMM, s.yMM - self.robot.originYMM, ang)
            px, py = pos.p.x + rx, pos.p.y + ry
            sz = s.sizeMM
            sp = QPainterPath()
            sp.addRect(px - sz/2.0, py - sz/2.0, sz, sz)
            self.anim_items["sensors"].append(self.scene.addPath(sp, QPen(QColor("#FFFFFF"), 1)))

        self.anim_items["wheels"] = []
        for wdef in self.robot.wheels:
            rx, ry = rot(wdef.xMM + self.robot.originXMM, wdef.yMM + self.robot.originYMM, ang)
            px, py = pos.p.x + rx, pos.p.y + ry
            half_w = WHEEL_W_MM * 0.5
            half_h = WHEEL_H_MM * 0.5
            corners = [(-half_w, -half_h), ( half_w, -half_h), ( half_w,  half_h), (-half_w,  half_h)]
            wp = QPainterPath()
            for i, (cx, cy) in enumerate(corners + [corners[0]]):
                rx2, ry2 = rot(cx, cy, ang)
                vx, vy = px + rx2, py + ry2
                if i == 0: wp.moveTo(vx, vy)
                else:      wp.lineTo(vx, vy)
            item = self.scene.addPath(wp, WHEEL_PEN)
            item.setBrush(WHEEL_BRUSH)
            self.anim_items["wheels"].append(item)

    # ---------- Simulation ----------
    def on_simulate(self):
        if not (self.track and self.robot):
            QMessageBox.warning(self, "Missing data", "Load a track and a robot.")
            return

        # Clear any previous replay
        if self.timer.isActive():
            self.timer.stop()
        self.reset_anim_items()
        self._trail_last_pt = None
        self._draw_robot_at_initial_pose()

        # Reset animation buffers
        self.anim_steps.clear()
        self.anim_idx = 0
        self.progress.setValue(0)
        self.step_count = 0
        self.lbl_progress.setText("Executed steps: 0")

        self.btn_sim.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.btn_replay.setEnabled(False)
        self.v_max_mm_s = self.spin_vf.value() * 1000.0

        # Spawn worker
        self.worker = SimWorker(self.track, self.robot, self.controller_fn,
                                self.spin_vf.value(), self.spin_tau.value())
        self.worker.sig_chunk.connect(self.on_stream_chunk)
        self.worker.sig_done.connect(self.on_stream_done)
        self.worker.sig_fail.connect(self.on_stream_fail)
        self.worker.start(QThread.TimeCriticalPriority)

    def _speed_color(self, v_mm_s: float) -> QColor:
        """Color map for velocity: 0→blue, 0.5*vmax→magenta, 1.0*vmax→red."""
        vmax = max(1e-6, float(self.v_max_mm_s or (self.spin_vf.value()*1000.0)))
        t = max(0.0, min(1.0, v_mm_s / vmax))
        if t <= 0.5:
            u = t / 0.5
            r, g, b = int(255 * u), 0, 255
        else:
            u = (t - 0.5) / 0.5
            r, g, b = 255, 0, int(255 * (1.0 - u))
        return QColor(r, g, b)

    def on_stop(self):
        """Stop replay timer and cancel worker thread."""
        if self.timer.isActive():
            self.timer.stop()
        if self.worker and self.worker.isRunning():
            try:
                self.worker.cancelled = True
                self.worker.wait(2000)
            except Exception:
                pass
        self.btn_sim.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.btn_replay.setEnabled(bool(self.anim_steps))

    def on_stream_chunk(self, chunk: List[dict]):
        self.anim_steps.extend(chunk)
        self.step_count += len(chunk)
        self.lbl_progress.setText(f"Executed steps: {self.step_count}")
        if not self.timer.isActive():
            self.timer.start(16)

    def on_stream_done(self, info: dict):
        if self.worker and self.worker.isRunning():
            self.worker.wait(2000)
        self.worker = None

        self.btn_sim.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.btn_replay.setEnabled(True)

        if self.timer.isActive():
            self.timer.stop()
        self.reset_anim_items()
        self.anim_idx = 0
        self._trail_last_pt = None

        if self.anim_steps:
            self.timer.start(16)

    def on_stream_fail(self, msg: str):
        self.worker = None
        self.btn_stop.setEnabled(False)
        self.btn_sim.setEnabled(True)
        QMessageBox.critical(self, "Error", msg)

    # ---------- Replay ----------
    def reset_anim_items(self):
        for it in list(self.anim_items.values()):
            if isinstance(it, list):
                for sub in it:
                    if sub is not None:
                        self.scene.removeItem(sub)
            elif it is not None:
                self.scene.removeItem(it)
        self.anim_items.clear()
        self.anim_items["trail_items"] = []
        self._trail_last_pt = None
        self.anim_items["robot"] = self.scene.addPath(QPainterPath(), QPen(QColor("#FFEB3B"), 2))
        self.anim_items["sensors"] = []
        self.anim_items["wheels"] = []
        if self.robot:
            for _ in self.robot.sensors:
                self.anim_items["sensors"].append(self.scene.addPath(QPainterPath(), QPen(QColor("#FFFFFF"), 1)))
            for _ in self.robot.wheels:
                item = self.scene.addPath(QPainterPath(), WHEEL_PEN)
                item.setBrush(WHEEL_BRUSH)
                self.anim_items["wheels"].append(item)

    def on_replay(self):
        if not self.anim_steps: return
        self.reset_anim_items()
        self.anim_idx = 0
        self.timer.start(16)

    def closeEvent(self, event):
        try:
            if self.timer.isActive():
                self.timer.stop()
            if self.worker and self.worker.isRunning():
                self.worker.cancelled = True
                self.worker.wait(2000)
        finally:
            super().closeEvent(event)

    def tick(self):
        """Advance the animation by N stored steps per frame."""
        if not self.anim_steps:
            self.timer.stop()
            return

        if ("trail_items" not in self.anim_items) or ("robot" not in self.anim_items):
            self.reset_anim_items()

        steps_this_frame = getattr(self, "anim_spf", 17)

        for _ in range(max(1, steps_this_frame)):
            i = self.anim_idx
            if i >= len(self.anim_steps):
                self.timer.stop()
                return

            step = self.anim_steps[i]
            x = float(step.get("x_mm", 0.0))
            y = float(step.get("y_mm", 0.0))
            h = float(step.get("heading_deg", 0.0))

            sim_t_s = i * 0.001
            self.lbl_time.setText(f"Sim time: {sim_t_s:.2f} s | Anim speed: {self.anim_speed:.1f}×")

            v_now = float(step.get("v_mm_s", 0.0))

            if self.anim_idx == 0:
                self._trail_last_pt = None

            # Trail
            if self._trail_last_pt is None:
                self._trail_last_pt = (x, y)
            else:
                (px, py) = self._trail_last_pt
                pen = QPen(self._speed_color(v_now), 2, Qt.SolidLine, Qt.RoundCap)
                seg = self.scene.addLine(px, py, x, y, pen)
                self.anim_items.setdefault("trail_items", []).append(seg)
                self._trail_last_pt = (x, y)

            # Robot drawing (origin-offset envelope)
            if self.robot:
                hw = self.robot.envelope.widthMM / 2.0
                hh = self.robot.envelope.heightMM / 2.0
                ang = rad(h)
                ox, oy = self.robot.originXMM, self.robot.originYMM
                corners = [(-hw,-hh),(hw,-hh),(hw,hh),(-hw,hh)]
                poly = QPainterPath()
                for k,(cx,cy) in enumerate(corners + [corners[0]]):
                    lx, ly = cx - ox, cy - oy
                    rx, ry = rot(lx, ly, ang)
                    px, py = x + rx, y + ry
                    poly.moveTo(px, py) if k == 0 else poly.lineTo(px, py)
                self.anim_items["robot"].setPath(poly)

                # Sensors
                for k, s in enumerate(self.robot.sensors):
                    px, py = s.xMM - self.robot.originXMM, s.yMM - self.robot.originYMM
                    rx, ry = rot(px, py, ang)
                    cx, cy = x + rx, y + ry
                    sp = QPainterPath()
                    sp.addRect(cx - 2, cy - 2, 4, 4)
                    if k < len(self.anim_items["sensors"]):
                        self.anim_items["sensors"][k].setPath(sp)

                # Wheels (22x15 mm)
                half_w = WHEEL_W_MM * 0.5
                half_h = WHEEL_H_MM * 0.5
                for k, wdef in enumerate(self.robot.wheels):
                    px, py = wdef.xMM - self.robot.originXMM, wdef.yMM - self.robot.originYMM
                    rx, ry = rot(px, py, ang)
                    cx, cy = x + rx, y + ry
                    corners = [(-half_w, -half_h), ( half_w, -half_h),
                               ( half_w,  half_h), (-half_w,  half_h)]
                    wp = QPainterPath()
                    for i, (lx, ly) in enumerate(corners + [corners[0]]):
                        rlx, rly = rot(lx, ly, ang)
                        vx, vy = cx + rlx, cy + rly
                        if i == 0: wp.moveTo(vx, vy)
                        else:      wp.lineTo(vx, vy)
                    if k < len(self.anim_items["wheels"]):
                        self.anim_items["wheels"][k].setPath(wp)

            self.anim_idx += 1

        sim_t_s = (self.anim_idx * 0.001)
        self.lbl_time.setText("Sim time: {:.2f} s | Anim speed: {:.1f}×".format(sim_t_s, self.anim_speed))

# ---------- Main ----------
def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
