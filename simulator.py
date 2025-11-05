"""Line-Follower Simulator (PySide6 + C backend)

This module provides a minimal GUI to load a track, a robot, and a Python
controller, then simulate the robot following the line. It uses a small C
library (linesim.dll) for the heavy geometry/physics and draws results with Qt.

Comments and docstrings are written in plain English to help beginners.
"""
from __future__ import annotations
import os, sys, math, json, csv, ctypes, importlib.util, random, time, zlib
from dataclasses import dataclass
from typing import List, Tuple, Dict, Any, Optional

from PySide6.QtCore import Qt, QPointF, QThread, Signal, QTimer, QElapsedTimer
from PySide6.QtGui import QPen, QColor, QPainterPath, QPainter
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QFormLayout, QPushButton,
    QFileDialog, QDoubleSpinBox, QLabel, QSplitter, QGraphicsView,
    QGraphicsScene, QMessageBox, QComboBox, QCheckBox, QDialog,
    QDialogButtonBox, QVBoxLayout, QHBoxLayout, QSpinBox, QSizePolicy
)

from ctypes import c_double, c_int, POINTER

class CPoint(ctypes.Structure):
    """ctypes struct that holds a 2D point (x, y) in millimeters for the C API."""
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

_linesim.step_dynamics_C = getattr(_linesim, "step_dynamics_C")
_linesim.step_dynamics_C.argtypes = [
    c_double, c_double, c_double,
    c_double, c_double,
    c_int, c_int,
    c_double, c_double, c_double, c_double,
    POINTER(c_double), POINTER(c_double), POINTER(c_double),
    POINTER(c_double), POINTER(c_double),
    POINTER(c_double), POINTER(c_double)
]
_linesim.step_dynamics_C.restype = None

try:
    _linesim.envelope_contacts_raster_C.argtypes = [
        c_double, c_double, c_double,
        c_double, c_double,
        ctypes.POINTER(ctypes.c_ubyte), c_int, c_int,
        c_double, c_double, c_double
    ]
    _linesim.envelope_contacts_raster_C.restype = c_int
except Exception:
    pass



WHEEL_W_MM = 22.0
WHEEL_H_MM = 15.0
WHEEL_PEN   = QPen(QColor("#000000"), 1)
WHEEL_BRUSH = QColor("#dddddd")

@dataclass(slots=True)
class Pt:
    """Lightweight 2D point used by the Python code (millimeters)."""
    x: float
    y: float

@dataclass(slots=True)
class Pose:
    """Robot pose with a point (x, y) and a heading in degrees."""
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

@dataclass(slots=True)
class SegStraight:
    """Track straight segment starting at from_pose with a given length (mm)."""
    kind: str
    id: str
    lengthMM: float
    from_pose: Pose

@dataclass(slots=True)
class SegArc:
    """Track circular arc segment starting at from_pose with radius (mm) and sweep (deg)."""
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

START_FINISH_GAP_MM = 1000.0
STRAIGHT_NEAR_XING_MM = 250.0
MARKER_OFFSET_MM = 40.0
MARKER_LENGTH_MM = 40.0
MARKER_THICKNESS_MM = 20.0

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

def ensure_track_raster(track_path: str, track_json: dict, segs: list, tapeW: float, gates) -> dict:
    rpath = _raster_paths_for_track(track_path)
    cached = _rmap_load(rpath)
    if cached is not None:
        meta, data = cached
        return {"path": rpath, "meta": meta, "data": data}

    pts = segments_polyline(segs, step=1.0)
    markers = SimWorker.build_markers(self=None, segs=segs, tapeW=tapeW, gates=gates) if hasattr(SimWorker, "build_markers") else []

    xs = [p.x for p in pts] + [m[0] for m in markers]
    ys = [p.y for p in pts] + [m[1] for m in markers]
    if not xs or not ys:
        raise RuntimeError("Invalid track geometry for rasterization.")
    minx = math.floor(min(xs) - 80.0)
    miny = math.floor(min(ys) - 80.0)
    maxx = math.ceil(max(xs) + 80.0)
    maxy = math.ceil(max(ys) + 80.0)

    W = int(maxx - minx)
    H = int(maxy - miny)
    origin_x = float(minx)
    origin_y = float(miny)
    pixel_mm = 1.0

    buf = bytearray(W * H)

    half = tapeW * 0.5
    if len(pts) >= 2:
        for i in range(len(pts) - 1):
            x1, y1 = pts[i].x, pts[i].y
            x2, y2 = pts[i+1].x, pts[i+1].y
            mnx = math.floor(min(x1, x2) - half); mxx = math.ceil(max(x1, x2) + half)
            mny = math.floor(min(y1, y2) - half); mxy = math.ceil(max(y1, y2) + half)
            ix0 = max(0, int(mnx - origin_x)); iy0 = max(0, int(mny - origin_y))
            ix1 = min(W-1, int(mxx - origin_x)); iy1 = min(H-1, int(mxy - origin_y))
            dx = x2 - x1; dy = y2 - y1
            segL2 = dx*dx + dy*dy or 1e-9
            for py in range(iy0, iy1+1):
                wy = origin_y + (py + 0.5) * pixel_mm
                for px in range(ix0, ix1+1):
                    wx = origin_x + (px + 0.5) * pixel_mm
                    t = ((wx - x1)*dx + (wy - y1)*dy) / segL2
                    if t < 0.0:  qx, qy = x1, y1
                    elif t > 1.0: qx, qy = x2, y2
                    else:        qx, qy = x1 + t*dx, y1 + t*dy
                    ddx = wx - qx; ddy = wy - qy
                    if (ddx*ddx + ddy*ddy) <= (half*half):
                        buf[py*W + px] = 255

    halfL = MARKER_LENGTH_MM * 0.5
    halfW = MARKER_THICKNESS_MM * 0.5
    for (cx, cy, ux, uy, vx, vy, hL, hW) in markers:
        corners = [
            (cx - ux*hL - vx*hW, cy - uy*hL - vy*hW),
            (cx + ux*hL - vx*hW, cy + uy*hL - vy*hW),
            (cx + ux*hL + vx*hW, cy + uy*hL + vy*hW),
            (cx - ux*hL + vx*hW, cy - uy*hL + vy*hW),
        ]
        mnx = math.floor(min(p[0] for p in corners)); mxx = math.ceil(max(p[0] for p in corners))
        mny = math.floor(min(p[1] for p in corners)); mxy = math.ceil(max(p[1] for p in corners))
        ix0 = max(0, int(mnx - origin_x)); iy0 = max(0, int(mny - origin_y))
        ix1 = min(W-1, int(mxx - origin_x)); iy1 = min(H-1, int(mxy - origin_y))
        for py in range(iy0, iy1+1):
            wy = origin_y + (py + 0.5) * pixel_mm
            for px in range(ix0, ix1+1):
                wx = origin_x + (px + 0.5) * pixel_mm
                dx = wx - cx; dy = wy - cy
                t = dx*ux + dy*uy
                w = dx*vx + dy*vy
                if (abs(t) <= hL) and (abs(w) <= hW):
                    buf[py*W + px] = 255

    meta = {"origin_x": origin_x, "origin_y": origin_y, "W": W, "H": H, "pixel_mm": pixel_mm}
    _rmap_save(rpath, meta, bytes(buf))
    return {"path": rpath, "meta": meta, "data": bytes(buf)}

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
    """Compute Start and Finish gate lines from the track definition (if enabled)."""
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

PARAMS_JSON_PATH = os.path.join(_here, "simulation_parameters.json")

DEFAULT_SIM_PARAMS = {
    "final_linear_speed_mps": 2.0,
    "motor_time_constant_s": 0.010,
    "simulation_step_dt_ms": 1.0,
    "sensor_mode": "analog",
    "value_of_line": 0,
    "value_of_background": 255,
    "analog_variation": 50
}

def load_sim_params() -> dict:
    try:
        with open(PARAMS_JSON_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
        out = DEFAULT_SIM_PARAMS.copy()
        out.update({k: data.get(k, out[k]) for k in out.keys()})
        return out
    except Exception:
        return DEFAULT_SIM_PARAMS.copy()

def save_sim_params(params: dict) -> None:
    p = DEFAULT_SIM_PARAMS.copy()
    p.update(params or {})
    p["final_linear_speed_mps"] = max(0.1, min(20.0, float(p["final_linear_speed_mps"])))
    p["motor_time_constant_s"]  = max(0.001, min(0.100, float(p["motor_time_constant_s"])))
    p["simulation_step_dt_ms"] = max(0.5,  min(100.0, float(p["simulation_step_dt_ms"])))
    p["sensor_mode"]            = "digital" if str(p.get("sensor_mode","analog")).lower().startswith("d") else "analog"
    p["value_of_line"]          = int(max(0, min(255, int(p["value_of_line"]))))
    p["value_of_background"]         = int(max(0, min(255, int(p["value_of_background"]))))
    p["analog_variation"]       = int(max(0, min(255, int(p["analog_variation"]))))
    with open(PARAMS_JSON_PATH, "w", encoding="utf-8") as f:
        json.dump(p, f, indent=2, ensure_ascii=False)


class SimulationParamsDialog(QDialog):
    """Small dialog to view/edit and persist simulation parameters to JSON."""
    def __init__(self, parent=None):
        """Set up references, parameters, and optional logger for a simulation run."""
        super().__init__(parent)
        self.setWindowTitle("Simulation parameters")
        self.setModal(True)

        params = load_sim_params()

        root = QVBoxLayout(self)

        row1 = QHBoxLayout()
        lbl1 = QLabel("Final linear speed (m/s)")
        self.ed_vf = QDoubleSpinBox()
        self.ed_vf.setRange(0.1, 20.0)
        self.ed_vf.setSingleStep(0.1)
        self.ed_vf.setValue(float(params["final_linear_speed_mps"]))
        row1.addWidget(lbl1); row1.addWidget(self.ed_vf)
        root.addLayout(row1)

        row2 = QHBoxLayout()
        lbl2 = QLabel("Motor time constant τ (s)")
        self.ed_tau = QDoubleSpinBox()
        self.ed_tau.setRange(0.001, 0.100)
        self.ed_tau.setDecimals(3)
        self.ed_tau.setSingleStep(0.001)
        self.ed_tau.setValue(float(params["motor_time_constant_s"]))
        row2.addWidget(lbl2); row2.addWidget(self.ed_tau)
        root.addLayout(row2)

        row3 = QHBoxLayout()
        lbl3 = QLabel("Simulation step dt (ms)")
        self.ed_dt = QDoubleSpinBox()
        self.ed_dt.setRange(0.5, 100.0)
        self.ed_dt.setDecimals(1)
        self.ed_dt.setSingleStep(0.5)
        self.ed_dt.setValue(float(params["simulation_step_dt_ms"]))
        row3.addWidget(lbl3); row3.addWidget(self.ed_dt)
        root.addLayout(row3)

        row4 = QHBoxLayout()
        lbl4 = QLabel("Sensors type")
        self.combo_sensor = QComboBox()
        self.combo_sensor.addItems(["analog", "digital"])
        idx = 1 if str(params["sensor_mode"]).lower().startswith("d") else 0
        self.combo_sensor.setCurrentIndex(idx)
        row4.addWidget(lbl4); row4.addWidget(self.combo_sensor)
        root.addLayout(row4)

        row5 = QHBoxLayout()
        lbl5 = QLabel("Value of line (0..255)")
        self.sp_line = QSpinBox()
        self.sp_line.setRange(0, 255)
        self.sp_line.setValue(int(params["value_of_line"]))
        row5.addWidget(lbl5); row5.addWidget(self.sp_line)
        root.addLayout(row5)

        row6 = QHBoxLayout()
        lbl6 = QLabel("Value of table (0..255)")
        self.sp_table = QSpinBox()
        self.sp_table.setRange(0, 255)
        self.sp_table.setValue(int(params["value_of_background"]))
        row6.addWidget(lbl6); row6.addWidget(self.sp_table)
        root.addLayout(row6)

        row7 = QHBoxLayout()
        lbl7 = QLabel("Analog variation (amplitude)")
        self.sp_vari = QSpinBox()
        self.sp_vari.setRange(0, 255)
        self.sp_vari.setValue(int(params["analog_variation"]))
        row7.addWidget(lbl7); row7.addWidget(self.sp_vari)
        root.addLayout(row7)

        btns = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel, parent=self)
        btns.accepted.connect(self._on_save)
        btns.rejected.connect(self.reject)
        root.addWidget(btns)

    def _on_save(self):
        data = {
            "final_linear_speed_mps": float(self.ed_vf.value()),
            "motor_time_constant_s":  float(self.ed_tau.value()),
            "simulation_step_dt_ms": float(self.ed_dt.value()),
            "sensor_mode":            str(self.combo_sensor.currentText()).lower(),
            "value_of_line":          int(self.sp_line.value()),
            "value_of_background":         int(self.sp_table.value()),
            "analog_variation":       int(self.sp_vari.value())
        }
        save_sim_params(data)
        self.accept()

@dataclass(slots=True)
class Envelope:
    """Robot outer rectangle (width × height) used for collisions and drawing."""
    widthMM: float
    heightMM: float

@dataclass(slots=True)
class Wheel:
    """Wheel definition with id and position in robot coordinates (mm)."""
    id: str
    xMM: float
    yMM: float

@dataclass(slots=True)
class Sensor:
    """Square sensor definition with id, position, and size (mm)."""
    id: str
    xMM: float
    yMM: float
    sizeMM: float = 5.0

@dataclass(slots=True)
class Robot:
    """Robot model grouping envelope, wheels, sensors and grid/origin settings."""
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

def sensor_value_from_coverage(cov: float,
                               sensor_mode: str,
                               value_of_line: int,
                               value_of_background: int,
                               analog_variation: int) -> int:
    cov = max(0.0, min(1.0, float(cov)))
    is_line = (cov >= 0.5)

    mode = ("digital" if str(sensor_mode).lower().startswith("d") else "analog")
    v_line  = int(max(0, min(1023, int(value_of_line))))
    v_table = int(max(0, min(1023, int(value_of_background))))
    amp     = int(max(0, min(1023, int(analog_variation))))

    if mode == "digital":
        base = v_line if is_line else v_table
        return base

    base = v_line if is_line else v_table
    if is_line:
        hi = base
        lo = max(0, base - amp)
        return random.randint(lo, hi)
    else:
        lo = base
        hi = min(1023, base + amp)
        return random.randint(lo, hi)

def load_python_controller(path: str):
    """Load a Python file and return its control_step(state) function."""
    spec = importlib.util.spec_from_file_location("controller_mod", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    if not hasattr(mod, "control_step"):
        raise ValueError("Python controller must define control_step(state)->{'pwm_left','pwm_right'}.")
    return mod.control_step

def import_controller(path: str):
    """Dispatch loader by file extension. Currently only .py is supported."""
    ext = os.path.splitext(path)[1].lower()
    if ext == ".py": return load_python_controller(path)
    raise ValueError("Only .py is supported.")

class SimLogger:
    """Helper that records steps/events and writes CSV + JSON logs under Logs/."""
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

        try:
            with open(self.json_path, "w", encoding="utf-8") as f:
                json.dump({"steps": self.steps, "events": self.events}, f, indent=2, ensure_ascii=False)
        except Exception as e:
            print("JSON log error:", e)

class NoopLogger:
    """Logger stub used when file logging is disabled."""
    def __init__(self):
        self.csv_path = ""
        self.json_path = ""
    def log_step(self, *args, **kwargs):
        return
    def log_event(self, *args, **kwargs):
        return
    def flush(self):
        return

class FinishZoneChecker:
    """Detects valid Start→Finish crossing using a rectangular zone pair."""
    __slots__ = ('s_mid','f_mid','ux','uy','nx','ny','L','HALF_W','EPS','started_inside','last_inside','exited_once','entered_once','armed','last_event','_cross','_sa','_sb','_fa','_fb','_finish_cross_t_ms','_require_envelope_ms')
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

def poly_area(poly):
    """Compute polygon signed area magnitude using the shoelace formula."""
    if len(poly) < 3: return 0.0
    a = 0.0
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i+1) % len(poly)]
        a += x1*y2 - x2*y1
    return abs(a) * 0.5

def suth_hodg_clip(subject, clip):
    """Sutherland–Hodgman polygon clipping of *subject* by convex *clip* polygon."""
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

def point_in_obb(px: float, py: float,
                 cx: float, cy: float,
                 ux: float, uy: float, halfL: float,
                 vx: float, vy: float, halfW: float) -> bool:
    dx = px - cx
    dy = py - cy
    t = dx*ux + dy*uy
    w = dx*vx + dy*vy
    return (abs(t) <= halfL) and (abs(w) <= halfW)

def oriented_rect(cx, cy, ux, uy, halfL, vx, vy, halfW):
    """Return the 4 corners of an oriented rectangle as (x, y) tuples."""
    return [
        (cx - ux*halfL - vx*halfW, cy - uy*halfL - vy*halfW),
        (cx + ux*halfL - vx*halfW, cy + uy*halfL - vy*halfW),
        (cx + ux*halfL + vx*halfW, cy + uy*halfL + vy*halfW),
        (cx - ux*halfL + vx*halfW, cy - uy*halfL + vy*halfW),
    ]

def rect_rect_overlap_area(R1, R2):
    """Compute overlap area between two rectangles by polygon clipping."""
    poly = suth_hodg_clip(R1, R2)
    return poly_area(poly)

class SimWorker(QThread):
    """Background thread that runs the physics loop and streams steps to the UI."""
    sig_chunk = Signal(list)
    sig_done  = Signal(dict)
    sig_fail  = Signal(str)

    def __init__(self, track: Dict[str, Any], robot: Robot, controller_fn,
                params: dict, save_logs: bool, parent=None, track_path: str | None = None):
        super().__init__(parent)
        self.track = track
        self.robot = robot
        self.controller_fn = controller_fn
        self.cancelled = False

        self.v_final = float(params.get("final_linear_speed_mps", 2.0)) * 1000.0
        self.tau     = max(0.001, min(0.100, float(params.get("motor_time_constant_s", 0.01))))
        self.dt_s    = max(0.0005, min(0.1,  float(params.get("simulation_step_dt_ms", 1.0)) / 1000.0))

        self.sensor_mode      = params.get("sensor_mode", "analog")
        self.value_of_line    = int(params.get("value_of_line", 255))
        self.value_of_background   = int(params.get("value_of_background", 0))
        self.analog_variation = int(params.get("analog_variation", 50))

        self.logger = SimLogger(base_dir=_here) if save_logs else NoopLogger()
        self._marker_logged = False
        self.track_path = track_path

    def _initial_pose(self):
        """Choose the starting pose: behind the Start gate if available, otherwise the track origin."""
        segs = getattr(self, "_segs", None)
        origin = getattr(self, "_origin", None)
        tapeW = getattr(self, "_tapeW", None)
        gates = getattr(self, "_gates", None)
        if (segs is None) or (origin is None):
            segs, origin, tapeW = segments_from_json(self.track)
        if gates:
            (sa, sb, shdg_run, shdg_base), _ = gates
            back = (self.robot.envelope.heightMM/2.0) + 250.0
            pose_gate = Pose(Pt((sa.x + sb.x)*0.5, (sa.y + sb.y)*0.5), shdg_run)
            pos = advance_straight(pose_gate, -back)
        else:
            pos = origin
        return pos

    def _sensors_world_xy(self, x, y, h_deg):
        """Return lists of sensor world coordinates (sx, sy) from robot pose and sensor local coordinates."""
        ang = math.radians(h_deg)
        ox, oy = self.robot.originXMM, self.robot.originYMM
        sx, sy = [], []
        for s in self.robot.sensors:
            rx, ry = rot(s.xMM - ox, s.yMM - oy, ang)
            sx.append(x + rx); sy.append(y + ry)
        return sx, sy

    def _coverage_from_raster_batch(self, sx, sy, size_mm, grid_n: int = 3):
        """Return fractional coverage [0..1] for each sensor by sampling the 1mm raster."""
        meta = getattr(self, "_rmap_meta", None)
        data = getattr(self, "_rmap_data", None)
        if not meta or not data:
            return [0.0 for _ in sx]
        W = int(meta["W"]); H = int(meta["H"])
        origin_x = float(meta["origin_x"]); origin_y = float(meta["origin_y"])
        pix = float(meta.get("pixel_mm", 1.0)) or 1.0
        mv = memoryview(data)
        half = float(size_mm) * 0.5
        if grid_n <= 1:
            offs = [(0.0, 0.0)]
        else:
            step = (2.0*half)/(grid_n-1)
            offs = [(i*step-half, j*step-half) for j in range(grid_n) for i in range(grid_n)]
        denom = float(len(offs))
        out = []
        for cx, cy in zip(sx, sy):
            k = 0
            for dx, dy in offs:
                wx = cx + dx; wy = cy + dy
                px = int((wx - origin_x) // pix)
                py = int((wy - origin_y) // pix)
                if 0 <= px < W and 0 <= py < H:
                    if mv[py*W + px] != 0:
                        k += 1
            out.append(k/denom if denom > 0 else 0.0)
        return out


    def _estimate_coverage_batch(self, sx, sy, x, y, h_deg, tape_half, sensor_half_unused):
        """Fallback to C function that estimates coverage along the polyline (if available)."""
        try:
            N = len(sx)
            xs = (c_double * N)(*sx)
            ys = (c_double * N)(*sy)
            out = (c_double * N)()
            if self.robot.sensors:
                size_default = float(self.robot.sensors[0].sizeMM)
            else:
                size_default = 5.0
            _linesim.estimate_sensors_coverage_batch_C(
                xs, ys, N,
                self._poly_ptr, self._poly_n,
                c_double(tape_half),
                None, c_double(size_default),
                c_int(3),
                out
            )
            return [out[i] for i in range(N)]
        except Exception:
            return [0.0 for _ in sx]

    def _apply_marker_overrides(self, sx, sy, base_cov):
        """Force coverage=1.0 when a sensor lies on a marker OBB (used for visualization cues)."""
        if not self._markers_obb:
            return base_cov
        out = list(base_cov)
        for i in range(len(sx)):
            px, py = sx[i], sy[i]
            for (mcx, mcy, mux, muy, mvx, mvy, mhalfL, mhalfW) in self._markers_obb:
                if point_in_obb(px, py, mcx, mcy, mux, muy, mhalfL, mvx, mvy, mhalfW):
                    out[i] = 1.0
                    break
        return out

    def envelope_contacts_tape(self, x, y, h_deg, tape_half_with_margin) -> bool:
        cx = x - self.robot.originXMM
        cy = y - self.robot.originYMM
        h_rad = math.radians(h_deg)
        hit = _linesim.envelope_contacts_tape_C(
            cx, cy,
            h_rad,
            self.robot.envelope.widthMM,
            self.robot.envelope.heightMM,
            self._poly_ptr, self._poly_n,
            tape_half_with_margin,
            0
        )
        return bool(hit)

    def build_markers(self, segs, tapeW, gates):
        """Build oriented rectangles (position and axes) used to draw/override curvature markers and gates."""
        rects = []
        halfL = MARKER_LENGTH_MM * 0.5
        halfW = MARKER_THICKNESS_MM * 0.5

        for (pp, hdg) in curvature_change_markers(segs):
            a = math.radians(hdg)
            tx, ty = math.cos(a), math.sin(a)
            nx, ny = math.sin(a), -math.cos(a)
            base = (tapeW*0.5) + MARKER_OFFSET_MM
            cx, cy = pp.x + nx*base, pp.y + ny*base
            rects.append((cx, cy, nx, ny, tx, ty, halfL, halfW))

        if gates:
            (sa, sb, shdg_run, shdg_base), (fa, fb, fhdg_run, fhdg_base) = gates

            def add_right_rect(pa, pb, base_hdg):
                a = math.radians(base_hdg)
                tx, ty = math.cos(a), math.sin(a)
                nx, ny = -math.sin(a), math.cos(a)
                mx, my = (pa.x + pb.x)*0.5, (pa.y + pb.y)*0.5
                base = (tapeW*0.5) + MARKER_OFFSET_MM
                cx, cy = mx + nx*base, my + ny*base
                rects.append((cx, cy, nx, ny, tx, ty, halfL, halfW))

            add_right_rect(sa, sb, shdg_base)
            add_right_rect(fa, fb, fhdg_base)

        return rects

    def _prepare_track_geometry(self):
        """Create C arrays for track polyline, precompute gates/markers, and optional raster cache."""
        segs, origin, tapeW = segments_from_json(self.track)
        pts = segments_polyline(segs, step=1.0)
        n = len(pts)
        poly_arr = (CPoint * n)(*[(CPoint(p.x, p.y)) for p in pts])
        self._poly_ptr, self._poly_n = poly_arr, n
        self._segs, self._origin, self._tapeW = segs, origin, tapeW
        self._gates = start_finish_lines(self.track, segs, tapeW)
        self._markers_obb = self.build_markers(segs, tapeW, self._gates)

        if self.track_path:
            info = ensure_track_raster(self.track_path, self.track, segs, tapeW, self._gates)
            self._rmap_meta = info["meta"]
            self._rmap_data = info["data"]
            self._rmap_arr = (ctypes.c_ubyte * len(self._rmap_data)).from_buffer_copy(self._rmap_data)
            self._rmap_ptr = ctypes.cast(self._rmap_arr, ctypes.POINTER(ctypes.c_ubyte))
        else:
            self._rmap_meta = None
            self._rmap_data = b""
            self._rmap_ptr = None

        return segs, origin, tapeW

    def run(self):
        """Main simulation loop: query controller, integrate dynamics via C API, stream steps, and stop on finish/collision."""
        try:
            segs, origin, tapeW = self._prepare_track_geometry()
            tape_half = float(tapeW) * 0.5

            start_pose = self._initial_pose()
            x, y, h = start_pose.p.x, start_pose.p.y, float(start_pose.headingDeg)

            vL = 0.0; vR = 0.0; v = 0.0; w = 0.0
            prev_v = 0.0; prev_w = 0.0
            trackW = abs(self.robot.wheels[-1].yMM - self.robot.wheels[0].yMM) if len(self.robot.wheels) >= 2 else 120.0  # use lateral (Y) distance between wheels
            dt = float(self.dt_s)
            t_ms = 0

            zone = None
            if self._gates:
                (sa, sb, shdg_run, shdg_base), (fa, fb, *_rest) = self._gates
                zone = FinishZoneChecker(sa, sb, fa, fb)
                zone.prime(x, y)

            CHUNK = 200
            chunk_buf = []

            env_w = float(self.robot.envelope.widthMM)
            env_h = float(self.robot.envelope.heightMM)
            tape_half_with_margin = tape_half

            sensor_half = float(self.robot.sensors[0].sizeMM) * 0.5 if self.robot.sensors else 2.5

            while not self.cancelled:
                sx, sy = self._sensors_world_xy(x, y, h)
                cov = self._coverage_from_raster_batch(sx, sy, sensor_half*2.0, grid_n=3)
                sn_vals = [sensor_value_from_coverage(
                    cov[i], self.sensor_mode, self.value_of_line, self.value_of_background, self.analog_variation
                ) for i in range(len(cov))]

                a_lin = (v - prev_v) / max(1e-9, dt)
                a_ang = (w - prev_w) / max(1e-9, dt)

                state = {
                    "t_ms": t_ms,
                    "x_mm": x, "y_mm": y, "heading_deg": h,
                    "v_mm_s": v, "omega_rad_s": w,
                    "a_lin_mm_s2": a_lin, "alpha_rad_s2": a_ang,
                    "sensors": sn_vals,
                    "v_left_mm_s": vL, "v_right_mm_s": vR,
                }

                try:
                    out = self.controller_fn(state)
                    pwmL = int(out.get("pwm_left", 1500)) if isinstance(out, dict) else 1500
                    pwmR = int(out.get("pwm_right", 1500)) if isinstance(out, dict) else 1500
                except Exception as e:
                    print(f"[Controller Error] {e}")
                    pwmL, pwmR = 1500, 1500

                ox = c_double(); oy = c_double(); oh = c_double()
                o_vL = c_double(); o_vR = c_double(); o_v = c_double(); o_w = c_double()
                prev_v, prev_w = v, w
                _linesim.step_dynamics_C(
                    c_double(x), c_double(y), c_double(h),
                    c_double(vL), c_double(vR),
                    c_int(pwmL), c_int(pwmR),
                    c_double(self.v_final), c_double(self.tau), c_double(trackW), c_double(dt),
                    ctypes.byref(ox), ctypes.byref(oy), ctypes.byref(oh),
                    ctypes.byref(o_vL), ctypes.byref(o_vR),
                    ctypes.byref(o_v), ctypes.byref(o_w)
                )
                prev_pose = (x, y, h)
                x, y, h = ox.value, oy.value, oh.value
                vL, vR, v, w = o_vL.value, o_vR.value, o_v.value, o_w.value


                try:
                    if self._rmap_ptr and self._rmap_meta:
                        cx = x - self.robot.originXMM
                        cy = y - self.robot.originYMM
                        hit = _linesim.envelope_contacts_raster_C(
                            c_double(cx), c_double(cy), c_double(math.radians(h)),
                            c_double(env_w), c_double(env_h),
                            self._rmap_ptr, c_int(self._rmap_meta["W"]), c_int(self._rmap_meta["H"]),
                            c_double(self._rmap_meta["origin_x"]), c_double(self._rmap_meta["origin_y"]), c_double(self._rmap_meta["pixel_mm"])
                        )
                    else:
                        hit = 1
                except Exception:
                    hit = 1

                finished = False
                if zone is not None:
                    finished = zone.update(prev_pose, (x, y, h), env_w, env_h, t_ms)

                step = {
                    "t_ms": t_ms,
                    "x_mm": x, "y_mm": y, "heading_deg": h,
                    "v_mm_s": v, "omega_rad_s": w,
                    "a_lin_mm_s2": a_lin, "alpha_rad_s2": a_ang,
                    "sensors": sn_vals,
                    "v_left_mm_s": vL, "v_right_mm_s": vR,
                }
                self.logger.log_step(t_ms, x, y, h, v, w, pwmL, pwmR, sensors=sn_vals)
                chunk_buf.append(step)
                if len(chunk_buf) >= CHUNK:
                    self.sig_chunk.emit(chunk_buf)
                    chunk_buf = []
                t_ms += int(round(dt * 1000.0))
                if finished or (self._rmap_ptr and not hit):
                    break
                if t_ms > 100000:
                    break

            if chunk_buf:
                self.sig_chunk.emit(chunk_buf)
            self.logger.flush()
            self.sig_done.emit({"dt_s": self.dt_s})
        except Exception as e:
            try:
                self.sig_fail.emit(str(e))
            except Exception:
                pass

class SimScene(QGraphicsScene):
    """QGraphicsScene that draws the background grid and simulation items."""
    def __init__(self):
        super().__init__()
        self.setBackgroundBrush(QColor("#0c0c0c"))

    def drawBackground(self, painter: QPainter, rect):
        """Draw a dark grid to help with spatial orientation (no heavy painting)."""
        super().drawBackground(painter, rect)
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
    """QGraphicsView with smooth zoom/pan suitable for inspecting the scene."""
    def __init__(self, scene: SimScene):
        super().__init__(scene)
        self.setRenderHints(self.renderHints() | QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setViewportUpdateMode(QGraphicsView.BoundingRectViewportUpdate)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)

    def wheelEvent(self, e):
        """Zoom in/out around the mouse position using the wheel delta; accept the event."""
        s = 1.15 if e.angleDelta().y() > 0 else 1/1.15
        self.scale(s, s)
        e.accept()

class MainWindow(QMainWindow):
    """Main GUI: loads files, starts simulation, and replays results."""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Line-Follower Simulator")

        self.track: Optional[Dict[str, Any]] = None
        self.robot: Optional[Robot] = None
        self.controller_fn = lambda state: {"pwm_left": 2000, "pwm_right": 2000}

        self.scene = SimScene()
        self.view = SimView(self.scene)

        controls = QWidget(); form = QFormLayout(controls)

        self.anim_interval_ms = 42
        self.stream_draw_interval_ms = 200
        self._last_stream_draw_ns = 0
        self.streaming = False

        self.anim_speed = 1.0
        self.anim_spf = 17
        self.anim_idx = 0
        self.anim_steps: List[Dict[str, Any]] = []
        self.anim_items: Dict[str, Any] = {}

        self.worker: Optional[SimWorker] = None
        self.v_max_mm_s = None

        self.controller_path = None
        self.lbl_ctrl_status = QLabel("—")
        self.lbl_ctrl_status.setStyleSheet("color: #aaaaaa; font-weight: 500;")

        self.btn_track = QPushButton("Load track (.json)")
        self.btn_track.setToolTip("Select a track file (.json).")
        self.btn_robot = QPushButton("Load robot (.json)")
        self.btn_robot.setToolTip("Select a robot description (.json).")
        self.btn_ctrl  = QPushButton("Load controller (.py)")
        self.btn_ctrl.setToolTip("Select a Python file implementing control_step(state).")
        self.btn_params = QPushButton("Simulation parameters")
        self.btn_params.setToolTip("Open a dialog to edit and persist simulation parameters to simulation_parameters.json.")
        self.spin_vf   = QDoubleSpinBox(); self.spin_vf.setRange(0.1, 20.0); self.spin_vf.setValue(2.0); self.spin_vf.setSingleStep(0.1)
        self.spin_tau  = QDoubleSpinBox(); self.spin_tau.setRange(0.001, 0.100); self.spin_tau.setDecimals(3); self.spin_tau.setSingleStep(0.001); self.spin_tau.setValue(0.010)

        self.spin_dt  = QDoubleSpinBox()
        self.spin_dt.setRange(0.5, 100.0)
        self.spin_dt.setDecimals(1)
        self.spin_dt.setSingleStep(0.5)
        self.spin_dt.setValue(1.0)
        self.sim_dt_s = float(self.spin_dt.value()) / 1000.0

        self.btn_sim   = QPushButton("Start")
        self.btn_sim.setToolTip("Start a new simulation with the currently loaded track, robot and controller.")
        self.btn_stop  = QPushButton("Stop")
        self.btn_stop.setToolTip("Stop the running simulation.")
        self.btn_replay= QPushButton("Play")
        self.btn_replay.setToolTip("Play the last finished simulation steps.")

        self.lbl_time = QLabel("Sim time: 0.00 s")
        self.lbl_time.setToolTip("Simulation time in seconds based on dt × steps. Updates only during live streaming or replay.")

        self.chk_log = QCheckBox("Save logs to file (CSV+JSON)")
        self.chk_log.setToolTip("If enabled, write CSV + JSON logs under the Logs/ folder for each run.")

        self.combo_speed = QComboBox()
        self.combo_speed.addItems(["0.1×", "0.5×", "1×", "2×", "4×"])
        self.combo_speed.setToolTip("Playback speed for visualization only. It does not affect the physics or logged data.")
        self.combo_speed.setCurrentIndex(2)
        self.combo_speed.currentIndexChanged.connect(self.on_speed_change)
        self.on_speed_change(self.combo_speed.currentIndex())

        form.addRow(QLabel("<b>Simulation Files</b>"))

        form.addRow(self.btn_params)

        # Controller row: button + status
        self.lbl_ctrl_status = QLabel("—")
        self.lbl_ctrl_status.setStyleSheet("color: #aaaaaa; font-weight: 500;")
        self.lbl_ctrl_status.setToolTip("Loaded controller file.")
        row_ctrl = QWidget()
        row_ctrl_layout = QHBoxLayout(row_ctrl); row_ctrl_layout.setContentsMargins(0,0,0,0)
        row_ctrl_layout.addWidget(self.btn_ctrl)
        row_ctrl_layout.addWidget(self.lbl_ctrl_status)
        form.addRow(row_ctrl)

        # Robot row: button + status
        self.lbl_robot_status = QLabel("—")
        self.lbl_robot_status.setStyleSheet("color: #aaaaaa; font-weight: 500;")
        self.lbl_robot_status.setToolTip("Loaded robot file.")
        row_robot = QWidget()
        row_robot_layout = QHBoxLayout(row_robot); row_robot_layout.setContentsMargins(0,0,0,0)
        row_robot_layout.addWidget(self.btn_robot)
        row_robot_layout.addWidget(self.lbl_robot_status)
        form.addRow(row_robot)

        self.lbl_track_status = QLabel("—")
        self.lbl_track_status.setStyleSheet("color: #aaaaaa; font-weight: 500;")
        self.lbl_track_status.setToolTip("Loaded track file.")
        row_track = QWidget()
        row_track_layout = QHBoxLayout(row_track); row_track_layout.setContentsMargins(0,0,0,0)
        row_track_layout.addWidget(self.btn_track)
        row_track_layout.addWidget(self.lbl_track_status)
        form.addRow(row_track)
        form.addRow(self.chk_log)
        form.addRow(QLabel("<b>Simulation Control</b>"))
        self.step_count = 0
        self.lbl_progress = QLabel("Executed steps: 0")
        self.lbl_progress.setToolTip("Number of simulation steps computed so far. During replay this shows the total steps generated.")
        form.addRow(self.lbl_progress)
        ctrl_row = QWidget()
        ctrl_layout = QHBoxLayout(ctrl_row)
        ctrl_layout.setContentsMargins(0, 0, 0, 0)
        ctrl_layout.addWidget(self.btn_sim)
        ctrl_layout.addWidget(self.btn_stop)
        form.addRow(ctrl_row)

        form.addRow(QLabel("<b>Replay</b>"))

        self.lbl_time = QLabel("Sim time: 0.00 s")
        self.lbl_time.setToolTip("Simulation time in seconds from replay/stream only.")
        replay_info_row = QWidget()
        replay_info_layout = QHBoxLayout(replay_info_row); replay_info_layout.setContentsMargins(0,0,0,0)
        replay_info_layout.addWidget(self.lbl_time)
        replay_info_layout.addWidget(QLabel("Animation speed"))
        replay_info_layout.addWidget(self.combo_speed)
        form.addRow(replay_info_row)

        self.btn_replay_stop = QPushButton("Stop")
        self.btn_replay_stop.setToolTip("Stop the current replay playback.")
        replay_ctrl_row = QWidget()
        replay_ctrl_layout = QHBoxLayout(replay_ctrl_row); replay_ctrl_layout.setContentsMargins(0,0,0,0)
        replay_ctrl_layout.addWidget(self.btn_replay)
        replay_ctrl_layout.addWidget(self.btn_replay_stop)
        form.addRow(replay_ctrl_row)

        self.btn_params.clicked.connect(self.on_open_params)
        self.btn_stop.setEnabled(False)
        self.btn_replay.setEnabled(False)
        self.is_replaying = False
        self.update_replay_buttons()
        self.btn_replay_stop.setEnabled(False)
        self.update_replay_buttons()

        splitter = QSplitter()
        splitter.addWidget(self.view)
        splitter.addWidget(controls)
        controls.setMinimumWidth(320)
        controls.setMaximumWidth(420)
        controls.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 0)
        self.setCentralWidget(splitter)
        self.resize(1280, 800)

        self.btn_track.clicked.connect(self.on_load_track)
        self.btn_robot.clicked.connect(self.on_load_robot)
        self.btn_ctrl.clicked.connect(self.on_load_controller)
        self.btn_sim.clicked.connect(self.on_simulate)
        self.btn_stop.clicked.connect(self.on_stop)
        self.btn_replay.clicked.connect(self.on_replay)
        self.btn_replay_stop.clicked.connect(self.on_stop_replay)

        self.timer = QTimer()
        self.timer.timeout.connect(self.tick)
        self._replay_elapsed = QElapsedTimer()
        self._replay_elapsed.start()
        self._sim_time_acc_s = 0.0
        self.anim_idx = 0
        self.is_replaying = False
        self.update_replay_buttons()
        self.btn_replay.setEnabled(False)
        self.btn_replay_stop.setEnabled(False)
        self.update_replay_buttons()

        self.spin_dt.valueChanged.connect(self._refresh_title)


        # Replay state flag
        self.is_replaying = False

    def update_replay_buttons(self):
        """Enable/disable replay buttons depending on current state."""
        has_steps = bool(self.anim_steps)
        self.btn_replay.setEnabled(has_steps and not self.is_replaying and not self.streaming)
        self.btn_replay_stop.setEnabled(self.is_replaying)
        self._refresh_title()

    def on_open_params(self):
        """Open the parameters dialog, then refresh spin boxes and window title."""
        dlg = SimulationParamsDialog(self)
        if dlg.exec() == QDialog.Accepted:
            p = load_sim_params()
            self.spin_vf.setValue(float(p["final_linear_speed_mps"]))
            self.spin_tau.setValue(float(p["motor_time_constant_s"]))
            self.spin_dt.setValue(float(p["simulation_step_dt_ms"]))
            self.sim_dt_s = float(p["simulation_step_dt_ms"]) / 1000.0
            self._refresh_title()
            QMessageBox.information(self, "Simulation parameters", "Parameters saved to simulation_parameters.json")
            try:
                self.on_speed_change(self.combo_speed.currentIndex())
            except Exception:
                pass

    def _refresh_title(self):
        """Show the current dt (ms) in the window title for quick reference."""
        self.setWindowTitle(f"Line-Follower Simulator (dt = {self.sim_dt_s*1000.0:.1f} ms)")

    def _draw_robot_at_initial_pose(self):
        """Draw the robot body/wheels/sensors at the initial pose onto the scene."""
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

        for k, s in enumerate(self.robot.sensors):
            px, py = s.xMM - ox, s.yMM - oy
            rx, ry = rot(px, py, ang)
            cx, cy = pos.p.x + rx, pos.p.y + ry
            sp = QPainterPath()
            sp.addRect(cx - 2, cy - 2, 4, 4)
            if k < len(self.anim_items["sensors"]):
                self.anim_items["sensors"][k].setPath(sp)

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

    def on_speed_change(self, idx: int):
        """Map speed combo to playback speed and compute steps-per-frame for replay."""
        mapping = {0: 0.1, 1: 0.5, 2: 1.0, 3: 2.0, 4: 4.0}
        self.anim_speed = mapping.get(idx, 1.0)
        steps_per_sec = 1.0 / max(1e-6, self.sim_dt_s)
        fps = 1000.0 / max(1.0, float(self.anim_interval_ms))
        self.anim_spf = max(1, int(round((steps_per_sec * self.anim_speed) / fps)))
        i = self.anim_idx if self.anim_steps else 0

    def on_load_track(self):
        """Read a track JSON, build raster cache, and draw the static track."""
        path, _ = QFileDialog.getOpenFileName(self, "Track file", "", "JSON (*.json)")
        if not path: return
        with open(path, "r", encoding="utf-8") as f:
            self.track = json.load(f)
        self.track_path = path
        segs, origin, tapeW = segments_from_json(self.track)
        gates = start_finish_lines(self.track, segs, tapeW)
        _ = ensure_track_raster(self.track_path, self.track, segs, tapeW, gates)
        self.draw_static_track()
        if self.robot: self.draw_robot_outline_preview()
        base = os.path.basename(path)
        if hasattr(self, "lbl_track_status"):
            self.lbl_track_status.setText(f"{base}   ✓")
            self.lbl_track_status.setStyleSheet("color: #2e7d32; font-weight: 600;")
            self.lbl_track_status.setToolTip(path)
        self.statusBar().showMessage(f"Track: {base}")

    def on_load_robot(self):
        """Read a robot JSON and update preview widgets; show errors if parsing fails."""
        path, _ = QFileDialog.getOpenFileName(self, "Robot file", "", "JSON (*.json)")
        if not path: return
        try:
            with open(path, "r", encoding="utf-8") as f:
                self.robot = robot_from_json(json.load(f))
            base = os.path.basename(path)
            self.statusBar().showMessage(f"Robot: {base}", 5000)
            if hasattr(self, "lbl_robot_status"):
                self.lbl_robot_status.setText(f"{base}   ✓")
                self.lbl_robot_status.setStyleSheet("color: #2e7d32; font-weight: 600;")
                self.lbl_robot_status.setToolTip(path)
            if self.track: self.draw_robot_outline_preview()
        except Exception as e:
            self.robot = None
            QMessageBox.critical(self, "Robot load error",
                                 f"Failed to read '{os.path.basename(path)}':\n{e}")
            try:
                if hasattr(self, "lbl_robot_status"):
                    self.lbl_robot_status.setText("failed")
                    self.lbl_robot_status.setStyleSheet("color: #c62828; font-weight: 600;")
            except Exception:
                pass
            if self.worker and self.worker.isRunning():
                try:
                    self.worker.cancelled = True
                    self.worker.wait(500)
                except Exception:
                    pass

    def on_load_controller(self):
        """Load a Python controller and update the status label with success/failure."""
        path, _ = QFileDialog.getOpenFileName(self, "Controller", "", "Python (*.py)")
        if not path:
            return
        try:
            self.controller_fn = import_controller(path)
            self.controller_path = path

            base = os.path.basename(path)
            self.lbl_ctrl_status.setText(f"{base}   ✓")
            self.lbl_ctrl_status.setStyleSheet("color: #2e7d32; font-weight: 600;")
            self.lbl_ctrl_status.setToolTip(path)
            self.statusBar().showMessage(f"Controller loaded: {base}", 4000)

            self.btn_ctrl.setStyleSheet("background: #e8f5e9;")
            QTimer.singleShot(600, lambda: self.btn_ctrl.setStyleSheet(""))

        except Exception as e:
            self.controller_path = None
            self.lbl_ctrl_status.setText("failed")
            self.lbl_ctrl_status.setStyleSheet("color: #c62828; font-weight: 600;")
            self.statusBar().showMessage("Controller load failed", 5000)
            QMessageBox.critical(self, "Controller load error", str(e))

    def clear_static(self):
        """Remove all static scene items and forget cached QGraphicsPathItems."""
        self.scene.clear()
        self.anim_items.clear()

    def draw_static_track(self):
        """Add the tape polyline and markers to the scene for a loaded track."""
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

        bbox = self.scene.itemsBoundingRect().adjusted(-200, -200, +200, +200)
        self.scene.setSceneRect(bbox)
        self.view.fitInView(bbox, Qt.KeepAspectRatio)

    def draw_robot_outline_preview(self):
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

    def on_simulate(self):
        self.streaming = True
        if not (self.track and self.robot):
            self.streaming = True
            QMessageBox.warning(self, "Missing data", "Load a track and a robot.")
            return

        if self.timer.isActive():
            self.timer.stop()
        self.reset_anim_items()
        self._trail_last_pt = None
        self._draw_robot_at_initial_pose()

        self.anim_steps.clear()
        self.anim_idx = 0
        self.step_count = 0
        self.lbl_progress.setText("Executed steps: 0")

        self.btn_sim.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.btn_replay.setEnabled(False)

        p = load_sim_params()
        self.v_max_mm_s = float(p.get("final_linear_speed_mps", 2.0)) * 1000.0
        self.sim_dt_s = max(0.0005, min(0.1, float(p.get("simulation_step_dt_ms", 1.0)) / 1000.0))

        try:
            self.on_speed_change(self.combo_speed.currentIndex())
        except Exception:
            pass

        save_logs = self.chk_log.isChecked()
        self.worker = SimWorker(
            self.track, self.robot, self.controller_fn,
            params=p,
            save_logs=save_logs,
            track_path=getattr(self, 'track_path', None)
        )
        self.worker.sig_chunk.connect(self.on_stream_chunk)
        self.worker.sig_done.connect(self.on_stream_done)
        self.worker.sig_fail.connect(self.on_stream_fail)
        self.worker.start(QThread.TimeCriticalPriority)

    def _speed_color(self, v_mm_s: float) -> QColor:
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
        self.streaming = False
        self._last_stream_draw_ns = 0
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
        self.is_replaying = False
        self.update_replay_buttons()
        if hasattr(self, "btn_replay_stop"):
            self.btn_replay_stop.setEnabled(False)

    def on_stream_chunk(self, chunk: List[dict]):
        self.anim_steps.extend(chunk)
        self.step_count += len(chunk)
        self.lbl_progress.setText(f"Executed steps: {self.step_count}")
        if not getattr(self, 'streaming', False):
            if not self.timer.isActive():
                self._sim_time_acc_s = 0.0 if self.anim_idx == 0 else self.anim_idx * self.sim_dt_s
                if self._replay_elapsed is None:
                    self._replay_elapsed = QElapsedTimer(); self._replay_elapsed.start()
                else:
                    self._replay_elapsed.restart()
                self.timer.start(self.anim_interval_ms)
        self.is_replaying = False
        self.update_replay_buttons()

    def on_stream_done(self, info: dict):
        self._last_stream_draw_ns = 0
        self.streaming = False
        if self.worker and self.worker.isRunning():
            self.worker.wait(2000)
        self.worker = None

        self.btn_sim.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.is_replaying = False
        self.update_replay_buttons()
        if hasattr(self, "btn_replay_stop"):
            self.btn_replay_stop.setEnabled(False)

        if self.timer.isActive():
            self.timer.stop()
        self.reset_anim_items()
        self.anim_idx = 0
        self._trail_last_pt = None

        if isinstance(info, dict) and "dt_s" in info and isinstance(info["dt_s"], (int, float)):
            self.sim_dt_s = float(info["dt_s"])

        self._sim_time_acc_s = 0.0
        if self._replay_elapsed is None:
            self._replay_elapsed = QElapsedTimer()
            self._replay_elapsed.start()
        else:
            self._replay_elapsed.restart()

        if self.anim_steps:
            self.timer.start(self.anim_interval_ms)
        self.is_replaying = False
        self.update_replay_buttons()

    def on_stream_fail(self, msg: str):
        self.worker = None
        self.btn_stop.setEnabled(False)
        self.btn_sim.setEnabled(True)
        QMessageBox.critical(self, "Error", msg)

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
        self._last_stream_draw_ns = 0
        if not self.anim_steps:
            return
        self.reset_anim_items()
        self.anim_idx = 0
        self._sim_time_acc_s = 0.0
        self.is_replaying = True
        self.update_replay_buttons()
        if self._replay_elapsed is None:
            self._replay_elapsed = QElapsedTimer()
            self._replay_elapsed.start()
        else:
            self._replay_elapsed.restart()
        self.timer.start(self.anim_interval_ms)
        self.is_replaying = False
        self.update_replay_buttons()

    def on_stop_replay(self):
        if self.timer.isActive():
            self.timer.stop()
        self.is_replaying = False
        self.update_replay_buttons()
        if hasattr(self, "btn_replay_stop"):
            self.btn_replay_stop.setEnabled(False)
        self.btn_replay_stop.setEnabled(False)

    def closeEvent(self, event):
        try:
            self.streaming = False
            if self.timer.isActive():
                self.timer.stop()
            if self.worker and self.worker.isRunning():
                self.worker.cancelled = True
                self.worker.wait(2000)
        finally:
            super().closeEvent(event)

    def tick(self):
        if not self.anim_steps:
            self.timer.stop()
            return

        if getattr(self, 'streaming', False):
            return

        if ("trail_items" not in self.anim_items) or ("robot" not in self.anim_items):
            self.reset_anim_items()

        if self._replay_elapsed is None:
            self._replay_elapsed = QElapsedTimer(); self._replay_elapsed.start()
        elapsed_s = self._replay_elapsed.restart() / 1000.0

        self._sim_time_acc_s += elapsed_s * self.anim_speed
        target_ms = self._sim_time_acc_s * 1000.0

        new_idx = self.anim_idx
        n_steps = len(self.anim_steps)
        while new_idx < n_steps:
            step_ms = float(self.anim_steps[new_idx].get("t_ms", new_idx * self.sim_dt_s * 1000.0))
            if step_ms > target_ms:
                break
            new_idx += 1

        if new_idx == self.anim_idx:
            return

        step = self.anim_steps[new_idx - 1]
        x = float(step.get("x_mm", 0.0))
        y = float(step.get("y_mm", 0.0))
        h = float(step.get("heading_deg", 0.0))
        v_now = float(step.get("v_mm_s", 0.0))
        ang = rad(h)

        if self._trail_last_pt is None:
            self._trail_last_pt = (x, y)
        else:
            (px, py) = self._trail_last_pt
            pen = QPen(self._speed_color(v_now), 8, Qt.SolidLine, Qt.RoundCap)
            seg = self.scene.addLine(px, py, x, y, pen)
            self.anim_items.setdefault("trail_items", []).append(seg)
            self._trail_last_pt = (x, y)

        if self.robot:
            hw = self.robot.envelope.widthMM / 2.0
            hh = self.robot.envelope.heightMM / 2.0
            ox, oy = self.robot.originXMM, self.robot.originYMM
            corners = [(-hw,-hh),(hw,-hh),(hw,hh),(-hw,hh)]
            poly = QPainterPath()
            for k,(cx,cy) in enumerate(corners + [corners[0]]):
                lx, ly = cx - ox, cy - oy
                rx, ry = rot(lx, ly, ang)
                px, py = x + rx, y + ry
                poly.moveTo(px, py) if k == 0 else poly.lineTo(px, py)
            self.anim_items["robot"].setPath(poly)

            for k, s in enumerate(self.robot.sensors):
                px, py = s.xMM - self.robot.originXMM, s.yMM - self.robot.originYMM
                rx, ry = rot(px, py, ang)
                cx, cy = x + rx, y + ry
                r = s.sizeMM / 2.0
                sp = QPainterPath(); sp.addEllipse(cx - r, cy - r, 2*r, 2*r)
                if k < len(self.anim_items["sensors"]):
                    self.anim_items["sensors"][k].setPath(sp)

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

        self.anim_idx = new_idx
        if self.anim_idx >= len(self.anim_steps):
            self.timer.stop()
            return

        last_ms = float(self.anim_steps[self.anim_idx-1].get("t_ms", (self.anim_idx-1)*self.sim_dt_s*1000.0))
        self.lbl_time.setText(f"Sim time: {last_ms/1000.0:.2f} s")

def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
