# simulation.py
from __future__ import annotations
import os, sys, math, json, time, ctypes, importlib.util, csv
from dataclasses import dataclass
from typing import List, Tuple, Dict, Any, Optional

from PySide6.QtCore import Qt, QPointF, QRectF, QThread, Signal, QObject, QTimer
from PySide6.QtGui import QPen, QColor, QPainterPath, QPainter
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QFormLayout, QHBoxLayout, QPushButton,
    QFileDialog, QDoubleSpinBox, QLabel, QSplitter, QGraphicsView, QGraphicsScene,
    QProgressBar, QMessageBox, QComboBox
)

# ======================
# Util (geom de pista)
# ======================
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

def deg(radv: float) -> float:
    return math.degrees(radv)

def rot(x: float, y: float, a: float) -> Tuple[float, float]:
    c, s = math.cos(a), math.sin(a)
    return (c*x - s*y, s*x + c*y)

def advance_straight(pose: Pose, d: float) -> Pose:
    a = rad(pose.headingDeg)
    return Pose(Pt(pose.p.x + d*math.cos(a), pose.p.y + d*math.sin(a)), pose.headingDeg)

def advance_arc(pose: Pose, R: float, sweepDeg: float) -> Pose:
    a0 = rad(pose.headingDeg)
    # centro do giro (esquerda)
    cx = pose.p.x - R*math.sin(a0)
    cy = pose.p.y + R*math.cos(a0)
    a1 = a0 + rad(sweepDeg)
    x = cx + R*math.sin(a1)
    y = cy - R*math.cos(a1)
    return Pose(Pt(x, y), pose.headingDeg + sweepDeg)

# ======================
# Modelos simples
# ======================
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

AnySeg = Tuple[str, object]

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

def segments_polyline(segs: List[object], step: float = 8.0) -> List[Pt]:
    pts: List[Pt] = []
    for s in segs:
        if isinstance(s, SegStraight):
            n = max(2, int(math.ceil(s.lengthMM/step)))
            for i in range(n):
                t = i/(n-1)
                p = advance_straight(s.from_pose, s.lengthMM*t).p
                pts.append(Pt(p.x, p.y))
        else:
            arc = s
            L = abs(rad(arc.sweepDeg))*arc.radiusMM
            n = max(6, int(math.ceil(L/step)))
            for i in range(n):
                t = i/(n-1)
                p = advance_arc(arc.from_pose, arc.radiusMM, arc.sweepDeg*t).p
                pts.append(Pt(p.x, p.y))
    return pts


# Constantes alinhadas com o editor (Utils/track_geometry.py)
START_FINISH_GAP_MM = 1000.0     # distância entre partida e chegada (em -s)
STRAIGHT_NEAR_XING_MM = 250.0    # afastamento de emendas de segmento
MARKER_OFFSET_MM = 40.0
MARKER_LENGTH_MM = 40.0
MARKER_THICKNESS_MM = 20.0


def curvature_change_markers(segs: List[object]) -> List[Tuple[Pt, float]]:
    """
    Retorna (ponto, headingDeg) sempre que a curvatura muda.
    Inclui também a transição entre o último e o primeiro segmento
    quando a pista é fechada (emenda final→início).
    """
    def kappa(s: object) -> float:
        if isinstance(s, SegArc):
            return (1.0 if s.sweepDeg >= 0.0 else -1.0)/max(1e-9, s.radiusMM)
        return 0.0

    def end_pose(s: object) -> Pose:
        return advance_straight(s.from_pose, s.lengthMM) if isinstance(s, SegStraight) \
               else advance_arc(s.from_pose, s.radiusMM, s.sweepDeg)

    out: List[Tuple[Pt, float]] = []
    if not segs:
        return out

    # mudanças internas
    for i in range(len(segs) - 1):
        a, b = segs[i], segs[i+1]
        if abs(kappa(a) - kappa(b)) > 1e-6:
            ep = end_pose(a)
            out.append((ep.p, ep.headingDeg))

    # mudança na emenda final → início (pista fechada)
    first, last = segs[0], segs[-1]
    ep_last = end_pose(last)
    is_closed = (math.hypot(ep_last.p.x - first.from_pose.p.x,
                            ep_last.p.y - first.from_pose.p.y) <= 1.0)
    if is_closed and abs(kappa(last) - kappa(first)) > 1e-6:
        out.append((ep_last.p, ep_last.headingDeg))
    return out


def start_finish_lines(track: Dict[str, Any], segs: List[object], tapeW: float):
    sf = track.get("startFinish") or {}
    if not sf.get("enabled", False): return None
    segId = sf.get("onSegmentId")
    startIsFwd = sf.get("startIsForward", True)
    sParam = float(sf.get("sParamMM", 0.0))

    straight = next((s for s in segs if isinstance(s, SegStraight) and s.id == segId), None)
    if not straight: return None

    # === clamp igual ao editor ===
    def clamp_start_on_seg(seg: SegStraight, t: float) -> float:
        mn = START_FINISH_GAP_MM + STRAIGHT_NEAR_XING_MM
        mx = seg.lengthMM - STRAIGHT_NEAR_XING_MM
        return max(mn, min(mx, t))

    sParam = clamp_start_on_seg(straight, sParam)

    def gate_at(d: float) -> Tuple[Pt, Pt, float]:
        pose = advance_straight(straight.from_pose, max(0.0, min(straight.lengthMM, d)))
        a = rad(pose.headingDeg)
        nx, ny = -math.sin(a), math.cos(a)  # lado ESQUERDO
        half = (tapeW * 1.2) * 0.5
        ax, ay = pose.p.x + nx*half, pose.p.y + ny*half
        bx, by = pose.p.x - nx*half, pose.p.y - ny*half
        return (Pt(ax, ay), Pt(bx, by), pose.headingDeg)

    start_pose  = gate_at(sParam)
    finish_pose = gate_at(max(0.0, sParam - START_FINISH_GAP_MM))
    return (start_pose, finish_pose) if startIsFwd else (finish_pose, start_pose)

# ======================
# Robô
# ======================
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
    """
    Aceita os formatos:
      {
        "envelope": {"widthMM":..., "heightMM":...},     # preferido
        "origin": {"xMM":..., "yMM":...},                # opcional
        "wheels": [{"id":..., "xMM":..., "yMM":...}, ...],
        "sensors": [{"id":..., "xMM":..., "yMM":..., "sizeMM":5.0}, ...],
        "gridStepMM": 5.0, "originXMM": 0.0, "originYMM": 0.0
      }
    …ou variantes antigas sem "envelope" e/ou com origin solto.
    """
    # --- envelope (obrigatório no resultado; se faltar, tenta inferir) ---
    env = obj.get("envelope")
    if not env:
        # tenta pegar campos soltos ou dentro de um "body"
        width  = obj.get("widthMM")  or (obj.get("body") or {}).get("widthMM")
        height = obj.get("heightMM") or (obj.get("body") or {}).get("heightMM")
        if width is None or height is None:
            # fallback seguro (evita KeyError e permite visualizar algo)
            width, height = 160.0, 140.0
        env = {"widthMM": float(width), "heightMM": float(height)}

    # --- origin (aceita objeto {"xMM","yMM"} ou campos soltos) ---
    ox = float(obj.get("originXMM", (obj.get("origin") or {}).get("xMM", 0.0)))
    oy = float(obj.get("originYMM", (obj.get("origin") or {}).get("yMM", 0.0)))

    # --- rodas/sensores (validação leve) ---
    if "wheels" not in obj or not obj["wheels"]:
        raise ValueError("Arquivo de robô inválido: lista 'wheels' ausente ou vazia.")
    if "sensors" not in obj or not obj["sensors"]:
        raise ValueError("Arquivo de robô inválido: lista 'sensors' ausente ou vazia.")

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

# ======================
# Aux de leitura de pista
# ======================
def point_to_polyline_distance_mm(px: float, py: float, poly: List[Pt]) -> float:
    best = 1e18
    for i in range(len(poly)-1):
        ax, ay = poly[i].x, poly[i].y
        bx, by = poly[i+1].x, poly[i+1].y
        vx, vy = bx-ax, by-ay
        wx, wy = px-ax, py-ay
        L2 = vx*vx + vy*vy
        if L2 < 1e-12:
            d = math.hypot(px-ax, py-ay)
        else:
            t = max(0.0, min(1.0, (wx*vx + wy*vy)/L2))
            cx, cy = ax + t*vx, ay + t*vy
            d = math.hypot(px-cx, py-cy)
        if d < best: best = d
    return best

# envelope toca a fita?
def envelope_contacts_tape(robot: Robot, x: float, y: float, heading_deg: float,
                           tape_poly: List[Pt], tape_w: float,
                           grid_n: int = 9, margin_mm: float = 0.5) -> bool:
    """
    Retorna True se QUALQUER parte do envelope está sobre a fita.
    Amostra uma malha grid_n x grid_n no retângulo do robô (rotacionado)
    e faz early-exit no primeiro ponto em contato.
    """
    hw = robot.envelope.widthMM * 0.5
    hh = robot.envelope.heightMM * 0.5
    ang = rad(heading_deg)
    half = tape_w * 0.5 + margin_mm  # margem anti-erro numérico

    if grid_n < 3:
        grid_n = 3

    # gera coordenadas locais uniformes em [-hw, +hw] x [-hh, +hh]
    for iy in range(grid_n):
        ly = -hh + (2.0 * hh) * (iy / (grid_n - 1))
        for ix in range(grid_n):
            lx = -hw + (2.0 * hw) * (ix / (grid_n - 1))
            rx, ry = rot(lx, ly, ang)
            px, py = x + rx, y + ry
            if point_to_polyline_distance_mm(px, py, tape_poly) <= half:
                return True  # early-exit no primeiro contato

    return False

def estimate_sensor_coverage(px: float, py: float, tape_poly: List[Pt], tape_w: float, sensor_size: float) -> float:
    n = 5
    step = sensor_size/(n-1)
    half = tape_w*0.5
    inside = 0
    for iy in range(n):
        for ix in range(n):
            sx = px + (ix - (n-1)/2.0)*step
            sy = py + (iy - (n-1)/2.0)*step
            if point_to_polyline_distance_mm(sx, sy, tape_poly) <= half:
                inside += 1
    return inside/(n*n)

def sensor_value_from_coverage(cov: float) -> int:
    cov = max(0.0, min(1.0, cov))
    if cov >= 0.5:
        return min(255, max(200, 200 + int(round(cov * 55))))
    else:
        return min(100, max(0, int(round(cov * 100))))

# ======================
# Controlador (plugin)
# ======================
def load_python_controller(path: str):
    spec = importlib.util.spec_from_file_location("controller_mod", path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)  # type: ignore
    if not hasattr(mod, "control_step"):
        raise ValueError("Controlador Python deve definir control_step(state)->{'pwm_left','pwm_right'}.")
    return mod.control_step

def import_controller(path: str):
    ext = os.path.splitext(path)[1].lower()
    if ext == ".py": return load_python_controller(path)
    raise ValueError("Somente .py aqui para simplificar; adicione .so/.dll se precisar.")

# ======================
# Worker de simulação
# ======================
class SimWorker(QThread):
    sig_chunk = Signal(list)     # lista de steps (parcial)
    sig_done  = Signal(dict)     # resumo
    sig_fail  = Signal(str)

    def __init__(self, track: Dict[str, Any], robot: Robot, controller_fn,
                 v_final_mps: float, tau_s: float, parent=None):
        super().__init__(parent)
        self.track = track
        self.robot = robot
        self.controller_fn = controller_fn
        self.v_final = float(v_final_mps)*1000.0  # mm/s
        self.tau = max(1e-6, float(tau_s))
        self.cancelled = False
        self.track_length_mm: float = 0.0
        self.sim_distance_mm: float = 0.0  # acumulada durante a sim

    def run(self):
        try:
            segs, origin, tapeW = segments_from_json(self.track)
            tape_poly = segments_polyline(segs, step=8.0)
            # partida/chegada
            gates = start_finish_lines(self.track, segs, tapeW)
            start_gate, finish_gate = (None, None) if (gates is None) else gates

            # posição inicial (logo ANTES da partida, voltado para frente)
            if start_gate is None:
                x0, y0, h0 = origin.p.x, origin.p.y, origin.headingDeg
            else:
                (a, b, hdg) = start_gate
                back = (self.robot.envelope.heightMM/2.0) + 10.0
                pose = Pose(Pt((a.x+b.x)/2.0, (a.y+b.y)/2.0), hdg)
                start_pose = advance_straight(pose, -back)
                x0, y0, h0 = start_pose.p.x, start_pose.p.y, start_pose.headingDeg

            # estado
            dt = 0.001  # 1ms
            v = 0.0
            w = 0.0
            x, y, h = x0, y0, h0

            # mapeamento PWM->velocidade de roda (simples)
            def pwm_to_wheel_v(pwm: int) -> float:
                pwm = max(0, min(4095, int(pwm)))
                return (pwm/4095.0) * self.v_final  # mm/s

            # distância entre rodas (track width) a partir do modelo
            wl = next((w for w in self.robot.wheels if w.id.lower() == "left"), None)
            wr = next((w for w in self.robot.wheels if w.id.lower() == "right"), None)
            trackW = abs((wr.yMM if wr else 35.0) - (wl.yMM if wl else -35.0))

            # loop
            steps_out = []
            max_ms = 300000  # 5min máx
            finished = False
            crossed_finish = False
            last_side = None

            # >>>> NOVO: enviar chunks menores para atualizar a UI <<<<
            CHUNK_STEPS = 100  # antes: 1000

            def side_of_gate(px: float, py: float, gate) -> float:
                (a, b, _) = gate
                vx, vy = (b.x - a.x), (b.y - a.y)
                wx, wy = (px - a.x), (py - a.y)
                return math.copysign(1.0, vx*wy - vy*wx)

            if finish_gate is not None:
                last_side = side_of_gate(x, y, finish_gate)

            for t_ms in range(max_ms):
                if self.cancelled: return
                # sensores
                a = rad(h)
                sn_vals = []
                for s in self.robot.sensors:
                    rx, ry = rot(s.xMM + self.robot.originXMM, s.yMM + self.robot.originYMM, a)
                    px, py = x + rx, y + ry
                    cov = estimate_sensor_coverage(px, py, tape_poly, tapeW, s.sizeMM)
                    sn_vals.append(sensor_value_from_coverage(cov))

                # controlador
                state = {
                    "t_ms": t_ms,
                    "pose": {"x_mm": x, "y_mm": y, "heading_deg": h},
                    "vel": {"v_mm_s": v, "omega_rad_s": w},
                    "sensors": {"values": sn_vals}
                }
                ctrl = self.controller_fn(state)
                pwmL = int(ctrl.get("pwm_left", 0))
                pwmR = int(ctrl.get("pwm_right", 0))

                # dinâmica 1ª ordem
                vL_cmd = pwm_to_wheel_v(pwmL)
                vR_cmd = pwm_to_wheel_v(pwmR)
                v_cmd = 0.5*(vL_cmd + vR_cmd)
                w_cmd = (vR_cmd - vL_cmd)/max(1e-6, trackW)
                alpha = 1.0 - math.exp(-dt/self.tau)
                v += (v_cmd - v)*alpha
                w += (w_cmd - w)*alpha

                # integra
                h += deg(w*dt)
                a = rad(h)
                x += v*dt*math.cos(a)
                y += v*dt*math.sin(a)

                steps_out.append({
                    "t_ms": t_ms,
                    "x_mm": x, "y_mm": y, "heading_deg": h,
                    "v_mm_s": v, "omega_rad_s": w,
                    "pwmL": pwmL, "pwmR": pwmR
                })

                # paradas
                if not envelope_contacts_tape(self.robot, x, y, h, tape_poly, tapeW):
                    finished = True
                    reason = "offtrack"
                    break

                if finish_gate is not None:
                    cur_side = side_of_gate(x, y, finish_gate)
                    if last_side is not None and cur_side != last_side:
                        crossed_finish = True
                    last_side = cur_side
                    if crossed_finish:
                        finished = True
                        reason = "finished"
                        break

                # >>>> NOVO: emite a cada 100 steps (UI atualiza “Steps executados” sem lag)
                if len(steps_out) >= CHUNK_STEPS:
                    self.sig_chunk.emit(steps_out)
                    steps_out = []

            if steps_out:
                self.sig_chunk.emit(steps_out)

            self.sig_done.emit({"ok": True, "reason": reason if finished else "timeout"})
        except Exception as e:
            self.sig_fail.emit(str(e))

# ======================
# Cena/View
# ======================
class SimScene(QGraphicsScene):
    def __init__(self):
        super().__init__()
        self.setBackgroundBrush(QColor("#0c0c0c"))

    def drawBackground(self, painter: QPainter, rect):
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
    def __init__(self, scene: SimScene):
        super().__init__(scene)
        self.setRenderHints(self.renderHints() | QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self.setDragMode(QGraphicsView.ScrollHandDrag)

    def wheelEvent(self, e):
        s = 1.15 if e.angleDelta().y() > 0 else 1/1.15
        self.scale(s, s)

# ======================
# Janela principal
# ======================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Line-Follower Simulator (dt=1ms)")

        # dados
        self.track: Optional[Dict[str, Any]] = None
        self.robot: Optional[Robot] = None
        self.controller_fn = lambda state: {"pwm_left": 2000, "pwm_right": 2000}

        # GUI
        self.scene = SimScene()
        self.view = SimView(self.scene)
        self.view.setSceneRect(0, 0, 4000, 3000)

        left = QWidget(); form = QFormLayout(left)

        self.btn_track = QPushButton("Carregar pista (.json)")
        self.btn_robot = QPushButton("Carregar robô (.json)")
        self.btn_ctrl  = QPushButton("Carregar controlador (.py/.so/.dll)")
        self.spin_vf   = QDoubleSpinBox(); self.spin_vf.setRange(0.1, 20.0); self.spin_vf.setValue(2.0); self.spin_vf.setSingleStep(0.1)
        self.spin_tau  = QDoubleSpinBox(); self.spin_tau.setRange(0.001, 5.0); self.spin_tau.setValue(0.05); self.spin_tau.setSingleStep(0.01)
        self.btn_sim   = QPushButton("Simular")
        self.btn_stop  = QPushButton("Parar")
        self.btn_replay= QPushButton("Rever animação")
        self.progress  = QProgressBar(); self.progress.setValue(0)  # pode manter a barra, mas ela não será usada
        self.step_count = 0
        self.lbl_progress = QLabel("Steps executados: 0")
        form.addRow(self.lbl_progress)
        self.lbl_time = QLabel("Tempo sim: 0.00 s | Vel. anim: 1.0×")

        form.addRow(self.lbl_time)

        self.combo_speed = QComboBox()
        self.combo_speed.addItems(["0.1×", "0.5×", "1×", "2×", "4×"])
        self.combo_speed.setCurrentIndex(2)  # 1×
        form.addRow(QLabel("Velocidade da animação"), self.combo_speed)
        
        self.anim_speed = 1.0
        self.anim_spf = 17  # ~1000/60
        self.combo_speed.currentIndexChanged.connect(self.on_speed_change)
        
        
        form.addRow(self.btn_track)
        form.addRow(self.btn_robot)
        form.addRow(self.btn_ctrl)
        form.addRow(QLabel("Velocidade linear final (m/s)"), self.spin_vf)
        form.addRow(QLabel("Constante de tempo do motor τ (s)"), self.spin_tau)
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

        # sinais
        self.btn_track.clicked.connect(self.on_load_track)
        self.btn_robot.clicked.connect(self.on_load_robot)
        self.btn_ctrl.clicked.connect(self.on_load_controller)
        self.btn_sim.clicked.connect(self.on_simulate)
        self.btn_stop.clicked.connect(self.on_stop)
        self.btn_replay.clicked.connect(self.on_replay)

        # animação
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.anim_idx = 0
        self.anim_steps: List[Dict[str, Any]] = []
        self.anim_items: Dict[str, Any] = {}  # itens gráficos vivos

        # worker
        self.worker: Optional[SimWorker] = None

    def on_speed_change(self, idx: int):
        mapping = {0: 0.1, 1: 0.5, 2: 1.0, 3: 2.0, 4: 4.0}
        self.anim_speed = mapping.get(idx, 1.0)
        # 1000 steps/s a 1×; com ~60 FPS => base ≈ 16.7 steps/frame
        base_spf = int(round(1000.0 / 60.0))
        self.anim_spf = max(1, int(round(base_spf * self.anim_speed)))
        # reflete no label imediatamente (mesmo parado)
        i = self.anim_idx if self.anim_steps else 0
        self.lbl_time.setText("Tempo sim: {:.2f} s | Vel. anim: {:.1f}×".format(i*0.001, self.anim_speed))

    # ---------- Loaders ----------
    def on_load_track(self):
        path, _ = QFileDialog.getOpenFileName(self, "Pista", "", "JSON (*.json)")
        if not path: return
        with open(path, "r", encoding="utf-8") as f:
            self.track = json.load(f)
        self.draw_static_track()
        self.statusBar().showMessage(f"Pista: {os.path.basename(path)}")

    def on_load_robot(self):
        path, _ = QFileDialog.getOpenFileName(self, "Carregar robô (JSON)", "", "JSON (*.json)")
        if not path: return
        try:
            with open(path, "r", encoding="utf-8") as f:
                self.robot = robot_from_json(json.load(f))
            self.statusBar().showMessage(f"Robô: {os.path.basename(path)}", 5000)
        except Exception as e:
            # Mostra erro amigável e garante que não há thread pendurada
            self.robot = None
            QMessageBox.critical(self, "Erro ao carregar robô",
                                f"Falha ao ler '{os.path.basename(path)}':\n{e}")
            # se por acaso existir worker rodando, peça cancelamento e espere
            if hasattr(self, "worker") and self.worker and self.worker.isRunning():
                try:
                    self.worker.cancelled = True
                    self.worker.wait(500)
                except Exception:
                    pass


    def on_load_controller(self):
        path, _ = QFileDialog.getOpenFileName(self, "Controlador", "", "Python (*.py);;Shared lib (*.so *.dll *.dylib)")
        if not path: return
        self.controller_fn = import_controller(path)
        QMessageBox.information(self, "Controlador", "Controlador carregado com sucesso.")

    # ---------- Desenho estático ----------
    def clear_static(self):
        # remove tudo e limpa referências de itens animados também
        self.scene.clear()
        self.anim_items.clear()

    def draw_static_track(self):
        if not self.track: return
        self.clear_static()
        segs, origin, tapeW = segments_from_json(self.track)
        pts = segments_polyline(segs, step=8.0)
        if len(pts) >= 2:
            path = QPainterPath(QPointF(pts[0].x, pts[0].y))
            for p in pts[1:]:
                path.lineTo(p.x, p.y)
            self.scene.addPath(path, QPen(QColor("#f5f5f5"), tapeW, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            self.scene.addPath(path, QPen(QColor("#444444"), 1, Qt.DashLine))
    
        # marcadores de curvatura (brancos) – lado ESQUERDO
        for (pp, hdg) in curvature_change_markers(segs):
            a = rad(hdg)
            nx, ny = math.sin(a), -math.cos(a)   # normal p/ ESQUERDA
            base = (tapeW*0.5) + MARKER_OFFSET_MM
            ax, ay = pp.x + nx*base, pp.y + ny*base
            bx, by = ax + nx*MARKER_LENGTH_MM, ay + ny*MARKER_LENGTH_MM
            self.scene.addLine(ax, ay, bx, by, QPen(QColor("#FFFFFF"), MARKER_THICKNESS_MM, Qt.SolidLine, Qt.RoundCap))
    
        # partida/chegada (linhas + pinos, ESQUERDA)
        gates = start_finish_lines(self.track, segs, tapeW)
        if gates:
            for (a, b, hdg) in gates:
                # linha do gate atravessando a fita
                self.scene.addLine(a.x, a.y, b.x, b.y, QPen(QColor("#FFD54F"), 4))
    
                # pino amarelo do lado ESQUERDO
                a_rad = rad(hdg)
                nx, ny = -math.sin(a_rad), math.cos(a_rad)
                base = (tapeW * 0.5) + MARKER_OFFSET_MM
                sx, sy = ((a.x + b.x)/2.0) + nx*base, ((a.y + b.y)/2.0) + ny*base
                ex, ey = sx + nx*MARKER_LENGTH_MM, sy + ny*MARKER_LENGTH_MM
                self.scene.addLine(sx, sy, ex, ey, QPen(QColor("#FFD54F"), MARKER_THICKNESS_MM, Qt.SolidLine, Qt.RoundCap))
    
        # --- comprimento total da pista (aprox. pela polilinha) ---
        self.track_length_mm = 0.0
        for i in range(len(pts)-1):
            dx = pts[i+1].x - pts[i].x
            dy = pts[i+1].y - pts[i].y
            self.track_length_mm += math.hypot(dx, dy)
    
        # ajusta view
        self.view.fitInView(self.scene.itemsBoundingRect().adjusted(-100, -100, +100, +100), Qt.KeepAspectRatio)

    def draw_robot_outline_preview(self):
        if not (self.track and self.robot): return
        # apenas redesenha a pista e coloca um retângulo “fantasma” no start
        self.draw_static_track()
        segs, origin, tapeW = segments_from_json(self.track)
        gates = start_finish_lines(self.track, segs, tapeW)
        if gates:
            (a, b, hdg), _ = gates
            back = (self.robot.envelope.heightMM/2.0) + 10.0
            pose = Pose(Pt((a.x+b.x)/2.0, (a.y+b.y)/2.0), hdg)
            pos = advance_straight(pose, -back)
            hw = self.robot.envelope.widthMM/2.0
            hh = self.robot.envelope.heightMM/2.0
            ang = rad(pos.headingDeg)
            corners = [(-hw,-hh),(hw,-hh),(hw,hh),(-hw,hh)]
            poly = QPainterPath()
            for i,(cx,cy) in enumerate(corners + [corners[0]]):
                rx, ry = rot(cx, cy, ang)
                px, py = pos.p.x + rx, pos.p.y + ry
                if i == 0: poly.moveTo(px, py)
                else: poly.lineTo(px, py)
            self.scene.addPath(poly, QPen(QColor("#9e9e9e"), 1, Qt.DashLine))

    # ---------- Simulação ----------
    def on_simulate(self):
        if not (self.track and self.robot):
            QMessageBox.warning(self, "Faltando", "Carregue pista e robô.")
            return
        self.anim_steps.clear()
        self.anim_idx = 0
        self.progress.setValue(0)
        self.btn_sim.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.btn_replay.setEnabled(False)
    
        # zera contadores de “steps” e distância (distância não será mais exibida)
        self.step_count = 0
        self.sim_distance_mm = 0.0
        self.lbl_progress.setText("Steps executados: 0")
    
        self.worker = SimWorker(self.track, self.robot, self.controller_fn,
                                self.spin_vf.value(), self.spin_tau.value())
        self.worker.sig_chunk.connect(self.on_stream_chunk)
        self.worker.sig_done.connect(self.on_stream_done)
        self.worker.sig_fail.connect(self.on_stream_fail)
        self.worker.start()


    def on_stop(self):
        """Parar simulação/ani­mação com segurança."""
        # para animação, se estiver rodando
        if self.timer.isActive():
            self.timer.stop()

        # se existir um worker rodando, peça cancelamento e ESPERE ele terminar
        if hasattr(self, "worker") and self.worker and self.worker.isRunning():
            try:
                self.worker.cancelled = True
                self.worker.wait(2000)  # espera até 2s
            except Exception:
                pass

        # estado dos botões
        self.btn_sim.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.btn_replay.setEnabled(bool(self.anim_steps))


    def on_stream_chunk(self, chunk: List[dict]):
        # guarda steps para animação
        self.anim_steps.extend(chunk)

        # conta steps executados
        self.step_count += len(chunk)

        # (opcional) ainda acumulo distância, caso queira usar depois
        for step in chunk:
            self.sim_distance_mm += float(step.get("v_mm_s", 0.0)) * 0.001

        # atualiza o texto (não usamos mais a barra de progresso como %)
        self.lbl_progress.setText(f"Steps executados: {self.step_count}")

    def _save_logs(self, summary: dict) -> None:
        """
        Salva os passos da simulação e o resumo em ./Logs/sim_YYYYmmdd-HHMMSS.json.
        """
        import datetime
        base_dir = os.path.join(os.path.dirname(__file__), "Logs")
        os.makedirs(base_dir, exist_ok=True)

        ts = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        out_path = os.path.join(base_dir, f"sim_{ts}.json")

        payload = {
            "summary": summary,
            "steps": self.anim_steps,  # lista de steps consumida pela animação
        }

        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)


    def on_stream_done(self, summary: dict):
        """Chamado quando a simulação em thread terminou e mandou o resumo."""
        # garante que o worker realmente saiu antes de seguir
        if hasattr(self, "worker") and self.worker:
            try:
                if self.worker.isRunning():
                    self.worker.wait(2000)
            finally:
                self.worker = None

        # habilita botões e inicia a animação do replay
        self.btn_sim.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.btn_replay.setEnabled(True)

        # salva logs na pasta Logs/
        try:
            self._save_logs(summary)
            self.statusBar().showMessage("Log salvo em ./Logs", 4000)
        except Exception as e:
            # não derruba a UI se der erro ao salvar log
            print("Falha ao salvar log:", e)

        # reinicia o replay desde o começo
        self.anim_idx = 0
        if self.anim_steps:
            self.timer.start(16)   # ~60 FPS


    def on_stream_fail(self, msg: str):
        self.worker = None
        self.btn_stop.setEnabled(False)
        self.btn_sim.setEnabled(True)
        QMessageBox.critical(self, "Erro", msg)

    # ---------- Replay ----------
    def reset_anim_items(self):
        # remove itens antigos (se existirem) e limpa refs
        for it in list(self.anim_items.values()):
            if isinstance(it, list):
                for sub in it:
                    if sub is not None:
                        self.scene.removeItem(sub)
            elif it is not None:
                self.scene.removeItem(it)
        self.anim_items.clear()

        # cria novos recipientes
        self.anim_items["trail"] = self.scene.addPath(QPainterPath(), QPen(QColor("#80CBC4"), 2))
        self.anim_items["robot"] = self.scene.addPath(QPainterPath(), QPen(QColor("#FFEB3B"), 2))
        self.anim_items["sensors"] = []

        # cria “pinos” para sensores
        if self.robot:
            for _ in self.robot.sensors:
                self.anim_items["sensors"].append(self.scene.addPath(QPainterPath(), QPen(QColor("#FFFFFF"), 1)))

    def on_replay(self):
        if not self.anim_steps:
            return
        # recria itens SEM limpar a cena toda (evita invalidar refs)
        self.reset_anim_items()
        self.anim_idx = 0
        self.timer.start(16)  # ~60 FPS

    def closeEvent(self, event):
        """
        Garante que nenhuma thread de simulação fique viva ao fechar a janela.
        Evita 'QThread: Destroyed while thread is still running'.
        """
        try:
            if self.timer.isActive():
                self.timer.stop()
            if hasattr(self, "worker") and self.worker and self.worker.isRunning():
                self.worker.cancelled = True
                self.worker.wait(2000)
        finally:
            super().closeEvent(event)


    def tick(self):
        if not self.anim_steps:
            self.timer.stop(); return

        # quantos steps desenhar neste frame
        steps_this_frame = getattr(self, "anim_spf", 17)  # ~1000/60 em 1×

        for _ in range(steps_this_frame):
            i = self.anim_idx
            if i >= len(self.anim_steps):
                self.timer.stop()
                break

            step = self.anim_steps[i]
            x, y, h = step["x_mm"], step["y_mm"], step["heading_deg"]

            # trilha
            trail: QPainterPath = self.anim_items["trail"].path()
            if trail.elementCount() == 0:
                trail.moveTo(x, y)
            else:
                trail.lineTo(x, y)
            self.anim_items["trail"].setPath(trail)

            # robô (retângulo) + sensores
            if self.robot:
                hw = self.robot.envelope.widthMM/2.0
                hh = self.robot.envelope.heightMM/2.0
                ang = rad(h)
                corners = [(-hw,-hh),(hw,-hh),(hw,hh),(-hw,hh)]
                poly = QPainterPath()
                for idx,(cx,cy) in enumerate(corners + [corners[0]]):
                    rx, ry = rot(cx, cy, ang)
                    px, py = x + rx, y + ry
                    if idx == 0: poly.moveTo(px, py)
                    else: poly.lineTo(px, py)
                self.anim_items["robot"].setPath(poly)

                for k, s in enumerate(self.robot.sensors):
                    rx, ry = rot(s.xMM + self.robot.originXMM, s.yMM + self.robot.originYMM, ang)
                    px, py = x + rx, y + ry
                    sz = s.sizeMM
                    sp = QPainterPath()
                    sp.addRect(px - sz/2.0, py - sz/2.0, sz, sz)
                    self.anim_items["sensors"][k].setPath(sp)

            self.anim_idx += 1

        # atualiza label após consumir os steps deste frame
        sim_t_s = (self.anim_idx * 0.001)
        self.lbl_time.setText("Tempo sim: {:.2f} s | Vel. anim: {:.1f}×".format(sim_t_s, self.anim_speed))

# ======================
# Main
# ======================
def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
