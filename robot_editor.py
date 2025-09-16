
from __future__ import annotations
import sys, json
from typing import Optional, Tuple

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QGraphicsScene, QGraphicsView, QFormLayout,
    QVBoxLayout, QHBoxLayout, QLabel, QDoubleSpinBox, QPushButton, QListWidget,
    QListWidgetItem, QFileDialog, QMessageBox, QSplitter, QCheckBox, QInputDialog
)
from PySide6.QtGui import QPen, QColor, QAction, QPainter, QKeyEvent
from PySide6.QtCore import Qt, QPointF, QRectF, Signal

from Utils.robot_geometry import (
    Pt, Envelope,
    DEFAULT_WHEEL_W, DEFAULT_WHEEL_H, DEFAULT_SENSOR_SIZE,
)
from Utils.robot_model import RobotModel, Wheel, Sensor

TAPE_THICKNESS_MM = 20.0  # linha horizontal no y=0

class RobotScene(QGraphicsScene):
    # notifica quando usuário clica em um elemento na cena
    selChanged = Signal(object, object)  # (kind or None, id or None)

    def __init__(self, model: RobotModel):
        super().__init__()
        self.model = model
        self.setBackgroundBrush(QColor("#0c0c0c"))
        # onde desenhar o centro do envelope (em coordenadas da cena, em mm)
        self.center_scene = QPointF(600.0, 420.0)

        # seleção
        self.selected_kind: Optional[str] = None  # "wheel" | "sensor"
        self.selected_id: Optional[str] = None

    # ---- picking ----
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            pos = event.scenePos()
            clicked = self._hit_test(pos)
            if clicked is None:
                # clique no fundo -> desseleciona
                self.selected_kind, self.selected_id = None, None
            else:
                k, _id = clicked
                # clicar de novo no mesmo -> desseleciona
                if self.selected_kind == k and self.selected_id == _id:
                    self.selected_kind, self.selected_id = None, None
                else:
                    self.selected_kind, self.selected_id = k, _id
            # avisa a UI
            self.selChanged.emit(self.selected_kind, self.selected_id)
            self.rebuild()
        elif event.button() == Qt.RightButton:
            # Clique direito: definir a ORIGEM (0,0) no ponto clicado
            p = event.scenePos()
            ox = p.x() - self.center_scene.x()
            oy = p.y() - self.center_scene.y()
            self.model.originXMM = ox
            self.model.originYMM = oy
            self.model.clamp_all_inside()
            self.selChanged.emit(None, None)
            self.rebuild()
        super().mousePressEvent(event)

    def _hit_test(self, p: QPointF) -> Optional[Tuple[str, str]]:
        """
        Retorna ("wheel","left/right") ou ("sensor","Si") se clicar sobre um elemento.
        """
        # verifica rodas
        for w in self.model.wheels:
            x = self.center_scene.x() + w.xMM
            y = self.center_scene.y() + w.yMM
            if (abs(p.x() - x) <= DEFAULT_WHEEL_W/2.0) and (abs(p.y() - y) <= DEFAULT_WHEEL_H/2.0):
                return ("wheel", w.id)
        # sensores
        for s in self.model.sensors:
            x = self.center_scene.x() + s.xMM
            y = self.center_scene.y() + s.yMM
            if (abs(p.x() - x) <= s.sizeMM/2.0) and (abs(p.y() - y) <= s.sizeMM/2.0):
                return ("sensor", s.id)
        return None

    # ---- render ----
    def drawBackground(self, painter: QPainter, rect):
        super().drawBackground(painter, rect)
        step = max(5.0, self.model.gridStepMM)
        pen = QPen(QColor(25, 25, 25), 1)
        painter.setPen(pen)
        left, top, right, bottom = rect.left(), rect.top(), rect.right(), rect.bottom()
        # vertical
        x = (left // step) * step
        while x <= right:
            painter.drawLine(x, top, x, bottom)
            x += step
        # horizontal
        y = (top // step) * step
        while y <= bottom:
            painter.drawLine(left, y, right, y)
            y += step

    def rebuild(self):
        self.clear()
        c = self.center_scene
        halfW = self.model.envelope.widthMM / 2.0
        halfH = self.model.envelope.heightMM / 2.0

        # área útil (apenas para referência)
        area_pen = QPen(QColor("#1f1f1f"), 1)
        self.addRect(0, 0, c.x()*2, c.y()*2, area_pen)

        # envelope amarelo
        env_pen = QPen(QColor("#FFD54F"))
        env_pen.setWidth(1)
        env_pen.setStyle(Qt.DashLine)
        env_brush = QColor(255, 213, 79, 40)
        self.addRect(c.x()-halfW, c.y()-halfH, self.model.envelope.widthMM, self.model.envelope.heightMM, env_pen, env_brush)

        # eixos (centro geométrico do envelope)
        axis_pen = QPen(QColor("#303030"), 1)
        self.addLine(c.x()-halfW-60, c.y(), c.x()+halfW+60, c.y(), axis_pen)  # eixo X
        self.addLine(c.x(), c.y()-halfH-60, c.x(), c.y()+halfH+60, axis_pen)  # eixo Y

        # "fita" da pista (linha horizontal y=0) — cinza escuro
        tape_pen = QPen(QColor("#2b2b2b"), TAPE_THICKNESS_MM, Qt.SolidLine, Qt.RoundCap)
        self.addLine(c.x()-halfW-200, c.y(), c.x()+halfW+200, c.y(), tape_pen)

        # origem (0,0) do robô (marcador em cruz ciano)
        ox = c.x() + self.model.originXMM
        oy = c.y() + self.model.originYMM
        o_pen = QPen(QColor("#00E5FF"), 2, Qt.SolidLine, Qt.RoundCap)
        self.addLine(ox-8, oy, ox+8, oy, o_pen)
        self.addLine(ox, oy-8, ox, oy+8, o_pen)

        # rodas
        for w in self.model.wheels:
            x = c.x() + w.xMM
            y = c.y() + w.yMM
            pen = QPen(QColor("#000000"), 1)
            brush = QColor("#dddddd")
            self.addRect(x-DEFAULT_WHEEL_W/2.0, y-DEFAULT_WHEEL_H/2.0,
                                DEFAULT_WHEEL_W, DEFAULT_WHEEL_H, pen, brush)
            if self.selected_kind == "wheel" and self.selected_id == w.id:
                sel_pen = QPen(QColor("#00E5FF"), 2)
                self.addRect(x-DEFAULT_WHEEL_W/2.0, y-DEFAULT_WHEEL_H/2.0,
                             DEFAULT_WHEEL_W, DEFAULT_WHEEL_H, sel_pen)

        # sensores
        for s in self.model.sensors:
            x = c.x() + s.xMM
            y = c.y() + s.yMM
            pen = QPen(QColor("#000000"), 1)
            brush = QColor("#ffffff")
            self.addRect(x-s.sizeMM/2.0, y-s.sizeMM/2.0, s.sizeMM, s.sizeMM, pen, brush)
            if self.selected_kind == "sensor" and self.selected_id == s.id:
                sel_pen = QPen(QColor("#00E5FF"), 2)
                self.addRect(x-s.sizeMM/2.0, y-s.sizeMM/2.0, s.sizeMM, s.sizeMM, sel_pen)

class RobotView(QGraphicsView):
    def __init__(self, scene: RobotScene):
        super().__init__(scene)
        self.setRenderHints(self.renderHints() | QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)

    def wheelEvent(self, event):
        factor = 1.15 if event.angleDelta().y() > 0 else 1/1.15
        self.scale(factor, factor)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Editor (Python/PySide6)")

        self.model = RobotModel()
        self.scene = RobotScene(self.model)
        self.view = RobotView(self.scene)
        self.view.setSceneRect(0, 0, 1600, 1000)

        # quando clica na cena, refletir seleção na UI
        self.scene.selChanged.connect(self.on_scene_selection)

        # Painel esquerdo: canvas; direito: propriedades
        props = QWidget()
        form = QFormLayout(props)

        # Envelope
        self.spin_env_w = QDoubleSpinBox(); self.spin_env_w.setRange(10, 250); self.spin_env_w.setValue(self.model.envelope.widthMM)
        self.spin_env_h = QDoubleSpinBox(); self.spin_env_h.setRange(10, 250); self.spin_env_h.setValue(self.model.envelope.heightMM)
        self.spin_grid = QDoubleSpinBox();  self.spin_grid.setRange(2, 200);   self.spin_grid.setValue(self.model.gridStepMM)

        form.addRow(QLabel("<b>Envelope (limite 250 x 250 mm)</b>"))
        form.addRow("Largura (mm)", self.spin_env_w)
        form.addRow("Altura (mm)", self.spin_env_h)
        form.addRow("Grid (mm)", self.spin_grid)

        # Origem (0,0)
        form.addRow(QLabel("<b>Origem (0,0) — posição relativa ao centro do envelope</b>"))
        self.spin_origin_x = QDoubleSpinBox(); self.spin_origin_x.setRange(-10000, 10000); self.spin_origin_x.setValue(self.model.originXMM)
        self.spin_origin_y = QDoubleSpinBox(); self.spin_origin_y.setRange(-10000, 10000); self.spin_origin_y.setValue(self.model.originYMM)
        self.btn_origin_from_sensor = QPushButton("Origem = sensor selecionado")
        form.addRow("Origem X", self.spin_origin_x)
        form.addRow("Origem Y", self.spin_origin_y)
        form.addRow(self.btn_origin_from_sensor)

        # Rodas
        form.addRow(QLabel("<b>Rodas (22 x 15 mm)</b>"))
        self.spin_wl_x = QDoubleSpinBox(); self.spin_wl_x.setRange(-10000, 10000)
        self.spin_wl_y = QDoubleSpinBox(); self.spin_wl_y.setRange(-10000, 10000)
        self.spin_wr_x = QDoubleSpinBox(); self.spin_wr_x.setRange(-10000, 10000)
        self.spin_wr_y = QDoubleSpinBox(); self.spin_wr_y.setRange(-10000, 10000)
        form.addRow("Left X", self.spin_wl_x); form.addRow("Left Y", self.spin_wl_y)
        form.addRow("Right X", self.spin_wr_x); form.addRow("Right Y", self.spin_wr_y)

        # Sensores
        form.addRow(QLabel("<b>Sensores (5 x 5 mm)</b>"))
        self.list_sensors = QListWidget()
        btn_row = QHBoxLayout()
        self.btn_add_s = QPushButton("Adicionar Sensor")
        self.btn_del_s = QPushButton("Remover Sensor")
        self.btn_rename_s = QPushButton("Renomear (F2)")
        btn_row.addWidget(self.btn_add_s); btn_row.addWidget(self.btn_del_s); btn_row.addWidget(self.btn_rename_s)
        form.addRow(self.list_sensors)
        form.addRow(btn_row)

        self.spin_s_x = QDoubleSpinBox(); self.spin_s_x.setRange(-10000, 10000)
        self.spin_s_y = QDoubleSpinBox(); self.spin_s_y.setRange(-10000, 10000)
        self.chk_snap = QCheckBox("Snap à malha (usa Grid mm)")
        form.addRow("Sensor X", self.spin_s_x)
        form.addRow("Sensor Y", self.spin_s_y)
        form.addRow(self.chk_snap)

        # Arquivo
        files_row = QHBoxLayout()
        self.btn_import = QPushButton("Importar JSON")
        self.btn_export = QPushButton("Exportar JSON")
        self.btn_fit = QPushButton("Fit View")
        files_row.addWidget(self.btn_import)
        files_row.addWidget(self.btn_export)
        files_row.addWidget(self.btn_fit)
        form.addRow(files_row)

        # Splitter
        splitter = QSplitter()
        splitter.addWidget(self.view)
        splitter.addWidget(props)
        splitter.setSizes([1000, 400])
        self.setCentralWidget(splitter)

        # Menus/atalhos
        act_open = QAction("Importar", self); act_open.setShortcut("Ctrl+O"); act_open.triggered.connect(self.on_import); self.addAction(act_open)
        act_save = QAction("Exportar", self); act_save.setShortcut("Ctrl+S"); act_save.triggered.connect(self.on_export); self.addAction(act_save)
        act_fit  = QAction("Fit", self);     act_fit.setShortcut("Ctrl+F");  act_fit.triggered.connect(self.on_fit);    self.addAction(act_fit)
        act_rename = QAction("Renomear sensor", self); act_rename.setShortcut("F2"); act_rename.triggered.connect(self.rename_sensor); self.addAction(act_rename)

        # Sinais
        self.spin_env_w.valueChanged.connect(self.on_env_change)
        self.spin_env_h.valueChanged.connect(self.on_env_change)
        self.spin_grid.valueChanged.connect(self.on_grid_change)

        self.spin_origin_x.valueChanged.connect(self.on_origin_change)
        self.spin_origin_y.valueChanged.connect(self.on_origin_change)
        self.btn_origin_from_sensor.clicked.connect(self.on_origin_from_sensor)

        self.spin_wl_x.valueChanged.connect(self.on_wheel_change)
        self.spin_wl_y.valueChanged.connect(self.on_wheel_change)
        self.spin_wr_x.valueChanged.connect(self.on_wheel_change)
        self.spin_wr_y.valueChanged.connect(self.on_wheel_change)

        self.list_sensors.currentRowChanged.connect(self.on_select_sensor)
        self.btn_add_s.clicked.connect(self.on_add_sensor)
        self.btn_del_s.clicked.connect(self.on_del_sensor)
        self.btn_rename_s.clicked.connect(self.rename_sensor)
        self.spin_s_x.valueChanged.connect(self.on_sensor_change)
        self.spin_s_y.valueChanged.connect(self.on_sensor_change)

        self.btn_import.clicked.connect(self.on_import)
        self.btn_export.clicked.connect(self.on_export)
        self.btn_fit.clicked.connect(self.on_fit)

        # Inicializa UI com dados do modelo
        self.refresh_all()
        self.scene.rebuild()
        self.resize(1280, 800)

    # ---------- helpers UI ----------
    def on_fit(self):
        # envelope + margens
        halfW = self.model.envelope.widthMM/2.0
        halfH = self.model.envelope.heightMM/2.0
        c = self.scene.center_scene
        rect = QRectF(c.x()-halfW-100, c.y()-halfH-100, self.model.envelope.widthMM+200, self.model.envelope.heightMM+200)
        self.view.fitInView(rect, Qt.KeepAspectRatio)

    def on_env_change(self):
        self.model.set_envelope(self.spin_env_w.value(), self.spin_env_h.value())
        self.refresh_wheels_fields()
        self.refresh_sensors_list()   # <- também atualiza coordenadas clampadas na lista
        self.refresh_sensors_fields()
        self.scene.rebuild()

    def on_grid_change(self):
        self.model.gridStepMM = self.spin_grid.value()
        if self.chk_snap.isChecked():
            # re-snap sensor atual
            self.on_sensor_change()
        self.scene.rebuild()

    def on_origin_change(self):
        self.model.originXMM = self.spin_origin_x.value()
        self.model.originYMM = self.spin_origin_y.value()
        self.model.clamp_all_inside()
        # reflete clamp
        self.spin_origin_x.blockSignals(True); self.spin_origin_y.blockSignals(True)
        self.spin_origin_x.setValue(self.model.originXMM); self.spin_origin_y.setValue(self.model.originYMM)
        self.spin_origin_x.blockSignals(False); self.spin_origin_y.blockSignals(False)
        self.scene.rebuild()

    def on_origin_from_sensor(self):
        sid = self.current_sensor_id()
        if not sid: return
        s = self.model.find_sensor(sid)
        if not s: return
        self.model.originXMM = s.xMM
        self.model.originYMM = s.yMM
        self.model.clamp_all_inside()
        self.spin_origin_x.blockSignals(True); self.spin_origin_y.blockSignals(True)
        self.spin_origin_x.setValue(self.model.originXMM); self.spin_origin_y.setValue(self.model.originYMM)
        self.spin_origin_x.blockSignals(False); self.spin_origin_y.blockSignals(False)
        self.scene.rebuild()

    def on_wheel_change(self):
        # aplica nos dois (com clamp)
        wl = self.model.find_wheel("left")
        wr = self.model.find_wheel("right")
        if wl:
            wl.xMM = self.spin_wl_x.value()
            wl.yMM = self.spin_wl_y.value()
        if wr:
            wr.xMM = self.spin_wr_x.value()
            wr.yMM = self.spin_wr_y.value()
        self.model.clamp_all_inside()
        self.refresh_wheels_fields()  # mostra valores clampados
        self.scene.selected_kind, self.scene.selected_id = "wheel", "left"  # mantém foco visual
        self.scene.rebuild()

    def on_add_sensor(self):
        # novo sensor nasce nos valores atuais dos campos X/Y
        x, y = self.spin_s_x.value(), self.spin_s_y.value()
        if self.chk_snap.isChecked():
            step = max(1e-6, self.model.gridStepMM)
            x = round(x / step) * step
            y = round(y / step) * step
        s = self.model.add_sensor(x, y)
        self.refresh_sensors_list()
        # seleciona o novo
        self.select_sensor_id(s.id)
        self.scene.selected_kind, self.scene.selected_id = "sensor", s.id
        self.scene.rebuild()

    def on_del_sensor(self):
        sid = self.current_sensor_id()
        if sid is None: return
        self.model.remove_sensor(sid)
        self.refresh_sensors_list()
        self.scene.selected_kind, self.scene.selected_id = None, None
        self.scene.rebuild()

    def on_select_sensor(self, row: int):
        sid = self.current_sensor_id()
        if sid is None:
            return
        s = self.model.find_sensor(sid)
        if not s:
            return
        # atualiza campos
        self.spin_s_x.blockSignals(True); self.spin_s_y.blockSignals(True)
        self.spin_s_x.setValue(s.xMM); self.spin_s_y.setValue(s.yMM)
        self.spin_s_x.blockSignals(False); self.spin_s_y.blockSignals(False)
        self.scene.selected_kind, self.scene.selected_id = "sensor", s.id
        self.scene.rebuild()

    def on_sensor_change(self):
        sid = self.current_sensor_id()
        if sid is None: return
        s = self.model.find_sensor(sid)
        if not s: return
        x = self.spin_s_x.value()
        y = self.spin_s_y.value()
        if self.chk_snap.isChecked():
            step = max(1e-6, self.model.gridStepMM)
            x = round(x / step) * step
            y = round(y / step) * step
        s.xMM = x
        s.yMM = y
        self.model.clamp_all_inside()
        # reflete clamp (e snap, se ativo)
        self.spin_s_x.blockSignals(True); self.spin_s_y.blockSignals(True)
        self.spin_s_x.setValue(s.xMM); self.spin_s_y.setValue(s.yMM)
        self.spin_s_x.blockSignals(False); self.spin_s_y.blockSignals(False)
        # atualiza a linha correspondente na lista
        self.update_current_sensor_list_item_text()
        self.scene.rebuild()

    def on_scene_selection(self, kind: Optional[str], sid: Optional[str]):
        """Quando usuário clica no desenho, sincroniza a lista/editores."""
        if kind == "sensor" and sid:
            self.select_sensor_id(sid)
        elif kind == "wheel" and sid:
            # apenas reflete seleção visual e preenche campos das rodas
            wl = self.model.find_wheel("left")
            wr = self.model.find_wheel("right")
            if wl and wr:
                self.refresh_wheels_fields()

    def rename_sensor(self):
        sid = self.current_sensor_id()
        if sid is None: return
        s = self.model.find_sensor(sid)
        if not s: return
        # diálogo para novo ID
        entered, ok = QInputDialog.getText(self, "Renomear sensor", "Novo ID:", text=s.id)
        if not ok or not entered:
            return
        new_id = str(entered).strip().upper()
        # força prefixo S + dígitos quando possível
        import re
        m = re.match(r'^\s*S?(\d+)\s*$', new_id)
        if m:
            new_id = f"S{int(m.group(1))}"
        # unicidade
        existing = {x.id.upper() for x in self.model.sensors if x is not s}
        base = new_id
        i = 1
        while new_id.upper() in existing:
            # tenta incrementar sufijo numérico
            m = re.match(r'^(.*?)(\d+)$', base)
            if m:
                base_root, num = m.group(1), int(m.group(2))
                num += 1
                new_id = f"{base_root}{num}"
            else:
                new_id = f"{base}{i}"
                i += 1
        s.id = new_id
        # reflete na lista
        self.refresh_sensors_list()
        self.select_sensor_id(s.id)
        self.scene.selected_kind, self.scene.selected_id = "sensor", s.id
        self.scene.rebuild()

    # ---------- teclado: nudge ----------
    def keyPressEvent(self, event: QKeyEvent):
        handled = False
        sid = self.current_sensor_id()
        if sid is not None:
            s = self.model.find_sensor(sid)
            if s:
                # passo base 1mm; Shift=5mm; Ctrl=0.5mm; Alt=10mm
                step = 1.0
                mods = event.modifiers()
                if mods & Qt.ShiftModifier: step = 5.0
                if mods & Qt.ControlModifier: step = 0.5
                if mods & Qt.AltModifier: step = 10.0
                dx = dy = 0.0
                if event.key() == Qt.Key_Left:  dx = -step; handled = True
                elif event.key() == Qt.Key_Right: dx = +step; handled = True
                elif event.key() == Qt.Key_Up:    dy = +step; handled = True
                elif event.key() == Qt.Key_Down:  dy = -step; handled = True
                if handled:
                    nx = s.xMM + dx
                    ny = s.yMM + dy
                    if self.chk_snap.isChecked():
                        grid = max(1e-6, self.model.gridStepMM)
                        nx = round(nx / grid) * grid
                        ny = round(ny / grid) * grid
                    s.xMM, s.yMM = nx, ny
                    self.model.clamp_all_inside()
                    # refletir
                    self.spin_s_x.blockSignals(True); self.spin_s_y.blockSignals(True)
                    self.spin_s_x.setValue(s.xMM); self.spin_s_y.setValue(s.yMM)
                    self.spin_s_x.blockSignals(False); self.spin_s_y.blockSignals(False)
                    self.update_current_sensor_list_item_text()
                    self.scene.selected_kind, self.scene.selected_id = "sensor", s.id
                    self.scene.rebuild()
        if not handled:
            super().keyPressEvent(event)

    def on_import(self):
        path, _ = QFileDialog.getOpenFileName(self, "Importar robô", "", "JSON (*.json)")
        if not path: return
        try:
            with open(path, "r", encoding="utf-8") as f:
                obj = json.load(f)
            self.model = RobotModel.from_json(obj)  # valida e substitui TUDO
            self.scene.model = self.model
            self.refresh_all()
            self.scene.selected_kind, self.scene.selected_id = None, None
            self.scene.rebuild()
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Falha ao importar:\n{e}")

    def on_export(self):
        path, _ = QFileDialog.getSaveFileName(self, "Exportar robô", "robot-spec.json", "JSON (*.json)")
        if not path: return
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self.model.to_json(), f, ensure_ascii=False, indent=2)
        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Falha ao exportar:\n{e}")

    # ---------- util ----------
    def refresh_all(self):
        self.spin_env_w.blockSignals(True); self.spin_env_h.blockSignals(True); self.spin_grid.blockSignals(True)
        self.spin_env_w.setValue(self.model.envelope.widthMM)
        self.spin_env_h.setValue(self.model.envelope.heightMM)
        self.spin_grid.setValue(self.model.gridStepMM)
        self.spin_env_w.blockSignals(False); self.spin_env_h.blockSignals(False); self.spin_grid.blockSignals(False)

        # origem
        self.spin_origin_x.blockSignals(True); self.spin_origin_y.blockSignals(True)
        self.spin_origin_x.setValue(self.model.originXMM)
        self.spin_origin_y.setValue(self.model.originYMM)
        self.spin_origin_x.blockSignals(False); self.spin_origin_y.blockSignals(False)

        self.refresh_wheels_fields()
        self.refresh_sensors_list()
        self.refresh_sensors_fields()

    def refresh_wheels_fields(self):
        wl = self.model.find_wheel("left")
        wr = self.model.find_wheel("right")
        self.spin_wl_x.blockSignals(True); self.spin_wl_y.blockSignals(True)
        self.spin_wr_x.blockSignals(True); self.spin_wr_y.blockSignals(True)
        if wl:
            self.spin_wl_x.setValue(wl.xMM); self.spin_wl_y.setValue(wl.yMM)
        if wr:
            self.spin_wr_x.setValue(wr.xMM); self.spin_wr_y.setValue(wr.yMM)
        self.spin_wl_x.blockSignals(False); self.spin_wl_y.blockSignals(False)
        self.spin_wr_x.blockSignals(False); self.spin_wr_y.blockSignals(False)

    def refresh_sensors_list(self):
        cur_sid = self.current_sensor_id()
        self.list_sensors.blockSignals(True)
        self.list_sensors.clear()
        for s in self.model.sensors:
            item = QListWidgetItem(f"{s.id}  (x={s.xMM:.1f}, y={s.yMM:.1f})")
            item.setData(Qt.UserRole, s.id)
            self.list_sensors.addItem(item)
        self.list_sensors.blockSignals(False)
        # restaura seleção se possível
        if cur_sid:
            self.select_sensor_id(cur_sid)
        elif self.model.sensors:
            self.list_sensors.setCurrentRow(0)

    def refresh_sensors_fields(self):
        sid = self.current_sensor_id()
        s = self.model.find_sensor(sid) if sid else None
        self.spin_s_x.blockSignals(True); self.spin_s_y.blockSignals(True)
        if s:
            self.spin_s_x.setValue(s.xMM); self.spin_s_y.setValue(s.yMM)
        else:
            self.spin_s_x.setValue(0.0);   self.spin_s_y.setValue(0.0)
        self.spin_s_x.blockSignals(False); self.spin_s_y.blockSignals(False)

    def update_current_sensor_list_item_text(self):
        """Atualiza o texto da linha selecionada na lista com os valores atuais."""
        row = self.list_sensors.currentRow()
        if 0 <= row < self.list_sensors.count():
            sid = self.list_sensors.item(row).data(Qt.UserRole)
            s = self.model.find_sensor(sid)
            if s:
                self.list_sensors.item(row).setText(f"{s.id}  (x={s.xMM:.1f}, y={s.yMM:.1f})")

    def current_sensor_id(self) -> Optional[str]:
        item = self.list_sensors.currentItem()
        return item.data(Qt.UserRole) if item else None

    def select_sensor_id(self, sid: str):
        for i in range(self.list_sensors.count()):
            if self.list_sensors.item(i).data(Qt.UserRole) == sid:
                self.list_sensors.setCurrentRow(i)
                # também assegura que os campos X/Y estejam sincronizados
                s = self.model.find_sensor(sid)
                if s:
                    self.spin_s_x.blockSignals(True); self.spin_s_y.blockSignals(True)
                    self.spin_s_x.setValue(s.xMM); self.spin_s_y.setValue(s.yMM)
                    self.spin_s_x.blockSignals(False); self.spin_s_y.blockSignals(False)
                break

def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
