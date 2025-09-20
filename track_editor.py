from __future__ import annotations
import sys, json, math
from typing import Any

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGraphicsView,
    QGraphicsScene, QFileDialog, QPushButton, QFormLayout, QDoubleSpinBox, QComboBox,
    QLabel, QSplitter, QMessageBox, QListWidget, QListWidgetItem, QCheckBox, QInputDialog,
    QSlider
)
from PySide6.QtGui import QPainterPath, QPen, QColor, QAction, QPainter, QFont
from PySide6.QtCore import Qt, QPointF, QRectF

from Utils.track_geometry import (
    Pt, Pose, SegStraight, SegArc, AnySeg,
    rad, advance_straight, advance_arc,
    segments_to_polyline, curvature_change_markers, clamp_start_on_seg,
    START_FINISH_GAP_MM, MARKER_OFFSET_MM, MARKER_LENGTH_MM, MARKER_THICKNESS_MM
)
from Utils.track_model import TrackModel, StartFinishCfg


# ---------- Scene ----------
class TrackScene(QGraphicsScene):
    """Draws the track, curvature markers, and start/finish gates."""
    def __init__(self, model: TrackModel):
        super().__init__()
        self.model = model
        self.highlight_index: int | None = None
        self.setBackgroundBrush(QColor("#0c0c0c"))
        self._label_font = QFont()
        self._label_font.setPointSize(9)
        self._label_font.setBold(True)

    def drawBackground(self, painter: QPainter, rect):
        super().drawBackground(painter, rect)
        # Subtle grid based on model grid step
        step = max(5.0, self.model.gridStepMM)
        pen = QPen(QColor(25, 25, 25), 1)
        painter.setPen(pen)
        left, top, right, bottom = rect.left(), rect.top(), rect.right(), rect.bottom()
        x = math.floor(left / step) * step
        while x <= right:
            painter.drawLine(QPointF(x, top), QPointF(x, bottom)); x += step
        y = math.floor(top / step) * step
        while y <= bottom:
            painter.drawLine(QPointF(left, y), QPointF(right, y)); y += step

    def _add_label(self, text: str, x: float, y: float, color: QColor):
        t = self.addText(text, self._label_font)
        t.setDefaultTextColor(color)
        t.setPos(x, y)

    def rebuild(self):
        """Regenerate all scene items from the current model state."""
        self.clear()

        # Build segments with absolute poses
        cur = self.model.origin
        segs: list[AnySeg] = []
        seg_paths: list[QPainterPath] = []
        for s in self.model.segments:
            if isinstance(s, SegStraight):
                seg = SegStraight(kind="straight", id=s.id, from_pose=cur, lengthMM=s.lengthMM)
                cur = advance_straight(seg.from_pose, seg.lengthMM)
            else:
                seg = SegArc(kind="arc", id=s.id, from_pose=cur, radiusMM=s.radiusMM, sweepDeg=s.sweepDeg)
                cur = advance_arc(seg.from_pose, seg.radiusMM, seg.sweepDeg)
            pts = segments_to_polyline([seg])
            path = QPainterPath(QPointF(pts[0].x, pts[0].y))
            for p in pts[1:]:
                path.lineTo(p.x, p.y)
            seg_paths.append(path)
            segs.append(seg)

        # Full track polyline
        pts = segments_to_polyline(segs)
        if len(pts) >= 2:
            full = QPainterPath(QPointF(pts[0].x, pts[0].y))
            for p in pts[1:]:
                full.lineTo(p.x, p.y)
            self.addPath(full, QPen(QColor("#f5f5f5"), self.model.tapeWidthMM,
                                    Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            self.addPath(full, QPen(QColor("#444444"), 1, Qt.DashLine))

        # Curvature-change markers (left side of the track)
        for (p, hdg) in curvature_change_markers(segs):
            hrad = rad(hdg)
            nx, ny = math.sin(hrad), -math.cos(hrad)
            base = (self.model.tapeWidthMM * 0.5) + MARKER_OFFSET_MM
            ax, ay = p.x + nx * base, p.y + ny * base
            bx, by = ax + nx * MARKER_LENGTH_MM, ay + ny * MARKER_LENGTH_MM
            self.addLine(ax, ay, bx, by, QPen(QColor("#FFFFFF"), MARKER_THICKNESS_MM,
                                              Qt.SolidLine, Qt.RoundCap))

        # Start/Finish gates and their right-side ticks
        if self.model.startFinish.enabled and self.model.startFinish.onSegmentId:
            seg = next((s for s in segs if isinstance(s, SegStraight) and s.id == self.model.startFinish.onSegmentId), None)
            if seg:
                t = clamp_start_on_seg(seg, self.model.startFinish.sParamMM)
                start_pose = advance_straight(seg.from_pose, t)
                finish_pose = advance_straight(seg.from_pose, max(0.0, t - START_FINISH_GAP_MM))

                mapping = ((start_pose, "Start"), (finish_pose, "Finish")) \
                          if self.model.startFinish.startIsForward else \
                          ((finish_pose, "Start"), (start_pose, "Finish"))

                for gate_pose, label in mapping:
                    # Cross bar
                    hrad = rad(gate_pose.headingDeg)
                    nx, ny = -math.sin(hrad), math.cos(hrad)
                    half = (self.model.tapeWidthMM * 1.2) * 0.5
                    ax, ay = gate_pose.p.x + nx*half, gate_pose.p.y + ny*half
                    bx, by = gate_pose.p.x - nx*half, gate_pose.p.y - ny*half
                    self.addLine(ax, ay, bx, by, QPen(QColor("#FFD54F"), 4))

                    # Right-side tick (visual asymmetry to distinguish sides)
                    base = (self.model.tapeWidthMM * 0.5) + MARKER_OFFSET_MM
                    sx, sy = gate_pose.p.x + nx*base, gate_pose.p.y + ny*base
                    ex, ey = sx + nx*MARKER_LENGTH_MM, sy + ny*MARKER_LENGTH_MM
                    self.addLine(sx, sy, ex, ey, QPen(QColor("#FFD54F"), MARKER_THICKNESS_MM,
                                                      Qt.SolidLine, Qt.RoundCap))

                    # Label near the bar
                    cx, cy = (ax+bx)/2.0, (ay+by)/2.0
                    lx, ly = cx + nx*14, cy + ny*14
                    self._add_label(label, lx, ly, QColor("#FFD54F"))

        # Highlight currently selected segment
        if self.highlight_index is not None and 0 <= self.highlight_index < len(seg_paths):
            self.addPath(seg_paths[self.highlight_index], QPen(QColor("#00E5FF"), 3, Qt.DashLine))


# ---------- View ----------
class TrackView(QGraphicsView):
    def __init__(self, scene: TrackScene):
        super().__init__(scene)
        self.setRenderHints(self.renderHints() | QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)

    def wheelEvent(self, event):
        s = 1.15 if event.angleDelta().y() > 0 else 1/1.15
        self.scale(s, s)


# ---------- Main Window ----------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Track Editor (Python/PySide6)")

        self.model = TrackModel()
        self.scene = TrackScene(self.model)
        self.view = TrackView(self.scene)
        self.view.setSceneRect(0, 0, self.model.area_widthMM, self.model.area_heightMM)
        self._updating_ui = False

        controls = QWidget(); form = QFormLayout(controls)

        # Area
        self.spin_area_w = QDoubleSpinBox(); self.spin_area_w.setRange(100, 100000); self.spin_area_w.setValue(self.model.area_widthMM)
        self.spin_area_h = QDoubleSpinBox(); self.spin_area_h.setRange(100, 100000); self.spin_area_h.setValue(self.model.area_heightMM)
        self.spin_grid   = QDoubleSpinBox(); self.spin_grid.setRange(5, 1000); self.spin_grid.setValue(self.model.gridStepMM)
        self.spin_tape   = QDoubleSpinBox(); self.spin_tape.setRange(1, 1000); self.spin_tape.setValue(self.model.tapeWidthMM)

        # Origin
        self.spin_ox = QDoubleSpinBox(); self.spin_ox.setRange(-100000,100000); self.spin_ox.setValue(self.model.origin.p.x)
        self.spin_oy = QDoubleSpinBox(); self.spin_oy.setRange(-100000,100000); self.spin_oy.setValue(self.model.origin.p.y)
        self.spin_oh = QDoubleSpinBox(); self.spin_oh.setRange(-360,360); self.spin_oh.setDecimals(1); self.spin_oh.setValue(self.model.origin.headingDeg)

        form.addRow(QLabel("<b>Area (mm)</b>"))
        form.addRow("Width (mm)", self.spin_area_w)
        form.addRow("Height (mm)", self.spin_area_h)
        form.addRow("Grid step (mm)", self.spin_grid)
        form.addRow("Tape width (mm)", self.spin_tape)

        form.addRow(QLabel("<b>Origin</b>"))
        form.addRow("X (mm)", self.spin_ox)
        form.addRow("Y (mm)", self.spin_oy)
        form.addRow("Angle (°)", self.spin_oh)

        # Segments
        form.addRow(QLabel("<b>Segments</b>"))
        self.list_segments = QListWidget(); form.addRow(self.list_segments)
        self.spin_length = QDoubleSpinBox(); self.spin_length.setRange(1, 1e6)
        self.spin_radius = QDoubleSpinBox(); self.spin_radius.setRange(1, 1e6)
        self.spin_sweep  = QDoubleSpinBox(); self.spin_sweep.setRange(-1080,1080); self.spin_sweep.setDecimals(1)
        form.addRow("Length (straight)", self.spin_length)
        form.addRow("Radius (arc)", self.spin_radius)
        form.addRow("Sweep (°)", self.spin_sweep)

        row_btns = QHBoxLayout()
        self.btn_add_st = QPushButton("Add Straight")
        self.btn_add_ar = QPushButton("Add Arc")
        self.btn_del    = QPushButton("Remove")
        self.btn_ren    = QPushButton("Rename (F2)")
        row_btns.addWidget(self.btn_add_st); row_btns.addWidget(self.btn_add_ar); row_btns.addWidget(self.btn_del); row_btns.addWidget(self.btn_ren)
        form.addRow(row_btns)

        # Start/Finish
        form.addRow(QLabel("<b>Start/Finish</b>"))
        self.chk_sf = QCheckBox("Enable Start/Finish")
        self.combo_sf_seg = QComboBox()
        self.spin_sf_s = QDoubleSpinBox(); self.spin_sf_s.setRange(0, 1000000); self.spin_sf_s.setValue(500.0)
        self.slider_sf = QSlider(Qt.Horizontal); self.slider_sf.setRange(0, 10000)
        self.chk_sf_forward = QCheckBox("Start is forward (+s)")
        form.addRow(self.chk_sf)
        form.addRow("Segment (straight)", self.combo_sf_seg)
        form.addRow("s position (mm)", self.spin_sf_s)
        form.addRow("Fine adjust s", self.slider_sf)
        form.addRow(self.chk_sf_forward)

        # File actions
        row1 = QHBoxLayout(); row2 = QHBoxLayout()
        btn_new = QPushButton("New")
        btn_open = QPushButton("Import JSON")
        btn_save = QPushButton("Export JSON")
        btn_fit  = QPushButton("Fit to View")
        row1.addWidget(btn_new); row1.addWidget(btn_open)
        row2.addWidget(btn_save); row2.addWidget(btn_fit)
        form.addRow(row1); form.addRow(row2)

        splitter = QSplitter()
        splitter.addWidget(self.view)
        splitter.addWidget(controls)
        splitter.setSizes([900, 360])
        self.setCentralWidget(splitter)

        # Connections
        self.list_segments.currentRowChanged.connect(self.on_select_segment)
        self.spin_length.valueChanged.connect(self.apply_seg_edit)
        self.spin_radius.valueChanged.connect(self.apply_seg_edit)
        self.spin_sweep.valueChanged.connect(self.apply_seg_edit)

        self.btn_add_st.clicked.connect(self.add_straight)
        self.btn_add_ar.clicked.connect(self.add_arc)
        self.btn_del.clicked.connect(self.del_segment)
        self.btn_ren.clicked.connect(self.rename_segment)

        self.spin_area_w.valueChanged.connect(self.on_area_change)
        self.spin_area_h.valueChanged.connect(self.on_area_change)
        self.spin_grid.valueChanged.connect(self.on_grid_change)
        self.spin_tape.valueChanged.connect(self.on_tape_change)

        self.spin_ox.valueChanged.connect(self.on_origin_change)
        self.spin_oy.valueChanged.connect(self.on_origin_change)
        self.spin_oh.valueChanged.connect(self.on_origin_change)

        self.chk_sf.toggled.connect(self.on_sf_change)
        self.combo_sf_seg.currentIndexChanged.connect(self.on_sf_change)
        self.spin_sf_s.valueChanged.connect(self.on_sf_change)
        self.slider_sf.valueChanged.connect(self.on_sf_slider)
        self.chk_sf_forward.toggled.connect(self.on_sf_change)

        btn_new.clicked.connect(self.new_file)
        btn_open.clicked.connect(self.import_json)
        btn_save.clicked.connect(self.export_json)
        btn_fit.clicked.connect(self.fit_view)

        # Shortcuts (action labels translated)
        act_open = QAction("Import", self);  act_open.setShortcut("Ctrl+O"); act_open.triggered.connect(self.import_json); self.addAction(act_open)
        act_save = QAction("Export", self);  act_save.setShortcut("Ctrl+S"); act_save.triggered.connect(self.export_json); self.addAction(act_save)
        act_fit  = QAction("Fit", self);     act_fit.setShortcut("Ctrl+F");  act_fit.triggered.connect(self.fit_view);    self.addAction(act_fit)
        act_rename = QAction("Rename", self); act_rename.setShortcut("F2");  act_rename.triggered.connect(self.rename_segment); self.addAction(act_rename)

        self.refresh_list()
        self.refresh_sf_combo()
        self.scene.rebuild()
        self.resize(1320, 820)

    # ---------- helpers ----------
    def fmt_item(self, s: AnySeg) -> str:
        if isinstance(s, SegStraight):
            return f"Straight  {s.id} — L={s.lengthMM:.1f} mm"
        else:
            return f"Arc  {s.id} — R={s.radiusMM:.1f} mm, θ={s.sweepDeg:.1f}°"

    def fit_view(self):
        rect = QRectF(0, 0, self.model.area_widthMM, self.model.area_heightMM)
        self.view.fitInView(rect, Qt.KeepAspectRatio)

    # id helpers
    def next_id_for(self, kind: str) -> str:
        if kind == 'straight':
            base = "R"
            nums = [int(''.join(ch for ch in s.id if ch.isdigit()) or "0")
                    for s in self.model.segments if isinstance(s, SegStraight)]
        else:
            base = "C"
            nums = [int(''.join(ch for ch in s.id if ch.isdigit()) or "0")
                    for s in self.model.segments if isinstance(s, SegArc)]
        n = max(nums) + 1 if nums else 1
        return f"{base}{n}"

    def ensure_unique_id(self, candidate: str, ignore_index: int | None = None) -> str:
        ids = {s.id.upper() for s in self.model.segments}
        if ignore_index is not None and 0 <= ignore_index < len(self.model.segments):
            ids.discard(self.model.segments[ignore_index].id.upper())
        base = candidate; i = 1
        while candidate.upper() in ids:
            candidate = f"{base}{i}"; i += 1
        return candidate

    # ---------- handlers ----------
    def refresh_list(self, preserve_index=True):
        cur = self.list_segments.currentRow()
        self.list_segments.blockSignals(True)
        self.list_segments.clear()
        for s in self.model.segments:
            self.list_segments.addItem(QListWidgetItem(self.fmt_item(s)))
        self.list_segments.blockSignals(False)
        if preserve_index and 0 <= cur < self.list_segments.count():
            self.list_segments.setCurrentRow(cur)
        elif self.list_segments.count():
            self.list_segments.setCurrentRow(0)

    def refresh_sf_combo(self):
        self.combo_sf_seg.blockSignals(True)
        self.combo_sf_seg.clear()
        for s in self.model.segments:
            if isinstance(s, SegStraight):
                self.combo_sf_seg.addItem(s.id)
        sf = self.model.startFinish
        if sf.enabled and sf.onSegmentId:
            idx = self.combo_sf_seg.findText(sf.onSegmentId)
            if idx >= 0: self.combo_sf_seg.setCurrentIndex(idx)
        self.combo_sf_seg.blockSignals(False)
        self.chk_sf.setChecked(self.model.startFinish.enabled)
        self.spin_sf_s.setValue(self.model.startFinish.sParamMM)
        self.chk_sf_forward.setChecked(self.model.startFinish.startIsForward)

    def on_select_segment(self, row: int):
        self.scene.highlight_index = row
        if 0 <= row < len(self.model.segments):
            s = self.model.segments[row]
            self.spin_length.blockSignals(True); self.spin_radius.blockSignals(True); self.spin_sweep.blockSignals(True)
            if isinstance(s, SegStraight):
                self.spin_length.setEnabled(True); self.spin_radius.setEnabled(False); self.spin_sweep.setEnabled(False)
                self.spin_length.setValue(s.lengthMM)
            else:
                self.spin_length.setEnabled(False); self.spin_radius.setEnabled(True); self.spin_sweep.setEnabled(True)
                self.spin_radius.setValue(s.radiusMM); self.spin_sweep.setValue(s.sweepDeg)
            self.spin_length.blockSignals(False); self.spin_radius.blockSignals(False); self.spin_sweep.blockSignals(False)
        self.scene.rebuild()

    def apply_seg_edit(self):
        row = self.list_segments.currentRow()
        if 0 <= row < len(self.model.segments):
            s = self.model.segments[row]
            if isinstance(s, SegStraight) and self.spin_length.isEnabled():
                s.lengthMM = self.spin_length.value()
            elif isinstance(s, SegArc) and (self.spin_radius.isEnabled() or self.spin_sweep.isEnabled()):
                s.radiusMM = self.spin_radius.value(); s.sweepDeg = self.spin_sweep.value()
            item = self.list_segments.item(row)
            if item: item.setText(self.fmt_item(s))
            self.scene.rebuild(); self.refresh_sf_combo()

    def add_straight(self):
        seg = SegStraight(kind="straight", id=self.ensure_unique_id(self.next_id_for('straight')),
                          from_pose=self.model.origin, lengthMM=500.0)
        self.model.segments.append(seg)
        self.refresh_list(False); self.list_segments.setCurrentRow(self.list_segments.count()-1)
        self.scene.rebuild(); self.refresh_sf_combo()

    def add_arc(self):
        seg = SegArc(kind="arc", id=self.ensure_unique_id(self.next_id_for('arc')),
                     from_pose=self.model.origin, radiusMM=300.0, sweepDeg=90.0)
        self.model.segments.append(seg)
        self.refresh_list(False); self.list_segments.setCurrentRow(self.list_segments.count()-1)
        self.scene.rebuild(); self.refresh_sf_combo()

    def del_segment(self):
        row = self.list_segments.currentRow()
        if 0 <= row < len(self.model.segments):
            if self.model.startFinish.onSegmentId == self.model.segments[row].id:
                self.model.startFinish.onSegmentId = None
            del self.model.segments[row]
            self.refresh_list(True); self.scene.rebuild(); self.refresh_sf_combo()

    def rename_segment(self):
        row = self.list_segments.currentRow()
        if 0 <= row < len(self.model.segments):
            s = self.model.segments[row]
            old_id = s.id
            entered, ok = QInputDialog.getText(self, "Rename segment", "New ID:", text=old_id)
            if ok and entered:
                entered = str(entered).strip().upper()
                digits = ''.join(ch for ch in entered if ch.isdigit())
                candidate = f"{'R' if isinstance(s, SegStraight) else 'C'}{digits}" if digits else self.next_id_for('straight' if isinstance(s, SegStraight) else 'arc')
                candidate = self.ensure_unique_id(candidate, row)
                s.id = candidate
                if self.model.startFinish.onSegmentId == old_id:
                    self.model.startFinish.onSegmentId = s.id
                item = self.list_segments.item(row)
                if item: item.setText(self.fmt_item(s))
                self.refresh_sf_combo()
                self.scene.rebuild()

    # Area/origin/tape
    def on_area_change(self):
        self.model.area_widthMM = self.spin_area_w.value()
        self.model.area_heightMM = self.spin_area_h.value()
        self.view.setSceneRect(0, 0, self.model.area_widthMM, self.model.area_heightMM)
        self.scene.rebuild()

    def on_grid_change(self):
        self.model.gridStepMM = self.spin_grid.value()
        self.scene.rebuild()

    def on_tape_change(self):
        self.model.tapeWidthMM = self.spin_tape.value()
        self.scene.rebuild()

    def on_origin_change(self):
        self.model.origin = Pose(Pt(self.spin_ox.value(), self.spin_oy.value()), self.spin_oh.value())
        # Origin is only a reference for visualization; segments keep their accumulated poses
        self.scene.rebuild()

    # Start/finish controls
    def on_sf_change(self):
        sf = self.model.startFinish
        sf.enabled = self.chk_sf.isChecked()
        if self.combo_sf_seg.currentIndex() >= 0:
            sf.onSegmentId = self.combo_sf_seg.currentText()
        sf.sParamMM = self.spin_sf_s.value()
        sf.startIsForward = self.chk_sf_forward.isChecked()
        self.scene.rebuild()

    def on_sf_slider(self, v: int):
        # Linear mapping 0..10000 -> s in millimeters (simple; can be tied to actual segment length)
        self.spin_sf_s.setValue(float(v))
        self.on_sf_change()

    # File I/O
    def new_file(self):
        self.model = TrackModel()
        self.scene.model = self.model
        self.refresh_list(); self.refresh_sf_combo(); self.scene.rebuild()

    def import_json(self):
        path, _ = QFileDialog.getOpenFileName(self, "Import track", "", "JSON (*.json)")
        if not path: return
        try:
            with open(path, "r", encoding="utf-8") as f:
                obj = json.load(f)
            self.model = TrackModel.from_json(obj)
            self.scene.model = self.model
            self.refresh_list(); self.refresh_sf_combo(); self.scene.rebuild()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Import failed:\n{e}")

    def export_json(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export track", "track.json", "JSON (*.json)")
        if not path: return
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self.model.to_json(), f, ensure_ascii=False, indent=2)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Export failed:\n{e}")


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
