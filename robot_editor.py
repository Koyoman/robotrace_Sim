"""Robot Editor UI (PySide6)

This module provides a simple, beginner-friendly graphical editor for a small robot.
You can see the robot envelope (overall size), place the left/right wheels, and add
square sensors on a grid. The editor lets you import/export a JSON robot definition,
zoom/pan the view, and nudge sensors with the keyboard.

The comments and docstrings in this file aim to explain each part in plain English
so that someone new to Python/Qt can follow along.
"""
from __future__ import annotations
import sys, json
from typing import Optional, Tuple

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QGraphicsScene, QGraphicsView, QFormLayout,
    QVBoxLayout, QHBoxLayout, QLabel, QDoubleSpinBox, QPushButton, QListWidget,
    QListWidgetItem, QFileDialog, QMessageBox, QSplitter, QCheckBox, QInputDialog, QGridLayout,
    QDialog
)
from PySide6.QtGui import QPen, QColor, QAction, QPainter, QKeyEvent
from PySide6.QtCore import Qt, QPointF, QRectF, Signal, QTimer

from Utils.robot_geometry import (
    Pt, Envelope,
    DEFAULT_WHEEL_W, DEFAULT_WHEEL_H, DEFAULT_SENSOR_SIZE,
)
from Utils.robot_model import RobotModel, Wheel, Sensor

TAPE_THICKNESS_MM = 20.0

class ElectricalDialog(QDialog):
    """Modal dialog to capture basic electrical parameters (battery and motor).
    Values are written back into the provided model on accept()."""
    def __init__(self, model: RobotModel, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Electrical parameters")
        self.model = model

        form = QFormLayout(self)

        self.spin_batt_v = QDoubleSpinBox()
        self.spin_batt_v.setRange(0.1, 60.0)
        self.spin_batt_v.setDecimals(2)
        self.spin_batt_v.setSuffix(" V")
        self.spin_batt_v.setToolTip("Battery nominal voltage in Volts.")
        self.spin_batt_cap = QDoubleSpinBox()
        self.spin_batt_cap.setRange(1.0, 20000.0)
        self.spin_batt_cap.setDecimals(0)
        self.spin_batt_cap.setSuffix(" mAh")
        self.spin_batt_cap.setToolTip("Battery capacity in milliamp-hours.")

        self.spin_motor_rpm = QDoubleSpinBox()
        self.spin_motor_rpm.setRange(1.0, 100000.0)
        self.spin_motor_rpm.setDecimals(0)
        self.spin_motor_rpm.setSuffix(" rpm")
        self.spin_motor_rpm.setToolTip("Motor no-load RPM at battery voltage.")
        self.spin_motor_i = QDoubleSpinBox()
        self.spin_motor_i.setRange(0.01, 100.0)
        self.spin_motor_i.setDecimals(2)
        self.spin_motor_i.setSuffix(" A")
        self.spin_motor_i.setToolTip("Estimated max current per motor (Amps).")

        bv = getattr(self.model, "batteryVoltageV", 7.4)
        bc = getattr(self.model, "batteryCapacitymAh", 850.0)
        mr = getattr(self.model, "motorNoLoadRPM", 10000.0)
        mi = getattr(self.model, "motorMaxCurrentA", 1.00)
        self.spin_batt_v.setValue(float(bv))
        self.spin_batt_cap.setValue(float(bc))
        self.spin_motor_rpm.setValue(float(mr))
        self.spin_motor_i.setValue(float(mi))

        form.addRow(QLabel("<b>Battery</b>"))
        form.addRow("Voltage", self.spin_batt_v)
        form.addRow("Capacity", self.spin_batt_cap)
        form.addRow(QLabel("<b>Motor</b>"))
        form.addRow("No-load RPM", self.spin_motor_rpm)
        form.addRow("Max current", self.spin_motor_i)

        btns = QHBoxLayout()
        self.btn_ok = QPushButton("OK")
        self.btn_cancel = QPushButton("Cancel")
        btns.addWidget(self.btn_ok); btns.addWidget(self.btn_cancel)
        form.addRow(btns)
        self.btn_ok.clicked.connect(self.accept)
        self.btn_cancel.clicked.connect(self.reject)

    def accept(self):
        """Copy values to model and close."""
        self.model.batteryVoltageV = float(self.spin_batt_v.value())
        self.model.batteryCapacitymAh = float(self.spin_batt_cap.value())
        self.model.motorNoLoadRPM = float(self.spin_motor_rpm.value())
        self.model.motorMaxCurrentA = float(self.spin_motor_i.value())
        super().accept()

class RobotScene(QGraphicsScene):
    """QGraphicsScene that draws the robot, grid, and selection.
Left-click selects items; right-click moves the origin.
It emits signals when the selection or origin changes for the UI to update."""
    selChanged = Signal(object, object)
    originMoved = Signal(float, float)

    def __init__(self, model: RobotModel):
        """Set up the window, build the form controls, connect signals, and show the editor."""
        super().__init__()
        self.model = model
        self.setBackgroundBrush(QColor("#0c0c0c"))
        self.center_scene = QPointF(600.0, 420.0)

        self.selected_kind: Optional[str] = None
        self.selected_id: Optional[str] = None

    def wheel_w(self) -> float:
        return float(getattr(self.model, "wheelWidthMM", DEFAULT_WHEEL_W if 'DEFAULT_WHEEL_W' in globals() else 22.0))

    def wheel_h(self) -> float:
        return float(getattr(self.model, "wheelHeightMM", DEFAULT_WHEEL_H if 'DEFAULT_WHEEL_H' in globals() else 15.0))

    def mousePressEvent(self, event):
        """Handle mouse clicks. Left-click selects wheels/sensors; right-click moves the origin (0,0)."""
        if event.button() == Qt.LeftButton:
            pos = event.scenePos()
            clicked = self._hit_test(pos)
            if clicked is None:
                self.selected_kind, self.selected_id = None, None
            else:
                k, _id = clicked
                if self.selected_kind == k and self.selected_id == _id:
                    self.selected_kind, self.selected_id = None, None
                else:
                    self.selected_kind, self.selected_id = k, _id
            self.selChanged.emit(self.selected_kind, self.selected_id)
            self.rebuild()
        elif event.button() == Qt.RightButton:
            p = event.scenePos()
            ox = p.x() - self.center_scene.x()
            oy = p.y() - self.center_scene.y()
            self.model.originXMM = ox
            self.model.originYMM = oy
            self.model.clamp_all_inside()
            self.originMoved.emit(self.model.originXMM, self.model.originYMM)
            self.selChanged.emit(None, None)
            self.rebuild()
        super().mousePressEvent(event)

    def _hit_test(self, p: QPointF) -> Optional[Tuple[str, str]]:
        ww = self.wheel_w()
        wh = self.wheel_h()
        for w in self.model.wheels:
            x = self.center_scene.x() + w.xMM
            y = self.center_scene.y() + w.yMM
            if (abs(p.x() - x) <= ww/2.0) and (abs(p.y() - y) <= wh/2.0):
                return ("wheel", w.id)
        for s in self.model.sensors:
            x = self.center_scene.x() + s.xMM
            y = self.center_scene.y() + s.yMM
            if (abs(p.x() - x) <= s.sizeMM/2.0) and (abs(p.y() - y) <= s.sizeMM/2.0):
                return ("sensor", s.id)
        return None

    def drawBackground(self, painter: QPainter, rect):
        """Draw a dark grid to help the user align parts."""
        super().drawBackground(painter, rect)
        step = max(5.0, self.model.gridStepMM)
        pen = QPen(QColor(25, 25, 25), 1)
        painter.setPen(pen)
        left, top, right, bottom = rect.left(), rect.top(), rect.right(), rect.bottom()
        x = (left // step) * step
        while x <= right:
            painter.drawLine(x, top, x, bottom)
            x += step
        y = (top // step) * step
        while y <= bottom:
            painter.drawLine(left, y, right, y)
            y += step

    def rebuild(self):
        """Repaint the entire scene: envelope, axes, tape line, origin crosshair, wheels, and sensors."""
        self.clear()
        c = self.center_scene
        halfW = self.model.envelope.widthMM / 2.0
        halfH = self.model.envelope.heightMM / 2.0

        area_pen = QPen(QColor("#1f1f1f"), 1)
        self.addRect(0, 0, c.x()*2, c.y()*2, area_pen)

        env_pen = QPen(QColor("#FFD54F")); env_pen.setWidth(1); env_pen.setStyle(Qt.DashLine)
        env_brush = QColor(255, 213, 79, 40)
        self.addRect(c.x()-halfW, c.y()-halfH, self.model.envelope.widthMM, self.model.envelope.heightMM, env_pen, env_brush)

        axis_pen = QPen(QColor("#303030"), 1)
        self.addLine(c.x()-halfW-60, c.y(), c.x()+halfW+60, c.y(), axis_pen)
        self.addLine(c.x(), c.y()-halfH-60, c.x(), c.y()+halfH+60, axis_pen)

        tape_pen = QPen(QColor("#2b2b2b"), TAPE_THICKNESS_MM, Qt.SolidLine, Qt.RoundCap)
        self.addLine(c.x()-halfW-200, c.y(), c.x()+halfW+200, c.y(), tape_pen)

        ox = c.x() + self.model.originXMM
        oy = c.y() + self.model.originYMM
        o_pen = QPen(QColor("#00E5FF"), 2, Qt.SolidLine, Qt.RoundCap)
        self.addLine(ox-8, oy, ox+8, oy, o_pen)
        self.addLine(ox, oy-8, ox, oy+8, o_pen)

        ww = self.wheel_w()
        wh = self.wheel_h()
        for w in self.model.wheels:
            x = c.x() + w.xMM
            y = c.y() + w.yMM
            pen = QPen(QColor("#000000"), 1)
            brush = QColor("#dddddd")
            self.addRect(
                x-ww/2.0, y-wh/2.0, ww, wh, pen, brush
            )
            if self.selected_kind == "wheel" and self.selected_id == w.id:
                sel_pen = QPen(QColor("#00E5FF"), 2)
                self.addRect(x-ww/2.0, y-wh/2.0, ww, wh, sel_pen)

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
    """QGraphicsView wrapper that enables smooth zooming and panning
        so the user can inspect the robot easily."""
    def __init__(self, scene: RobotScene):
        super().__init__(scene)
        self.setRenderHints(self.renderHints() | QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)

    def wheelEvent(self, event):
        """Zoom in/out with the mouse wheel so the user can inspect details."""
        factor = 1.15 if event.angleDelta().y() > 0 else 1/1.15
        self.scale(factor, factor)

class MainWindow(QMainWindow):
    """Main application window that assembles the scene, view, and
        property panel. It wires up the buttons/spin boxes to the robot model."""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Editor (Python/PySide6)")

        self.model = RobotModel()

        if not hasattr(self.model, "wheelWidthMM"):  self.model.wheelWidthMM = 22.0
        if not hasattr(self.model, "wheelHeightMM"): self.model.wheelHeightMM = 15.0
        if not hasattr(self.model, "batteryVoltageV"):     self.model.batteryVoltageV = 7.4
        if not hasattr(self.model, "batteryCapacitymAh"):  self.model.batteryCapacitymAh = 850.0
        if not hasattr(self.model, "motorNoLoadRPM"):      self.model.motorNoLoadRPM = 10000.0
        if not hasattr(self.model, "motorMaxCurrentA"):    self.model.motorMaxCurrentA = 1.0

        self.scene = RobotScene(self.model)
        self.view = RobotView(self.scene)
        self.view.setSceneRect(0, 0, 1600, 1000)

        self.scene.selChanged.connect(self.on_scene_selection)
        self.scene.originMoved.connect(self.on_scene_origin_moved)

        props = QWidget()
        form = QFormLayout(props)

        envelope_hint = (
            "Defines the robot’s bounding box used for simulation limits.\n"
            "Width and height are given in millimeters (mm)."
        )
        grid_hint = "Grid spacing used for snapping sensors and position alignment."

        envelope_title = QLabel("<b>Envelope</b>")
        envelope_title.setToolTip(envelope_hint)
        form.addRow(envelope_title)

        self.spin_env_w = QDoubleSpinBox()
        self.spin_env_w.setRange(10, 250)
        self.spin_env_w.setValue(self.model.envelope.widthMM)
        self.spin_env_w.setSuffix(" mm")
        self.spin_env_w.setToolTip("Width of the robot envelope in millimeters.")

        self.spin_env_h = QDoubleSpinBox()
        self.spin_env_h.setRange(10, 250)
        self.spin_env_h.setValue(self.model.envelope.heightMM)
        self.spin_env_h.setSuffix(" mm")
        self.spin_env_h.setToolTip("Height of the robot envelope in millimeters.")

        self.spin_grid = QDoubleSpinBox()
        self.spin_grid.setRange(2, 200)
        self.spin_grid.setValue(self.model.gridStepMM)
        self.spin_grid.setSuffix(" mm")
        self.spin_grid.setToolTip(grid_hint)

        env_grid = QGridLayout()
        env_grid.setHorizontalSpacing(8)
        env_grid.addWidget(QLabel("Width"), 0, 0)
        env_grid.addWidget(QLabel("Height"), 0, 1)
        env_grid.addWidget(QLabel("Grid"), 0, 2)
        env_grid.addWidget(self.spin_env_w, 1, 0)
        env_grid.addWidget(self.spin_env_h, 1, 1)
        env_grid.addWidget(self.spin_grid, 1, 2)
        env_widget = QWidget()
        env_widget.setLayout(env_grid)
        form.addRow(env_widget)

        origin_hint = (
            "The <b>Origin</b> defines the <b>X,Y position of the robot in the simulation</b>.\n"
            "Right-click on the canvas to move the origin or use the fields below."
        )

        self.spin_origin_x = QDoubleSpinBox()
        self.spin_origin_x.setRange(-10000, 10000)
        self.spin_origin_x.setValue(self.model.originXMM)
        self.spin_origin_x.setSuffix(" mm")
        self.spin_origin_x.setToolTip(origin_hint)

        self.spin_origin_y = QDoubleSpinBox()
        self.spin_origin_y.setRange(-10000, 10000)
        self.spin_origin_y.setValue(self.model.originYMM)
        self.spin_origin_y.setSuffix(" mm")
        self.spin_origin_y.setToolTip(origin_hint)

        origin_grid = QGridLayout()
        origin_grid.addWidget(QLabel("Origin X"), 0, 0)
        origin_grid.addWidget(QLabel("Origin Y"), 0, 1)
        origin_grid.addWidget(self.spin_origin_x, 1, 0)
        origin_grid.addWidget(self.spin_origin_y, 1, 1)
        origin_widget = QWidget()
        origin_widget.setLayout(origin_grid)
        form.addRow(origin_widget)

        self.btn_origin_mid_wheels = QPushButton("Origin = middle of wheels")
        self.btn_origin_mid_wheels.setToolTip(
            "Set the origin exactly at the midpoint between the left and right wheel centers."
        )
        form.addRow(self.btn_origin_mid_wheels)

        self.spin_wl_x = QDoubleSpinBox()
        self.spin_wl_x.setRange(-10000, 10000)
        self.spin_wl_x.setSuffix(" mm")
        self.spin_wl_x.setToolTip("Left wheel X position relative to the origin (mm).")

        self.spin_wl_y = QDoubleSpinBox()
        self.spin_wl_y.setRange(-10000, 10000)
        self.spin_wl_y.setSuffix(" mm")
        self.spin_wl_y.setToolTip("Left wheel Y position relative to the origin (mm).")

        self.spin_wr_x = QDoubleSpinBox()
        self.spin_wr_x.setRange(-10000, 10000)
        self.spin_wr_x.setSuffix(" mm")
        self.spin_wr_x.setToolTip("Right wheel X position relative to the origin (mm).")

        self.spin_wr_y = QDoubleSpinBox()
        self.spin_wr_y.setRange(-10000, 10000)
        self.spin_wr_y.setSuffix(" mm")
        self.spin_wr_y.setToolTip("Right wheel Y position relative to the origin (mm).")

        self.spin_wheel_w = QDoubleSpinBox()
        self.spin_wheel_w.setRange(1.0, 200.0)
        self.spin_wheel_w.setValue(float(getattr(self.model, "wheelWidthMM", 22.0)))
        self.spin_wheel_w.setSuffix(" mm")
        self.spin_wheel_w.setToolTip("Wheel rectangle width (X) in mm. Default 22.")

        self.spin_wheel_h = QDoubleSpinBox()
        self.spin_wheel_h.setRange(1.0, 200.0)
        self.spin_wheel_h.setValue(float(getattr(self.model, "wheelHeightMM", 15.0)))
        self.spin_wheel_h.setSuffix(" mm")
        self.spin_wheel_h.setToolTip("Wheel rectangle height (Y) in mm. Default 15.")

        wheels_grid = QGridLayout()
        wheels_grid.setHorizontalSpacing(8)
        wheels_grid.addWidget(QLabel("<b>Wheels</b>"), 0, 0)
        wheels_grid.addWidget(QLabel("X"), 0, 1)
        wheels_grid.addWidget(QLabel("Y"), 0, 2)
        wheels_grid.addWidget(QLabel("Left"), 1, 0)
        wheels_grid.addWidget(self.spin_wl_x, 1, 1)
        wheels_grid.addWidget(self.spin_wl_y, 1, 2)
        wheels_grid.addWidget(QLabel("Right"), 2, 0)
        wheels_grid.addWidget(self.spin_wr_x, 2, 1)
        wheels_grid.addWidget(self.spin_wr_y, 2, 2)
        wheels_grid.addWidget(QLabel("Size"), 3, 0)
        wheels_grid.addWidget(self.spin_wheel_w, 3, 1)
        wheels_grid.addWidget(self.spin_wheel_h, 3, 2)
        wheels_widget = QWidget()
        wheels_widget.setLayout(wheels_grid)
        form.addRow(wheels_widget)

        sensors_hint = (
            "Defines the positions of all sensors (e.g., line or distance sensors)\n"
            "relative to the robot’s origin in millimeters."
        )
        sensors_title = QLabel("<b>Sensors (5 x 5 mm)</b>")
        sensors_title.setToolTip(sensors_hint)
        form.addRow(sensors_title)

        self.list_sensors = QListWidget()
        self.list_sensors.setToolTip("List of all defined sensors and their coordinates.")
        btn_row = QHBoxLayout()
        self.btn_add_s = QPushButton("Add Sensor");   self.btn_add_s.setToolTip("Add a new sensor to the robot.")
        self.btn_del_s = QPushButton("Remove Sensor"); self.btn_del_s.setToolTip("Remove the selected sensor.")
        self.btn_rename_s = QPushButton("Rename (F2)"); self.btn_rename_s.setToolTip("Rename the selected sensor.")
        btn_row.addWidget(self.btn_add_s)
        btn_row.addWidget(self.btn_del_s)
        btn_row.addWidget(self.btn_rename_s)
        form.addRow(self.list_sensors)
        form.addRow(btn_row)

        self.spin_s_x = QDoubleSpinBox()
        self.spin_s_x.setRange(-10000, 10000)
        self.spin_s_x.setSuffix(" mm")
        self.spin_s_x.setToolTip("Sensor X position relative to the origin (mm).")

        self.spin_s_y = QDoubleSpinBox()
        self.spin_s_y.setRange(-10000, 10000)
        self.spin_s_y.setSuffix(" mm")
        self.spin_s_y.setToolTip("Sensor Y position relative to the origin (mm).")

        self.chk_snap = QCheckBox("Snap to grid (uses Grid mm)")
        self.chk_snap.setToolTip("When enabled, sensor coordinates snap to the defined grid step.")

        # Place Sensor X and Sensor Y on a single row (labels on top, fields below).
        sensor_xy_grid = QGridLayout()
        sensor_xy_grid.setHorizontalSpacing(8)
        sensor_xy_grid.addWidget(QLabel("Sensor X"), 0, 0)
        sensor_xy_grid.addWidget(QLabel("Sensor Y"), 0, 1)
        sensor_xy_grid.addWidget(self.spin_s_x, 1, 0)
        sensor_xy_grid.addWidget(self.spin_s_y, 1, 1)
        sensor_xy_widget = QWidget()
        sensor_xy_widget.setLayout(sensor_xy_grid)
        form.addRow(sensor_xy_widget)

        form.addRow(self.chk_snap)

        self.btn_elec   = QPushButton("Electrical parameters…"); self.btn_elec.setToolTip("Open a dialog to edit battery/motor parameters. These are saved to JSON.")
        form.addRow(self.btn_elec)

        file_hint = "Import or export robot configuration files, or fit the view to the current layout."
        files_row = QHBoxLayout()
        self.btn_import = QPushButton("Import JSON"); self.btn_import.setToolTip("Load a robot configuration from a JSON file.")
        self.btn_export = QPushButton("Export JSON"); self.btn_export.setToolTip("Save the current robot configuration to a JSON file.")
        self.btn_fit    = QPushButton("Fit View");    self.btn_fit.setToolTip("Adjust zoom to fit the entire robot in view.")
        files_row.addWidget(self.btn_import)
        files_row.addWidget(self.btn_export)
        files_row.addWidget(self.btn_fit)
        form.addRow(files_row)

        splitter = QSplitter()
        splitter.addWidget(self.view)
        splitter.addWidget(props)
        splitter.setSizes([1000, 400])
        self.setCentralWidget(splitter)

        act_open = QAction("Import", self); act_open.setShortcut("Ctrl+O"); act_open.triggered.connect(self.on_import); self.addAction(act_open)
        act_save = QAction("Export", self); act_save.setShortcut("Ctrl+S"); act_save.triggered.connect(self.on_export); self.addAction(act_save)
        act_fit  = QAction("Fit", self);    act_fit.setShortcut("Ctrl+F");  act_fit.triggered.connect(self.on_fit);    self.addAction(act_fit)
        act_rename = QAction("Rename sensor", self); act_rename.setShortcut("F2"); act_rename.triggered.connect(self.rename_sensor); self.addAction(act_rename)

        self.spin_env_w.valueChanged.connect(self.on_env_change)
        self.spin_env_h.valueChanged.connect(self.on_env_change)
        self.spin_grid.valueChanged.connect(self.on_grid_change)

        self.spin_origin_x.valueChanged.connect(self.on_origin_change)
        self.spin_origin_y.valueChanged.connect(self.on_origin_change)
        self.btn_origin_mid_wheels.clicked.connect(self.on_origin_from_mid_wheels)

        self.spin_wheel_w.valueChanged.connect(self.on_wheel_size_change)
        self.spin_wheel_h.valueChanged.connect(self.on_wheel_size_change)

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
        self.btn_elec.clicked.connect(self.on_open_electrical)

        self.refresh_all()
        self.scene.rebuild()
        self.resize(1280, 800)

        QTimer.singleShot(0, self.on_fit)

    def on_fit(self):
        """Adjust the zoom so the whole robot fits nicely on screen."""
        halfW = self.model.envelope.widthMM/2.0
        halfH = self.model.envelope.heightMM/2.0
        c = self.scene.center_scene
        rect = QRectF(c.x()-halfW-100, c.y()-halfH-100, self.model.envelope.widthMM+200, self.model.envelope.heightMM+200)
        self.view.fitInView(rect, Qt.KeepAspectRatio)

    def on_env_change(self):
        """Apply new envelope size (width/height). Also refresh wheel/sensor panels and redraw."""
        self.model.set_envelope(self.spin_env_w.value(), self.spin_env_h.value())
        self.refresh_wheels_fields()
        self.refresh_sensors_list()
        self.refresh_sensors_fields()
        self.scene.rebuild()

    def on_grid_change(self):
        """Apply a new grid spacing. Optionally snap sensor positions to the grid."""
        self.model.gridStepMM = self.spin_grid.value()
        if self.chk_snap.isChecked():
            self.on_sensor_change()
        self.scene.rebuild()

    def on_scene_origin_moved(self, x: float, y: float):
        """Update Origin X/Y spin boxes when the origin is moved directly on the canvas."""
        self.spin_origin_x.blockSignals(True); self.spin_origin_y.blockSignals(True)
        self.spin_origin_x.setValue(x); self.spin_origin_y.setValue(y)
        self.spin_origin_x.blockSignals(False); self.spin_origin_y.blockSignals(False)

        self.model.originXMM = x
        self.model.originYMM = y
        self.scene.rebuild()

    def on_origin_change(self):
        """Apply new origin coordinates typed by the user, clamping to safe limits."""
        self.model.originXMM = self.spin_origin_x.value()
        self.model.originYMM = self.spin_origin_y.value()
        self.model.clamp_all_inside()
        self.spin_origin_x.blockSignals(True); self.spin_origin_y.blockSignals(True)
        self.spin_origin_x.setValue(self.model.originXMM); self.spin_origin_y.setValue(self.model.originYMM)
        self.spin_origin_x.blockSignals(False); self.spin_origin_y.blockSignals(False)
        self.scene.rebuild()

    def on_origin_from_mid_wheels(self):
        """Place the origin exactly at the midpoint between the two wheels."""
        wl = self.model.find_wheel("left")
        wr = self.model.find_wheel("right")
        if not wl or not wr:
            QMessageBox.warning(self, "Wheels not found",
                                "You need to have the left and right wheels configured to calculate the midpoint.")
            return
        self.model.originXMM = (wl.xMM + wr.xMM) / 2.0
        self.model.originYMM = (wl.yMM + wr.yMM) / 2.0
        self.model.clamp_all_inside()
        self.spin_origin_x.blockSignals(True); self.spin_origin_y.blockSignals(True)
        self.spin_origin_x.setValue(self.model.originXMM); self.spin_origin_y.setValue(self.model.originYMM)
        self.spin_origin_x.blockSignals(False); self.spin_origin_y.blockSignals(False)
        self.scene.rebuild()

    def on_wheel_size_change(self):
        """Update wheel rectangle width/height and redraw."""
        self.model.wheelWidthMM = float(self.spin_wheel_w.value())
        self.model.wheelHeightMM = float(self.spin_wheel_h.value())
        self.scene.rebuild()

    def on_wheel_change(self):
        """Save wheel positions from the spin boxes and redraw the scene."""
        wl = self.model.find_wheel("left")
        wr = self.model.find_wheel("right")
        if wl:
            wl.xMM = self.spin_wl_x.value()
            wl.yMM = self.spin_wl_y.value()
        if wr:
            wr.xMM = self.spin_wr_x.value()
            wr.yMM = self.spin_wr_y.value()
        self.model.clamp_all_inside()
        self.refresh_wheels_fields()
        self.scene.selected_kind, self.scene.selected_id = "wheel", "left"
        self.scene.rebuild()

    def on_add_sensor(self):
        """Create a new sensor at the current X/Y fields (snaps to grid if enabled)."""
        x, y = self.spin_s_x.value(), self.spin_s_y.value()
        if self.chk_snap.isChecked():
            step = max(1e-6, self.model.gridStepMM)
            x = round(x / step) * step
            y = round(y / step) * step
        s = self.model.add_sensor(x, y)
        self.refresh_sensors_list()
        self.select_sensor_id(s.id)
        self.scene.selected_kind, self.scene.selected_id = "sensor", s.id
        self.scene.rebuild()

    def on_del_sensor(self):
        """Delete the currently selected sensor from the list.
        After deletion, automatically select the last remaining sensor (if any)
        so the user can keep pressing Delete to remove all without reselecting."""
        sid = self.current_sensor_id()
        if sid is None:
            return
        # Remove from model
        self.model.remove_sensor(sid)
        # Refresh UI list
        self.refresh_sensors_list()
        # If there are sensors left, select the last one; otherwise clear selection/fields
        count = self.list_sensors.count()
        if count > 0:
            last_row = count - 1
            self.list_sensors.setCurrentRow(last_row)
            # Ensure scene selection and fields reflect the new current item
            new_sid = self.list_sensors.item(last_row).data(Qt.UserRole)
            self.scene.selected_kind, self.scene.selected_id = "sensor", new_sid
            # Sync fields for the selected sensor
            self.on_select_sensor(last_row)
        else:
            # No sensors left: clear selection and fields
            self.scene.selected_kind, self.scene.selected_id = None, None
            self.refresh_sensors_fields()
        self.scene.rebuild()

    def on_select_sensor(self, row: int):
        """When the user selects a sensor in the list, reflect its coordinates in the fields."""
        sid = self.current_sensor_id()
        if sid is None:
            return
        s = self.model.find_sensor(sid)
        if not s:
            return
        self.spin_s_x.blockSignals(True); self.spin_s_y.blockSignals(True)
        self.spin_s_x.setValue(s.xMM); self.spin_s_y.setValue(s.yMM)
        self.spin_s_x.blockSignals(False); self.spin_s_y.blockSignals(False)
        self.scene.selected_kind, self.scene.selected_id = "sensor", s.id
        self.scene.rebuild()

    def on_sensor_change(self):
        """Apply new sensor coordinates from the fields (with optional snapping and clamping)."""
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
        self.spin_s_x.blockSignals(True); self.spin_s_y.blockSignals(True)
        self.spin_s_x.setValue(s.xMM); self.spin_s_y.setValue(s.yMM)
        self.spin_s_x.blockSignals(False); self.spin_s_y.blockSignals(False)
        self.update_current_sensor_list_item_text()
        self.scene.rebuild()

    def on_scene_selection(self, kind: Optional[str], sid: Optional[str]):
        """Keep the list/fields in sync when an item is selected directly on the canvas."""
        if kind == "sensor" and sid:
            self.select_sensor_id(sid)
        elif kind == "wheel" and sid:
            wl = self.model.find_wheel("left")
            wr = self.model.find_wheel("right")
            if wl and wr:
                self.refresh_wheels_fields()

    def rename_sensor(self):
        """Rename the selected sensor. Ensures IDs stay unique and tidy (e.g., S1, S2…)."""
        sid = self.current_sensor_id()
        if sid is None: return
        s = self.model.find_sensor(sid)
        if not s: return
        entered, ok = QInputDialog.getText(self, "Rename sensor", "New ID:", text=s.id)
        if not ok or not entered:
            return
        new_id = str(entered).strip().upper()
        import re
        m = re.match(r'^\s*S?(\d+)\s*$', new_id)
        if m:
            new_id = f"S{int(m.group(1))}"
        existing = {x.id.upper() for x in self.model.sensors if x is not s}
        base = new_id
        i = 1
        while new_id.upper() in existing:
            m = re.match(r'^(.*?)(\d+)$', base)
            if m:
                base_root, num = m.group(1), int(m.group(2))
                num += 1
                new_id = f"{base_root}{num}"
            else:
                new_id = f"{base}{i}"
                i += 1
        s.id = new_id
        self.refresh_sensors_list()
        self.select_sensor_id(s.id)
        self.scene.selected_kind, self.scene.selected_id = "sensor", s.id
        self.scene.rebuild()

    def keyPressEvent(self, event: QKeyEvent):
        """Allow nudging the selected sensor with arrow keys. Hold Shift/Ctrl/Alt for bigger/smaller steps."""
        handled = False
        sid = self.current_sensor_id()
        if sid is not None:
            s = self.model.find_sensor(sid)
            if s:
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
                    self.spin_s_x.blockSignals(True); self.spin_s_y.blockSignals(True)
                    self.spin_s_x.setValue(s.xMM); self.spin_s_y.setValue(s.yMM)
                    self.spin_s_x.blockSignals(False); self.spin_s_y.blockSignals(False)
                    self.update_current_sensor_list_item_text()
                    self.scene.selected_kind, self.scene.selected_id = "sensor", s.id
                    self.scene.rebuild()
        if not handled:
            super().keyPressEvent(event)

    def on_import(self):
        """Load a robot definition from a JSON file and refresh the UI.
        Also loads optional 'wheelSizeMM' and 'electrical' sections."""
        path, _ = QFileDialog.getOpenFileName(self, "Import robot", "", "JSON (*.json)")
        if not path: return
        try:
            with open(path, "r", encoding="utf-8") as f:
                obj = json.load(f)
            self.model = RobotModel.from_json(obj)
            if not hasattr(self.model, "wheelWidthMM"):  self.model.wheelWidthMM = 22.0
            if not hasattr(self.model, "wheelHeightMM"): self.model.wheelHeightMM = 15.0
            ws = obj.get("wheelSizeMM") or obj.get("wheel_size_mm")
            if isinstance(ws, dict):
                self.model.wheelWidthMM = float(ws.get("x", self.model.wheelWidthMM))
                self.model.wheelHeightMM = float(ws.get("y", self.model.wheelHeightMM))

            elec = obj.get("electrical", {})
            self.model.batteryVoltageV   = float(elec.get("batteryVoltageV", getattr(self.model, "batteryVoltageV", 7.4)))
            self.model.batteryCapacitymAh= float(elec.get("batteryCapacitymAh", getattr(self.model, "batteryCapacitymAh", 850.0)))
            self.model.motorNoLoadRPM    = float(elec.get("motorNoLoadRPM", getattr(self.model, "motorNoLoadRPM", 10000.0)))
            self.model.motorMaxCurrentA  = float(elec.get("motorMaxCurrentA", getattr(self.model, "motorMaxCurrentA", 1.0)))

            self.scene.model = self.model
            self.refresh_all()
            self.scene.selected_kind, self.scene.selected_id = None, None
            self.scene.rebuild()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to import:\n{e}")

    def on_export(self):
        """Save the current robot definition to a JSON file.
        Exports 'wheelSizeMM' and an 'electrical' block along with the base model."""
        path, _ = QFileDialog.getSaveFileName(self, "Export robot", "robot-spec.json", "JSON (*.json)")
        if not path: return
        try:
            data = self.model.to_json()
            # Inject our additional fields without requiring RobotModel changes.
            data["wheelSizeMM"] = {
                "x": float(getattr(self.model, "wheelWidthMM", 22.0)),
                "y": float(getattr(self.model, "wheelHeightMM", 15.0)),
            }
            data["electrical"] = {
                "batteryVoltageV":   float(getattr(self.model, "batteryVoltageV", 7.4)),
                "batteryCapacitymAh":float(getattr(self.model, "batteryCapacitymAh", 850.0)),
                "motorNoLoadRPM":    float(getattr(self.model, "motorNoLoadRPM", 10000.0)),
                "motorMaxCurrentA":  float(getattr(self.model, "motorMaxCurrentA", 1.0)),
            }
            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to export:\n{e}")

    def on_open_electrical(self):
        """Open the electrical parameters dialog and apply changes if accepted."""
        dlg = ElectricalDialog(self.model, self)
        if dlg.exec() == QDialog.Accepted:
            pass

    def refresh_all(self):
        """Refresh all form sections from the model (envelope, origin, wheels, sensors)."""
        self.spin_env_w.blockSignals(True); self.spin_env_h.blockSignals(True); self.spin_grid.blockSignals(True)
        self.spin_env_w.setValue(self.model.envelope.widthMM)
        self.spin_env_h.setValue(self.model.envelope.heightMM)
        self.spin_grid.setValue(self.model.gridStepMM)
        self.spin_env_w.blockSignals(False); self.spin_env_h.blockSignals(False); self.spin_grid.blockSignals(False)

        self.spin_origin_x.blockSignals(True); self.spin_origin_y.blockSignals(True)
        self.spin_origin_x.setValue(self.model.originXMM)
        self.spin_origin_y.setValue(self.model.originYMM)
        self.spin_origin_x.blockSignals(False); self.spin_origin_y.blockSignals(False)

        self.spin_wheel_w.blockSignals(True); self.spin_wheel_h.blockSignals(True)
        self.spin_wheel_w.setValue(float(getattr(self.model, "wheelWidthMM", 22.0)))
        self.spin_wheel_h.setValue(float(getattr(self.model, "wheelHeightMM", 15.0)))
        self.spin_wheel_w.blockSignals(False); self.spin_wheel_h.blockSignals(False)

        self.refresh_wheels_fields()
        self.refresh_sensors_list()
        self.refresh_sensors_fields()

    def refresh_wheels_fields(self):
        """Show the current wheel coordinates in the spin boxes."""
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
        """Rebuild the sensor list with their IDs and coordinates."""
        cur_sid = self.current_sensor_id()
        self.list_sensors.blockSignals(True)
        self.list_sensors.clear()
        for s in self.model.sensors:
            item = QListWidgetItem(f"{s.id}  (x={s.xMM:.1f} mm, y={s.yMM:.1f} mm)")
            item.setData(Qt.UserRole, s.id)
            self.list_sensors.addItem(item)
        self.list_sensors.blockSignals(False)
        if cur_sid:
            self.select_sensor_id(cur_sid)
        elif self.model.sensors:
            self.list_sensors.setCurrentRow(0)

    def refresh_sensors_fields(self):
        """Show the selected sensor coordinates in the X/Y fields (or zeros if none)."""
        sid = self.current_sensor_id()
        s = self.model.find_sensor(sid) if sid else None
        self.spin_s_x.blockSignals(True); self.spin_s_y.blockSignals(True)
        if s:
            self.spin_s_x.setValue(s.xMM); self.spin_s_y.setValue(s.yMM)
        else:
            self.spin_s_x.setValue(0.0);   self.spin_s_y.setValue(0.0)
        self.spin_s_x.blockSignals(False); self.spin_s_y.blockSignals(False)

    def update_current_sensor_list_item_text(self):
        """Update the text of the currently selected sensor list row."""
        row = self.list_sensors.currentRow()
        if 0 <= row < self.list_sensors.count():
            sid = self.list_sensors.item(row).data(Qt.UserRole)
            s = self.model.find_sensor(sid)
            if s:
                self.list_sensors.item(row).setText(f"{s.id}  (x={s.xMM:.1f} mm, y={s.yMM:.1f} mm)")

    def current_sensor_id(self) -> Optional[str]:
        item = self.list_sensors.currentItem()
        return item.data(Qt.UserRole) if item else None

    def select_sensor_id(self, sid: str):
        """Select a sensor by ID in the list and show its coordinates in the fields."""
        for i in range(self.list_sensors.count()):
            if self.list_sensors.item(i).data(Qt.UserRole) == sid:
                self.list_sensors.setCurrentRow(i)
                s = self.model.find_sensor(sid)
                if s:
                    self.spin_s_x.blockSignals(True); self.spin_s_y.blockSignals(True)
                    self.spin_s_x.setValue(s.xMM); self.spin_s_y.setValue(s.yMM)
                    self.spin_s_x.blockSignals(False); self.spin_s_y.blockSignals(False)
                break

def main():
    """Create the Qt application, show the main window, and start the event loop."""
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
