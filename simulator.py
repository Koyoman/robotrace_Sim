"""Line-Follower Simulator (PySide6 + C backend)

This module provides a minimal GUI to load a track, a robot, and a Python
controller, then simulate the robot following the line. It uses a small C
library (linesim.dll) for the heavy geometry/physics and draws results with Qt.

Comments and docstrings are written in plain English to help beginners.
"""
from __future__ import annotations

import importlib
import math
import os
import sys
from typing import Any, Dict, List, Optional

from PySide6.QtCore import Qt, QPointF, QThread, Signal, QTimer, QElapsedTimer
from PySide6.QtGui import QPen, QColor, QPainterPath, QPainter
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QFormLayout, QPushButton,
    QFileDialog, QLabel, QSplitter, QGraphicsView,
    QGraphicsScene, QMessageBox, QComboBox, QCheckBox, QVBoxLayout, QHBoxLayout, QSizePolicy,
    QDialog, QDialogButtonBox, QDoubleSpinBox, QGroupBox, QSpinBox, QScrollArea
)

from Utils.robot_spec import RobotSpec
from Utils.simulation_config import SimulationConfig, derive_runtime_params
from Utils.track_geometry import (
    MARKER_LENGTH_MM, MARKER_OFFSET_MM, MARKER_THICKNESS_MM,
    Pt, Pose, SegStraight, advance_straight, rad,
)
from Utils.track_spec import TrackSpec
from Utils.validation import ValidationError
from Utils.robot_runtime import robot_local_to_world
from sim.controller_loader import ControllerLoadError, load_controller
from sim.track_runtime import (
    curvature_change_markers, ensure_track_raster, oriented_rect,
    rot, segments_from_track as segments_from_json, segments_polyline, start_finish_lines,
)
from sim.worker import SimWorker


WHEEL_W_MM = 22.0
WHEEL_H_MM = 15.0
WHEEL_PEN = QPen(QColor("#000000"), 1)
WHEEL_BRUSH = QColor("#dddddd")


def _friendly_error(exc: Exception) -> str:
    if isinstance(exc, ValidationError):
        return str(exc)
    if isinstance(exc, ControllerLoadError):
        return str(exc)
    return str(exc)


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


class CustomPhysicsDialog(QDialog):
    """Single dialog for all Phase 4 custom physics options."""

    def __init__(self, parent=None, *, settings: dict[str, Any] | None = None):
        super().__init__(parent)
        self.setWindowTitle("Custom physics settings")
        self.setModal(True)
        self.setMinimumWidth(620)
        self.resize(720, 760)
        self._settings = dict(settings or {})

        root = QVBoxLayout(self)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        body = QWidget()
        layout = QVBoxLayout(body)
        scroll.setWidget(body)
        root.addWidget(scroll)

        self._checks: dict[str, QCheckBox] = {}
        self._double: dict[str, QDoubleSpinBox] = {}
        self._int: dict[str, QSpinBox] = {}

        def add_check(key: str, text: str, parent_layout: QFormLayout | QVBoxLayout) -> QCheckBox:
            chk = QCheckBox(text)
            self._checks[key] = chk
            if isinstance(parent_layout, QFormLayout):
                parent_layout.addRow(chk)
            else:
                parent_layout.addWidget(chk)
            return chk

        def add_double(key: str, label: str, parent_layout: QFormLayout, *, lo=-1e9, hi=1e9, step=1.0, decimals=3, suffix="") -> QDoubleSpinBox:
            spin = QDoubleSpinBox()
            spin.setRange(float(lo), float(hi))
            spin.setDecimals(decimals)
            spin.setSingleStep(float(step))
            if suffix:
                spin.setSuffix(suffix)
            self._double[key] = spin
            parent_layout.addRow(label, spin)
            return spin

        def add_int(key: str, label: str, parent_layout: QFormLayout, *, lo=0, hi=10_000_000, step=1) -> QSpinBox:
            spin = QSpinBox()
            spin.setRange(int(lo), int(hi))
            spin.setSingleStep(int(step))
            self._int[key] = spin
            parent_layout.addRow(label, spin)
            return spin

        model_group = QGroupBox("1. Modelo base")
        model_layout = QFormLayout(model_group)
        self.chk_use_dc = add_check("custom_use_dc_motor_model", "Usar modelo DC", model_layout)
        self.chk_use_c = add_check("custom_use_c_backend", "Exigir backend C modular ABI compatível", model_layout)
        self.chk_allow_py_fallback = add_check("custom_allow_python_fallback", "Permitir fallback Python explícito", model_layout)
        self.chk_use_kin = add_check("custom_use_kinematic_model", "Permitir fallback cinemático Python", model_layout)
        add_double("basic_max_wheel_speed_mm_s", "Velocidade máxima manual", model_layout, lo=1, hi=100000, step=50, suffix=" mm/s")
        self.chk_auto_speed = QCheckBox("Velocidade automática a partir do robô")
        model_layout.addRow(self.chk_auto_speed)
        layout.addWidget(model_group)

        accel_group = QGroupBox("2. Limite de aceleração")
        accel_layout = QFormLayout(accel_group)
        self.chk_use_accel = add_check("custom_use_acceleration_limit", "Habilitar limite de aceleração", accel_layout)
        self.chk_auto_accel = add_check("custom_use_auto_acceleration_limit", "Automático a partir do robô", accel_layout)
        add_double("custom_max_wheel_accel_mm_s2", "Aceleração máxima", accel_layout, lo=1, hi=1_000_000, step=500, suffix=" mm/s²")
        layout.addWidget(accel_group)

        batt_group = QGroupBox("3. Bateria")
        batt_layout = QFormLayout(batt_group)
        add_check("custom_use_battery_model", "Habilitar descarga de bateria", batt_layout)
        add_double("custom_battery_initial_voltage_v", "Tensão inicial", batt_layout, lo=0.1, hi=100, step=0.1, suffix=" V")
        add_double("custom_battery_nominal_voltage_v", "Tensão nominal", batt_layout, lo=0.1, hi=100, step=0.1, suffix=" V")
        add_double("custom_battery_min_voltage_v", "Tensão mínima", batt_layout, lo=0.0, hi=100, step=0.1, suffix=" V")
        add_double("custom_battery_capacity_mah", "Capacidade", batt_layout, lo=1, hi=100000, step=100, suffix=" mAh")
        add_double("custom_battery_internal_resistance_ohm", "Resistência interna", batt_layout, lo=0, hi=100, step=0.01, decimals=4, suffix=" Ω")
        add_double("custom_battery_soc_initial", "SOC inicial", batt_layout, lo=0, hi=1, step=0.05, decimals=3)
        layout.addWidget(batt_group)

        sensor_group = QGroupBox("4. Sensores com ruído")
        sensor_layout = QFormLayout(sensor_group)
        add_check("custom_use_sensor_noise", "Habilitar ruído nos sensores de linha", sensor_layout)
        add_double("custom_sensor_noise_std", "Desvio padrão legado", sensor_layout, lo=0, hi=65535, step=0.005, decimals=5)
        add_double("sensor_common_noise_std", "Ruído comum", sensor_layout, lo=0, hi=65535, step=0.005, decimals=5)
        add_double("sensor_individual_noise_std", "Ruído individual", sensor_layout, lo=0, hi=65535, step=0.005, decimals=5)
        add_double("sensor_filter_tau_ms", "Filtro temporal tau", sensor_layout, lo=0, hi=10000, step=1, suffix=" ms")
        add_double("sensor_latency_ms", "Latência", sensor_layout, lo=0, hi=10000, step=1, suffix=" ms")
        add_double("sensor_update_rate_Hz", "Update rate", sensor_layout, lo=0, hi=10000, step=1, suffix=" Hz")
        add_double("sensor_gain_default", "Ganho default", sensor_layout, lo=0.001, hi=1000, step=0.01, decimals=4)
        add_double("sensor_offset_default", "Offset default", sensor_layout, lo=-65535, hi=65535, step=1)
        add_check("verbose_sensor_log", "Log detalhado raw/calibrated/filtered", sensor_layout)
        add_int("custom_sensor_noise_seed", "Seed", sensor_layout, lo=0, hi=2_147_483_647)
        layout.addWidget(sensor_group)

        enc_group = QGroupBox("5. Encoder")
        enc_layout = QFormLayout(enc_group)
        add_check("custom_use_encoder_model", "Habilitar encoder", enc_layout)
        add_int("custom_encoder_ticks_per_rev", "Ticks por volta", enc_layout, lo=1, hi=1_000_000)
        add_double("custom_encoder_noise_std_ticks", "Ruído", enc_layout, lo=0, hi=1_000_000, step=0.1, suffix=" ticks")
        add_check("custom_encoder_quantization", "Quantizar ticks", enc_layout)
        layout.addWidget(enc_group)

        imu_group = QGroupBox("6. IMU")
        imu_layout = QFormLayout(imu_group)
        add_check("custom_use_imu_model", "Habilitar IMU", imu_layout)
        add_double("custom_imu_gyro_noise_std_rad_s", "Ruído gyro", imu_layout, lo=0, hi=1000, step=0.001, decimals=6, suffix=" rad/s")
        add_double("custom_imu_accel_noise_std_mm_s2", "Ruído acelerômetro", imu_layout, lo=0, hi=1_000_000, step=1, suffix=" mm/s²")
        layout.addWidget(imu_group)

        slip_group = QGroupBox("7. Slip de roda")
        slip_layout = QFormLayout(slip_group)
        add_check("custom_use_wheel_slip", "Habilitar slip", slip_layout)
        add_double("custom_slip_ratio_left", "Slip manual esquerdo", slip_layout, lo=0, hi=0.95, step=0.01, decimals=4)
        add_double("custom_slip_ratio_right", "Slip manual direito", slip_layout, lo=0, hi=0.95, step=0.01, decimals=4)
        add_double("custom_slip_noise_std", "Ruído de slip", slip_layout, lo=0, hi=0.95, step=0.001, decimals=5)
        add_double("custom_slip_stiffness_factor", "Rigidez slip contínuo", slip_layout, lo=0, hi=10, step=0.01, decimals=4)
        add_double("custom_slip_at_limit", "Slip no limite", slip_layout, lo=0, hi=0.99, step=0.01, decimals=4)
        add_double("custom_slip_max_ratio", "Slip máximo", slip_layout, lo=0, hi=0.99, step=0.01, decimals=4)
        add_check("custom_use_combined_slip_limit", "Limite combinado longitudinal + lateral", slip_layout)
        add_check("custom_use_lateral_slip", "Slip lateral em curva", slip_layout)
        add_double("custom_mu_static_left", "µ estático esquerdo override", slip_layout, lo=0, hi=10, step=0.01, decimals=4)
        add_double("custom_mu_static_right", "µ estático direito override", slip_layout, lo=0, hi=10, step=0.01, decimals=4)
        add_double("custom_mu_kinetic_left", "µ cinético esquerdo override", slip_layout, lo=0, hi=10, step=0.01, decimals=4)
        add_double("custom_mu_kinetic_right", "µ cinético direito override", slip_layout, lo=0, hi=10, step=0.01, decimals=4)
        layout.addWidget(slip_group)

        track_group = QGroupBox("8. Imperfeições da pista")
        track_layout = QFormLayout(track_group)
        add_check("custom_use_track_imperfections", "Habilitar imperfeições procedurais", track_layout)
        add_double("custom_track_imperfection_amplitude_mm", "Amplitude", track_layout, lo=0, hi=1000, step=0.1, suffix=" mm")
        add_double("custom_track_imperfection_wavelength_mm", "Comprimento de onda", track_layout, lo=1, hi=100000, step=10, suffix=" mm")
        add_double("custom_track_imperfection_noise_std", "Ruído", track_layout, lo=0, hi=1000, step=0.1, suffix=" mm")
        layout.addWidget(track_group)

        adv_group = QGroupBox("9. Parâmetros avançados do motor/C")
        adv_layout = QFormLayout(adv_group)
        add_double("custom_motor_deadzone_pwm", "Deadzone PWM", adv_layout, lo=0, hi=100000, step=1)
        add_double("custom_current_limit_a", "Limite de corrente", adv_layout, lo=0, hi=1000, step=0.1, suffix=" A")
        add_double("custom_drivetrain_efficiency", "Eficiência drivetrain", adv_layout, lo=0.001, hi=1.0, step=0.01, decimals=4)
        add_double("custom_viscous_friction", "Atrito viscoso", adv_layout, lo=0, hi=1000, step=0.000001, decimals=9)
        add_double("custom_coulomb_friction", "Atrito Coulomb", adv_layout, lo=0, hi=1000, step=0.000001, decimals=9)
        add_check("custom_pwm_saturation_enabled", "Saturar PWM", adv_layout)
        add_double("custom_max_pwm", "PWM máximo manual", adv_layout, lo=1, hi=1_000_000, step=1)
        self.chk_auto_pwm = QCheckBox("PWM máximo automático do robô")
        adv_layout.addRow(self.chk_auto_pwm)
        layout.addWidget(adv_group)

        note = QLabel(
            "As mudanças são salvas ao clicar OK e passam a valer apenas na próxima simulação. "
            "Cada opção exibida é repassada para SimulationConfig e consumida pela engine, pelo modelo físico ou pelo backend C."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color: #888888;")
        layout.addWidget(note)

        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        root.addWidget(buttons)

        self.chk_use_dc.toggled.connect(self._update_enabled)
        self.chk_use_accel.toggled.connect(self._update_enabled)
        self.chk_auto_speed.toggled.connect(self._update_enabled)
        self.chk_auto_accel.toggled.connect(self._update_enabled)
        self.chk_auto_pwm.toggled.connect(self._update_enabled)
        self._load_settings(self._settings)
        self._update_enabled()

    @staticmethod
    def default_settings() -> dict[str, Any]:
        return {
            "custom_use_dc_motor_model": True,
            "custom_use_kinematic_model": True,
            "custom_use_acceleration_limit": True,
            "custom_use_battery_model": False,
            "custom_use_sensor_noise": False,
            "custom_use_encoder_model": False,
            "custom_use_imu_model": False,
            "custom_use_wheel_slip": False,
            "custom_use_track_imperfections": False,
            "custom_use_c_backend": True,
            "custom_allow_python_fallback": False,
            "custom_use_auto_acceleration_limit": True,
            "basic_max_wheel_speed_mm_s": None,
            "basic_max_wheel_accel_mm_s2": None,
            "custom_max_wheel_accel_mm_s2": 9810.0,
            "custom_battery_initial_voltage_v": 7.4,
            "custom_battery_nominal_voltage_v": 7.4,
            "custom_battery_min_voltage_v": 6.0,
            "custom_battery_capacity_mah": 1000.0,
            "custom_battery_internal_resistance_ohm": 0.15,
            "custom_battery_soc_initial": 1.0,
            "custom_sensor_noise_std": 0.02,
            "sensor_common_noise_std": 0.0,
            "sensor_individual_noise_std": 0.0,
            "sensor_filter_tau_ms": 0.0,
            "sensor_latency_ms": 0.0,
            "sensor_update_rate_Hz": 0.0,
            "sensor_gain_default": 1.0,
            "sensor_offset_default": 0.0,
            "verbose_sensor_log": False,
            "custom_sensor_noise_seed": 12345,
            "custom_encoder_ticks_per_rev": 1024,
            "custom_encoder_noise_std_ticks": 0.0,
            "custom_encoder_quantization": True,
            "custom_imu_gyro_noise_std_rad_s": 0.0,
            "custom_imu_accel_noise_std_mm_s2": 0.0,
            "custom_slip_ratio_left": 0.0,
            "custom_slip_ratio_right": 0.0,
            "custom_slip_noise_std": 0.0,
            "custom_slip_stiffness_factor": 0.03,
            "custom_slip_at_limit": 0.05,
            "custom_slip_max_ratio": 0.95,
            "custom_use_combined_slip_limit": True,
            "custom_use_lateral_slip": True,
            "custom_mu_static_left": 0.0,
            "custom_mu_static_right": 0.0,
            "custom_mu_kinetic_left": 0.0,
            "custom_mu_kinetic_right": 0.0,
            "custom_track_imperfection_amplitude_mm": 0.0,
            "custom_track_imperfection_wavelength_mm": 500.0,
            "custom_track_imperfection_noise_std": 0.0,
            "custom_motor_deadzone_pwm": 0.0,
            "custom_current_limit_a": 0.0,
            "custom_drivetrain_efficiency": 0.9,
            "custom_viscous_friction": 0.0,
            "custom_coulomb_friction": 0.0,
            "custom_pwm_saturation_enabled": True,
            "custom_max_pwm": None,
        }

    def _load_settings(self, settings: dict[str, Any]) -> None:
        merged = self.default_settings()
        merged.update(settings or {})
        for key, chk in self._checks.items():
            chk.setChecked(bool(merged.get(key, False)))
        for key, spin in self._double.items():
            value = merged.get(key, None)
            if value is None:
                value = 500.0 if key == "basic_max_wheel_speed_mm_s" else 4095.0 if key == "custom_max_pwm" else self.default_settings().get(key, 0.0)
            spin.setValue(float(value))
        for key, spin in self._int.items():
            spin.setValue(int(merged.get(key, self.default_settings().get(key, 0))))
        self.chk_auto_speed.setChecked(merged.get("basic_max_wheel_speed_mm_s", None) is None)
        self.chk_auto_pwm.setChecked(merged.get("custom_max_pwm", None) is None)

    def _update_enabled(self) -> None:
        use_kinematic = not self.chk_use_dc.isChecked()
        self.chk_use_accel.setEnabled(True)
        self.chk_use_kin.setEnabled(True)
        self.chk_use_c.setEnabled(self.chk_use_dc.isChecked())
        self.chk_allow_py_fallback.setEnabled(self.chk_use_dc.isChecked() and self.chk_use_c.isChecked())
        self.chk_auto_speed.setEnabled(use_kinematic)
        self._double["basic_max_wheel_speed_mm_s"].setEnabled(use_kinematic and not self.chk_auto_speed.isChecked())
        self.chk_auto_accel.setEnabled(self.chk_use_accel.isChecked())
        self._double["custom_max_wheel_accel_mm_s2"].setEnabled(self.chk_use_accel.isChecked() and not self.chk_auto_accel.isChecked())
        self._double["custom_max_pwm"].setEnabled(not self.chk_auto_pwm.isChecked())

    def settings(self) -> dict[str, Any]:
        data = self.default_settings()
        for key, chk in self._checks.items():
            data[key] = chk.isChecked()
        for key, spin in self._double.items():
            data[key] = float(spin.value())
        for key, spin in self._int.items():
            data[key] = int(spin.value())
        if self.chk_auto_speed.isChecked():
            data["basic_max_wheel_speed_mm_s"] = None
        if self.chk_auto_accel.isChecked():
            data["basic_max_wheel_accel_mm_s2"] = None
        else:
            data["basic_max_wheel_accel_mm_s2"] = data["custom_max_wheel_accel_mm_s2"]
        if self.chk_auto_pwm.isChecked():
            data["custom_max_pwm"] = None
        return data

class MainWindow(QMainWindow):
    """Main GUI: loads files, starts simulation, and replays results."""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Line-Follower Simulator")

        self.track: Optional[TrackSpec] = None
        self.robot: Optional[RobotSpec] = None
        self.controller_fn = lambda state: {"pwm_left": 2000, "pwm_right": 2000}

        self.scene = SimScene()
        self.view = SimView(self.scene)

        controls = QWidget(); form = QFormLayout(controls)

        self.anim_interval_ms = 42
        self.sim_dt_s = 0.001
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
        self.robot_path = None
        self.robot_data_raw = {}
        self.track_path = None
        self.lbl_ctrl_status = QLabel("—")
        self.lbl_ctrl_status.setStyleSheet("color: #aaaaaa; font-weight: 500;")

        self.btn_track = QPushButton("Load track (.json)")
        self.btn_track.setToolTip("Select a track file (.json).")
        self.btn_robot = QPushButton("Load robot (.json)")
        self.btn_robot.setToolTip("Select a robot description (.json).")
        self.btn_ctrl  = QPushButton("Load controller (.py)")
        self.btn_ctrl.setToolTip("Select a Python file implementing control_step(state).")

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

        self.combo_physics = QComboBox()
        self.combo_physics.addItem("Realistic", "realistic")
        self.combo_physics.addItem("Basic", "basic")
        self.combo_physics.addItem("Ideal", "ideal")
        self.combo_physics.addItem("Custom", "custom")
        self.combo_physics.setCurrentIndex(0)
        self.combo_physics.setToolTip("Physics profile used by SimulationEngine. Realistic now enables the complete Phase 4 physics preset.")

        self.custom_physics_settings = CustomPhysicsDialog.default_settings()
        self.btn_custom_physics = QPushButton("Custom settings…")
        self.btn_custom_physics.setToolTip("Configure all custom physics options used on the next simulation run.")
        self.lbl_custom_physics = QLabel("")
        self.lbl_custom_physics.setWordWrap(True)
        self.lbl_custom_physics.setStyleSheet("color: #888888;")

        self.combo_speed = QComboBox()
        self.combo_speed.addItems(["0.1×", "0.5×", "1×", "2×", "4×"])
        self.combo_speed.setToolTip("Playback speed for visualization only. It does not affect the physics or logged data.")
        self.combo_speed.setCurrentIndex(2)
        self.combo_speed.currentIndexChanged.connect(self.on_speed_change)
        self.on_speed_change(self.combo_speed.currentIndex())

        form.addRow(QLabel("<b>Simulation Files</b>"))

        self.lbl_ctrl_status = QLabel("—")
        self.lbl_ctrl_status.setStyleSheet("color: #aaaaaa; font-weight: 500;")
        self.lbl_ctrl_status.setToolTip("Loaded controller file.")
        row_ctrl = QWidget()
        row_ctrl_layout = QHBoxLayout(row_ctrl); row_ctrl_layout.setContentsMargins(0,0,0,0)
        row_ctrl_layout.addWidget(self.btn_ctrl)
        row_ctrl_layout.addWidget(self.lbl_ctrl_status)
        form.addRow(row_ctrl)

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
        physics_row = QWidget()
        physics_layout = QHBoxLayout(physics_row); physics_layout.setContentsMargins(0, 0, 0, 0)
        physics_layout.addWidget(self.combo_physics)
        physics_layout.addWidget(self.btn_custom_physics)
        form.addRow("Physics profile", physics_row)
        form.addRow("Custom physics", self.lbl_custom_physics)
        self.btn_reload = QPushButton("Reload files")
        self.btn_reload.setToolTip("Reload the last loaded track, robot, and controller from disk (no dialogs). Useful when tuning the PID/controller to grab the latest code and JSONs.")
        form.addRow(self.btn_reload)
        self.btn_reload.clicked.connect(self.on_reload_files)
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
        self.btn_track_robot = QPushButton("Track robot")
        self.btn_track_robot.setToolTip("Keep the view centered on the robot during replay.")
        self.btn_track_robot.setCheckable(True)
        self.btn_track_robot.setChecked(False)
        replay_ctrl_row = QWidget()
        replay_ctrl_layout = QHBoxLayout(replay_ctrl_row); replay_ctrl_layout.setContentsMargins(0,0,0,0)
        replay_ctrl_layout.addWidget(self.btn_replay)
        replay_ctrl_layout.addWidget(self.btn_replay_stop)
        replay_ctrl_layout.addWidget(self.btn_track_robot)
        form.addRow(replay_ctrl_row)

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
        self.combo_physics.currentIndexChanged.connect(self.on_physics_profile_changed)
        self.btn_custom_physics.clicked.connect(self.on_configure_custom_physics)
        self.on_physics_profile_changed()

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

        self.setWindowTitle(f"Line-Follower Simulator")

        self.is_replaying = False

    def update_replay_buttons(self):
        """Enable/disable replay buttons depending on current state."""
        has_steps = bool(self.anim_steps)
        self.btn_replay.setEnabled(has_steps and not self.is_replaying and not self.streaming)
        self.btn_replay_stop.setEnabled(self.is_replaying)

    def _draw_robot_at_initial_pose(self):
        """Draw the robot body/wheels/sensors at the initial pose onto the scene."""
        if not (self.track and self.robot):
            return
        segs, origin, tapeW = segments_from_json(self.track)
        gates = start_finish_lines(self.track)

        if gates:
            (sa, sb, shdg_run, shdg_base), _ = gates
            back = (self.robot.envelope.height_mm/2.0) + 250.0
            pose_gate = Pose(Pt((sa.x + sb.x)*0.5, (sa.y + sb.y)*0.5), shdg_run)
            pos = advance_straight(pose_gate, -back)
        else:
            pos = origin

        hw = self.robot.envelope.width_mm * 0.5
        hh = self.robot.envelope.height_mm * 0.5
        ang = rad(pos.headingDeg)
        ox, oy = self.robot.origin_x_mm, self.robot.origin_y_mm

        corners = [(-hw,-hh),(hw,-hh),(hw,hh),(-hw,hh)]
        poly = QPainterPath()
        for k,(cx,cy) in enumerate(corners + [corners[0]]):
            lx, ly = cx - ox, cy - oy
            rx, ry = rot(lx, ly, ang)
            px, py = pos.p.x + rx, pos.p.y + ry
            poly.moveTo(px, py) if k == 0 else poly.lineTo(px, py)
        self.anim_items["robot"].setPath(poly)

        for k, s in enumerate(self.robot.sensors):
            px, py = s.x_mm - ox, s.y_mm - oy
            rx, ry = rot(px, py, ang)
            cx, cy = pos.p.x + rx, pos.p.y + ry
            sp = QPainterPath()
            sp.addRect(cx - 2, cy - 2, 4, 4)
            if k < len(self.anim_items["sensors"]):
                self.anim_items["sensors"][k].setPath(sp)

        for k, wdef in enumerate(self.robot.wheels):
            half_w = float(getattr(wdef, 'width_mm', WHEEL_W_MM)) * 0.5
            half_h = float(getattr(wdef, 'height_mm', WHEEL_H_MM)) * 0.5

            px, py = wdef.x_mm - ox, wdef.y_mm - oy
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
        """Read and validate a TrackSpec, build raster cache, and draw the static track."""
        path, _ = QFileDialog.getOpenFileName(self, "Track file", "", "JSON (*.json)")
        if not path:
            return
        try:
            self.track = TrackSpec.from_json_file(path)
            self.track_path = path
            segs, origin, tapeW = segments_from_json(self.track)
            gates = start_finish_lines(self.track)
            _ = ensure_track_raster(self.track_path, self.track, segs, tapeW, gates)
            self.draw_static_track()
            if self.robot:
                self.draw_robot_outline_preview()
            base = os.path.basename(path)
            if hasattr(self, "lbl_track_status"):
                self.lbl_track_status.setText(f"{base}   ✓")
                self.lbl_track_status.setStyleSheet("color: #2e7d32; font-weight: 600;")
                self.lbl_track_status.setToolTip(path)
            self.statusBar().showMessage(f"Track: {base}")
        except Exception as e:
            self.track = None
            QMessageBox.critical(self, "Track load error", f"Failed to read '{os.path.basename(path)}':\n{_friendly_error(e)}")
            if hasattr(self, "lbl_track_status"):
                self.lbl_track_status.setText("failed")
                self.lbl_track_status.setStyleSheet("color: #c62828; font-weight: 600;")

    def on_load_robot(self):
        """Read and validate a RobotSpec and update preview widgets."""
        path, _ = QFileDialog.getOpenFileName(self, "Robot file", "", "JSON (*.json)")
        if not path:
            return
        try:
            self.robot = RobotSpec.from_json_file(path)
            self.robot_data_raw = self.robot.to_dict()
            self.robot_path = path
            base = os.path.basename(path)
            self.statusBar().showMessage(f"Robot: {base}", 5000)
            if hasattr(self, "lbl_robot_status"):
                self.lbl_robot_status.setText(f"{base}   ✓")
                self.lbl_robot_status.setStyleSheet("color: #2e7d32; font-weight: 600;")
                self.lbl_robot_status.setToolTip(path)
            if self.track:
                self.draw_robot_outline_preview()
        except Exception as e:
            self.robot = None
            QMessageBox.critical(self, "Robot load error", f"Failed to read '{os.path.basename(path)}':\n{_friendly_error(e)}")
            if hasattr(self, "lbl_robot_status"):
                self.lbl_robot_status.setText("failed")
                self.lbl_robot_status.setStyleSheet("color: #c62828; font-weight: 600;")
            if self.worker and self.worker.isRunning():
                try:
                    self.worker.cancel()
                    self.worker.wait(500)
                except Exception:
                    pass

    def on_load_controller(self):
        """Load a Python controller and update the status label with success/failure."""
        path, _ = QFileDialog.getOpenFileName(self, "Controller", "", "Python (*.py)")
        if not path:
            return
        try:
            self.controller_fn = load_controller(path)
            self.controller_path = path
            base = os.path.basename(path)
            if hasattr(self, "lbl_ctrl_status"):
                self.lbl_ctrl_status.setText(f"{base}   ✓")
                self.lbl_ctrl_status.setStyleSheet("color: #2e7d32; font-weight: 600;")
                self.lbl_ctrl_status.setToolTip(path)
            self.statusBar().showMessage(f"Controller loaded: {base}", 5000)
        except Exception as e:
            QMessageBox.critical(self, "Controller error", _friendly_error(e))
            if hasattr(self, "lbl_ctrl_status"):
                self.lbl_ctrl_status.setText("failed")
                self.lbl_ctrl_status.setStyleSheet("color: #c62828; font-weight: 600;")

    def on_reload_files(self):
        """Reload the last loaded files from disk without opening dialogs."""
        reloaded = []
        if getattr(self, "track_path", None):
            try:
                self.track = TrackSpec.from_json_file(self.track_path)
                segs, origin, tapeW = segments_from_json(self.track)
                gates = start_finish_lines(self.track)
                try:
                    _ = ensure_track_raster(self.track_path, self.track, segs, tapeW, gates)
                except Exception:
                    pass
                self.draw_static_track()
                if self.robot:
                    self.draw_robot_outline_preview()
                base = os.path.basename(self.track_path)
                if hasattr(self, "lbl_track_status"):
                    self.lbl_track_status.setText(f"{base}   ✓")
                    self.lbl_track_status.setStyleSheet("color: #2e7d32; font-weight: 600;")
                    self.lbl_track_status.setToolTip(self.track_path)
                reloaded.append("track")
            except Exception as e:
                QMessageBox.critical(self, "Track reload error", f"{self.track_path}\n{_friendly_error(e)}")

        if getattr(self, "robot_path", None):
            try:
                self.robot = RobotSpec.from_json_file(self.robot_path)
                self.robot_data_raw = self.robot.to_dict()
                if self.track:
                    self.draw_robot_outline_preview()
                base = os.path.basename(self.robot_path)
                if hasattr(self, "lbl_robot_status"):
                    self.lbl_robot_status.setText(f"{base}   ✓")
                    self.lbl_robot_status.setStyleSheet("color: #2e7d32; font-weight: 600;")
                    self.lbl_robot_status.setToolTip(self.robot_path)
                reloaded.append("robot")
            except Exception as e:
                QMessageBox.critical(self, "Robot reload error", f"{self.robot_path}\n{_friendly_error(e)}")

        if getattr(self, "controller_path", None):
            try:
                importlib.invalidate_caches()
                self.controller_fn = load_controller(self.controller_path)
                base = os.path.basename(self.controller_path)
                if hasattr(self, "lbl_ctrl_status"):
                    self.lbl_ctrl_status.setText(f"{base}   ✓")
                    self.lbl_ctrl_status.setStyleSheet("color: #2e7d32; font-weight: 600;")
                    self.lbl_ctrl_status.setToolTip(self.controller_path)
                try:
                    self.btn_ctrl.setStyleSheet("background: #e8f5e9;")
                    QTimer.singleShot(600, lambda: self.btn_ctrl.setStyleSheet(""))
                except Exception:
                    pass
                reloaded.append("controller")
            except Exception as e:
                QMessageBox.critical(self, "Controller reload error", f"{self.controller_path}\n{_friendly_error(e)}")

        if reloaded:
            self.statusBar().showMessage("Reloaded: " + ", ".join(reloaded), 4000)
        else:
            self.statusBar().showMessage("Nothing to reload (no paths set).", 4000)

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

        gates = start_finish_lines(self.track)
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
        gates = start_finish_lines(self.track)
        if gates:
            (a, b, hdg_run, hdg_base), _ = gates
            back = (self.robot.envelope.height_mm / 2.0) + 250.0
            pose_gate = Pose(Pt((a.x + b.x) / 2.0, (a.y + b.y) / 2.0), hdg_run)
            pos = advance_straight(pose_gate, -back)
        else:
            pos = origin

        hw = self.robot.envelope.width_mm / 2.0
        hh = self.robot.envelope.height_mm / 2.0
        ang = rad(pos.headingDeg)

        ox, oy = self.robot.origin_x_mm, self.robot.origin_y_mm
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
            rx, ry = rot(s.x_mm - self.robot.origin_x_mm, s.y_mm - self.robot.origin_y_mm, ang)
            px, py = pos.p.x + rx, pos.p.y + ry
            sz = s.size_mm
            sp = QPainterPath()
            sp.addRect(px - sz/2.0, py - sz/2.0, sz, sz)
            self.anim_items["sensors"].append(self.scene.addPath(sp, QPen(QColor("#FFFFFF"), 1)))

        self.anim_items["wheels"] = []
        for wdef in self.robot.wheels:
            px, py = robot_local_to_world(self.robot, pos.p.x, pos.p.y, pos.headingDeg, wdef.x_mm, wdef.y_mm)
            half_w = float(getattr(wdef, 'width_mm', WHEEL_W_MM)) * 0.5
            half_h = float(getattr(wdef, 'height_mm', WHEEL_H_MM)) * 0.5
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

    def _current_physics_profile(self) -> str:
        return str(self.combo_physics.currentData() or "realistic").strip().lower()

    def _custom_physics_summary(self) -> str:
        st = self.custom_physics_settings
        enabled = []
        if bool(st.get("custom_use_dc_motor_model", True)):
            enabled.append("DC/C")
        else:
            enabled.append("cinemático")
        for key, label in [
            ("custom_use_acceleration_limit", "aceleração"),
            ("custom_use_battery_model", "bateria"),
            ("custom_use_sensor_noise", "ruído sensores"),
            ("custom_use_encoder_model", "encoder"),
            ("custom_use_imu_model", "IMU"),
            ("custom_use_wheel_slip", "slip"),
            ("custom_use_track_imperfections", "pista imperfeita"),
        ]:
            if bool(st.get(key, False)):
                enabled.append(label)
        return "Custom ativo: " + ", ".join(enabled) + "."

    def _update_custom_physics_summary(self) -> None:
        is_custom = self._current_physics_profile() == "custom"
        self.btn_custom_physics.setEnabled(is_custom)
        self.lbl_custom_physics.setVisible(is_custom)
        if is_custom:
            self.lbl_custom_physics.setText(self._custom_physics_summary())
        else:
            self.lbl_custom_physics.setText("Only used when Physics profile is Custom.")

    def on_physics_profile_changed(self, *_args) -> None:
        self._update_custom_physics_summary()

    def on_configure_custom_physics(self) -> None:
        dialog = CustomPhysicsDialog(self, settings=self.custom_physics_settings)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            self.custom_physics_settings = dialog.settings()
            self._update_custom_physics_summary()

    def _apply_custom_physics_settings(self, cfg: SimulationConfig) -> None:
        """Copy UI-selected custom settings into the config used by the next worker."""
        if str(cfg.physics_profile).strip().lower() != "custom":
            return
        for key, value in self.custom_physics_settings.items():
            if hasattr(cfg, key):
                setattr(cfg, key, value)
        # The Phase 3 kinematic fields are still accepted and now mirror the
        # Phase 4 manual speed/acceleration settings.
        cfg.basic_max_wheel_speed_mm_s = self.custom_physics_settings.get("basic_max_wheel_speed_mm_s", None)
        cfg.basic_max_wheel_accel_mm_s2 = self.custom_physics_settings.get("basic_max_wheel_accel_mm_s2", None)

    def _build_simulation_config(self) -> SimulationConfig:
        if self.robot is None:
            return SimulationConfig()
        cfg = SimulationConfig.from_robot_spec(self.robot)
        cfg.save_logs = self.chk_log.isChecked()
        cfg.physics_profile = self._current_physics_profile()
        self._apply_custom_physics_settings(cfg)
        return cfg

    def on_simulate(self):
        self.streaming = True
        if not (self.track and self.robot):
            self.streaming = False
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

        cfg = self._build_simulation_config()
        p = derive_runtime_params(self.robot, cfg)
        self.v_max_mm_s = float(p.get("final_linear_speed_mps", 2.0)) * 1000.0
        self.sim_dt_s = max(0.0005, min(0.1, float(p.get("simulation_step_dt_ms", 1.0)) / 1000.0))

        try:
            self.on_speed_change(self.combo_speed.currentIndex())
        except Exception:
            pass

        self.worker = SimWorker(
            self.track,
            self.robot,
            self.controller_fn,
            config=cfg,
            save_logs=cfg.save_logs,
            track_path=getattr(self, 'track_path', None),
        )
        self.worker.sig_chunk.connect(self.on_stream_chunk)
        self.worker.sig_done.connect(self.on_stream_done)
        self.worker.sig_fail.connect(self.on_stream_fail)
        self.worker.start(QThread.TimeCriticalPriority)

    def _speed_color(self, v_mm_s: float) -> QColor:
        vmax = max(1e-6, float(self.v_max_mm_s or 1000.0))
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
                self.worker.cancel()
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
                self.worker.cancel()
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
        if hasattr(self, "btn_track_robot") and self.btn_track_robot.isChecked():
            try:
                self.view.centerOn(x, y)
            except Exception:
                pass
        if hasattr(self, 'btn_track_robot'):
            self.btn_track_robot.setEnabled(bool(self.anim_steps) and not self.streaming)

        if self._trail_last_pt is None:
            self._trail_last_pt = (x, y)
        else:
            (px, py) = self._trail_last_pt
            pen = QPen(self._speed_color(v_now), 8, Qt.SolidLine, Qt.RoundCap)
            seg = self.scene.addLine(px, py, x, y, pen)
            self.anim_items.setdefault("trail_items", []).append(seg)
            self._trail_last_pt = (x, y)

        if self.robot:
            hw = self.robot.envelope.width_mm / 2.0
            hh = self.robot.envelope.height_mm / 2.0
            ox, oy = self.robot.origin_x_mm, self.robot.origin_y_mm
            corners = [(-hw,-hh),(hw,-hh),(hw,hh),(-hw,hh)]
            poly = QPainterPath()
            for k,(cx,cy) in enumerate(corners + [corners[0]]):
                lx, ly = cx - ox, cy - oy
                rx, ry = rot(lx, ly, ang)
                px, py = x + rx, y + ry
                poly.moveTo(px, py) if k == 0 else poly.lineTo(px, py)
            self.anim_items["robot"].setPath(poly)

            for k, s in enumerate(self.robot.sensors):
                px, py = s.x_mm - self.robot.origin_x_mm, s.y_mm - self.robot.origin_y_mm
                rx, ry = rot(px, py, ang)
                cx, cy = x + rx, y + ry
                r = s.size_mm / 2.0
                sp = QPainterPath(); sp.addEllipse(cx - r, cy - r, 2*r, 2*r)
                if k < len(self.anim_items["sensors"]):
                    self.anim_items["sensors"][k].setPath(sp)

            for k, wdef in enumerate(self.robot.wheels):
                half_w = float(getattr(wdef, 'width_mm', WHEEL_W_MM)) * 0.5
                half_h = float(getattr(wdef, 'height_mm', WHEEL_H_MM)) * 0.5

                px, py = wdef.x_mm - self.robot.origin_x_mm, wdef.y_mm - self.robot.origin_y_mm
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
