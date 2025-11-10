"""Robot Editor UI (PySide6)

This module provides a simple, beginner-friendly graphical editor for a small robot.
You can see the robot envelope (overall size), place the left/right wheels, and add
square sensors on a grid. The editor lets you import/export a JSON robot definition,
zoom/pan the view, and nudge sensors with the keyboard.

The comments and docstrings in this file aim to explain each part in plain English
so that someone new to Python/Qt can follow along.
"""
from __future__ import annotations
import sys, json, math
from typing import Optional, Tuple, List

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QGraphicsScene, QGraphicsView, QFormLayout,
    QVBoxLayout, QHBoxLayout, QLabel, QDoubleSpinBox, QPushButton, QListWidget,
    QListWidgetItem, QFileDialog, QMessageBox, QSplitter, QCheckBox, QInputDialog, QGridLayout,
    QDialog, QSpinBox, QComboBox, QTabWidget, QPlainTextEdit
)
from PySide6.QtGui import QPen, QColor, QAction, QPainter, QKeyEvent
from PySide6.QtCore import Qt, QPointF, QRectF, Signal, QTimer

from Utils.robot_geometry import (
    Pt, Envelope,
    DEFAULT_WHEEL_W, DEFAULT_WHEEL_H, DEFAULT_SENSOR_SIZE,
)
from Utils.robot_model import RobotModel, Wheel, Sensor

TAPE_THICKNESS_MM = 20.0

def _safe_json_loads(txt: str, fallback):
    """Parse JSON from a plain text box; if invalid, return fallback."""
    try:
        obj = json.loads(txt)
        return obj
    except Exception:
        return fallback

class ElectricalDialog(QDialog):
    """Modal dialog to capture user-supplied robot parameters grouped into tabs.
    Geometry stays computed by the editor.

    This dialog was simplified to keep only the STRICTLY necessary parameters for the simulator
    and to rename/add fields according to the new robot-spec structure.
    """
    def __init__(self, model: RobotModel, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Electrical / Motor / Dynamics parameters")
        self.model = model

        root = QVBoxLayout(self)
        tabs = QTabWidget(self)
        root.addWidget(tabs)

        tab_batt = QWidget(); form_batt = QFormLayout(tab_batt)

        self.spin_batt_v = QDoubleSpinBox(); self.spin_batt_v.setRange(0.1, 60.0); self.spin_batt_v.setDecimals(3); self.spin_batt_v.setSuffix(" V")
        self.spin_batt_v.setToolTip("Open-circuit voltage at the CURRENT state of charge (SoC). Used as initial condition / sanity check. Effective terminal V = OCV(SoC) − I·(R_batt + wiring_R) − driver_drop_V. If an OCV table is provided, the simulator will prefer it to map SoC→OCV over time.")
        form_batt.addRow(QLabel("batteryVoltageV — OCV @ current SoC"), self.spin_batt_v)

        self.spin_batt_cap = QDoubleSpinBox(); self.spin_batt_cap.setRange(1.0, 20000.0); self.spin_batt_cap.setDecimals(0); self.spin_batt_cap.setSuffix(" mAh")
        self.spin_batt_cap.setToolTip("Battery nominal capacity in mAh. Drives SoC integration (discharge/charge) and range estimation.")
        form_batt.addRow(QLabel("Capacity"), self.spin_batt_cap)

        self.sp_Rbatt = QDoubleSpinBox(); self.sp_Rbatt.setRange(0.0, 1.0); self.sp_Rbatt.setDecimals(9); self.sp_Rbatt.setSuffix(" Ω")
        self.sp_Rbatt.setToolTip("Battery internal series resistance (Ω). Models instantaneous voltage sag under load.")
        form_batt.addRow(QLabel("Battery ESR"), self.sp_Rbatt)

        self.sp_wiring_R = QDoubleSpinBox(); self.sp_wiring_R.setRange(0.0, 1.0); self.sp_wiring_R.setDecimals(9); self.sp_wiring_R.setSuffix(" Ω")
        self.sp_wiring_R.setToolTip("Series resistance of cables/connectors between battery and drivers (Ω). Add to ESR for drop calculations.")
        form_batt.addRow(QLabel("wiring_R_ohm — harness resistance"), self.sp_wiring_R)

        self.sp_driver_drop = QDoubleSpinBox(); self.sp_driver_drop.setRange(0.0, 12.0); self.sp_driver_drop.setDecimals(3); self.sp_driver_drop.setSuffix(" V")
        self.sp_driver_drop.setToolTip("Approx. fixed drop across the motor driver/H-bridge (V). Represents diode/bridge losses or regulator headroom.")
        form_batt.addRow(QLabel("H-bridge Drop"), self.sp_driver_drop)

        self.sp_batt_rc_ms = QDoubleSpinBox(); self.sp_batt_rc_ms.setRange(0.0, 1e6); self.sp_batt_rc_ms.setDecimals(1); self.sp_batt_rc_ms.setSuffix(" ms")
        self.sp_batt_rc_ms.setToolTip("Optional: RC time constant (ms) of a single-pole Thévenin battery model. Adds transient recovery. Set 0 to disable.")
        form_batt.addRow(QLabel("batt_RC_timeconst_ms — Thévenin pole (opt)"), self.sp_batt_rc_ms)

        self.txt_batt_ocv_tbl = QPlainTextEdit()
        self.txt_batt_ocv_tbl.setPlaceholderText('[ [0.0, 7.2], [0.2, 7.6], [0.5, 7.9], [0.8, 8.2], [1.0, 8.4] ]  # SoC–OCV pairs')
        self.txt_batt_ocv_tbl.setToolTip("battery_OCV_table: JSON list of [SoC, OCV(V)] pairs used to map SoC→OCV during simulation. SoC in [0..1]. Must be sorted by SoC. Example:\n[[0.0, 7.2],[0.20, 7.6],[0.50, 7.9],[0.80, 8.2],[1.00, 8.4]]")
        self.txt_batt_ocv_tbl.setFixedHeight(90)
        form_batt.addRow(QLabel("battery_OCV_table (JSON)"), self.txt_batt_ocv_tbl)

        tabs.addTab(tab_batt, "Battery")

        tab_mot = QWidget(); form_mot = QFormLayout(tab_mot)

        self.sp_gear_ratio = QDoubleSpinBox(); self.sp_gear_ratio.setRange(0.001, 1000.0); self.sp_gear_ratio.setDecimals(9)
        self.sp_gear_ratio.setToolTip("Gear ratio defined as motor:wheel. Example: 10 ⇒ motor rotates 10× faster than the wheel. Affects torque/speed trade-offs and reflected inertia.")
        form_mot.addRow(QLabel("gear_ratio — motor:wheel"), self.sp_gear_ratio)

        self.sp_eta = QDoubleSpinBox(); self.sp_eta.setRange(0.0, 1.0); self.sp_eta.setDecimals(9)
        self.sp_eta.setToolTip("Transmission efficiency η in [0..1]. Multiplies available torque at the wheel and reduces back-EMF reflected to the motor.")
        form_mot.addRow(QLabel("Transmission efficiency"), self.sp_eta)

        self.sp_Rm = QDoubleSpinBox(); self.sp_Rm.setRange(0.0, 100.0); self.sp_Rm.setDecimals(9); self.sp_Rm.setSuffix(" Ω")
        self.sp_Rm.setToolTip("Motor phase/winding resistance (Ω). Sets stall current and I·R losses with the driver limits.")
        form_mot.addRow(QLabel("Winding Resistance"), self.sp_Rm)

        self.sp_Lm = QDoubleSpinBox(); self.sp_Lm.setRange(0.0, 10.0); self.sp_Lm.setDecimals(9); self.sp_Lm.setSuffix(" H")
        self.sp_Lm.setToolTip("Motor inductance (H). Shapes current ripple and response to PWM/step inputs; improves realism in transients.")
        form_mot.addRow(QLabel("Inductance"), self.sp_Lm)

        self.sp_Kv_rpmV = QDoubleSpinBox(); self.sp_Kv_rpmV.setRange(0.0, 100000.0); self.sp_Kv_rpmV.setDecimals(9)
        self.sp_Kv_rpmV.setToolTip("Speed constant Kv (rpm/V). You may fill Kv OR Kt. If both are given, the dialog derives a coherent pair and Kt takes precedence.")
        form_mot.addRow(QLabel("Speed constant Kv"), self.sp_Kv_rpmV)

        self.sp_Kt = QDoubleSpinBox(); self.sp_Kt.setRange(0.0, 10.0); self.sp_Kt.setDecimals(9); self.sp_Kt.setSuffix(" N·m/A")
        self.sp_Kt.setToolTip("Torque constant Kt (N·m/A). If >0, Kv is derived by Kv_rpm_per_V = 60/(2π·Kt).")
        form_mot.addRow(QLabel("Torque constant Kt"), self.sp_Kt)

        self.sp_I0 = QDoubleSpinBox(); self.sp_I0.setRange(0.0, 20.0); self.sp_I0.setDecimals(9); self.sp_I0.setSuffix(" A")
        self.sp_I0.setToolTip("No-load current (A) at nominal voltage; accounts for iron/friction losses when torque ≈ 0.")
        form_mot.addRow(QLabel("No-Load Current"), self.sp_I0)

        self.sp_b_visc = QDoubleSpinBox(); self.sp_b_visc.setRange(0.0, 1.0); self.sp_b_visc.setDecimals(9); self.sp_b_visc.setSuffix(" N·m·s/rad")
        self.sp_b_visc.setToolTip("Viscous friction coefficient (N·m·s/rad). Produces torque proportional to angular speed.")
        form_mot.addRow(QLabel("Viscous Friction"), self.sp_b_visc)

        self.sp_tau_c = QDoubleSpinBox(); self.sp_tau_c.setRange(0.0, 1.0); self.sp_tau_c.setDecimals(9); self.sp_tau_c.setSuffix(" N·m")
        self.sp_tau_c.setToolTip("Coulomb (dry) friction torque (N·m). Static offset opposing motion.")
        form_mot.addRow(QLabel("Coulomb Friction"), self.sp_tau_c)

        self.sp_Jm = QDoubleSpinBox(); self.sp_Jm.setRange(0.0, 1.0); self.sp_Jm.setDecimals(9); self.sp_Jm.setSuffix(" kg·m²")
        self.sp_Jm.setToolTip("Motor rotor inertia (kg·m²). Affects acceleration and current spikes during transients.")
        form_mot.addRow(QLabel("Motor Inertia"), self.sp_Jm)

        self.sp_Jl = QDoubleSpinBox(); self.sp_Jl.setRange(0.0, 1.0); self.sp_Jl.setDecimals(9); self.sp_Jl.setSuffix(" kg·m²")
        self.sp_Jl.setToolTip("Load inertia reflected to the motor side (kg·m²). Include wheels/gear/load referred by gear_ratio².")
        form_mot.addRow(QLabel("Reflected Load Inertia"), self.sp_Jl)

        self.sp_stall_current = QDoubleSpinBox(); self.sp_stall_current.setRange(0.0, 200.0); self.sp_stall_current.setDecimals(2); self.sp_stall_current.setSuffix(" A")
        self.sp_stall_current.setToolTip("Datasheet stall current per motor (A). Useful for sanity checks vs. R_motor and driver limits.")
        form_mot.addRow(QLabel("Stall Current"), self.sp_stall_current)

        self.sp_driver_ilim = QDoubleSpinBox(); self.sp_driver_ilim.setRange(0.0, 200.0); self.sp_driver_ilim.setDecimals(2); self.sp_driver_ilim.setSuffix(" A")
        self.sp_driver_ilim.setToolTip("Motor driver current limit (A). If 0, the simulator will not clamp current at the driver stage.")
        form_mot.addRow(QLabel("Driver Current Limit"), self.sp_driver_ilim)

        tabs.addTab(tab_mot, "Motor")

        tab_dyn = QWidget(); form_dyn = QFormLayout(tab_dyn)

        self.sp_mass = QDoubleSpinBox(); self.sp_mass.setRange(0.001, 100.0); self.sp_mass.setDecimals(3); self.sp_mass.setSuffix(" kg")
        self.sp_mass.setToolTip("Total mass (kg) of the robot including battery and payload. Used for linear dynamics and traction.")
        form_dyn.addRow(QLabel("mass_kg — total mass"), self.sp_mass)

        self.sp_Rwheel_geom = QDoubleSpinBox(); self.sp_Rwheel_geom.setRange(0.1, 500.0); self.sp_Rwheel_geom.setDecimals(3); self.sp_Rwheel_geom.setSuffix(" mm")
        self.sp_Rwheel_geom.setToolTip("Geometric wheel radius (mm). The simulator uses an effective radius internally to account for tire deformation/slip.")
        form_dyn.addRow(QLabel("wheel_radius_mm — geometric"), self.sp_Rwheel_geom)

        self.sp_Rwheel_eff = QDoubleSpinBox(); self.sp_Rwheel_eff.setRange(0.0, 500.0); self.sp_Rwheel_eff.setDecimals(3); self.sp_Rwheel_eff.setSuffix(" mm")
        self.sp_Rwheel_eff.setToolTip("Optional effective wheel radius (mm). If 0, it may be computed from slip/contact model.")
        form_dyn.addRow(QLabel("R_wheel_eff_mm — effective (opt)"), self.sp_Rwheel_eff)

        self.sp_track = QDoubleSpinBox(); self.sp_track.setRange(0.0, 2000.0); self.sp_track.setDecimals(3); self.sp_track.setSuffix(" mm")
        self.sp_track.setToolTip("Track (mm): center-to-center spacing of left vs right wheels. Critical for differential-drive kinematics/turning radius.")
        form_dyn.addRow(QLabel("track_mm — wheel spacing"), self.sp_track)

        self.sp_wheelbase = QDoubleSpinBox(); self.sp_wheelbase.setRange(0.0, 2000.0); self.sp_wheelbase.setDecimals(3); self.sp_wheelbase.setSuffix(" mm")
        self.sp_wheelbase.setToolTip("Wheelbase (mm): distance from CG to tractive axle along X. Leave 0 if not modeling CG offset effects.")
        form_dyn.addRow(QLabel("wheelbase_mm — CG↔axle"), self.sp_wheelbase)

        self.sp_wheelbase_off = QDoubleSpinBox(); self.sp_wheelbase_off.setRange(-2000.0, 2000.0); self.sp_wheelbase_off.setDecimals(3); self.sp_wheelbase_off.setSuffix(" mm")
        self.sp_wheelbase_off.setToolTip("Signed offset (mm) if tractive axle does not pass through the CG. Impacts load transfer/turning dynamics.")
        form_dyn.addRow(QLabel("wheelbase_offset_mm — axle offset"), self.sp_wheelbase_off)

        self.sp_Jbody = QDoubleSpinBox(); self.sp_Jbody.setRange(0.0, 10.0); self.sp_Jbody.setDecimals(9); self.sp_Jbody.setSuffix(" kg·m²")
        self.sp_Jbody.setToolTip("Yaw moment of inertia of the body (kg·m²) about vertical axis. Shapes angular acceleration and turning response.")
        form_dyn.addRow(QLabel("J_body_kgm2 — yaw inertia"), self.sp_Jbody)

        self.sp_Jwheel = QDoubleSpinBox(); self.sp_Jwheel.setRange(0.0, 1.0); self.sp_Jwheel.setDecimals(9); self.sp_Jwheel.setSuffix(" kg·m²")
        self.sp_Jwheel.setToolTip("Optional wheel/axle inertia per wheel (kg·m²). Improves transient accuracy when accelerating/decelerating.")
        form_dyn.addRow(QLabel("J_wheel_kgm2 — wheel inertia (opt)"), self.sp_Jwheel)

        self.sp_mu_static = QDoubleSpinBox(); self.sp_mu_static.setRange(0.0, 5.0); self.sp_mu_static.setDecimals(3)
        self.sp_mu_static.setToolTip("Static friction coefficient (wheel–ground). Sets peak tractive force before slip.")
        form_dyn.addRow(QLabel("mu_static — static friction"), self.sp_mu_static)

        self.sp_mu_kin = QDoubleSpinBox(); self.sp_mu_kin.setRange(0.0, 5.0); self.sp_mu_kin.setDecimals(3)
        self.sp_mu_kin.setToolTip("Kinetic friction coefficient (wheel–ground). Applies after slip occurs; typically < mu_static.")
        form_dyn.addRow(QLabel("mu_kinetic — kinetic friction"), self.sp_mu_kin)

        self.sp_Crr = QDoubleSpinBox(); self.sp_Crr.setRange(0.0, 0.5); self.sp_Crr.setDecimals(9)
        self.sp_Crr.setToolTip("Rolling resistance coefficient Crr (dimensionless). Typical 0.002–0.02 for small robots; opposes motion at all speeds.")
        form_dyn.addRow(QLabel("Crr — rolling resistance"), self.sp_Crr)

        tabs.addTab(tab_dyn, "Dynamics/Geom")

        tab_sens = QWidget(); form_sens = QFormLayout(tab_sens)

        enc_title = QLabel("<b>Encoders</b>"); enc_title.setToolTip("Wheel encoder configuration used for odometry and speed estimates.")
        form_sens.addRow(enc_title)
        self.chk_enc_enable = QCheckBox("Enable encoders"); self.chk_enc_enable.setToolTip("Enable/disable encoders in the simulation.")
        form_sens.addRow(self.chk_enc_enable)

        self.combo_enc_type = QComboBox(); self.combo_enc_type.addItems(["incremental","absolute"])
        self.combo_enc_type.setToolTip("Type of encoder: incremental or absolute. Affects how odometry is synthesized.")
        form_sens.addRow(QLabel("Type"), self.combo_enc_type)

        self.sp_enc_ppr = QSpinBox(); self.sp_enc_ppr.setRange(0, 1_000_000)
        self.sp_enc_ppr.setToolTip("Encoder pulses per mechanical revolution (PPR). Used to compute distance per pulse.")
        form_sens.addRow(QLabel("PPR"), self.sp_enc_ppr)

        self.sp_enc_res_bits = QSpinBox(); self.sp_enc_res_bits.setRange(0, 32)
        self.sp_enc_res_bits.setToolTip("Resolution in bits if the encoder is absolute.")
        form_sens.addRow(QLabel("Resolution (bits)"), self.sp_enc_res_bits)

        self.sp_enc_noise = QDoubleSpinBox(); self.sp_enc_noise.setRange(0.0, 10000.0); self.sp_enc_noise.setDecimals(3)
        self.sp_enc_noise.setToolTip("Standard deviation of encoder measurement noise (pulses).")
        form_sens.addRow(QLabel("Noise std (pulses)"), self.sp_enc_noise)

        self.sp_enc_rate = QDoubleSpinBox(); self.sp_enc_rate.setRange(0.0, 100000.0); self.sp_enc_rate.setDecimals(1); self.sp_enc_rate.setSuffix(" Hz")
        self.sp_enc_rate.setToolTip("Encoder update rate (Hz) in the simulation.")
        form_sens.addRow(QLabel("Update rate"), self.sp_enc_rate)

        imu_title = QLabel("<b>IMU</b>"); imu_title.setToolTip("Simulated inertial sensor for heading/acceleration measurements.")
        form_sens.addRow(imu_title)
        self.chk_imu_enable = QCheckBox("Enable IMU"); self.chk_imu_enable.setToolTip("Enable/disable simulated IMU readings.")
        form_sens.addRow(self.chk_imu_enable)

        self.sp_imu_std = QDoubleSpinBox(); self.sp_imu_std.setRange(0.0, 360.0); self.sp_imu_std.setDecimals(3); self.sp_imu_std.setSuffix(" °")
        self.sp_imu_std.setToolTip("IMU angular noise standard deviation (degrees).")
        form_sens.addRow(QLabel("Std"), self.sp_imu_std)

        self.sp_imu_bias = QDoubleSpinBox(); self.sp_imu_bias.setRange(0.0, 360.0); self.sp_imu_bias.setDecimals(3); self.sp_imu_bias.setSuffix(" °/s")
        self.sp_imu_bias.setToolTip("IMU angular bias/drift (degrees per second).")
        form_sens.addRow(QLabel("Bias"), self.sp_imu_bias)

        self.sp_imu_rate = QDoubleSpinBox(); self.sp_imu_rate.setRange(0.0, 100000.0); self.sp_imu_rate.setDecimals(1); self.sp_imu_rate.setSuffix(" Hz")
        self.sp_imu_rate.setToolTip("IMU update frequency (Hz) in the simulation.")
        form_sens.addRow(QLabel("Update rate"), self.sp_imu_rate)

        self.sp_imu_lat = QDoubleSpinBox(); self.sp_imu_lat.setRange(0.0, 10000.0); self.sp_imu_lat.setDecimals(1); self.sp_imu_lat.setSuffix(" ms")
        self.sp_imu_lat.setToolTip("IMU latency (ms) applied to sensor updates.")
        form_sens.addRow(QLabel("Latency"), self.sp_imu_lat)

        ir_title = QLabel("<b>IR line sensors</b>"); ir_title.setToolTip("Signal model for IR line sensors (analog or digital).")
        form_sens.addRow(ir_title)

        self.combo_sensor_mode = QComboBox(); self.combo_sensor_mode.addItems(["analog", "digital"])
        self.combo_sensor_mode.setToolTip("Select whether sensors output analog values or digital levels.")
        form_sens.addRow(QLabel("Mode"), self.combo_sensor_mode)

        self.sp_sensor_bits = QSpinBox(); self.sp_sensor_bits.setRange(1, 16)
        self.sp_sensor_bits.setToolTip("ADC resolution in bits for analog mode (sets max value to 2^n-1). Ignored in digital mode.")
        form_sens.addRow(QLabel("Bits"), self.sp_sensor_bits)

        self.sp_value_line = QSpinBox(); self.sp_value_line.setRange(0, 65535)
        self.sp_value_line.setToolTip("Output value when the sensor sees the line (dark tape).")
        form_sens.addRow(QLabel("Value of line"), self.sp_value_line)

        self.sp_value_bg = QSpinBox(); self.sp_value_bg.setRange(0, 65535)
        self.sp_value_bg.setToolTip("Output value when the sensor sees the background (board).")
        form_sens.addRow(QLabel("Value of background"), self.sp_value_bg)

        self.sp_noise_line = QSpinBox(); self.sp_noise_line.setRange(0, 65535)
        self.sp_noise_line.setToolTip("Analog noise amplitude on line readings. Used only in analog mode.")
        form_sens.addRow(QLabel("Analog noise (line)"), self.sp_noise_line)

        self.sp_noise_bg = QSpinBox(); self.sp_noise_bg.setRange(0, 65535)
        self.sp_noise_bg.setToolTip("Analog noise amplitude on background readings. Used only in analog mode.")
        form_sens.addRow(QLabel("Analog noise (background)"), self.sp_noise_bg)

        tabs.addTab(tab_sens, "Sensors")

        tab_odom = QWidget(); form_odom = QFormLayout(tab_odom)
        self.chk_odom_enable = QCheckBox("Enable odometry"); self.chk_odom_enable.setToolTip("Enable/disable synthesized odometry from encoders.")
        form_odom.addRow(self.chk_odom_enable)

        self.sp_odom_std_mm = QDoubleSpinBox(); self.sp_odom_std_mm.setRange(0.0, 10000.0); self.sp_odom_std_mm.setDecimals(3); self.sp_odom_std_mm.setSuffix(" mm")
        self.sp_odom_std_mm.setToolTip("Odometer noise standard deviation for position (mm).")
        form_odom.addRow(QLabel("Noise std"), self.sp_odom_std_mm)

        self.sp_odom_std_deg = QDoubleSpinBox(); self.sp_odom_std_deg.setRange(0.0, 360.0); self.sp_odom_std_deg.setDecimals(3); self.sp_odom_std_deg.setSuffix(" °")
        self.sp_odom_std_deg.setToolTip("Odometer noise standard deviation for heading (degrees).")
        form_odom.addRow(QLabel("Noise std"), self.sp_odom_std_deg)

        self.sp_odom_bias_mm = QDoubleSpinBox(); self.sp_odom_bias_mm.setRange(0.0, 1000.0); self.sp_odom_bias_mm.setDecimals(3); self.sp_odom_bias_mm.setSuffix(" mm")
        self.sp_odom_bias_mm.setToolTip("Odometer systematic bias for position (mm).")
        form_odom.addRow(QLabel("Bias"), self.sp_odom_bias_mm)

        self.sp_odom_bias_deg = QDoubleSpinBox(); self.sp_odom_bias_deg.setRange(0.0, 360.0); self.sp_odom_bias_deg.setDecimals(3); self.sp_odom_bias_deg.setSuffix(" °")
        self.sp_odom_bias_deg.setToolTip("Odometer systematic bias for heading (degrees).")
        form_odom.addRow(QLabel("Bias"), self.sp_odom_bias_deg)

        self.sp_odom_rate = QDoubleSpinBox(); self.sp_odom_rate.setRange(0.0, 100000.0); self.sp_odom_rate.setDecimals(1); self.sp_odom_rate.setSuffix(" Hz")
        self.sp_odom_rate.setToolTip("Odometer update frequency (Hz) in the simulation.")
        form_odom.addRow(QLabel("Update rate"), self.sp_odom_rate)
        tabs.addTab(tab_odom, "Odometry")

        self.spin_batt_v.setValue(float(getattr(self.model, "batteryVoltageV", 7.4)))
        self.spin_batt_cap.setValue(float(getattr(self.model, "batteryCapacitymAh", 850.0)))
        self.sp_Rbatt.setValue(float(getattr(self.model, "RbattOhm", 0.0)))
        self.sp_wiring_R.setValue(float(getattr(self.model, "wiringROhm", 0.0)))
        self.sp_driver_drop.setValue(float(getattr(self.model, "driverDropV", 0.0)))
        self.sp_batt_rc_ms.setValue(float(getattr(self.model, "battRCTimeConstMs", 0.0)))
        ocv_tbl = getattr(self.model, "batteryOCVTable", [[0.0, 7.2], [0.5, 7.4], [1.0, 8.4]])
        try:
            self.txt_batt_ocv_tbl.setPlainText(json.dumps(ocv_tbl, ensure_ascii=False))
        except Exception:
            self.txt_batt_ocv_tbl.setPlainText("[]")

        self.sp_gear_ratio.setValue(float(getattr(self.model, "gearRatio", 1.0)))
        self.sp_eta.setValue(float(getattr(self.model, "transmissionEfficiency", 1.0)))
        self.sp_Rm.setValue(float(getattr(self.model, "RmotorOhm", 0.0)))
        self.sp_Lm.setValue(float(getattr(self.model, "LmotorH", 0.0)))
        self.sp_Kv_rpmV.setValue(float(getattr(self.model, "KvRPMperV", 0.0)))
        self.sp_Kt.setValue(float(getattr(self.model, "KtNmPerA", 0.0)))
        self.sp_I0.setValue(float(getattr(self.model, "I0NoLoadA", 0.0)))
        self.sp_b_visc.setValue(float(getattr(self.model, "bViscNmPerRadps", 0.0)))
        self.sp_tau_c.setValue(float(getattr(self.model, "tauCoulombNm", 0.0)))
        self.sp_Jm.setValue(float(getattr(self.model, "JmotorKGm2", 0.0)))
        self.sp_Jl.setValue(float(getattr(self.model, "JloadKGm2", 0.0)))
        self.sp_stall_current.setValue(float(getattr(self.model, "stallCurrentA", 0.0)))
        self.sp_driver_ilim.setValue(float(getattr(self.model, "driverCurrentLimitA", 0.0)))

        self.sp_mass.setValue(float(getattr(self.model, "massKG", 0.20)))
        self.sp_Rwheel_geom.setValue(float(getattr(self.model, "wheelRadiusMM", 11.0)))
        self.sp_Rwheel_eff.setValue(float(getattr(self.model, "RwheelEffMM", 0.0)))
        try:
            wl = self.model.find_wheel("left"); wr = self.model.find_wheel("right")
            default_track = abs((wl.yMM if wl else 0.0) - (wr.yMM if wr else 0.0))
        except Exception:
            default_track = 0.0
        self.sp_track.setValue(float(getattr(self.model, "trackMM", default_track)))
        self.sp_wheelbase.setValue(float(getattr(self.model, "wheelbaseMM", 0.0)))
        self.sp_wheelbase_off.setValue(float(getattr(self.model, "wheelbaseOffsetMM", 0.0)))
        self.sp_Jbody.setValue(float(getattr(self.model, "JbodyKGm2", 0.0)))
        self.sp_Jwheel.setValue(float(getattr(self.model, "JwheelKGm2", 0.0)))
        self.sp_mu_static.setValue(float(getattr(self.model, "muStatic", 1.0)))
        self.sp_mu_kin.setValue(float(getattr(self.model, "muKinetic", 0.8)))
        self.sp_Crr.setValue(float(getattr(self.model, "Crr", 0.005)))

        self.chk_enc_enable.setChecked(bool(getattr(self.model, "encoderEnable", False)))
        enc_type = str(getattr(self.model, "encoderType", "incremental")).lower()
        self.combo_enc_type.setCurrentIndex(0 if enc_type.startswith("i") else 1)
        self.sp_enc_ppr.setValue(int(getattr(self.model, "encoderPPR", 0)))
        self.sp_enc_res_bits.setValue(int(getattr(self.model, "encoderResolutionBits", 0)))
        self.sp_enc_noise.setValue(float(getattr(self.model, "encoderNoiseStdPulses", 0.0)))
        self.sp_enc_rate.setValue(float(getattr(self.model, "encoderUpdateRateHz", 0.0)))

        self.chk_imu_enable.setChecked(bool(getattr(self.model, "imuEnable", False)))
        self.sp_imu_std.setValue(float(getattr(self.model, "imuStdDeg", 0.0)))
        self.sp_imu_bias.setValue(float(getattr(self.model, "imuBiasDegPerS", 0.0)))
        self.sp_imu_rate.setValue(float(getattr(self.model, "imuUpdateRateHz", 0.0)))
        self.sp_imu_lat.setValue(float(getattr(self.model, "imuLatencyMs", 0.0)))

        self.combo_sensor_mode.setCurrentIndex(0 if str(getattr(self.model, "sensorMode", "analog")).lower().startswith("a") else 1)
        self.sp_sensor_bits.setValue(int(getattr(self.model, "sensorBits", 8)))
        maxv = (1 << int(self.sp_sensor_bits.value())) - 1
        self.sp_value_line.setMaximum(maxv); self.sp_value_bg.setMaximum(maxv); self.sp_noise_line.setMaximum(maxv); self.sp_noise_bg.setMaximum(maxv)
        self.sp_value_line.setValue(int(getattr(self.model, "valueOfLine", 0)))
        self.sp_value_bg.setValue(int(getattr(self.model, "valueOfBackground", 255)))
        self.sp_noise_line.setValue(int(getattr(self.model, "analogNoiseLine", 50)))
        self.sp_noise_bg.setValue(int(getattr(self.model, "analogNoiseBackground", 50)))

        def _apply_ir_bit_ranges():
            maxv = (1 << int(self.sp_sensor_bits.value())) - 1
            for w in (self.sp_value_line, self.sp_value_bg, self.sp_noise_line, self.sp_noise_bg):
                w.setMaximum(maxv)
        self.sp_sensor_bits.valueChanged.connect(_apply_ir_bit_ranges)

        btns = QHBoxLayout()
        self.btn_ok = QPushButton("OK")
        self.btn_cancel = QPushButton("Cancel")
        btns.addWidget(self.btn_ok); btns.addWidget(self.btn_cancel)
        root.addLayout(btns)
        self.btn_ok.clicked.connect(self.accept)
        self.btn_cancel.clicked.connect(self.reject)

    def accept(self):
        """Copy values to model and close. Geometry-related values remain computed elsewhere."""
        self.model.batteryVoltageV = float(self.spin_batt_v.value())
        self.model.batteryCapacitymAh = float(self.spin_batt_cap.value())
        self.model.RbattOhm = float(self.sp_Rbatt.value())
        self.model.wiringROhm = float(self.sp_wiring_R.value())
        self.model.driverDropV = float(self.sp_driver_drop.value())
        self.model.battRCTimeConstMs = float(self.sp_batt_rc_ms.value())
        self.model.batteryOCVTable = _safe_json_loads(self.txt_batt_ocv_tbl.toPlainText(), getattr(self.model, "batteryOCVTable", []))

        self.model.gearRatio = float(self.sp_gear_ratio.value())
        self.model.transmissionEfficiency = float(self.sp_eta.value())
        self.model.RmotorOhm = float(self.sp_Rm.value())
        self.model.LmotorH = float(self.sp_Lm.value())
        Kv = float(self.sp_Kv_rpmV.value())
        Kt = float(self.sp_Kt.value())
        if Kt > 0.0:
            self.model.KtNmPerA = Kt
            try:
                self.model.KvRPMperV = float(60.0/(2*math.pi*Kt))
            except ZeroDivisionError:
                self.model.KvRPMperV = 0.0
        else:
            self.model.KvRPMperV = Kv
            self.model.KtNmPerA = float(0.0 if Kv == 0.0 else 60.0/(2*math.pi*Kv))
        self.model.KvRadPerV = float(self.model.KvRPMperV * 2*math.pi/60.0) if self.model.KvRPMperV != 0 else 0.0
        self.model.I0NoLoadA = float(self.sp_I0.value())
        self.model.bViscNmPerRadps = float(self.sp_b_visc.value())
        self.model.tauCoulombNm = float(self.sp_tau_c.value())
        self.model.JmotorKGm2 = float(self.sp_Jm.value())
        self.model.JloadKGm2 = float(self.sp_Jl.value())
        self.model.JtotalKGm2 = float(self.model.JmotorKGm2 + self.model.JloadKGm2)
        self.model.stallCurrentA = float(self.sp_stall_current.value())
        self.model.driverCurrentLimitA = float(self.sp_driver_ilim.value())

        self.model.massKG = float(self.sp_mass.value())
        self.model.wheelRadiusMM = float(self.sp_Rwheel_geom.value())
        self.model.RwheelEffMM = float(self.sp_Rwheel_eff.value())
        self.model.trackMM = float(self.sp_track.value())
        self.model.wheelbaseMM = float(self.sp_wheelbase.value())
        self.model.wheelbaseOffsetMM = float(self.sp_wheelbase_off.value())
        self.model.JbodyKGm2 = float(self.sp_Jbody.value())
        self.model.JwheelKGm2 = float(self.sp_Jwheel.value())
        self.model.muStatic = float(self.sp_mu_static.value())
        self.model.muKinetic = float(self.sp_mu_kin.value())
        self.model.Crr = float(self.sp_Crr.value())

        self.model.encoderEnable = bool(self.chk_enc_enable.isChecked())
        self.model.encoderType = "incremental" if self.combo_enc_type.currentIndex() == 0 else "absolute"
        self.model.encoderPPR = int(self.sp_enc_ppr.value())
        self.model.encoderResolutionBits = int(self.sp_enc_res_bits.value())
        self.model.encoderNoiseStdPulses = float(self.sp_enc_noise.value())
        self.model.encoderUpdateRateHz = float(self.sp_enc_rate.value())

        self.model.imuEnable = bool(self.chk_imu_enable.isChecked())
        self.model.imuStdDeg = float(self.sp_imu_std.value())
        self.model.imuBiasDegPerS = float(self.sp_imu_bias.value())
        self.model.imuUpdateRateHz = float(self.sp_imu_rate.value())
        self.model.imuLatencyMs = float(self.sp_imu_lat.value())

        self.model.sensorMode = "analog" if self.combo_sensor_mode.currentIndex() == 0 else "digital"
        self.model.sensorBits = int(self.sp_sensor_bits.value())
        self.model.valueOfLine = int(self.sp_value_line.value())
        self.model.valueOfBackground = int(self.sp_value_bg.value())
        self.model.analogNoiseLine = int(self.sp_noise_line.value())
        self.model.analogNoiseBackground = int(self.sp_noise_bg.value())

        self.model.odomEnable = bool(self.chk_odom_enable.isChecked())
        self.model.odomNoiseStdMM = float(self.sp_odom_std_mm.value())
        self.model.odomNoiseStdDeg = float(self.sp_odom_std_deg.value())
        self.model.odomBiasMM = float(self.sp_odom_bias_mm.value())
        self.model.odomBiasDeg = float(self.sp_odom_bias_deg.value())
        self.model.odomUpdateRateHz = float(self.sp_odom_rate.value())

        super().accept()

class RobotScene(QGraphicsScene):
    """QGraphicsScene that draws the robot, grid, and selection.
        Left-click selects items; right-click moves the Rotation point (Alt+Right-click moves CG).
        It emits signals when the selection or origin changes for the UI to update."""
    selChanged = Signal(object, object)
    originMoved = Signal(float, float)
    cgMoved = Signal(float, float)

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
        """Handle mouse clicks. Left-click selects wheels/sensors; right-click moves the Rotation point; hold Alt for CG."""
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
            rx = p.x() - self.center_scene.x()
            ry = p.y() - self.center_scene.y()
            if event.modifiers() & Qt.AltModifier:
                self.model.cgXMM = rx; self.model.cgYMM = ry
                self.model.clamp_all_inside()
                self.cgMoved.emit(self.model.cgXMM, self.model.cgYMM)
            else:
                self.model.rotXMM = rx; self.model.rotYMM = ry
                self.model.clamp_all_inside()
                self.originMoved.emit(self.model.rotXMM, self.model.rotYMM)
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

        rotx = c.x() + float(getattr(self.model, 'rotXMM', 0.0))
        roty = c.y() + float(getattr(self.model, 'rotYMM', 0.0))
        cgx  = c.x() + float(getattr(self.model, 'cgXMM',  0.0))
        cgy  = c.y() + float(getattr(self.model, 'cgYMM',  0.0))
        rot_pen = QPen(QColor('#FF00AA'), 2, Qt.SolidLine, Qt.RoundCap)
        self.addLine(rotx-8, roty, rotx+8, roty, rot_pen)
        self.addLine(rotx, roty-8, rotx, roty+8, rot_pen)
        cg_pen = QPen(QColor('#00E5FF'), 2, Qt.SolidLine, Qt.RoundCap)
        self.addLine(cgx-8, cgy, cgx+8, cgy, cg_pen)
        self.addLine(cgx, cgy-8, cgx, cgy+8, cg_pen)

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

        if not hasattr(self.model, 'cgXMM'):  self.model.cgXMM  = float(getattr(self.model, 'originXMM', 0.0))
        if not hasattr(self.model, 'cgYMM'):  self.model.cgYMM  = float(getattr(self.model, 'originYMM', 0.0))
        if not hasattr(self.model, 'rotXMM'): self.model.rotXMM = 0.0
        if not hasattr(self.model, 'rotYMM'): self.model.rotYMM = 0.0

        if not hasattr(self.model, "wheelWidthMM"):  self.model.wheelWidthMM = 22.0
        if not hasattr(self.model, "wheelHeightMM"): self.model.wheelHeightMM = 15.0
        if not hasattr(self.model, "batteryVoltageV"):     self.model.batteryVoltageV = 7.4
        if not hasattr(self.model, "batteryCapacitymAh"):  self.model.batteryCapacitymAh = 850.0
        if not hasattr(self.model, "RbattOhm"):            self.model.RbattOhm = 0.0
        if not hasattr(self.model, "wiringROhm"):          self.model.wiringROhm = 0.0
        if not hasattr(self.model, "driverDropV"):         self.model.driverDropV = 0.0
        if not hasattr(self.model, "battRCTimeConstMs"):   self.model.battRCTimeConstMs = 0.0
        if not hasattr(self.model, "batteryOCVTable"):     self.model.batteryOCVTable = [[0.0, 7.2],[0.5, 7.4],[1.0, 8.4]]
        if not hasattr(self.model, "gearRatio"):                 self.model.gearRatio = 1.0
        if not hasattr(self.model, "transmissionEfficiency"):    self.model.transmissionEfficiency = 1.0
        if not hasattr(self.model, "RmotorOhm"):                 self.model.RmotorOhm = 0.0
        if not hasattr(self.model, "LmotorH"):                   self.model.LmotorH = 0.0
        if not hasattr(self.model, "KvRPMperV"):                 self.model.KvRPMperV = 0.0
        if not hasattr(self.model, "KvRadPerV"):                 self.model.KvRadPerV = 0.0
        if not hasattr(self.model, "KtNmPerA"):                  self.model.KtNmPerA = 0.0
        if not hasattr(self.model, "I0NoLoadA"):                 self.model.I0NoLoadA = 0.0
        if not hasattr(self.model, "bViscNmPerRadps"):           self.model.bViscNmPerRadps = 0.0
        if not hasattr(self.model, "tauCoulombNm"):              self.model.tauCoulombNm = 0.0
        if not hasattr(self.model, "JmotorKGm2"):                self.model.JmotorKGm2 = 0.0
        if not hasattr(self.model, "JloadKGm2"):                 self.model.JloadKGm2 = 0.0
        if not hasattr(self.model, "JtotalKGm2"):                self.model.JtotalKGm2 = 0.0
        if not hasattr(self.model, "stallCurrentA"):             self.model.stallCurrentA = 0.0
        if not hasattr(self.model, "driverCurrentLimitA"):       self.model.driverCurrentLimitA = 0.0
        if not hasattr(self.model, "controllerPwmResolutionBits"): self.model.controllerPwmResolutionBits = 12
        if not hasattr(self.model, "controllerPwmFrequencyHz"):    self.model.controllerPwmFrequencyHz = 20000.0
        if not hasattr(self.model, "controllerDeadbandPercent"):   self.model.controllerDeadbandPercent = 0.0
        if not hasattr(self.model, "motorPwmMin"):                 self.model.motorPwmMin = -4095
        if not hasattr(self.model, "motorPwmMax"):                 self.model.motorPwmMax = 4095
        if not hasattr(self.model, "simulationStepDtMs"):          self.model.simulationStepDtMs = 1.0
        if not hasattr(self.model, "massKG"):             self.model.massKG = 0.2
        if not hasattr(self.model, "wheelRadiusMM"):      self.model.wheelRadiusMM = 11.0
        if not hasattr(self.model, "RwheelEffMM"):        self.model.RwheelEffMM = 0.0
        if not hasattr(self.model, "trackMM"):            self.model.trackMM = 0.0
        if not hasattr(self.model, "wheelbaseMM"):        self.model.wheelbaseMM = 0.0
        if not hasattr(self.model, "wheelbaseOffsetMM"):  self.model.wheelbaseOffsetMM = 0.0
        if not hasattr(self.model, "JbodyKGm2"):          self.model.JbodyKGm2 = 0.0
        if not hasattr(self.model, "JwheelKGm2"):         self.model.JwheelKGm2 = 0.0
        if not hasattr(self.model, "muStatic"):           self.model.muStatic = 1.0
        if not hasattr(self.model, "muKinetic"):          self.model.muKinetic = 0.8
        if not hasattr(self.model, "Crr"):                self.model.Crr = 0.005
        if not hasattr(self.model, "encoderEnable"):    self.model.encoderEnable = False
        if not hasattr(self.model, "encoderType"):      self.model.encoderType = "incremental"
        if not hasattr(self.model, "encoderPPR"):       self.model.encoderPPR = 0
        if not hasattr(self.model, "encoderResolutionBits"): self.model.encoderResolutionBits = 0
        if not hasattr(self.model, "encoderNoiseStdPulses"): self.model.encoderNoiseStdPulses = 0.0
        if not hasattr(self.model, "encoderUpdateRateHz"): self.model.encoderUpdateRateHz = 0.0
        if not hasattr(self.model, "imuEnable"):        self.model.imuEnable = False
        if not hasattr(self.model, "imuStdDeg"):        self.model.imuStdDeg = 0.0
        if not hasattr(self.model, "imuBiasDegPerS"):   self.model.imuBiasDegPerS = 0.0
        if not hasattr(self.model, "imuUpdateRateHz"):  self.model.imuUpdateRateHz = 0.0
        if not hasattr(self.model, "imuLatencyMs"):     self.model.imuLatencyMs = 0.0
        if not hasattr(self.model, "odomEnable"):       self.model.odomEnable = False
        if not hasattr(self.model, "odomNoiseStdMM"):   self.model.odomNoiseStdMM = 0.0
        if not hasattr(self.model, "odomNoiseStdDeg"):  self.model.odomNoiseStdDeg = 0.0
        if not hasattr(self.model, "odomBiasMM"):       self.model.odomBiasMM = 0.0
        if not hasattr(self.model, "odomBiasDeg"):      self.model.odomBiasDeg = 0.0
        if not hasattr(self.model, "odomUpdateRateHz"): self.model.odomUpdateRateHz = 0.0
        if not hasattr(self.model, "sensorMode"):       self.model.sensorMode = "analog"
        if not hasattr(self.model, "sensorBits"):       self.model.sensorBits = 8
        if not hasattr(self.model, "valueOfLine"):      self.model.valueOfLine = 0
        if not hasattr(self.model, "valueOfBackground"): self.model.valueOfBackground = 255
        if not hasattr(self.model, "analogNoiseLine"):  self.model.analogNoiseLine = 50
        if not hasattr(self.model, "analogNoiseBackground"): self.model.analogNoiseBackground = 50

        self.scene = RobotScene(self.model)
        self.view = RobotView(self.scene)
        self.view.setSceneRect(0, 0, 1600, 1000)

        self.scene.selChanged.connect(self.on_scene_selection)
        self.scene.originMoved.connect(self.on_scene_rot_moved)
        self.scene.cgMoved.connect(self.on_scene_cg_moved)

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
            "<b>Rotation</b>: point the robot rotates about (e.g., axle midpoint). Right-click to move it."
            "<b>CG</b>: center of mass. Alt+Right-click to move it."
        )

        self.spin_rot_x = QDoubleSpinBox(); self.spin_rot_x.setRange(-10000,10000); self.spin_rot_x.setValue(self.model.rotXMM); self.spin_rot_x.setSuffix(" mm"); self.spin_rot_x.setToolTip(origin_hint)
        self.spin_rot_y = QDoubleSpinBox(); self.spin_rot_y.setRange(-10000,10000); self.spin_rot_y.setValue(self.model.rotYMM); self.spin_rot_y.setSuffix(" mm"); self.spin_rot_y.setToolTip(origin_hint)
        self.spin_cg_x  = QDoubleSpinBox();  self.spin_cg_x.setRange(-10000,10000);  self.spin_cg_x.setValue(self.model.cgXMM);  self.spin_cg_x.setSuffix(" mm");  self.spin_cg_x.setToolTip(origin_hint)
        self.spin_cg_y  = QDoubleSpinBox();  self.spin_cg_y.setRange(-10000,10000);  self.spin_cg_y.setValue(self.model.cgYMM);  self.spin_cg_y.setSuffix(" mm");  self.spin_cg_y.setToolTip(origin_hint)
        origin_grid = QGridLayout()
        origin_grid.addWidget(QLabel("Rotation X"), 0, 0)
        origin_grid.addWidget(QLabel("Rotation Y"), 0, 1)
        origin_grid.addWidget(self.spin_rot_x, 1, 0)
        origin_grid.addWidget(self.spin_rot_y, 1, 1)
        origin_grid.addWidget(QLabel("CG X"), 2, 0)
        origin_grid.addWidget(QLabel("CG Y"), 2, 1)
        origin_grid.addWidget(self.spin_cg_x, 3, 0)
        origin_grid.addWidget(self.spin_cg_y, 3, 1)
        origin_widget = QWidget(); origin_widget.setLayout(origin_grid)
        form.addRow(origin_widget)

        self.btn_rot_mid_wheels = QPushButton("Rotation = middle of wheels")
        self.btn_cg_mid_wheels  = QPushButton("CG = middle of wheels")
        form.addRow(self.btn_rot_mid_wheels)
        form.addRow(self.btn_cg_mid_wheels)

        self.spin_wl_x = QDoubleSpinBox(); self.spin_wl_x.setRange(-10000, 10000); self.spin_wl_x.setSuffix(" mm"); self.spin_wl_x.setToolTip("Left wheel X position relative to the origin (mm).")
        self.spin_wl_y = QDoubleSpinBox(); self.spin_wl_y.setRange(-10000, 10000); self.spin_wl_y.setSuffix(" mm"); self.spin_wl_y.setToolTip("Left wheel Y position relative to the origin (mm).")
        self.spin_wr_x = QDoubleSpinBox(); self.spin_wr_x.setRange(-10000, 10000); self.spin_wr_x.setSuffix(" mm"); self.spin_wr_x.setToolTip("Right wheel X position relative to the origin (mm).")
        self.spin_wr_y = QDoubleSpinBox(); self.spin_wr_y.setRange(-10000, 10000); self.spin_wr_y.setSuffix(" mm"); self.spin_wr_y.setToolTip("Right wheel Y position relative to the origin (mm).")

        self.spin_wheel_w = QDoubleSpinBox(); self.spin_wheel_w.setRange(1.0, 200.0); self.spin_wheel_w.setValue(float(getattr(self.model, "wheelWidthMM", 22.0))); self.spin_wheel_w.setSuffix(" mm"); self.spin_wheel_w.setToolTip("Wheel rectangle width (X) in mm. Visual only.")
        self.spin_wheel_h = QDoubleSpinBox(); self.spin_wheel_h.setRange(1.0, 200.0); self.spin_wheel_h.setValue(float(getattr(self.model, "wheelHeightMM", 15.0))); self.spin_wheel_h.setSuffix(" mm"); self.spin_wheel_h.setToolTip("Wheel rectangle height (Y) in mm. Visual only.")

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

        self.spin_s_x = QDoubleSpinBox(); self.spin_s_x.setRange(-10000, 10000); self.spin_s_x.setSuffix(" mm"); self.spin_s_x.setToolTip("Sensor X position relative to the origin (mm).")
        self.spin_s_y = QDoubleSpinBox(); self.spin_s_y.setRange(-10000, 10000); self.spin_s_y.setSuffix(" mm"); self.spin_s_y.setToolTip("Sensor Y position relative to the origin (mm).")

        self.chk_snap = QCheckBox("Snap to grid (uses Grid mm)"); self.chk_snap.setToolTip("When enabled, sensor coordinates snap to the defined grid step.")

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

        self.btn_elec   = QPushButton("Advanced parameters…"); self.btn_elec.setToolTip("Open a dialog to edit battery/motor/dynamics parameters.")
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

        self.spin_rot_x.valueChanged.connect(self.on_rot_change)
        self.spin_rot_y.valueChanged.connect(self.on_rot_change)
        self.spin_cg_x.valueChanged.connect(self.on_cg_change)
        self.spin_cg_y.valueChanged.connect(self.on_cg_change)
        self.btn_rot_mid_wheels.clicked.connect(self.on_rot_from_mid_wheels)
        self.btn_cg_mid_wheels.clicked.connect(self.on_cg_from_mid_wheels)

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


    def on_scene_rot_moved(self, x: float, y: float):
        """Update Rotation X/Y spin boxes when the rotation point is moved directly on the canvas."""
        self.spin_rot_x.blockSignals(True); self.spin_rot_y.blockSignals(True)
        self.spin_rot_x.setValue(x); self.spin_rot_y.setValue(y)
        self.spin_rot_x.blockSignals(False); self.spin_rot_y.blockSignals(False)
        self.model.rotXMM = x; self.model.rotYMM = y
        self.scene.rebuild()

    def on_scene_cg_moved(self, x: float, y: float):
        """Update CG X/Y spin boxes when the CG is moved directly on the canvas."""
        self.spin_cg_x.blockSignals(True); self.spin_cg_y.blockSignals(True)
        self.spin_cg_x.setValue(x); self.spin_cg_y.setValue(y)
        self.spin_cg_x.blockSignals(False); self.spin_cg_y.blockSignals(False)
        self.model.cgXMM = x; self.model.cgYMM = y
        self.scene.rebuild()

    def on_rot_change(self):
        """Apply new rotation coordinates typed by the user, clamping to safe limits."""
        self.model.rotXMM = float(self.spin_rot_x.value())
        self.model.rotYMM = float(self.spin_rot_y.value())
        self.model.clamp_all_inside()
        self.spin_rot_x.blockSignals(True); self.spin_rot_y.blockSignals(True)
        self.spin_rot_x.setValue(self.model.rotXMM); self.spin_rot_y.setValue(self.model.rotYMM)
        self.spin_rot_x.blockSignals(False); self.spin_rot_y.blockSignals(False)
        self.scene.rebuild()

    def on_cg_change(self):
        """Apply new CG coordinates typed by the user, clamping to safe limits."""
        self.model.cgXMM = float(self.spin_cg_x.value())
        self.model.cgYMM = float(self.spin_cg_y.value())
        self.model.clamp_all_inside()
        self.spin_cg_x.blockSignals(True); self.spin_cg_y.blockSignals(True)
        self.spin_cg_x.setValue(self.model.cgXMM); self.spin_cg_y.setValue(self.model.cgYMM)
        self.spin_cg_x.blockSignals(False); self.spin_cg_y.blockSignals(False)
        self.scene.rebuild()

    def on_rot_from_mid_wheels(self):
        """Place the rotation point at the midpoint between the two wheels."""
        wl = self.model.find_wheel("left"); wr = self.model.find_wheel("right")
        if not wl or not wr:
            QMessageBox.warning(self, "Wheels not found", "Left and right wheels are required to compute midpoint.")
            return
        self.model.rotXMM = (wl.xMM + wr.xMM)/2.0
        self.model.rotYMM = (wl.yMM + wr.yMM)/2.0
        self.spin_rot_x.blockSignals(True); self.spin_rot_y.blockSignals(True)
        self.spin_rot_x.setValue(self.model.rotXMM); self.spin_rot_y.setValue(self.model.rotYMM)
        self.spin_rot_x.blockSignals(False); self.spin_rot_y.blockSignals(False)
        self.scene.rebuild()

    def on_cg_from_mid_wheels(self):
        """Place the CG at the midpoint between the two wheels (simple assumption)."""
        wl = self.model.find_wheel("left"); wr = self.model.find_wheel("right")
        if not wl or not wr:
            QMessageBox.warning(self, "Wheels not found", "Left and right wheels are required to compute midpoint.")
            return
        self.model.cgXMM = (wl.xMM + wr.xMM)/2.0
        self.model.cgYMM = (wl.yMM + wr.yMM)/2.0
        self.spin_cg_x.blockSignals(True); self.spin_cg_y.blockSignals(True)
        self.spin_cg_x.setValue(self.model.cgXMM); self.spin_cg_y.setValue(self.model.cgYMM)
        self.spin_cg_x.blockSignals(False); self.spin_cg_y.blockSignals(False)
        self.scene.rebuild()

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

    def on_wheel_size_change(self):
        """Update wheel rectangle width/height and redraw.
            Also *derive* geometric wheel radius from editor width (radius = width/2)."""
        self.model.wheelWidthMM = float(self.spin_wheel_w.value())
        self.model.wheelHeightMM = float(self.spin_wheel_h.value())
        try:
            self.model.wheelRadiusMM = float(self.model.wheelWidthMM) / 2.0
        except Exception:
            pass
        self.scene.rebuild()

    def on_wheel_change(self):
        """Save wheel positions from the spin boxes and redraw the scene.
            Track (center-to-center Y spacing) is always derived from editor wheels."""
        wl = self.model.find_wheel("left")
        wr = self.model.find_wheel("right")
        if wl:
            wl.xMM = self.spin_wl_x.value()
            wl.yMM = self.spin_wl_y.value()
        if wr:
            wr.xMM = self.spin_wr_x.value()
            wr.yMM = self.spin_wr_y.value()
        if wl and wr:
            measured_track = abs(wl.yMM - wr.yMM)
            self.model.trackMM = measured_track
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
        self.model.remove_sensor(sid)
        self.refresh_sensors_list()
        count = self.list_sensors.count()
        if count > 0:
            last_row = count - 1
            self.list_sensors.setCurrentRow(last_row)
            new_sid = self.list_sensors.item(last_row).data(Qt.UserRole)
            self.scene.selected_kind, self.scene.selected_id = "sensor", new_sid
            self.on_select_sensor(last_row)
        else:
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
            Loads the new 'electrical', 'geometric_mechanical' and 'motor_transmission' sections"""
        path, _ = QFileDialog.getOpenFileName(self, "Import robot", "", "JSON (*.json)")
        if not path: return
        try:
            with open(path, "r", encoding="utf-8") as f:
                obj = json.load(f)
            self.model = RobotModel.from_json(obj)
            if not hasattr(self.model, "wheelWidthMM"):  self.model.wheelWidthMM = 22.0
            if not hasattr(self.model, "wheelHeightMM"): self.model.wheelHeightMM = 15.0

            elec = obj.get("electrical", {})
            self.model.batteryVoltageV   = float(elec.get("batteryVoltageV", getattr(self.model, "batteryVoltageV", 7.4)))
            self.model.batteryCapacitymAh= float(elec.get("batteryCapacitymAh", getattr(self.model, "batteryCapacitymAh", 850.0)))
            self.model.RbattOhm          = float(elec.get("R_batt_ohm", getattr(self.model, "RbattOhm", 0.0)))
            self.model.wiringROhm        = float(elec.get("wiring_R_ohm", getattr(self.model, "wiringROhm", 0.0)))
            self.model.driverDropV       = float(elec.get("driver_drop_V", getattr(self.model, "driverDropV", 0.0)))
            self.model.battRCTimeConstMs = float(elec.get("batt_RC_timeconst_ms", getattr(self.model, "battRCTimeConstMs", 0.0)))
            tbl = elec.get("battery_OCV_table", getattr(self.model, "batteryOCVTable", []))
            if isinstance(tbl, list):
                self.model.batteryOCVTable = tbl

            gm = obj.get("geometric_mechanical", {})
            if "body_length_mm" in gm and "body_width_mm" in gm:
                self.model.envelope = Envelope(widthMM=float(gm.get("body_length_mm", self.model.envelope.widthMM)),
                                               heightMM=float(gm.get("body_width_mm", self.model.envelope.heightMM)))
            self.model.wheelRadiusMM = float(gm.get("wheel_radius_mm", getattr(self.model, "wheelRadiusMM", 11.0)))
            self.model.trackMM = float(gm.get("track_mm", getattr(self.model, "trackMM", 0.0)))
            self.model.wheelbaseMM = float(gm.get("wheelbase_mm", getattr(self.model, "wheelbaseMM", 0.0)))
            self.model.wheelbaseOffsetMM = float(gm.get("wheelbase_offset_mm", getattr(self.model, "wheelbaseOffsetMM", 0.0)))
            self.model.massKG = float(gm.get("mass_kg", getattr(self.model, "massKG", 0.20)))
            self.model.JbodyKGm2 = float(gm.get("J_body_kgm2", getattr(self.model, "JbodyKGm2", 0.0)))
            self.model.muStatic = float(gm.get("mu_static", getattr(self.model, "muStatic", 1.0)))
            self.model.muKinetic = float(gm.get("mu_kinetic", getattr(self.model, "muKinetic", 0.8)))
            self.model.Crr = float(gm.get("Crr", getattr(self.model, "Crr", 0.005)))
            self.model.JwheelKGm2 = float(gm.get("J_wheel_kgm2", getattr(self.model, "JwheelKGm2", 0.0)))
            self.model.RwheelEffMM = float(gm.get("R_wheel_eff_mm", getattr(self.model, "RwheelEffMM", 0.0)))
            cg_pair = gm.get("cg_origin_xy_mm", None)
            rot_pair = gm.get("rot_origin_xy_mm", None)
            if isinstance(cg_pair, list) and len(cg_pair) == 2:
                self.model.cgXMM, self.model.cgYMM = float(cg_pair[0]), float(cg_pair[1])
            if isinstance(rot_pair, list) and len(rot_pair) == 2:
                self.model.rotXMM, self.model.rotYMM = float(rot_pair[0]), float(rot_pair[1])
            if not hasattr(self.model, "rotXMM") or not hasattr(self.model, "rotYMM"):
                legacy_origin = obj.get("origin", {})
                if isinstance(legacy_origin, dict) and "xMM" in legacy_origin and "yMM" in legacy_origin:
                    self.model.rotXMM = float(legacy_origin.get("xMM", 0.0))
                    self.model.rotYMM = float(legacy_origin.get("yMM", 0.0))
                else:
                    self.model.rotXMM = float(getattr(self.model, "rotXMM", 0.0) or 0.0)
                    self.model.rotYMM = float(getattr(self.model, "rotYMM", 0.0) or 0.0)
            if not hasattr(self.model, "cgXMM") or not hasattr(self.model, "cgYMM"):
                try:
                    wl = self.model.find_wheel("left"); wr = self.model.find_wheel("right")
                    if wl and wr:
                        self.model.cgXMM = float((wl.xMM + wr.xMM)/2.0)
                        self.model.cgYMM = float((wl.yMM + wr.yMM)/2.0)
                    else:
                        self.model.cgXMM = float(getattr(self.model, "rotXMM", 0.0))
                        self.model.cgYMM = float(getattr(self.model, "rotYMM", 0.0))
                except Exception:
                    self.model.cgXMM = float(getattr(self.model, "rotXMM", 0.0))
                    self.model.cgYMM = float(getattr(self.model, "rotYMM", 0.0))

            mt = obj.get("motor_transmission", {})
            self.model.gearRatio = float(mt.get("gear_ratio", getattr(self.model, "gearRatio", 1.0)))
            self.model.transmissionEfficiency = float(mt.get("eta", getattr(self.model, "transmissionEfficiency", 1.0)))
            self.model.RmotorOhm = float(mt.get("R_motor_ohm", getattr(self.model, "RmotorOhm", 0.0)))
            self.model.LmotorH = float(mt.get("L_motor_H", getattr(self.model, "LmotorH", 0.0)))
            Kv = float(mt.get("Kv_rpm_per_V", getattr(self.model, "KvRPMperV", 0.0)))
            Kt = float(mt.get("Kt_Nm_per_A", getattr(self.model, "KtNmPerA", 0.0)))
            if Kt > 0.0:
                self.model.KtNmPerA = Kt
                self.model.KvRPMperV = float(0.0 if Kt == 0 else 60.0/(2*math.pi*Kt))
            else:
                self.model.KvRPMperV = Kv
                self.model.KtNmPerA = float(0.0 if Kv == 0 else 60.0/(2*math.pi*Kv))
            self.model.KvRadPerV = float(self.model.KvRPMperV * 2*math.pi/60.0) if self.model.KvRPMperV != 0 else 0.0
            self.model.I0NoLoadA = float(mt.get("I0_noLoad_A", getattr(self.model, "I0NoLoadA", 0.0)))
            self.model.bViscNmPerRadps = float(mt.get("b_visc_Nm_per_radps", getattr(self.model, "bViscNmPerRadps", 0.0)))
            self.model.tauCoulombNm = float(mt.get("tau_coulomb_Nm", getattr(self.model, "tauCoulombNm", 0.0)))
            self.model.JmotorKGm2 = float(mt.get("J_motor_kgm2", getattr(self.model, "JmotorKGm2", 0.0)))
            self.model.JloadKGm2  = float(mt.get("J_load_kgm2", getattr(self.model, "JloadKGm2", 0.0)))
            self.model.JtotalKGm2 = float(self.model.JmotorKGm2 + self.model.JloadKGm2)
            self.model.stallCurrentA = float(mt.get("stallCurrent_A", getattr(self.model, "stallCurrentA", 0.0)))
            self.model.driverCurrentLimitA = float(mt.get("driver_current_limit_A", getattr(self.model, "driverCurrentLimitA", 0.0)))

            self.scene.model = self.model
            self.refresh_all()
            self.scene.selected_kind, self.scene.selected_id = None, None
            self.scene.rebuild()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to import:\\n{e}")

    def on_export(self):
        """Save the current robot definition to a JSON file.

        Exports a *lean* robot spec including only the essentials requested:
        - electrical: OCV/ESR/driver/wiring (+ optional RC pole, OCV table, metadata C-rate)
        - geometric_mechanical: wheel radius, track, wheelbase(+offset), mass, J_body, mu_static/kinetic, Crr (+optional J_wheel, R_wheel_eff)
        - motor_transmission: gear_ratio, eta, R_motor, L_motor, one of Kv/Kt (we export both coherent), I0, b_visc, tau_coulomb, inertias, stall/drv limits
        - controller, sensors, encoders, IMU, odometry as before.
        Visual-only items (wheel rectangles) are NOT exported under geometric_mechanical.
        """
        path, _ = QFileDialog.getSaveFileName(self, "Export robot", "robot-spec.json", "JSON (*.json)")
        if not path: return
        try:
            data = self.model.to_json()

            wl = self.model.find_wheel("left")
            wr = self.model.find_wheel("right")
            measured_track = abs(wl.yMM - wr.yMM) if (wl and wr) else 0.0
            track_mm = float(measured_track)

            data["geometric_mechanical"] = {
                "body_length_mm":     float(self.model.envelope.widthMM),
                "body_width_mm":      float(self.model.envelope.heightMM),
                "wheel_radius_mm":     float(getattr(self.model, "wheelWidthMM", 22.0)) / 2.0,
                "track_mm":            track_mm,
                "wheelbase_mm":        abs(float(getattr(self.model, 'cgXMM', 0.0)) - float(getattr(self.model, 'rotXMM', 0.0))),
                "wheelbase_offset_mm": float(getattr(self.model, 'cgYMM', 0.0)) - float(getattr(self.model, 'rotYMM', 0.0)),
                "mass_kg":             float(getattr(self.model, "massKG", 0.20)),
                "J_body_kgm2":         float(getattr(self.model, "JbodyKGm2", 0.0)),
                "mu_static":           float(getattr(self.model, "muStatic", 1.0)),
                "mu_kinetic":          float(getattr(self.model, "muKinetic", 0.8)),
                "Crr":                 float(getattr(self.model, "Crr", 0.005)),
                "cg_origin_xy_mm":     [float(getattr(self.model,'cgXMM',0.0)), float(getattr(self.model,'cgYMM',0.0))],
                "rot_origin_xy_mm":    [float(getattr(self.model,'rotXMM',0.0)), float(getattr(self.model,'rotYMM',0.0))],
            }
            if float(getattr(self.model, "JwheelKGm2", 0.0)) > 0.0:
                data["geometric_mechanical"]["J_wheel_kgm2"] = float(self.model.JwheelKGm2)
            if float(getattr(self.model, "RwheelEffMM", 0.0)) > 0.0:
                data["geometric_mechanical"]["R_wheel_eff_mm"] = float(self.model.RwheelEffMM)
            if float(getattr(self.model, "JwheelKGm2", 0.0)) > 0.0:
                data["geometric_mechanical"]["J_wheel_kgm2"] = float(self.model.JwheelKGm2)
            if float(getattr(self.model, "RwheelEffMM", 0.0)) > 0.0:
                data["geometric_mechanical"]["R_wheel_eff_mm"] = float(self.model.RwheelEffMM)

            data["electrical"] = {
                "batteryVoltageV":    float(getattr(self.model, "batteryVoltageV", 7.4)),   # OCV(IC)
                "batteryCapacitymAh": float(getattr(self.model, "batteryCapacitymAh", 850.0)),
                "R_batt_ohm":         float(getattr(self.model, "RbattOhm", 0.0)),
                "wiring_R_ohm":       float(getattr(self.model, "wiringROhm", 0.0)),
                "driver_drop_V":      float(getattr(self.model, "driverDropV", 0.0)),
            }
            if float(getattr(self.model, "battRCTimeConstMs", 0.0)) > 0.0:
                data["electrical"]["batt_RC_timeconst_ms"] = float(self.model.battRCTimeConstMs)
            tbl = getattr(self.model, "batteryOCVTable", None)
            if isinstance(tbl, list) and len(tbl) > 0:
                data["electrical"]["battery_OCV_table"] = tbl

            Kv = float(getattr(self.model, "KvRPMperV", 0.0))
            Kt = float(getattr(self.model, "KtNmPerA", 0.0))
            if Kt > 0.0:
                Kv_coh = float(0.0 if Kt == 0 else 60.0/(2*math.pi*Kt))
                Kt_coh = Kt
            else:
                Kv_coh = Kv
                Kt_coh = float(0.0 if Kv == 0 else 60.0/(2*math.pi*Kv))

            data["motor_transmission"] = {
                "gear_ratio":          float(getattr(self.model, "gearRatio", 1.0)),
                "eta":                 float(getattr(self.model, "transmissionEfficiency", 1.0)),
                "R_motor_ohm":         float(getattr(self.model, "RmotorOhm", 0.0)),
                "L_motor_H":           float(getattr(self.model, "LmotorH", 0.0)),
                "Kv_rpm_per_V":        Kv_coh,
                "Kv_rad_per_V":        float(Kv_coh * 2*math.pi/60.0) if Kv_coh != 0 else 0.0,
                "Kt_Nm_per_A":         Kt_coh,
                "I0_noLoad_A":         float(getattr(self.model, "I0NoLoadA", 0.0)),
                "b_visc_Nm_per_radps": float(getattr(self.model, "bViscNmPerRadps", 0.0)),
                "tau_coulomb_Nm":      float(getattr(self.model, "tauCoulombNm", 0.0)),
                "J_motor_kgm2":        float(getattr(self.model, "JmotorKGm2", 0.0)),
                "J_load_kgm2":         float(getattr(self.model, "JloadKGm2", 0.0)),
                "stallCurrent_A":      float(getattr(self.model, "stallCurrentA", 0.0)),
                "driver_current_limit_A": float(getattr(self.model, "driverCurrentLimitA", 0.0)),
            }

            # Controller (unchanged)
            data["controller"] = {
                "pwm_resolution_bits": int(getattr(self.model, "controllerPwmResolutionBits", 12)),
                "pwm_frequency_Hz":    float(getattr(self.model, "controllerPwmFrequencyHz", 20000.0)),
                "deadband_percent":    float(getattr(self.model, "controllerDeadbandPercent", 0.0)),
                "pwm_max":             int(getattr(self.model, "motorPwmMax", 4095)),
                "pwm_min":             int(getattr(self.model, "motorPwmMin", -4095)),
                "simulation_step_dt_ms": float(getattr(self.model, "simulationStepDtMs", 1.0)),
            }

            sensors_cfg = {}
            sensors_cfg["sensor_mode"] = str(getattr(self.model, "sensorMode", "analog"))
            sensors_cfg["sensor_bits"] = int(getattr(self.model, "sensorBits", 8))
            sensors_cfg["value_of_line"] = int(getattr(self.model, "valueOfLine", 0))
            sensors_cfg["value_of_background"] = int(getattr(self.model, "valueOfBackground", 255))
            sensors_cfg["analog_noise_line"] = int(getattr(self.model, "analogNoiseLine", 50))
            sensors_cfg["analog_noise_background"] = int(getattr(self.model, "analogNoiseBackground", 50))
            data["sensorsConfig"] = sensors_cfg

            data["encoders"] = {
                "enable":           bool(getattr(self.model, "encoderEnable", False)),
                "type":             str(getattr(self.model, "encoderType", "incremental")).lower(),
                "ppr":              int(getattr(self.model, "encoderPPR", 0)),
                "resolution_bits":  int(getattr(self.model, "encoderResolutionBits", 0)),
                "noise_std_pulses": float(getattr(self.model, "encoderNoiseStdPulses", 0.0)),
                "update_rate_Hz":   float(getattr(self.model, "encoderUpdateRateHz", 0.0)),
            }

            data["imu"] = {
                "enable": bool(getattr(self.model, "imuEnable", False)),
                "std_deg": float(getattr(self.model, "imuStdDeg", 0.0)),
                "bias_deg_s": float(getattr(self.model, "imuBiasDegPerS", 0.0)),
                "update_rate_Hz": float(getattr(self.model, "imuUpdateRateHz", 0.0)),
                "latency_ms": float(getattr(self.model, "imuLatencyMs", 0.0)),
            }

            data["odometry"] = {
                "enable": bool(getattr(self.model, "odomEnable", False)),
                "noise_std_mm": float(getattr(self.model, "odomNoiseStdMM", 0.0)),
                "noise_std_deg": float(getattr(self.model, "odomNoiseStdDeg", 0.0)),
                "bias_mm": float(getattr(self.model, "odomBiasMM", 0.0)),
                "bias_deg": float(getattr(self.model, "odomBiasDeg", 0.0)),
                "update_rate_Hz": float(getattr(self.model, "odomUpdateRateHz", 0.0)),
            }

            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to export:\\n{e}")

    def on_open_electrical(self):
        """Open the parameter dialog and apply changes if accepted."""
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

        self.spin_rot_x.blockSignals(True); self.spin_rot_y.blockSignals(True)
        self.spin_rot_x.setValue(self.model.rotXMM)
        self.spin_rot_y.setValue(self.model.rotYMM)
        self.spin_rot_x.blockSignals(False); self.spin_rot_y.blockSignals(False)
        self.spin_cg_x.blockSignals(True); self.spin_cg_y.blockSignals(True)
        self.spin_cg_x.setValue(self.model.cgXMM)
        self.spin_cg_y.setValue(self.model.cgYMM)
        self.spin_cg_x.blockSignals(False); self.spin_cg_y.blockSignals(False)

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
