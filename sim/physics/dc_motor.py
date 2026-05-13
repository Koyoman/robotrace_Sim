from __future__ import annotations

import math
from ctypes import byref, c_double, c_int
from typing import Any

from Utils.robot_spec import RobotSpec
from Utils.simulation_config import SimulationConfig, derive_runtime_params
from Utils.simulation_state import SimulationState

from sim.physics.base import PhysicsModel


class DCMotorPhysicsModel(PhysicsModel):
    """Wrapper around the existing native C DC drivetrain model."""

    requires_native = True

    def __init__(self, robot: RobotSpec | None = None, params: dict[str, Any] | None = None):
        self.params = dict(params or {})
        self._phys: dict[str, float] = {}
        self._v_mps = 0.0
        self._w_radps = 0.0
        self._i_left_A = 0.0
        self._i_right_A = 0.0
        if robot is not None:
            self.configure(robot, self.params)

    def configure(self, robot: RobotSpec, params: dict[str, Any] | None = None) -> None:
        if params is not None:
            self.params = dict(params)
        elif not self.params:
            self.params = derive_runtime_params(robot)

        gm = robot.geometric_mechanical
        p = self.params
        self._phys = {
            "Vb": float(p.get("V_batt_nom_V", 7.4)),
            "Rb": float(p.get("R_batt_ohm", 0.05)),
            "Rw": float(p.get("R_wiring_ohm", 0.02)),
            "Vdrop": float(p.get("driver_drop_V", 0.2)),
            "Rm": float(p.get("Rm_ohm", 3.0)),
            "Lm": float(p.get("Lm_H", 0.0001)),
            "Kt": float(p.get("Kt_Nm_per_A", 0.0015)),
            "Ke": float(p.get("Ke_V_per_rad", 1.0 / 500.0)),
            "gear": float(p.get("gear_ratio", 1.0)),
            "eta": float(p.get("eta_drive", 0.9)),
            "Jm": float(p.get("Jm_kgm2", 1e-8)),
            "Jload": float(p.get("Jload_kgm2", 0.0)),
            "b": float(p.get("b_visc_Nm_per_radps", 0.0)),
            "tau_c": float(p.get("tau_coulomb_Nm", 0.0)),
            "Imax": float(p.get("I_max_A", 3.0)),
            "mass": float(p.get("mass_kg", 0.2)),
            "track": float(p.get("track_m", 0.07)),
            "r": float(p.get("wheel_r_m", 0.011)),
            "Jz": float(p.get("Jz_kgm2", 1e-4)),
            "Crr": float(p.get("Crr", 0.005)),
            "rho": float(p.get("rho_air", 1.225)),
            "CdA": float(p.get("CdA", 0.02)),
            "mu_static": float(gm.mu_static),
            "mu_kinetic": float(gm.mu_kinetic),
            "pwm_max": float(p.get("pwm_max", 4095)),
            "pwm_min": float(p.get("pwm_min", -4095)),
            "deadband_percent": float(p.get("deadband_percent", 0.0)),
        }

    def reset(self, initial_state: SimulationState) -> None:
        self._v_mps = float(initial_state.v_mm_s) / 1000.0
        self._w_radps = float(initial_state.omega_rad_s)
        self._i_left_A = 0.0
        self._i_right_A = 0.0

    def step(
        self,
        state: SimulationState,
        pwm_left: int,
        pwm_right: int,
        dt_s: float,
        robot: RobotSpec,
        config: SimulationConfig,
        native: Any | None = None,
    ) -> SimulationState:
        if native is None:
            raise RuntimeError("O perfil realistic requer o backend nativo linesim.dll.")
        if not self._phys:
            self.configure(robot, self.params or derive_runtime_params(robot, config))

        phys = self._phys
        phys["pwm_min"] = float(phys.get("pwm_min", -4095.0))
        phys["pwm_max"] = float(phys.get("pwm_max", 4095.0))
        neutral_raw = self.params.get("pwm_neutral", None)
        if neutral_raw is None:
            pwm_center = 0.5 * (phys["pwm_min"] + phys["pwm_max"])
        else:
            pwm_center = float(neutral_raw)
        deadband = float(phys.get("deadband_percent", 0.0)) * 0.01

        ox = c_double(); oy = c_double(); oh = c_double()
        ov = c_double(); ow = c_double(); oil = c_double(); oir = c_double()

        native.step_motor_drivetrain_C(
            c_double(state.x_mm / 1000.0), c_double(state.y_mm / 1000.0), c_double(math.radians(state.heading_deg)),
            c_double(self._v_mps), c_double(self._w_radps), c_double(self._i_left_A), c_double(self._i_right_A),
            c_int(pwm_left), c_int(pwm_right),
            c_double(phys["pwm_min"]), c_double(phys["pwm_max"]), c_double(pwm_center), c_double(deadband),
            c_double(phys["Vb"]), c_double(phys["Rb"]), c_double(phys["Rw"]), c_double(phys["Vdrop"]),
            c_double(phys["Rm"]), c_double(phys["Lm"]), c_double(phys["Kt"]), c_double(phys["Ke"]),
            c_double(phys.get("b", 0.0)), c_double(phys.get("tau_c", 0.0)),
            c_double(phys["gear"]), c_double(phys["eta"]),
            c_double(phys["mass"]), c_double(phys["track"]), c_double(phys["r"]), c_double(phys["Jz"]),
            c_double(phys["Crr"]), c_double(phys["rho"]), c_double(phys["CdA"]),
            c_double(phys.get("mu_static", 1.0)), c_double(phys.get("mu_kinetic", 0.8)),
            c_double(phys["Imax"]),
            c_double(dt_s),
            byref(ox), byref(oy), byref(oh), byref(ov), byref(ow), byref(oil), byref(oir),
        )

        self._v_mps = ov.value
        self._w_radps = ow.value
        self._i_left_A = oil.value
        self._i_right_A = oir.value

        v_left_mps = self._v_mps - 0.5 * self._w_radps * phys["track"]
        v_right_mps = self._v_mps + 0.5 * self._w_radps * phys["track"]
        v_mm_s = self._v_mps * 1000.0
        omega = self._w_radps
        dt = max(1e-9, float(dt_s))

        return SimulationState(
            t_ms=state.t_ms,
            x_mm=ox.value * 1000.0,
            y_mm=oy.value * 1000.0,
            heading_deg=math.degrees(oh.value),
            v_mm_s=v_mm_s,
            omega_rad_s=omega,
            a_lin_mm_s2=(v_mm_s - state.v_mm_s) / dt,
            alpha_rad_s2=(omega - state.omega_rad_s) / dt,
            v_left_mm_s=v_left_mps * 1000.0,
            v_right_mm_s=v_right_mps * 1000.0,
            pwm_left=int(pwm_left),
            pwm_right=int(pwm_right),
            sensors=list(state.sensors),
        )
