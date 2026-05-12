from __future__ import annotations

from collections.abc import Callable

from PySide6.QtCore import QThread, Signal

from Utils.robot_spec import RobotSpec
from Utils.simulation_config import SimulationConfig
from Utils.track_spec import TrackSpec
from sim.engine import SimulationEngine


class SimWorker(QThread):
    """Thin QThread adapter around SimulationEngine."""

    sig_chunk = Signal(list)
    sig_done = Signal(dict)
    sig_fail = Signal(str)

    def __init__(
        self,
        track: TrackSpec,
        robot: RobotSpec,
        controller_fn: Callable[[dict], dict],
        config: SimulationConfig,
        save_logs: bool,
        parent=None,
        track_path: str | None = None,
    ):
        super().__init__(parent)
        self.cancelled = False
        self.engine = SimulationEngine(
            track=track,
            robot=robot,
            config=config,
            controller_fn=controller_fn,
            track_path=track_path,
            save_logs=save_logs,
        )

    @property
    def dt_s(self) -> float:
        return self.engine.dt_s

    def cancel(self) -> None:
        self.cancelled = True
        self.engine.cancel()

    def run(self) -> None:
        try:
            for chunk in self.engine.iter_chunks():
                if self.cancelled:
                    self.engine.cancel()
                    break
                self.sig_chunk.emit(chunk)
            self.sig_done.emit({"dt_s": self.engine.dt_s})
        except Exception as exc:
            try:
                self.sig_fail.emit(str(exc))
            except Exception:
                pass
