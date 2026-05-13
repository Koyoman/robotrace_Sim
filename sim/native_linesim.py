from __future__ import annotations

import ctypes
import os
import sys
from ctypes import POINTER, c_double, c_int


class CPoint(ctypes.Structure):
    """ctypes struct matching the C backend Pt type."""

    _fields_ = [("x", c_double), ("y", c_double)]


_linesim = None
_load_error: Exception | None = None


def _candidate_library_paths() -> list[str]:
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    lib_dir = os.path.join(root, "utills_c")
    names = ["linesim.dll"] if sys.platform.startswith("win") else ["liblinesim.so", "linesim.so", "linesim.dll"]
    return [os.path.join(lib_dir, name) for name in names]


def _dll_path() -> str:
    # Backward-compatible helper kept for callers/tests that expect the old name.
    return _candidate_library_paths()[0]


def get_linesim():
    """Load and configure the native backend lazily."""
    global _linesim, _load_error
    if _linesim is not None:
        return _linesim
    if _load_error is not None:
        raise RuntimeError(f"Não foi possível carregar o backend nativo linesim: {_load_error}")

    errors: list[Exception] = []
    try:
        if sys.platform.startswith("win") and hasattr(os, "add_dll_directory"):
            os.add_dll_directory(os.path.dirname(_dll_path()))
        lib = None
        for path in _candidate_library_paths():
            if not os.path.exists(path):
                continue
            try:
                lib = ctypes.CDLL(path)
                break
            except Exception as exc:
                errors.append(exc)
        if lib is None:
            detail = "; ".join(str(e) for e in errors) or "biblioteca nativa não encontrada"
            raise RuntimeError(detail)

        lib.envelope_contacts_tape_C.argtypes = [
            c_double, c_double, c_double,
            c_double, c_double,
            POINTER(CPoint), c_int,
            c_double, c_int,
        ]
        lib.envelope_contacts_tape_C.restype = c_int

        lib.estimate_sensor_coverage_C.argtypes = [
            c_double, c_double,
            POINTER(CPoint), c_int,
            c_double, c_double, c_int,
        ]
        lib.estimate_sensor_coverage_C.restype = c_double

        lib.estimate_sensors_coverage_batch_C = getattr(lib, "estimate_sensors_coverage_batch_C")
        lib.estimate_sensors_coverage_batch_C.argtypes = [
            POINTER(c_double), POINTER(c_double), c_int,
            POINTER(CPoint), c_int,
            c_double,
            POINTER(c_double), c_double,
            c_int,
            POINTER(c_double),
        ]
        lib.estimate_sensors_coverage_batch_C.restype = None

        lib.crossed_finish_C.argtypes = [
            c_double, c_double, c_double, c_double,
            c_double, c_double, c_double, c_double,
        ]
        lib.crossed_finish_C.restype = c_int

        lib.step_motor_drivetrain_C = getattr(lib, "step_motor_drivetrain_C")
        lib.step_motor_drivetrain_C.argtypes = [
            c_double, c_double, c_double,
            c_double, c_double, c_double, c_double,
            c_int, c_int,
            c_double, c_double, c_double, c_double,
            c_double, c_double, c_double, c_double,
            c_double, c_double, c_double, c_double,
            c_double, c_double,
            c_double, c_double,
            c_double, c_double, c_double, c_double,
            c_double, c_double, c_double,
            c_double, c_double,
            c_double,
            c_double,
            POINTER(c_double), POINTER(c_double), POINTER(c_double),
            POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_double),
        ]
        lib.step_motor_drivetrain_C.restype = None

        try:
            lib.envelope_contacts_raster_C.argtypes = [
                c_double, c_double, c_double,
                c_double, c_double,
                ctypes.POINTER(ctypes.c_ubyte), c_int, c_int,
                c_double, c_double, c_double,
            ]
            lib.envelope_contacts_raster_C.restype = c_int
        except Exception:
            pass

        _linesim = lib
        return lib
    except Exception as exc:  # Native backend may be unavailable during CI; defer as runtime error.
        _load_error = exc
        raise RuntimeError(f"Não foi possível carregar o backend nativo linesim: {exc}") from exc
