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


def _dll_path() -> str:
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    return os.path.join(root, "utills_c", "linesim.dll")


def get_linesim():
    """Load and configure the native backend lazily."""
    global _linesim, _load_error
    if _linesim is not None:
        return _linesim
    if _load_error is not None:
        raise RuntimeError(f"Não foi possível carregar linesim.dll: {_load_error}")

    dlldir = os.path.dirname(_dll_path())
    try:
        if sys.platform.startswith("win") and hasattr(os, "add_dll_directory"):
            os.add_dll_directory(dlldir)
        lib = ctypes.CDLL(_dll_path())

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
    except Exception as exc:  # DLL can be Windows-only during Linux CI; defer as runtime error.
        _load_error = exc
        raise RuntimeError(f"Não foi possível carregar linesim.dll: {exc}") from exc
