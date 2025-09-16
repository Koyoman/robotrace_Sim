from __future__ import annotations
from dataclasses import dataclass
from typing import Tuple

# Conversões e utilidades geométricas

@dataclass
class Pt:
    x: float
    y: float

@dataclass
class Envelope:
    widthMM: float
    heightMM: float

# Regras oficiais:
MAX_W = 250.0  # mm
MAX_H = 250.0  # mm
# (altura 200 mm é regra física; aqui o editor trata footprint 2D)

# Dimensões padrão
DEFAULT_WHEEL_W = 22.0
DEFAULT_WHEEL_H = 15.0
DEFAULT_SENSOR_SIZE = 5.0

def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))

def clamp_inside_envelope(x: float, y: float, halfW: float, halfH: float,
                          objHalfW: float, objHalfH: float) -> Tuple[float, float]:
    """
    Garante que um retângulo centrado (x,y) com meia-largura/altura objHalfW/H
    permaneça 100% dentro do envelope de meia-largura/altura halfW/H.
    """
    x_min = -halfW + objHalfW
    x_max =  halfW - objHalfW
    y_min = -halfH + objHalfH
    y_max =  halfH - objHalfH
    return (clamp(x, x_min, x_max), clamp(y, y_min, y_max))

def enforce_envelope_rules(widthMM: float, heightMM: float) -> Tuple[float, float]:
    """
    Não deixa ultrapassar 250 x 250 (regras oficiais de footprint).
    """
    return (min(MAX_W, max(10.0, widthMM)), min(MAX_H, max(10.0, heightMM)))
