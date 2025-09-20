# controller_p_weighted.py — Proportional controller with hysteresis (marker-safe)

BASE_PWM = 2048
KP = 400.0
PWM_MIN, PWM_MAX = -4095, 4095

# Hysteresis thresholds (dark tape = lower reading)
TH_LO = 110.0   # <= on tape
TH_HI = 170.0   # >= off tape

# Limit steering to avoid huge kicks when crossing side markers
MAX_DIFF = 1500.0  # keep P-only but capped

def _clamp(v: float) -> int:
    """Clamp PWM to valid range."""
    if v < PWM_MIN: return PWM_MIN
    if v > PWM_MAX: return PWM_MAX
    return int(v)

def _activation(val: float) -> float:
    """Map raw [0..255] to activation [0..1] with hysteresis (1 = on dark line)."""
    if val <= TH_LO: return 1.0
    if val >= TH_HI: return 0.0
    return 1.0 - (val - TH_LO) / (TH_HI - TH_LO)

def _weights_signed_even(n: int, step: float = 1000.0):
    """
    Symmetric signed weights. For even n, omit zero to avoid center deadband:
      n=8 → [-4,-3,-2,-1,+1,+2,+3,+4] * step
    For odd n, include zero:
      n=7 → [-3,-2,-1,0,+1,+2,+3] * step
    """
    if n <= 0: return []
    half = n // 2
    if n % 2 == 0:
        out = []
        for i in range(n):
            k = i - half
            if i >= half: k += 1
            out.append(k * step)
        return out
    else:
        return [(i - half) * step for i in range(n)]

def control_step(state):
    """
    Minimal P controller with:
    - Hysteresis-based activations (noise & marker resilience).
    - Weighted centroid error (negative = line to the left, positive = right).
    - Proportional correction with a simple clamp.
    """
    vals = list(state.get("sensors", {}).get("values", []))
    n = len(vals)
    if n == 0:
        return {"pwm_left": BASE_PWM, "pwm_right": BASE_PWM}

    acts = [_activation(v) for v in vals]
    W = _weights_signed_even(n, step=1000.0)

    sw = sum(acts)
    if sw <= 1e-9:
        err = 0.0
    else:
        err = sum(w * a for w, a in zip(W, acts)) / sw

    # Pure P (with a safety clamp to avoid huge kicks on markers)
    diff = KP * err
    if diff >  MAX_DIFF: diff =  MAX_DIFF
    if diff < -MAX_DIFF: diff = -MAX_DIFF

    pwm_left  = _clamp(BASE_PWM - diff)
    pwm_right = _clamp(BASE_PWM + diff)
    return {"pwm_left": pwm_left, "pwm_right": pwm_right}
