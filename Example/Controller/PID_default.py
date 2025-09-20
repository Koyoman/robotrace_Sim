# PID_default.py — PID basic with line-loss recovery (last-side memory)
#
# This is a minimal variation of the PID_basic controller:
# - Same noise cleaning (hysteresis TH_LO/TH_HI) and 1000-scaled symmetric weights.
# - Classic PID: diff = Kp*err + Ki*∫err + Kd*d(err)/dt (with simple exponential filtering on D).
# - Small extra behaviors only:
#     * Keeps the last side the line was on (left/right).
#     * If all sensors lose the line, sets err = ±FALLBACK_ERR based on last_side
#       so the robot turns toward the last seen direction to reacquire the track.
#
# With KI=0 and a modest KD, it behaves like your PID basic, only adding the
# 'find-the-line' behavior when the tape is lost.

BASE_PWM = 3000

# Gains (aligned with your PID basic style; tune as needed)
KP = 0.150
KI = 0.0
KD = 0.015

# Hysteresis thresholds (dark tape = lower reading)
TH_LO = 110.0
TH_HI = 170.0

# Derivative timing/filter
DT_S = 0.001
D_ALPHA = 0.30   # derivative low-pass (0..1), modest smoothing

# Fallback magnitude when line is lost
FALLBACK_ERR = 6000.0  # large enough to command a strong turn

# PWM rails
PWM_MIN, PWM_MAX = -4095, 4095

# Controller state
G = {
    "err_sum": 0.0,
    "prev_err": 0.0,
    "d_filt": 0.0,
    "last_side": 0   # -1 = left, +1 = right, 0 = unknown
}

def _clamp_pwm(v: float) -> int:
    """Clamp PWM to valid range."""
    if v < PWM_MIN: return PWM_MIN
    if v > PWM_MAX: return PWM_MAX
    return int(v)

def _activation(val: float) -> float:
    """Map raw [0..255] to activation [0..1] with hysteresis (1 = on dark tape)."""
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
    vals = list(state.get("sensors", {}).get("values", []))
    n = len(vals)
    if n == 0:
        # No sensors? just go forward
        return {"pwm_left": BASE_PWM, "pwm_right": BASE_PWM}

    # Hysteresis-based activations (noise cleaning)
    acts = [_activation(v) for v in vals]

    # Weighted centroid error
    W = _weights_signed_even(n, step=1000.0)
    sw = sum(acts)

    if sw <= 1e-9:
        # Line lost → turn toward last seen side
        side = G["last_side"] if G["last_side"] != 0 else 1
        err = FALLBACK_ERR * side
    else:
        center = sum(w * a for w, a in zip(W, acts)) / sw
        err = center
        # Update last_side if we have a sign
        if err < 0:   G["last_side"] = -1
        elif err > 0: G["last_side"] = +1

    # --- PID terms (close to PID_basic) ---
    P = KP * err

    # Integral (kept off by default; anti-windup kept small due to 1000-scaled units)
    G["err_sum"] += err
    if G["err_sum"] > 1000.0:  G["err_sum"] = 1000.0
    if G["err_sum"] < -1000.0: G["err_sum"] = -1000.0
    I = KI * G["err_sum"]

    # Derivative with dt and light smoothing
    d_raw = (err - G["prev_err"]) / DT_S
    G["d_filt"] = (1.0 - D_ALPHA) * G["d_filt"] + D_ALPHA * d_raw
    D = KD * G["d_filt"]
    G["prev_err"] = err

    diff = P + I + D

    # Same mixing pattern as PID_basic: base ± diff
    pwm_left  = _clamp_pwm(BASE_PWM - diff)
    pwm_right = _clamp_pwm(BASE_PWM + diff)
    return {"pwm_left": pwm_left, "pwm_right": pwm_right}
