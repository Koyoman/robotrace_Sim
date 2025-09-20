# PID_basic_untuned.py — PID that matches P_basic when KI=KD=0
#
# Behavior:
# - Same noise cleaning (hysteresis), weighting, and mixing as P_basic.
# - diff = Kp*err + Ki*∫err + Kd*d(err)/dt
# - Clamp on 'diff' identical to P_basic's MAX_DIFF, no slew-rate, no deadband.
# - With KI=0 and KD=0, this is effectively the same as P_basic.
#
# You only need to tweak KI and KD to explore integral/derivative effects.

BASE_PWM = 2048
KP = 0.07         # same Kp as P_basic
KI = 0.0           # you may tune
KD = 0.010           # you may tune

PWM_MIN, PWM_MAX = -4095, 4095

# Hysteresis thresholds (dark tape = lower reading)
TH_LO = 110.0
TH_HI = 170.0

# Derivative timing/filter (has effect only if KD > 0)
DT_S = 0.001
D_ALPHA = 0.30      # exponential smoothing for derivative

# PID state
G = {"err_sum": 0.0, "prev_err": 0.0, "d_filt": 0.0}

def _clamp_pwm(v: float) -> int:
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
    vals = list(state.get("sensors", {}).get("values", []))
    n = len(vals)
    if n == 0:
        return {"pwm_left": BASE_PWM, "pwm_right": BASE_PWM}

    # Hysteresis-based activations (same noise cleaning as P_basic)
    acts = [_activation(v) for v in vals]

    # Weighted centroid error
    W = _weights_signed_even(n, step=1000.0)
    sw = sum(acts)
    if sw <= 1e-9:
        err = 0.0
    else:
        err = sum(w * a for w, a in zip(W, acts)) / sw

    # --- PID terms ---
    P = KP * err

    # Integral (only effective if KI > 0)
    G["err_sum"] += err
    # tight anti-windup since units of 'err' are large (×1000 weights)
    if G["err_sum"] > 1000.0:  G["err_sum"] = 1000.0
    if G["err_sum"] < -1000.0: G["err_sum"] = -1000.0
    I = KI * G["err_sum"]

    # Derivative with dt and exponential smoothing (only effective if KD > 0)
    d_raw = (err - G["prev_err"]) / DT_S
    G["d_filt"] = (1.0 - D_ALPHA) * G["d_filt"] + D_ALPHA * d_raw
    D = KD * G["d_filt"]
    G["prev_err"] = err

    # Same mixing as P_basic
    diff = P + I + D

    pwm_left  = _clamp_pwm(BASE_PWM - diff)
    pwm_right = _clamp_pwm(BASE_PWM + diff)
    return {"pwm_left": pwm_left, "pwm_right": pwm_right}
