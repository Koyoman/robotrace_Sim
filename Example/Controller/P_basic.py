"""PID_basic.py — Extremely simple line-following PID.

Steps:
  1) Compute the centroid of the sensor array (lateral error).
  2) Apply raw PID on that error (no normalization).
  3) Adjust wheel speeds around SPEED_PWM.
"""

# ---------------- Parameters ----------------
SPEED_PWM = 1000        # Base forward PWM

KP = 300.0              # Proportional gain (tune this first)
KI = 0.0                # Integral gain
KD = 0.0                # Derivative gain

WEIGHT_STEP = 1.0
PWM_MIN, PWM_MAX = -4095, 4095
DEFAULT_DT = 0.001
LOST_SPIN_PWM = 600

# ---------------- Internal state -------------
_integral = 0.0
_last_err = 0.0
_has_last = False

# ---------------- Helpers -------------------
def _clamp_pwm(v):
    if v < PWM_MIN: return PWM_MIN
    if v > PWM_MAX: return PWM_MAX
    return int(v)

def _weights_centered(n, step):
    """Return n sensor positions symmetric around 0."""
    if n <= 0: return []
    half = n // 2
    if n % 2 == 0:
        out = []
        for i in range(n):
            k = i - half
            if i >= half: k += 1
            out.append(k * step)
        return out
    return [(i - half) * step for i in range(n)]

def _centroid(values):
    """Weighted average of sensor values (error in sensor units)."""
    total = float(sum(values))
    if total <= 1e-12:
        return None
    w = _weights_centered(len(values), WEIGHT_STEP)
    return sum(a * x for a, x in zip(w, values)) / total

# ---------------- Main PID ------------------
def control_step(state):
    """Compute next PWM commands."""
    global _integral, _last_err, _has_last

    sensors = state.get("sensors", [])
    if not sensors:
        # No readings → go straight slowly
        return {"pwm_left": SPEED_PWM, "pwm_right": SPEED_PWM}

    dt = float(state.get("dt_s", DEFAULT_DT))
    if dt <= 0.0: dt = DEFAULT_DT

    err = _centroid(sensors)
    if err is None:
        # Line lost → keep moving forward gently
        return {"pwm_left": SPEED_PWM, "pwm_right": SPEED_PWM}

    # ----- PID -----
    p = KP * err

    # ----- Map to wheel speeds -----
    left = _clamp_pwm(SPEED_PWM - p)
    right = _clamp_pwm(SPEED_PWM + p)

    return {"pwm_left": left, "pwm_right": right}
