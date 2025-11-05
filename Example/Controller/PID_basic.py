"""PID_basic.py — Minimal PID controller example for a line-following robot.

This controller extends the basic proportional (P) idea by adding:
- I (integral): accumulates small persistent errors to remove steady offsets.
- D (derivative): reacts to how fast the error is changing to reduce overshoot.

It reads the sensor array, computes a centroid-like lateral error, and maps the
PID output to a left/right wheel PWM difference. The code is intentionally small
and heavily commented for beginners.
"""

# ===== Configuration constants (you can tweak these) =====

# Base PWM (motor power) when perfectly centered on the line.
BASE_PWM = 2048

# PID gains. Start small and increase gradually while testing.
KP = 0.35   # proportional gain
KI = 0.0002 # integral gain (small; integral accumulates over time)
KD = 0.001   # derivative gain

# Safe PWM range for your motors/driver.
PWM_MIN, PWM_MAX = -4095, 4095

# Sensor "position" spacing used to compute the centroid (arbitrary units).
WEIGHT_STEP = 1000.0

# Integral clamp to avoid "windup" (integral growing too much when saturated).
I_MIN, I_MAX = -500.0, 500.0

# Default time step (seconds) in case the simulator does not provide one.
DEFAULT_DT = 0.001


# ===== Internal persistent state (kept across calls) =====
_integral = 0.0
_last_err = 0.0
_has_last = False


def _clamp_pwm(v: float) -> int:
    """Clamp *v* to the allowed PWM range and return an int."""
    if v < PWM_MIN:
        return PWM_MIN
    if v > PWM_MAX:
        return PWM_MAX
    return int(v)


def _weights_signed_even(n: int, step: float = WEIGHT_STEP):
    """Return *n* lateral positions centered at zero.

    Works for odd or even number of sensors:
    - odd  (e.g., 5): [-2, -1, 0, +1, +2] * step
    - even (e.g., 4): [-1.5, -0.5, +0.5, +1.5] * step (zero is skipped for symmetry)
    """
    if n <= 0:
        return []

    half = n // 2
    if n % 2 == 0:
        out = []
        for i in range(n):
            k = i - half
            if i >= half:
                k += 1  # skip zero
            out.append(k * step)
        return out
    return [(i - half) * step for i in range(n)]


def _centroid_error(values):
    """Compute lateral error from sensor *values* using a weighted average.

    If all sensors read ~0 (line lost), returns 0.0 to keep the robot calm.
    """
    total = float(sum(values))
    if total <= 1e-12:
        return 0.0
    weights = _weights_signed_even(len(values), step=WEIGHT_STEP)
    return sum(w * a for w, a in zip(weights, values)) / total


def control_step(state):
    """Compute one control action from the simulator *state*.

    Expected keys in *state*:
        - "sensors": list[float] with one reading per sensor (required)
        - "dt_s":    simulation time step in seconds (optional; DEFAULT_DT if missing)

    Returns:
        dict: motor commands {"pwm_left": int, "pwm_right": int}
    """
    global _integral, _last_err, _has_last

    sensors = state.get("sensors", [])
    if not sensors:
        # Fallback: drive straight (helps during loading or if line is lost).
        return {"pwm_left": BASE_PWM, "pwm_right": BASE_PWM}

    dt = float(state.get("dt_s", DEFAULT_DT))
    if dt <= 0.0:
        dt = DEFAULT_DT

    # --- Measure error from sensors ---
    err = _centroid_error(sensors)

    # --- PID terms ---
    # P: proportional to current error
    p = KP * err

    # I: accumulate error over time (with clamping to prevent windup)
    _integral += err * dt * KI
    if _integral < I_MIN:
        _integral = I_MIN
    elif _integral > I_MAX:
        _integral = I_MAX
    i = _integral

    # D: depends on how fast error changes (finite difference)
    if _has_last:
        d_raw = (err - _last_err) / dt
    else:
        d_raw = 0.0
        _has_last = True
    d = KD * d_raw
    _last_err = err

    # Combined steering command; sign convention matches _centroid_error.
    steer = p + i + d

    # Map steering into left/right PWM. Turning right = slow left, speed up right.
    pwm_left = _clamp_pwm(BASE_PWM - steer)
    pwm_right = _clamp_pwm(BASE_PWM + steer)

    return {"pwm_left": pwm_left, "pwm_right": pwm_right}
