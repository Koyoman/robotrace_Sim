"""P_basic.py — Minimal proportional (P) controller example for a line‑following robot.

This controller reads an array of light sensors (or similar) that tell how much each
segment of the sensor bar "sees" the line. It then computes a *centroid-like* error
telling how far the robot is from the center of the line and applies a proportional
(P-only) correction to the left/right wheel commands.

The goal of this file is educational: the comments explain each piece in simple,
non‑technical English so that someone new to robotics can follow along.
"""

# ===== Configuration constants (you can tweak these to change behavior) =====

# Base PWM (motor power) applied to both wheels when the robot is perfectly centered.
# Larger values make the robot drive faster in a straight line.
BASE_PWM = 2048

# Proportional gain: how strongly we react to the measured "off‑center" error.
# Bigger KP = stronger/ quicker turns; too big can make the robot oscillate.
KP = 0.3

# Allowed PWM range for the motors (typical 12‑bit DAC/PWM range is -4095..4095).
# We clamp (limit) all motor commands to stay inside this safe range.
PWM_MIN, PWM_MAX = -4095, 4095


def _clamp(v: float) -> int:
    """Limit value *v* to the safe PWM range.

    Why this matters:
    Motor drivers and simulators expect commands inside a valid range.
    Clamping protects against accidental overflow (e.g., from large gains)
    and keeps the robot stable.
    """
    if v < PWM_MIN:
        return PWM_MIN
    if v > PWM_MAX:
        return PWM_MAX
    return int(v)


def _weights_signed_even(n: int, step: float = 1000.0):
    """Create a list of *n* horizontal positions for the sensor bar, centered at zero.

    Each sensor gets a "position weight" that tells us where it sits left↔right.
    Example (n=5): [-2000, -1000, 0, 1000, 2000]
    Example (n=4): [-1500, -500, 500, 1500]  (note there is no exact center sensor)

    We use these weights together with the sensor readings to compute a
    weighted average (centroid). That gives a smooth estimate of how far
    the robot is from the line center.

    Args:
        n: Number of sensors.
        step: Distance between adjacent sensor positions (unitless scale).
              Bigger step ⇒ stronger position effect for the same readings.

    Returns:
        List[float]: signed positions for each sensor index.
    """
    if n <= 0:
        return []

    half = n // 2

    # If n is even, there is no middle sensor. We "skip" zero so left and right
    # are symmetric (…,-step, +step, …). If n is odd, we naturally get a 0 center.
    if n % 2 == 0:
        out = []
        for i in range(n):
            k = i - half
            if i >= half:
                k += 1  # skip 0 to keep symmetry when n is even
            out.append(k * step)
        return out
    else:
        return [(i - half) * step for i in range(n)]


def control_step(state):
    """Compute one control action from the current simulator *state*.

    The simulator passes in a dictionary with live information. We read the
    line sensors, estimate how far we are from the center, then return the
    left/right motor commands (PWM values). This is called every simulation tick.

    Expected *state* keys (only 'sensors' is required here):
        - "sensors": list of non‑negative numbers, one per sensor.
                     Larger number ⇒ that sensor sees more of the line.
        - "v_mm_s":     (optional) current linear speed, mm/s (not used here)
        - "omega_rad_s":(optional) current angular speed, rad/s (not used here)
        - "a_lin_mm_s2":(optional) linear acceleration, mm/s² (not used here)
        - "alpha_rad_s2":(optional) angular acceleration, rad/s² (not used here)
        - "v_left_mm_s": (optional) left wheel speed, mm/s (not used here)
        - "v_right_mm_s":(optional) right wheel speed, mm/s (not used here)

    Returns:
        dict: {"pwm_left": int, "pwm_right": int}
    """
    # Read the sensor array (could be empty if something went wrong).
    vals = state.get("sensors", [])
    n = len(vals)
    if n == 0:
        # No sensor data: just drive straight at base power as a safe fallback.
        return {"pwm_left": BASE_PWM, "pwm_right": BASE_PWM}

    # The lines below show how to read other telemetry fields if you want to log
    # or experiment with more advanced controllers later. We don't use them here.
    _v = float(state.get("v_mm_s", 0.0))
    _w = float(state.get("omega_rad_s", 0.0))
    _a_lin = float(state.get("a_lin_mm_s2", 0.0))
    _a_ang = float(state.get("alpha_rad_s2", 0.0))
    _vL = float(state.get("v_left_mm_s", 0.0))
    _vR = float(state.get("v_right_mm_s", 0.0))

    # Compute the position weights for each sensor index (left is negative, right positive).
    W = _weights_signed_even(n, step=1000.0)

    # Sum of sensor activations. We use it to compute a weighted average safely.
    sw = sum(vals)
    if sw <= 1e-12:
        # All sensors read ~zero (e.g., line lost): treat error as zero to avoid division by zero.
        err = 0.0
    else:
        # Centroid error: (sum(position_i * reading_i) / sum(reading_i)).
        # If the line is more under the right sensors, error is positive (turn right).
        err = sum(w * a for w, a in zip(W, vals)) / sw

    # Proportional steering: more error ⇒ bigger difference between wheels.
    diff = KP * err

    # Convert steering into motor commands. Turning right means slowing the left wheel
    # and speeding up the right wheel (signs are opposite).
    pwm_left = _clamp(BASE_PWM - diff)
    pwm_right = _clamp(BASE_PWM + diff)

    return {"pwm_left": pwm_left, "pwm_right": pwm_right}
