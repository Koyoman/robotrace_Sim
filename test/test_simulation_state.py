from Utils.simulation_state import SimulationState


def test_controller_state_has_dt_and_documented_fields():
    state = SimulationState(
        t_ms=1,
        x_mm=2.0,
        y_mm=3.0,
        heading_deg=4.0,
        v_mm_s=5.0,
        omega_rad_s=6.0,
        a_lin_mm_s2=7.0,
        alpha_rad_s2=8.0,
        v_left_mm_s=9.0,
        v_right_mm_s=10.0,
        pwm_left=11,
        pwm_right=12,
        sensors=[13],
    )
    payload = state.to_controller_state(0.001)
    for key in [
        "t_ms", "dt_s", "x_mm", "y_mm", "heading_deg", "v_mm_s", "omega_rad_s",
        "a_lin_mm_s2", "alpha_rad_s2", "sensors", "v_left_mm_s", "v_right_mm_s",
    ]:
        assert key in payload
    assert payload["dt_s"] == 0.001
