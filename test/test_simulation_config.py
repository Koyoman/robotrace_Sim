from Utils.simulation_config import SimulationConfig


def test_simulation_config_dt_s():
    cfg = SimulationConfig(simulation_step_dt_ms=2.5)
    assert cfg.dt_s == 0.0025
