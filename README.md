# RobotraceSim — Line-Follower Robot Simulator

RobotraceSim is a Python + PySide6 simulator for line-follower robots. It loads a track JSON, a robot JSON and a Python controller, then runs the simulation through the current `SimulationEngine`.

The active simulation path uses the native C backend in `utills_c/linesim.dll` through `sim/native_linesim.py`. The Python code handles validation, controller loading, state assembly, robot/track runtime conversion, cache management and UI/replay.

## Quick start

```bash
pip install PySide6 pytest

python track_editor.py
python robot_editor.py
python simulator.py
```

In the simulator:

1. Load a track JSON, for example `Example/Track/track_1_cw.json`.
2. Load a robot JSON, for example `Example/Robot/robot-spec.json`.
3. Load a controller, for example `Example/Controller/P_basic.py` or `Example/Controller/PID_basic.py`.
4. Click **Start**.

## Files used by the simulator

| File type | Example | Purpose |
|---|---|---|
| Track JSON | `Example/Track/track_1_cw.json` | Defines area, origin, tape width, segments and optional start/finish. |
| Robot JSON | `Example/Robot/robot-spec.json` | Defines envelope, origin, wheels, sensors, controller parameters and drivetrain/mechanical data. |
| Controller Python | `Example/Controller/P_basic.py` | Must expose `control_step(state)`. |
| `.rmap` cache | generated beside the track JSON | Derived raster cache for track/contact/sensor lookup. Do not edit manually. |

There is no required `simulation_parameters.json` in the current flow. At runtime, `SimulationConfig` is derived from the loaded robot spec, especially `controller.simulation_step_dt_ms` and `sensorsConfig`.

## Controller contract

The controller file must define:

```python
def control_step(state: dict) -> dict:
    return {"pwm_left": int_value, "pwm_right": int_value}
```

The simulator preserves this contract. Older controllers that ignore extra state fields continue to work.

### State payload sent to `control_step(state)`

Every simulation tick, the engine sends this dictionary:

```python
{
    "t_ms": int,               # elapsed simulation time [ms]
    "dt_s": float,             # simulation step [s], from SimulationConfig.dt_s
    "x_mm": float,             # robot pose origin X in world/track frame [mm]
    "y_mm": float,             # robot pose origin Y in world/track frame [mm]
    "heading_deg": float,      # robot heading [deg]
    "v_mm_s": float,           # linear velocity [mm/s]
    "omega_rad_s": float,      # angular velocity [rad/s]
    "a_lin_mm_s2": float,      # linear acceleration [mm/s²]
    "alpha_rad_s2": float,     # angular acceleration [rad/s²]
    "sensors": list[int],      # sensor readings
    "v_left_mm_s": float,      # left wheel linear speed [mm/s]
    "v_right_mm_s": float,     # right wheel linear speed [mm/s]
}
```

`dt_s` comes from `SimulationConfig.dt_s`, which is `simulation_step_dt_ms / 1000.0`. For example, `simulation_step_dt_ms = 1.0` produces `dt_s = 0.001`.

The expected return is:

```python
{"pwm_left": int, "pwm_right": int}
```

If the controller raises an exception or returns an invalid object, the current engine falls back to neutral/default PWM values for that tick and prints the controller error.

## Robot origin and runtime geometry

The robot JSON stores wheel and sensor positions in the editor/drawing coordinate system. The `origin` field defines the local point used as the simulation pose reference.

At runtime, local JSON coordinates are converted by subtracting the configured origin:

```python
local_runtime_x = local_json_x - robot.origin_x_mm
local_runtime_y = local_json_y - robot.origin_y_mm
```

Then the pose rotation/translation is applied:

```python
world_x = pose_x + cos(theta) * local_runtime_x - sin(theta) * local_runtime_y
world_y = pose_y + sin(theta) * local_runtime_x + cos(theta) * local_runtime_y
```

This applies consistently to sensors, wheels and the envelope center used by the runtime. The wheel track used by the drivetrain prefers `geometric_mechanical.track_mm`; if it is not valid, it falls back to the distance between the wheel centers. That fallback is not distorted by `origin`, because the same origin offset is subtracted from both wheels.

## Track raster cache (`.rmap`)

For a loaded track JSON, the simulator creates or reuses a `.rmap` file beside the track. This file is a derived cache, not source data.

The cache metadata stores a SHA-256 fingerprint built from:

- track area;
- track origin;
- `tapeWidthMM`;
- segments;
- start/finish configuration;
- rasterization parameters such as pixel size, margin, polyline step and marker dimensions.

When a track is loaded, the simulator compares the expected fingerprint with the fingerprint stored in the `.rmap` header. If the hash is missing, different, invalid or the cache is corrupted, the `.rmap` is regenerated automatically.

Do not edit `.rmap` files manually. Edit the track JSON instead.

## Active physics path

The current active physics path is the C drivetrain/backend exposed by `step_motor_drivetrain_C` in `linesim.dll`.

The Python engine passes drivetrain, battery, motor, mass, wheel radius, wheel track, friction and timestep values to the C backend. The public C API and function signatures are not changed by the Python refactor.

The previous/legacy simple physics switch is not exposed as a runtime option in this version. The stale `use_motor_dc_model` configuration flag was removed from `SimulationConfig` because it did not select a different code path. This avoids a misleading half-active physics option. New physics choices should be added later as explicit, tested options.

## Logging and replay

If **Save logs to file (CSV+JSON)** is enabled in the UI, logs are written under `Logs/`:

```text
Logs/sim_log_YYYYMMDD_HHMMSS.csv
Logs/sim_log_YYYYMMDD_HHMMSS.json
```

CSV step columns:

```text
t_ms, x_mm, y_mm, heading_deg, v_mm_s, omega_rad_s, pwm_left, pwm_right, s0, s1, ..., sN
```

Replay uses the stored simulation steps and does not rerun the controller.

## Project structure

```text
RobotraceSim/
├── simulator.py
├── track_editor.py
├── robot_editor.py
├── Utils/
│   ├── robot_spec.py
│   ├── robot_runtime.py
│   ├── track_spec.py
│   ├── simulation_config.py
│   ├── simulation_state.py
│   └── validation.py
├── sim/
│   ├── engine.py
│   ├── worker.py
│   ├── controller_loader.py
│   ├── track_runtime.py
│   └── native_linesim.py
├── Example/
│   ├── Controller/
│   ├── Robot/
│   └── Track/
├── test/
├── utills_c/
└── Logs/
```

## Testing

Compile Python files:

```bash
python -m compileall simulator.py robot_editor.py track_editor.py Utils sim
```

Run tests:

```bash
python -m pytest -q
```

On PowerShell, prefer `compileall` as shown above instead of `python -m py_compile Utils/*.py`, because Python on Windows does not expand wildcards in that form.

## Current limitations

- This phase corrects inconsistencies only; it does not add new physics.
- The active simulation path depends on the native `linesim.dll` backend.
- `.rmap` files are cache files and may be regenerated at any time.
- Physics-module selection is intentionally not exposed until separate implementations are explicit, tested and documented.
