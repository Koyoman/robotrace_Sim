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
4. Choose a physics profile if needed. The default is **Realistic**.
5. Click **Start**.

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

## Physics profiles

Physics is selected by `SimulationConfig.physics_profile`. Existing flows remain compatible because the default is:

```python
physics_profile = "realistic"
```

Accepted profiles:

| Profile | Model | Behavior | Native backend |
|---|---|---|---|
| `ideal` | `IdealPhysicsModel` | Converts PWM directly to wheel speed and updates pose with differential-drive kinematics. There is no inertia or gradual acceleration. | No |
| `basic` | `BasicKinematicPhysicsModel` | Converts PWM to target wheel speed, applies a simple wheel acceleration limit, then updates pose with differential-drive kinematics. | No |
| `realistic` | `DCMotorPhysicsModel` | Preserves the previous simulator behavior by calling the existing C drivetrain path exposed as `step_motor_drivetrain_C`. | Yes |
| `custom` | factory-selected | Uses explicit supported flags. By default it behaves like `realistic`; if `custom_use_dc_motor_model=False`, it uses the basic kinematic model and `custom_use_acceleration_limit` controls the acceleration limit. | Depends on flags |

### Physics configuration fields

`SimulationConfig` includes these fields:

```python
physics_profile: str = "realistic"
ideal_max_wheel_speed_mm_s: float | None = None
basic_max_wheel_speed_mm_s: float | None = None
basic_max_wheel_accel_mm_s2: float | None = None
custom_use_dc_motor_model: bool = True
custom_use_acceleration_limit: bool = True
```

For `ideal_max_wheel_speed_mm_s`, `basic_max_wheel_speed_mm_s` and
`basic_max_wheel_accel_mm_s2`, `None` means automatic derivation from the
loaded robot spec. This is important because the ideal/basic profiles must use
the same order of magnitude as the DC model. With the example N20 robot, the
full-PWM no-load wheel speed is derived from battery voltage, driver drop, motor
`Kv`/`Ke`, gear ratio and wheel radius instead of using a fixed 500 mm/s
fallback. Explicit numeric values are still supported when a deterministic
experiment needs a manually chosen scale.

The `custom_*` flags are intentionally limited to behavior implemented in this phase:

- `custom_use_dc_motor_model=True`: use the current DC/native model.
- `custom_use_dc_motor_model=False`: use the basic kinematic model.
- `custom_use_acceleration_limit=True`: when the custom profile is using the basic kinematic model, wheel speeds change gradually.
- `custom_use_acceleration_limit=False`: when the custom profile is using the basic kinematic model, wheel speeds jump directly to the target speed.

When the simulator UI is used, selecting **Custom** enables the **Custom settings…** button.
That dialog is the single place where the user can choose the currently supported
custom physics options for the next run:

- whether Custom uses the DC/native drivetrain or the Python kinematic drivetrain;
- whether the kinematic drivetrain uses an acceleration limit;
- whether the kinematic max wheel speed is automatic or manually overridden;
- whether the kinematic max wheel acceleration is automatic or manually overridden.

The dialog values are stored in the main window and copied into `SimulationConfig`
immediately before creating the `SimWorker`. Therefore changes made by the user are
not applied to a simulation that is already running; they are applied to the next
press of **Start**. The worker then passes the same config into `SimulationEngine`,
and the factory in `sim/physics/factory.py` creates the selected model from those
values.

No decorative physics flags were added for features not implemented in this phase.

### Native C backend compatibility

The public C API remains compatible. The signature of `step_motor_drivetrain_C` was not changed. The realistic profile still calls the native DC drivetrain through `sim/physics/dc_motor.py`.

One robustness fix was made inside `linesim.c`: the exact current update in `step_motor_drivetrain_C` now uses the same normalized SI-unit wheel radius and track width as the derivative calculation. Python already passes these values in meters, so this does not explain the large step-count difference by itself; it only prevents incorrect behavior if the C function is called directly with legacy millimeter values.

On Linux/macOS development environments, `sim/native_linesim.py` can also load `utills_c/liblinesim.so` or `utills_c/linesim.so` before falling back to `linesim.dll`. Windows still uses `linesim.dll`.

The Python engine now delegates motion updates to a model created by `sim/physics/factory.py`. The engine remains responsible for controller calls, sensor updates, finish/collision checks, chunk emission and logging.

### Why ideal/basic no longer use a fixed 500 mm/s scale

The original Phase 3 implementation used `500 mm/s` as the full-scale wheel
speed for `ideal` and `basic`. With `P_basic.py`, the base PWM is `1000`, and
the robot controller range is `-4095..4095`. Therefore the ideal/basic profiles
were running at roughly:

```text
1000 / 4095 * 500 mm/s ≈ 122 mm/s
```

The realistic DC model does not use that fixed scale. It derives motor speed
from the robot JSON electrical and motor parameters, so the same PWM produces a
wheel speed around five times higher for the example robot. This was the main
reason for the observed difference between about 39,700 steps in ideal/basic and
about 8,200 steps in realistic.

The fix was to let ideal/basic derive their default full-PWM scale from the same
robot parameters used by the DC model.

### Current physics limitations

This phase modularizes physics selection. It does not add:

- sensor noise beyond the existing sensor value noise path;
- battery state-of-charge simulation;
- wheel slip/derrapagem;
- tire model;
- lateral dynamics;
- new collision physics.

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
│   ├── native_linesim.py
│   └── physics/
│       ├── base.py
│       ├── ideal.py
│       ├── kinematic.py
│       ├── dc_motor.py
│       └── factory.py
├── Example/
│   ├── Controller/
│   ├── Robot/
│   └── Track/
├── test/
├── utills_c/
└── Logs/
```


## Building the native C backend

The repository includes the existing Windows `utills_c/linesim.dll`. If you
change `utills_c/linesim.c`, rebuild the native library for your platform before
running the `realistic` profile.

Linux/macOS development build:

```bash
cd utills_c
gcc -shared -fPIC -O2 -o liblinesim.so linesim.c -lm
```

Windows MinGW-w64 example:

```bat
cd utills_c
x86_64-w64-mingw32-gcc -shared -O2 -DLINESIM_EXPORTS -o linesim.dll linesim.c -Wl,--out-implib,linesim.lib
```

Helper scripts are available as:

- `utills_c/build_linesim_linux.sh`
- `utills_c/build_linesim_windows_mingw.bat`

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

- The `realistic` profile depends on the native `linesim.dll` backend.
- The `ideal` and `basic` profiles are deterministic Python models intended for debugging and controller iteration.
- `.rmap` files are cache files and may be regenerated at any time.
- Advanced physics such as battery discharge, slip, tire modeling and lateral dynamics are intentionally outside this phase.
