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

Physics is selected by `SimulationConfig.physics_profile`. The default remains:

```python
physics_profile = "realistic"
```

Accepted profiles:

| Profile | Model | Behavior | Native backend |
|---|---|---|---|
| `ideal` | `IdealPhysicsModel` | PWM maps directly to wheel speed. No inertia, battery, slip, encoder, IMU, sensor noise or track imperfections. Useful for controller/debug checks. | No |
| `basic` | `BasicKinematicPhysicsModel` | PWM maps to a target wheel speed and a simple wheel acceleration limit can be applied. | No |
| `realistic` | `RealisticPhysicsModel` | Phase 4 complete preset. It enables the DC motor path, acceleration limit, battery, PWM/deadzone/current-limit parameters, slip, encoder, IMU, sensor noise and procedural track imperfections with safe defaults. | Yes, uses `step_physics_modular_C` when available |
| `custom` | `CustomPhysicsModel` | User-selectable flags and parameters from the **Custom settings…** dialog. It can use the modular C/DC path or the Python kinematic fallback. | Depends on flags |

### Phase 4 — expanded physics

Phase 4 changed the meaning of **Realistic**. It no longer means “preserve the old DC behavior”. It now means “use the most complete physical model implemented by the project”. Because of this, realistic results may differ from Phase 3.

Implemented resources:

- wheel acceleration limit;
- battery discharge with voltage/SOC state;
- line sensor noise with reproducible seed;
- wheel encoder ticks, delta ticks and angular speed fields;
- simple IMU channels for gyro/alpha/acceleration;
- wheel slip, including asymmetric slip;
- procedural track imperfections applied during runtime sensor sampling;
- modular C backend entry point `step_physics_modular_C`;
- explicit configuration for parameters that were previously implicit or hardcoded in the C model.

### State payload additions

The original controller fields are preserved. Phase 4 adds optional fields that old controllers can ignore:

```python
{
    "battery_voltage_v": float,
    "battery_soc": float,
    "enc_left_ticks": float,
    "enc_right_ticks": float,
    "enc_left_delta_ticks": float,
    "enc_right_delta_ticks": float,
    "enc_left_rad_s": float,
    "enc_right_rad_s": float,
    "imu_omega_rad_s": float,
    "imu_alpha_rad_s2": float,
    "imu_accel_x_mm_s2": float,
    "imu_accel_y_mm_s2": float,
    "slip_ratio_left": float,
    "slip_ratio_right": float,
}
```

The controller signature is still:

```python
control_step(state) -> {"pwm_left": int, "pwm_right": int}
```

### Custom settings dialog

Selecting **Custom** enables **Custom settings…**. The dialog now has one window with these sections:

1. Modelo base;
2. Limite de aceleração;
3. Bateria;
4. Sensores com ruído;
5. Encoder;
6. IMU;
7. Slip de roda;
8. Imperfeições da pista;
9. Parâmetros avançados do motor/C.

The values flow through the project as:

```text
UI Custom settings
    -> MainWindow.custom_physics_settings
    -> MainWindow._build_simulation_config()
    -> SimulationConfig
    -> SimWorker
    -> SimulationEngine
    -> create_physics_model()
    -> CustomPhysicsModel / RealisticPhysicsModel
    -> NativeLineSim / step_physics_modular_C, when DC/C is enabled
```

Changes are applied only when the next simulation starts. They are not applied to a run already in progress.

### Main Phase 4 configuration fields

`SimulationConfig` now includes the following real-effect flags and parameters:

```python
custom_use_dc_motor_model: bool = True
custom_use_kinematic_model: bool = True
custom_use_acceleration_limit: bool = True
custom_use_battery_model: bool = False
custom_use_sensor_noise: bool = False
custom_use_encoder_model: bool = False
custom_use_imu_model: bool = False
custom_use_wheel_slip: bool = False
custom_use_track_imperfections: bool = False

custom_use_auto_acceleration_limit: bool = True
custom_max_wheel_accel_mm_s2: float = 9810.0

custom_battery_initial_voltage_v: float = 7.4
custom_battery_nominal_voltage_v: float = 7.4
custom_battery_min_voltage_v: float = 6.0
custom_battery_capacity_mah: float = 1000.0
custom_battery_internal_resistance_ohm: float = 0.15
custom_battery_soc_initial: float = 1.0

custom_sensor_noise_std: float = 0.02
custom_sensor_noise_seed: int = 12345

custom_encoder_ticks_per_rev: int = 1024
custom_encoder_noise_std_ticks: float = 0.0
custom_encoder_quantization: bool = True

custom_imu_gyro_noise_std_rad_s: float = 0.0
custom_imu_accel_noise_std_mm_s2: float = 0.0

custom_slip_ratio_left: float = 0.0
custom_slip_ratio_right: float = 0.0
custom_slip_noise_std: float = 0.0

custom_track_imperfection_amplitude_mm: float = 0.0
custom_track_imperfection_wavelength_mm: float = 500.0
custom_track_imperfection_noise_std: float = 0.0

custom_motor_deadzone_pwm: float = 0.0
custom_current_limit_a: float = 0.0
custom_drivetrain_efficiency: float = 0.9
custom_viscous_friction: float = 0.0
custom_coulomb_friction: float = 0.0
custom_pwm_saturation_enabled: bool = True
custom_max_pwm: float | None = None
```

### Battery model

The battery model is intentionally simple. It tracks SOC across steps, estimates current from the motor model or PWM load, converts capacity from mAh to As and reduces SOC. Voltage is clamped by the configured minimum voltage and includes a simple internal-resistance drop.

Known limitation: it is not an electrochemical battery model.

### Sensor noise

Sensor noise is applied after the normal line sensor value is computed. If `custom_sensor_noise_std <= 1.0`, it is interpreted as normalized ADC-scale noise. Larger values are interpreted as raw ADC counts. The configured seed makes runs reproducible.

### Encoder

Encoder ticks are derived from wheel displacement, wheel radius and ticks per revolution. The model keeps accumulated ticks and per-step delta ticks. Quantization can be disabled for continuous experiments.

### IMU

The IMU model exposes gyro Z and simple acceleration channels. `imu_omega_rad_s` follows the simulated angular speed. `imu_accel_x_mm_s2` and `imu_accel_y_mm_s2` use the estimated linear acceleration projected in world coordinates. This is a simplified model, not a full inertial/navigation model.

### Slip

Slip applies:

```text
effective_wheel_speed = modeled_wheel_speed * (1 - slip_ratio)
```

The ratio is clamped between `0.0` and `0.95`. Asymmetric slip produces curvature/heading changes.

### Track imperfections

Track imperfections do not modify the track JSON and do not invalidate the `.rmap` cache. They are procedural offsets applied at sensor sampling time, after the cached raster is loaded.

### Native C backend compatibility and modular backend

The old public function remains available:

```c
step_motor_drivetrain_C(...)
```

Phase 4 adds:

```c
int step_physics_modular_C(
    const PhysicsInputC* input,
    const PhysicsConfigC* config,
    PhysicsStateC* state
);
```

`sim/native_linesim.py` declares equivalent `ctypes.Structure` classes and configures the new function when the loaded library exports it. If a native library does not export the new function, the Python model uses a controlled fallback rather than exposing a raw traceback.

### C parameters made explicit

The C backend now receives these physical values explicitly through `PhysicsConfigC` instead of relying on hidden constants:

| Parameter | Previous location | New source | Unit | Default/fallback |
|---|---|---|---|---|
| `track_m` | robot/C call | `derive_runtime_params` | m | `geometric_mechanical.track_mm` or wheel spacing |
| `wheel_radius_m` | robot/C call | `derive_runtime_params` | m | `geometric_mechanical.wheel_radius_mm` |
| `pwm_min`, `pwm_max`, `pwm_center` | controller/C call | RobotSpec/SimulationConfig | PWM counts | robot controller limits |
| `motor_deadzone_pwm` | implicit/deadband | `custom_motor_deadzone_pwm` | PWM counts | `0.0` |
| `battery_voltage_v` | RobotSpec/DC call | RobotSpec/SimulationConfig | V | robot battery voltage |
| `battery_capacity_mah` | RobotSpec or default | RobotSpec/SimulationConfig | mAh | robot capacity or `1000.0` |
| `battery_internal_resistance_ohm` | RobotSpec/DC call | RobotSpec/SimulationConfig | Ω | robot value or `0.15` |
| `current_limit_a` | motor field/fallback | RobotSpec/SimulationConfig | A | driver/stall current or `3.0` |
| `drivetrain_efficiency` | motor field | RobotSpec/SimulationConfig | ratio | `0.9` |
| `viscous_friction` | motor field | RobotSpec/SimulationConfig | Nm/(rad/s) | motor field or `0.0` |
| `coulomb_friction` | motor field | RobotSpec/SimulationConfig | Nm | motor field or `0.0` |
| `max_wheel_accel_mm_s2` | Python default | SimulationConfig | mm/s² | auto from robot or `9810.0` |
| `slip_ratio_left/right` | absent | SimulationConfig | ratio | `0.0` |
| `encoder_ticks_per_rev` | absent | SimulationConfig | ticks/rev | `1024` |

### Rebuilding the native backend

Windows/MSYS2 UCRT64:

```bash
cd utills_c
build_linesim_windows_mingw.bat
```

Linux/WSL:

```bash
cd utills_c
bash build_linesim_linux.sh
```

Manual Linux build:

```bash
gcc -shared -O2 -fPIC -DLINESIM_EXPORTS -o utills_c/liblinesim.so utills_c/linesim.c -lm
```

### Test commands

```bash
python -m compileall simulator.py robot_editor.py track_editor.py Utils sim
python -m pytest -q
```

PowerShell note: do not use `python -m py_compile Utils/*.py`. Use `compileall` as shown above.

### Current physics limitations

Phase 4 adds useful physical effects, but it is still simplified:

- the battery model is approximate;
- the IMU model is simple;
- slip is a direct speed-loss model, not a full tire model;
- track imperfections are procedural sensor offsets;
- there is no advanced lateral dynamics or tire force model;
- sensor noise is stochastic but not based on an optical/electrical sensor model.


## Logging and replay

If **Save logs to file (CSV+JSON)** is enabled in the UI, logs are written under `Logs/`:

```text
Logs/sim_log_YYYYMMDD_HHMMSS.csv
Logs/sim_log_YYYYMMDD_HHMMSS.json
```

CSV step columns now include the complete Phase 4 telemetry available at each step. The first columns are ordered for analysis and the sensor columns are still expanded as `s0`, `s1`, ..., `sN`:

```text
t_ms, dt_s, physics_profile,
x_mm, y_mm, heading_deg,
v_mm_s, omega_rad_s, a_lin_mm_s2, alpha_rad_s2,
v_left_mm_s, v_right_mm_s,
pwm_left, pwm_right,
battery_voltage_v, battery_soc,
current_left_a, current_right_a, current_total_a, battery_power_w,
enc_left_ticks, enc_right_ticks,
enc_left_delta_ticks, enc_right_delta_ticks,
enc_left_rad_s, enc_right_rad_s,
imu_omega_rad_s, imu_alpha_rad_s2,
imu_accel_x_mm_s2, imu_accel_y_mm_s2,
slip_ratio_left, slip_ratio_right,
hit, finished,
s0, s1, ..., sN
```

The JSON log stores the same per-step telemetry, keeping `sensors` as a list. This makes the JSON better for programmatic analysis and the CSV better for spreadsheets.

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

## Fase 4.1 — correção/revalidação do realistic

Após a expansão física da Fase 4, foi feita uma revalidação do perfil `realistic` porque ele podia ficar artificialmente lento em comparação com `ideal` e `basic`.

Correções aplicadas:

- a bateria usa OCV/fonte derivada do SOC para alimentar o motor;
- a tensão terminal sob carga continua sendo reportada no `state`, mas não é realimentada como OCV;
- o slip foi separado entre velocidade periférica da roda e velocidade efetiva no chão;
- encoders usam velocidade da roda antes do slip;
- a pose do robô usa velocidade efetiva após slip;
- o backend C modular agora expõe `linesim_abi_version_C()`; DLLs antigas sem ABI 2 não são usadas no caminho modular, evitando o bug de tensão recursiva.

A fundamentação das fórmulas está em `docs/phase4_formula_validation.md`.

## Fase 4.3 — correções da física realistic/custom

A Fase 4.3 corrige pontos conceituais do modo `realistic/custom` sem alterar a assinatura do controller (`control_step(state)`). Os campos antigos continuam disponíveis e novos campos foram adicionados ao `state` e aos logs.

Principais correções:

- `current_left_a`, `current_right_a`, `current_total_a`, `battery_current_a` e `battery_power_w` agora representam consumo e nunca são negativos.
- A corrente assinada usada para torque foi separada em `motor_left_current_signed_a` e `motor_right_current_signed_a`.
- Quando o motor gira para frente e o PWM manda ré, o modelo usa `I = (V_aplicada - back_emf) / R`; por isso o módulo da corrente aumenta, como esperado fisicamente.
- O encoder agora mede a rotação da roda/pneu antes do slip e expõe ticks inteiros.
- A velocidade de roda foi separada da velocidade efetiva no chão: `wheel_*_surface_speed_mm_s` e `ground_*_speed_mm_s`.
- A IMU agora trabalha no referencial do corpo do robô: `imu_accel_x_mm_s2 = aceleração longitudinal` e `imu_accel_y_mm_s2 ≈ v * omega`.
- O slip do `realistic` agora é calculado por torque/força disponível e limites de atrito (`mu_static`, `mu_kinetic`, `mass_kg`, `Crr`), não por valor fixo.
- O `custom` mantém opção de slip manual com `custom_use_manual_slip` e também preserva compatibilidade com configurações antigas que já usavam `custom_slip_ratio_left/right`.
- O log agora inclui `duty_left/right`, corrente de bateria, corrente assinada/debug, tensão aplicada, back-EMF, torque, velocidade de roda, velocidade no chão, forças de tração, `physics_backend` e `linesim_abi_version`.

Dados do `robot-spec.json` agora usados de forma efetiva incluem geometria, massa, atrito, bateria, tabela OCV, resistências, queda do driver, constantes do motor, transmissão, PWM, sensores, encoder e IMU. O bloco `odometry` continua carregado e reservado para uma fase posterior, sem ser misturado ao encoder bruto.

Para detalhes de fórmulas, unidades, campos de log, limitações e sanity checks, consulte:

```text
docs/phase4_3_physics_corrections.md
```

### Recompilar backend C

Linux/macOS:

```bash
cd utills_c
gcc -O2 -shared -fPIC linesim.c -o liblinesim.so -lm
```

Windows com MinGW:

```bat
cd utills_c
build_linesim_windows_mingw.bat
```

### Testar

No PowerShell, use `compileall` em vez de glob com `py_compile`:

```powershell
python -m compileall simulator.py robot_editor.py track_editor.py Utils sim
python -m pytest -q
```

## Fase 4.4 — backend C real, dinâmica de roda e telemetria física

A Fase 4.4 torna o `realistic` dependente do backend C modular por padrão. O modo `realistic` não deve mais cair silenciosamente em `python_dc`: quando o C está válido, o log deve indicar `physics_backend = realistic_c_modular`, `linesim_abi_version >= 4`, `c_step_call_count > 0` e `using_python_fallback = false`. Se a DLL/SO estiver ausente, antiga ou incompatível, a simulação falha com erro claro, salvo quando o fallback Python for habilitado explicitamente em configuração.

Novos campos de política em `SimulationConfig`:

- `require_c_backend_for_realistic`
- `allow_python_fallback_for_realistic`
- `custom_use_c_backend`
- `custom_allow_python_fallback`

O backend C agora exporta `linesim_abi_version_C()` com `LINESIM_ABI_VERSION = 4`, além de `step_physics_modular_C(...)` com telemetria expandida. O Python valida ABI e disponibilidade da função antes de iniciar o caminho modular.

O modelo C do `realistic/custom_c` calcula a física principal: dinâmica própria de roda, inércia refletida motor/transmissão, torque de contato com o solo, slip contínuo, limite combinado longitudinal/lateral, potência, energia e canais separados de torque. O Python continua responsável por UI, controller, carregamento de JSON, sensores geométricos de linha e log.

Documentação detalhada:

```text
docs/phase4_4_realistic_backend_and_dynamics.md
```

### Recompilar backend C — Fase 4.4

Linux/WSL:

```bash
./utills_c/build_linesim_linux.sh
```

Windows/MSYS2 UCRT64:

```bat
utills_c\build_linesim_windows_mingw.bat
```

### Validar uso real do C

```bash
python - <<'PY'
from sim.native_linesim import get_linesim, linesim_abi_version, has_modular_physics, backend_info
lib = get_linesim()
print(linesim_abi_version(lib), has_modular_physics(lib), backend_info(lib))
PY
```

O resultado esperado é ABI `>= 4`, `has_modular_physics=True` e `compatible_modular=True`.

### Testar

```bash
python -m compileall simulator.py robot_editor.py track_editor.py Utils sim
python -m pytest -q
```

> Observação: o pacote inclui `utills_c/liblinesim.so` recompilado para Linux/WSL. A DLL antiga de Windows não foi reaproveitada; em Windows, recompile com o `.bat` antes de rodar `realistic`.
