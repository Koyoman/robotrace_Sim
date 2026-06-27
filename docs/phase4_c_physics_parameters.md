# Phase 4 C physics parameter mapping

This document records the Phase 4 audit of the previous C drivetrain path and where each physical parameter now comes from.

| Physical parameter | Where it was before | New field/configuration | Unit | Default/fallback |
|---|---|---|---|---|
| `track_m` | Argument to `step_motor_drivetrain_C`; normalized inside C | `derive_runtime_params(...)["track_m"]`, then `PhysicsConfigC.track_m` | m | `geometric_mechanical.track_mm` or wheel spacing |
| `wheel_radius_m` | Argument to `step_motor_drivetrain_C`; normalized inside C | `derive_runtime_params(...)["wheel_r_m"]`, then `PhysicsConfigC.wheel_radius_m` | m | `geometric_mechanical.wheel_radius_mm` |
| `pwm_min`, `pwm_max` | Robot controller fields passed positionally | `PhysicsConfigC.pwm_min`, `PhysicsConfigC.pwm_max` | PWM counts | robot controller limits |
| `pwm_center` | Derived from controller or center of range | `PhysicsConfigC.pwm_center` | PWM counts | neutral or midpoint |
| `deadband_01` | Normalized positional argument | `custom_motor_deadzone_pwm` converted to normalized C deadband | ratio | `0.0` |
| `V_batt` | Robot electrical field passed to C | `custom_battery_initial_voltage_v` / RobotSpec | V | robot battery voltage or `7.4` |
| `R_batt` | Robot electrical field passed to C | `custom_battery_internal_resistance_ohm` / RobotSpec | Ω | robot value or `0.15` |
| `V_driver_drop` | Robot electrical field passed to C | RobotSpec through `driver_drop_V` | V | `0.2` |
| `Rm` | Robot motor field | RobotSpec `Rm_ohm` | Ω | `4.0` |
| `Lm` | Robot motor field | RobotSpec `Lm_H` | H | `0.00015` |
| `Kt` | Robot motor field | RobotSpec `Kt_Nm_per_A` | Nm/A | motor spec default |
| `Ke` | Derived in Python from `Kv_rpm_per_V` | `derive_runtime_params(...)["Ke_V_per_rad"]` | V/(rad/s) | `1/500` fallback |
| `gear` | Robot motor field | RobotSpec `gear_ratio` | ratio | `1.0` safe fallback |
| `eta` | Robot motor field | RobotSpec or `custom_drivetrain_efficiency` | ratio | `0.9` |
| `b_visc` | Robot motor field | RobotSpec or `custom_viscous_friction` | Nm/(rad/s) | `0.0` |
| `tau_coulomb` | Robot motor field | RobotSpec or `custom_coulomb_friction` | Nm | `0.0` |
| `I_max` | Driver/stall current fallback | RobotSpec or `custom_current_limit_a` | A | `3.0` |
| `mass` | Robot mechanical field | RobotSpec | kg | `0.2` |
| `Jz` | Robot mechanical field or estimate | RobotSpec/derived estimate | kg·m² | `mass*track²/12` |
| `Crr` | Robot mechanical field | RobotSpec | ratio | `0.005` |
| `rho`, `CdA` | Python defaults passed to C | `derive_runtime_params` explicit defaults | kg/m³, m² | `1.225`, `0.02` |
| `max_wheel_accel_mm_s2` | Python basic model only | `custom_max_wheel_accel_mm_s2` or auto | mm/s² | `9810.0` |
| `slip_ratio_left/right` | absent | `custom_slip_ratio_left/right` | ratio | `0.0` |
| `battery_capacity_mah`, `battery_soc` | absent from C | `custom_battery_capacity_mah`, `custom_battery_soc_initial` | mAh, ratio | `1000.0`, `1.0` |
| `encoder_ticks_per_rev` | absent from C | `custom_encoder_ticks_per_rev` | ticks/rev | `1024` |

The old `step_motor_drivetrain_C` is preserved for compatibility. New code should prefer `step_physics_modular_C` when the loaded native library exports it.
