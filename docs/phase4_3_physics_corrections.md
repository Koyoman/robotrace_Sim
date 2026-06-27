# Fase 4.3 — correções conceituais da física realista/custom

Esta fase corrige a interpretação física de corrente, encoder, IMU e slip no modo `realistic/custom`, mantendo compatibilidade com `control_step(state)` e com os perfis `ideal`, `basic`, `realistic` e `custom`.

## 1. Corrente do motor

A simulação agora separa duas grandezas:

| Campo | Unidade | Sinal | Uso |
|---|---:|---:|---|
| `motor_left_current_signed_a` / `motor_right_current_signed_a` | A | assinado | cálculo interno de torque |
| `current_left_a` / `current_right_a` | A | sempre `>= 0` | consumo exposto no state/log |
| `motor_left_current_a` / `motor_right_current_a` | A | sempre `>= 0` | módulo da corrente do motor |
| `battery_current_a` / `current_total_a` | A | sempre `>= 0` | consumo total da bateria |
| `battery_power_w` | W | sempre `>= 0` | potência consumida da bateria |

A corrente assinada usa o modelo quase-estático/RL:

```text
duty = clamp(pwm, pwm_min, pwm_max) -> [-1, 1]
V_terminal ≈ OCV(SOC) - I_batt_prev * (R_batt + R_wiring)
V_motor = duty * max(0, V_terminal - V_driver_drop)
E = Ke * omega_motor
I_target = (V_motor - E) / R_motor
I_next = I_prev + (1 - exp(-dt * R_motor / L_motor)) * (I_target - I_prev)
```

Se `L_motor_H <= 0`, a corrente vai diretamente para `I_target`. A corrente assinada é limitada por `driver_current_limit_A`, ou por `stallCurrent_A` quando o limite do driver não é informado.

O consumo exposto usa sempre o módulo:

```text
current_left_a  = abs(motor_left_current_signed_a)
current_right_a = abs(motor_right_current_signed_a)
battery_current_a = current_left_a + current_right_a
battery_power_w = battery_voltage_v * battery_current_a
```

Não há regeneração real nesta etapa. Em frenagem/reversão, energia mecânica negativa é contabilizada como `brake_dissipated_power_w`.

## 2. Reversão de polaridade

Quando a roda/motor gira para frente (`omega_motor > 0`) e o PWM manda ré (`duty < 0`):

```text
V_motor < 0
E > 0
V_motor - E fica mais negativo
abs(I) aumenta
```

Isso aumenta `current_left_a/current_right_a`, gera torque oposto e registra a corrente assinada negativa apenas nos campos `*_signed_a`.

## 3. Encoder

O encoder passou a medir a rotação da roda/pneu antes do slip, não a velocidade efetiva no chão.

Campos separados:

| Campo | Significado |
|---|---|
| `wheel_left_surface_speed_mm_s` / `wheel_right_surface_speed_mm_s` | velocidade periférica da roda antes do slip |
| `ground_left_speed_mm_s` / `ground_right_speed_mm_s` | velocidade efetiva aplicada à cinemática do robô |
| `v_left_mm_s` / `v_right_mm_s` | compatibilidade: velocidade efetiva no chão |

Com slip:

```text
ground_speed = wheel_surface_speed * (1 - slip_ratio)
encoder_rad_s = wheel_surface_speed / wheel_radius_mm
```

Os campos principais de ticks são inteiros:

```text
enc_left_ticks: int
enc_right_ticks: int
enc_left_delta_ticks: int
enc_right_delta_ticks: int
```

A acumulação fracionária fica interna no modelo.

## 4. IMU no referencial do corpo

A IMU agora usa o referencial preso ao robô:

```text
X_body: longitudinal/frente do robô
Y_body: lateral
Z_body: yaw
```

Campos:

```text
imu_omega_rad_s = omega_rad_s + bias + noise
imu_alpha_rad_s2 = alpha_rad_s2 + noise
imu_accel_x_mm_s2 = a_lin_mm_s2 + noise
imu_accel_y_mm_s2 = v_mm_s * omega_rad_s + noise
```

Não há rotação por `heading_deg` nos campos `imu_accel_x/y`, porque uma IMU real acompanha o corpo do robô. A mudança do heading global não deve alterar a leitura para o mesmo movimento relativo.

## 5. Slip por torque/força disponível

No `realistic`, o slip deixou de ser fixo. Ele é calculado por roda a partir da força demandada e do limite de atrito.

```text
N_per_wheel = mass_kg * g / 2
F_static_max = mu_static * N_per_wheel
F_kinetic_max = mu_kinetic * N_per_wheel
F_command = wheel_torque / wheel_radius
F_rr = Crr * N_per_wheel * sign(speed)
F_drive = F_command - F_rr
```

Critério:

```text
if abs(F_drive) <= F_static_max:
    slip_ratio = 0
    F_ground = F_drive
else:
    F_ground = sign(F_drive) * F_kinetic_max
    slip_ratio = clamp(1 - abs(F_ground) / abs(F_drive), 0, 0.95)
```

No `custom`, `custom_use_manual_slip=True` força o uso de `custom_slip_ratio_left/right`. Por compatibilidade com configurações antigas, o perfil `custom` também interpreta `custom_slip_ratio_left/right` não nulos como slip manual. O preset `realistic` nunca força slip fixo por padrão.

## 6. Dados do `robot-spec.json` usados

### `geometric_mechanical`

Usado em cinemática, encoder e slip:

- `wheel_radius_mm`
- `track_mm`
- `mass_kg`
- `J_body_kgm2`, com fallback quando zero
- `mu_static`
- `mu_kinetic`
- `Crr`
- `cg_origin_xy_mm` / `rot_origin_xy_mm` permanecem carregados/preparados para usos futuros de dinâmica de massa distribuída.

### `electrical`

Usado em bateria/corrente:

- `batteryVoltageV`
- `batteryCapacitymAh`
- `R_batt_ohm`
- `wiring_R_ohm`
- `driver_drop_V`
- `battery_OCV_table`, interpolada por SOC.

### `motor_transmission`

Usado no modelo elétrico e de torque:

- `gear_ratio`
- `eta`
- `R_motor_ohm`
- `L_motor_H`
- `Kv_rpm_per_V`
- `Kv_rad_per_V`
- `Kt_Nm_per_A`
- `I0_noLoad_A`
- `b_visc_Nm_per_radps`
- `tau_coulomb_Nm`
- `J_motor_kgm2`
- `J_load_kgm2`
- `stallCurrent_A`
- `driver_current_limit_A`

### `controller`

Usado em duty/deadband/dt:

- `pwm_min`
- `pwm_max`
- `pwm_neutral`
- `deadband_percent`
- `pwm_resolution_bits` fica disponível no spec
- `pwm_frequency_Hz` fica disponível no spec; ripple PWM ainda não é modelado
- `simulation_step_dt_ms`

### `sensorsConfig`

Usado no pipeline de sensores:

- `sensor_mode`
- `sensor_bits`
- `value_of_line`
- `value_of_background`
- `analog_noise_line`
- `analog_noise_background`

Os valores são clampados para `0 .. 2^sensor_bits - 1`.

### `encoders`

Usado quando encoder está habilitado:

- `enable`
- `ppr`
- `noise_std_pulses`
- `update_rate_Hz`
- `resolution_bits` fica preparado/documentado; limitação de rollover por bits ainda não é modelada.

### `imu`

Usado quando IMU está habilitada:

- `enable`
- `std_deg`
- `bias_deg_s`
- `update_rate_Hz`
- `latency_ms` fica preparado/documentado; a latência real ainda não possui buffer dedicado nesta etapa.

### `odometry`

O bloco continua carregado e separado de encoder bruto. A odometria com bias/ruído fica preparada para uma próxima fase para evitar misturar estimativa odométrica com telemetria real de encoder.

## 7. Campos novos/relevantes no log

| Campo | Unidade | Descrição |
|---|---:|---|
| `duty_left/right` | 1 | duty normalizado após deadband |
| `battery_current_a` | A | consumo total da bateria |
| `motor_*_current_signed_a` | A | corrente interna assinada/debug |
| `motor_*_current_a` | A | módulo da corrente do motor |
| `motor_*_voltage_v` | V | tensão aplicada assinada |
| `motor_*_back_emf_v` | V | back-EMF assinada |
| `motor_*_torque_nm` | Nm | torque líquido no eixo do motor |
| `wheel_*_torque_nm` | Nm | torque na roda após transmissão |
| `mechanical_power_*_w` | W | potência mecânica na roda |
| `brake_dissipated_power_w` | W | energia de frenagem dissipada |
| `wheel_*_surface_speed_mm_s` | mm/s | velocidade periférica da roda antes do slip |
| `ground_*_speed_mm_s` | mm/s | velocidade efetiva no chão |
| `traction_force_*_n` | N | força efetiva calculada no contato |
| `max_static_force_*_n` | N | limite de atrito estático por roda |
| `physics_backend` | texto | `python_dc`, `python_kinematic`, `c_modular`, `c_legacy` |
| `linesim_abi_version` | inteiro | ABI reportada pelo backend C, quando usado |

## 8. Backend C / ctypes

A ABI C foi marcada como versão 3 em `linesim_abi_version_C()`. O layout principal de `PhysicsConfigC`/`PhysicsStateC` foi mantido para não quebrar a DLL existente, mas a finalização física de consumo, slip, encoder e IMU agora acontece em Python de forma comum a todos os backends.

Quando o C modular está disponível, ele continua sendo usado para a etapa base. O Python aplica a etapa final para garantir:

- consumo não negativo;
- encoder pré-slip;
- IMU no corpo;
- slip calculado por força/torque;
- campos de log consistentes.

## 9. Sanity checks

- `current_left_a`, `current_right_a`, `current_total_a`, `battery_current_a` e `battery_power_w` nunca devem ser negativos.
- Com roda girando para frente e PWM reverso, `current_left_a/right_a` deve aumentar.
- Com `slip_ratio > 0`, `enc_left_rad_s * wheel_radius_mm` deve acompanhar `wheel_left_surface_speed_mm_s`, não `ground_left_speed_mm_s`.
- Em curva, `imu_accel_y_mm_s2 ≈ v_mm_s * omega_rad_s`.
- Mudar apenas `heading_deg` inicial não deve mudar a IMU para o mesmo movimento relativo.

## 10. Limitações conhecidas

- O modelo de pneu ainda é simplificado e longitudinal.
- Não há regeneração real para a bateria.
- PWM ripple/frequência ainda não é modelado eletricamente.
- `latency_ms` de IMU/sensores está documentado, mas ainda não tem buffer dedicado completo.
- `resolution_bits` do encoder ainda não modela rollover/overflow.
- O bloco `odometry` está carregado, mas uma estimativa odométrica separada fica para a próxima fase.
