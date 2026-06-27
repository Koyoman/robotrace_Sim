# Fase 4.1 — revalidação das fórmulas físicas do modo realistic

Este documento registra a revisão feita após observar que o perfil `realistic` fazia o robô completar um percurso muito mais lentamente que `ideal` e `basic`.

## Resultado da revisão

Foram encontrados dois problemas de implementação, não de conceito físico:

1. **Bateria:** a tensão terminal, já reduzida por resistência interna, era reutilizada como se fosse a tensão de circuito aberto no passo seguinte. Isso causava queda recursiva artificial da tensão até `battery_min_voltage_v`.
2. **Slip:** o slip de 1% era aplicado diretamente sobre a velocidade que voltava para o modelo do motor. Como a simulação roda com `dt = 1 ms`, isso virava uma redução multiplicativa a cada passo, equivalente a amortecimento artificial muito maior que 1%.

Correção aplicada:

- O motor agora recebe uma tensão de fonte/OCV calculada a partir do SOC.
- A tensão terminal continua sendo reportada no state, mas não é usada como OCV no passo seguinte.
- O slip agora altera apenas a velocidade efetiva do robô no chão.
- A velocidade periférica da roda usada pelo motor/back-EMF fica em estado interno separado.
- Encoders usam a velocidade da roda antes do slip, como em um encoder real de eixo/roda.

## Fórmulas usadas

### 1. Cinemática diferencial

Para robô diferencial:

```text
v = (v_r + v_l) / 2
omega = (v_r - v_l) / L
x[k+1] = x[k] + v cos(theta + omega dt / 2) dt
y[k+1] = y[k] + v sin(theta + omega dt / 2) dt
theta[k+1] = theta[k] + omega dt
```

Onde:

- `v_l`, `v_r`: velocidades lineares das rodas no contato com o chão;
- `L`: distância entre rodas;
- `theta`: heading do robô.

Referência principal: Thomas Hellström, *Kinematics Equations for Differential Drive and Articulated Steering*, Umeå University, 2011. O mesmo conjunto de relações aparece também nas notas de G. W. Lucas/Rossum, *A Tutorial and Elementary Trajectory Model for the Differential Steering System*.

### 2. Motor DC

Modelo elétrico usado:

```text
V_a = L di/dt + R i + K_e omega_m
```

Modelo de torque:

```text
T_m = K_t i - b omega_m - tau_c sign(omega_m)
```

Transmissão para a roda:

```text
omega_m = G omega_w
T_w = eta G T_m
F_w = T_w / r
```

Onde:

- `V_a`: tensão aplicada na armadura;
- `i`: corrente;
- `R`, `L`: resistência e indutância do motor;
- `K_e`: constante de back-EMF;
- `K_t`: constante de torque;
- `G`: redução;
- `eta`: eficiência;
- `r`: raio da roda.

Referências principais: LiU Automatic Control, *DC-motor modelling and parameter identification*, e P. Wolm et al., *Analysis of a PM DC Motor Model for Application in Feedback Design for Electric Mobility Vehicles*.

### 3. Dinâmica longitudinal simplificada

Força e aceleração longitudinal:

```text
F_net = F_left + F_right - F_roll - F_drag
a = F_net / m
```

Arrasto e rolamento:

```text
F_drag = 0.5 rho CdA v |v|
F_roll = Crr m g sign(v)
```

Limite simples de atrito:

```text
|F_wheel| <= mu N
```

Este ainda é um modelo simplificado. Ele não implementa Pacejka/brush tire completo.

### 4. Bateria

Modelo equivalente simples tipo Rint/Thevenin de ordem zero:

```text
SOC[k+1] = clamp(SOC[k] - I dt / Q, 0, 1)
OCV = V_min + SOC (V_initial - V_min)
V_terminal = max(V_min, OCV - I R_internal)
```

Onde:

- `Q = capacity_mAh * 3.6` em ampere-segundo;
- `OCV` é tensão de circuito aberto;
- `V_terminal` é a tensão sob carga.

O erro corrigido era alimentar o próximo passo com `V_terminal` no lugar de `OCV`.

Referências principais: H. He et al., *Evaluation of Lithium-Ion Battery Equivalent Circuit Models for State of Charge Estimation*, Energies, 2011; K. Movassagh et al., *A Critical Look at Coulomb Counting Approach for State of Charge Estimation in Batteries*, Energies, 2021.

### 5. Slip longitudinal

O slip longitudinal representa a diferença entre a velocidade periférica da roda e a velocidade efetiva no contato com o solo. Para o modelo simples desta simulação:

```text
v_ground_left = v_wheel_left * (1 - slip_left)
v_ground_right = v_wheel_right * (1 - slip_right)
```

A correção importante é que `v_wheel` deve continuar sendo a velocidade da roda/motor, enquanto `v_ground` é usada para integrar a pose do robô. Antes, `v_ground` era realimentada como se fosse `v_wheel`, criando amortecimento acumulado.

Referências principais: S. L. Miller e J. C. Gerdes, *Calculating Longitudinal Wheel Slip and Tire Parameters Using GPS Velocity*, American Control Conference; Hans B. Pacejka, *Tyre and Vehicle Dynamics*.

### 6. Encoder

Encoder incremental:

```text
delta_ticks = delta_theta_wheel * ticks_per_rev / (2 pi)
delta_theta_wheel = v_wheel dt / r
```

Com slip habilitado, o encoder mede rotação da roda, não deslocamento efetivo no solo. Portanto, foi mantido ligado à velocidade de roda antes do slip.

Referência principal: E. Olson, *A Primer on Odometry and Motor Control*, MIT, 2007.

## Teste numérico de sanidade

Com `robot-spec.json`, PWM máximo nos dois motores e `dt = 1 ms`:

- antes da correção, `realistic` estabilizava próximo de **0,77 m/s** com slip default de 1%;
- depois da correção, a velocidade de roda estabiliza perto de **1,93 m/s** e a velocidade efetiva com 1% de slip fica perto de **1,91 m/s**;
- a tensão terminal permanece perto de **7,99 V** em regime, em vez de cair artificialmente para **6,0 V** em poucos milissegundos.

## Comandos executados

```bash
python -m compileall simulator.py robot_editor.py track_editor.py Utils sim
python -m pytest -q
```

Resultado local após correção:

```text
39 passed
```
