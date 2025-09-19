# controller_example.py — Line Follower PD com modo "busca"
# Compatível com simulation.py (state: t_ms, pose, vel, accel, sensors.values)

# Estado persistente entre chamadas
G = {
    "prev_err": 0.0,
    "d_filt": 0.0,
    "lost_ms": 0,
    "last_seen_sign": 1.0,   # +1: linha à direita; -1: à esquerda
}

# ===== Parâmetros ajustáveis =====
# Base de avanço. 1850 funciona bem com v_final ≈ 2.0 m/s do simulador
BASE_PWM    = 1850

# Ganhos PD
KP          = 1300.0         # ganho proporcional
KD          = 220.0          # ganho derivativo (dt=1ms)
D_ALPHA     = 0.25           # suavização da derivada (0..1) — 0 = sem filtro

# Modo busca quando perde a linha
SEARCH_PWM  = 900            # PWM de avanço durante busca
SEARCH_TURN = 750            # PWM diferencial para girar procurando a fita

# Limiar de “linha presente” (soma de pesos normalizados 0..1)
# Quanto maior o número de sensores, maior a soma; esse valor funciona bem para 5~12 sensores.
LOSS_THRESH = 0.12

# Saturação e proteção
PWM_MIN     = 0
PWM_MAX     = 4095

# dt do simulador
DT_S        = 0.001

def _clamp_pwm(v):
    if v < PWM_MIN: return PWM_MIN
    if v > PWM_MAX: return PWM_MAX
    return int(v)

def _weights_from_raw(vals):
    """
    Converte leituras 0..255 para pesos 0..1.
    Pelo simulador: dentro da faixa → 0..100 ; fora → 200..255.
    Mapeamos monotonamente: valores menores => peso maior (mais 'preto').
    Usamos uma rampa entre [lo, hi] para suavizar ruído.
    """
    lo, hi = 60.0, 200.0      # região de transição suave
    ws = []
    for v in vals:
        if v <= lo:
            w = 1.0
        elif v >= hi:
            w = 0.0
        else:
            # entre lo e hi cai linearmente 1→0
            w = 1.0 - (v - lo) / (hi - lo)
        ws.append(w)
    return ws

def _centroid_error(ws):
    """
    Calcula erro lateral normalizado a partir dos pesos dos sensores.
    Considera sensores igualmente espaçados em [-1, +1] na ordem recebida.
    Retorna (erro, soma_pesos).
    """
    n = len(ws)
    if n == 0:
        return 0.0, 0.0

    if n == 1:
        xs = [0.0]
    else:
        step = 2.0 / (n - 1)
        xs = [-1.0 + i * step for i in range(n)]

    sw = sum(ws)
    if sw <= 1e-12:
        return 0.0, 0.0

    cx = sum(x * w for x, w in zip(xs, ws)) / sw
    # Convenção: erro > 0 => linha à DIREITA do centro do robô
    return cx, sw

def control_step(state):
    """
    Entrada esperada:
      state = {
        "t_ms": int,
        "pose": {"x_mm","y_mm","heading_deg"},
        "vel":  {"v_mm_s","omega_rad_s"},
        "accel":{"a_lin_mm_s2","alpha_rad_s2"},
        "sensors": {"values": [ints 0..255]}
      }
    Saída:
      {"pwm_left": int 0..4095, "pwm_right": int 0..4095}
    """
    vals = list(state.get("sensors", {}).get("values", []))
    t_ms = int(state.get("t_ms", 0))

    # Reset ao início de uma simulação
    if t_ms == 0:
        G["prev_err"] = 0.0
        G["d_filt"] = 0.0
        G["lost_ms"] = 0
        G["last_seen_sign"] = 1.0

    # 1) Erro a partir dos sensores
    ws = _weights_from_raw(vals)
    err, sumw = _centroid_error(ws)

    # 2) Perdeu/achou a linha?
    lost = (sumw < LOSS_THRESH)
    if lost:
        G["lost_ms"] += 1
    else:
        G["lost_ms"] = 0
        G["last_seen_sign"] = 1.0 if err >= 0.0 else -1.0

    # 3) Derivada suavizada
    d_raw = (err - G["prev_err"]) / DT_S
    G["d_filt"] = (1.0 - D_ALPHA) * G["d_filt"] + D_ALPHA * d_raw
    G["prev_err"] = err

    # 4) Modo busca se perdido
    if lost:
        # avança e gira para o último lado onde viu a linha
        turn = SEARCH_TURN * G["last_seen_sign"]
        left  = SEARCH_PWM - turn
        right = SEARCH_PWM + turn
        return {"pwm_left": _clamp_pwm(left), "pwm_right": _clamp_pwm(right)}

    # 5) Controlador PD
    diff = KP * err + KD * G["d_filt"]

    # Pequena zona morta na correção para reduzir ziguezague próximo ao centro
    if abs(err) < 0.10:
        diff *= 0.6

    left  = BASE_PWM - diff
    right = BASE_PWM + diff

    # 6) Saturação final
    return {"pwm_left": _clamp_pwm(left), "pwm_right": _clamp_pwm(right)}
