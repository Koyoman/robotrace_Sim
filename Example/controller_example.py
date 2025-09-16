# controller_example.py — line follower that can complete a lap
# Estratégia:
#  - Calcula erro lateral a partir de N sensores (centroide ponderado).
#  - Controlador PD no erro para dividir o PWM entre as rodas.
#  - Modo "busca" simples quando a fita some dos sensores.

# Estado persistente entre chamadas
G = {
    "prev_err": 0.0,
    "lost_ms": 0,
    "last_seen_sign": 1.0,  # +1 linha estava à direita; -1 à esquerda
}

# Parâmetros (pode ajustar se quiser)
BASE_PWM   = 1850         # viés de avanço (0..4095). Bom com v_final ≈ 2.0 m/s
KP         = 1400.0       # ganho proporcional (|err|<=1 → diff ≈ KP)
KD         = 250.0        # ganho derivativo (por 1 ms)
SEARCH_PWM = 950          # avanço durante busca
SEARCH_TURN = 700         # giro extra durante busca
LOSS_THRESH = 0.12        # se soma de pesos < isso → “perdeu a linha”
DT_S       = 0.001        # dt do simulador = 1 ms

def _saturate(pwm):
    if pwm < 0: return 0
    if pwm > 4095: return 4095
    return int(pwm)

def _error_from_sensors(vals):
    # Mapeia 0..255 (com salto perto de ~200) em pesos 0..1 mais suaves
    lo, hi = 60.0, 200.0
    ws = []
    for v in vals:
        if v >= hi:
            w = 1.0
        elif v <= lo:
            w = 0.0
        else:
            w = (v - lo) / (hi - lo)
        ws.append(w)

    n = len(ws)
    if n == 0:
        return 0.0, 0.0

    # posições idealizadas dos sensores de -1 (esquerda) a +1 (direita)
    if n == 1:
        xs = [0.0]
    else:
        step = 2.0 / (n - 1)
        xs = [-1.0 + i*step for i in range(n)]

    sw = sum(ws)
    if sw <= 1e-6:
        return 0.0, 0.0

    cx = sum(x*w for x, w in zip(xs, ws)) / sw
    # Convenção: erro positivo ⇒ linha à DIREITA do centro do robô
    err = cx
    return err, sw

def control_step(state):
    # state: { "t_ms": int, "pose":{...}, "vel":{...}, "sensors":{"values":[...]}}
    vals = list(state.get("sensors", {}).get("values", []))
    t_ms = int(state.get("t_ms", 0))

    # Reset no início da simulação
    if t_ms == 0:
        G["prev_err"] = 0.0
        G["lost_ms"] = 0
        G["last_seen_sign"] = 1.0

    err, sumw = _error_from_sensors(vals)

    # Perdeu/achou linha
    lost = (sumw < LOSS_THRESH)
    if not lost:
        G["last_seen_sign"] = 1.0 if err >= 0 else -1.0
        G["lost_ms"] = 0
    else:
        G["lost_ms"] += 1

    # Derivada
    derr = (err - G["prev_err"]) / DT_S
    G["prev_err"] = err

    if lost:
        # Busca simples: avança pouco e gira pro lado do último visto
        turn = SEARCH_TURN * G["last_seen_sign"]
        left  = SEARCH_PWM - turn
        right = SEARCH_PWM + turn
        return {"pwm_left": _saturate(left), "pwm_right": _saturate(right)}

    # Seguimento de linha PD
    diff = KP * err + KD * derr
    left  = BASE_PWM - diff
    right = BASE_PWM + diff

    # Suaviza quando erro é pequeno (reduz ziguezague)
    if abs(err) < 0.12:
        left  = 0.8*left  + 0.2*BASE_PWM
        right = 0.8*right + 0.2*BASE_PWM

    return {"pwm_left": _saturate(left), "pwm_right": _saturate(right)}
