# controller_example.py — Line Follower PD com thresholds e pesos definidos
# Formato do state (enviado pelo simulador):
#  - t_ms
#  - pose:   {x_mm, y_mm, heading_deg}
#  - vel:    {v_mm_s, omega_rad_s}
#  - accel:  {a_lin_mm_s2, alpha_rad_s2}
#  - sensors:{values: [0..255]}
#  - wheels: {v_left_mm_s, v_right_mm_s}   # lido mas não utilizado aqui

G = {"prev_err": 0.0, "d_filt": 0.0, "last_center": 0.0}

# PWM base e ganhos (ajuste conforme necessário)
BASE_PWM = 1850
KP = 1300.0
KD = 220.0
D_ALPHA = 0.25      # filtro exponencial p/ derivada
PWM_MIN, PWM_MAX = 0, 4095
DT_S = 0.001

# Limiares com histerese (linha = valores BAIXOS)
TH_LO = 120.0       # <= em cima da linha
TH_HI = 160.0       # >= fora da linha

def _clamp(v: float) -> int:
    return int(PWM_MIN if v < PWM_MIN else PWM_MAX if v > PWM_MAX else v)

def _activation(val: float) -> float:
    """Mapeia leitura [0..255] para ativação [0..1] (1 = em cima da linha)."""
    if val <= TH_LO: return 1.0
    if val >= TH_HI: return 0.0
    # transição suave reduz efeito do ruído
    return 1.0 - (val - TH_LO) / (TH_HI - TH_LO)

def _weights_signed_even(n: int, step: float = 1000.0):
    """
    Para n par, cria pesos sem zero: ...,-3,-2,-1,+1,+2,+3,... (× step)
    Ex.: n=8 -> [-4,-3,-2,-1,+1,+2,+3,+4] * 1000
    Para n ímpar, centraliza com zero: ...,-2,-1,0,+1,+2,... (× step)
    """
    if n <= 0: return []
    half = n // 2
    if n % 2 == 0:
        # desloca para não ter zero
        out = []
        for i in range(n):
            k = i - half
            if i >= half:
                k += 1
            out.append(k * step)
        return out
    else:
        return [ (i - half) * step for i in range(n) ]

def control_step(state):
    # velocidades de roda estão disponíveis, mas não usadas aqui
    _ = state.get("wheels", {})

    if int(state.get("t_ms", 0)) == 0:
        G["prev_err"] = 0.0
        G["d_filt"] = 0.0
        G["last_center"] = 0.0

    vals = list(state.get("sensors", {}).get("values", []))
    n = len(vals)

    # ativações [0..1]
    acts = [_activation(v) for v in vals]

    # pesos — para n=8 resultará: [-4000,-3000,-2000,-1000, +1000,+2000,+3000,+4000]
    W = _weights_signed_even(n, step=1000.0)

    # centroide assinado
    sw = sum(acts)
    if sw <= 1e-9:
        err = G["last_center"]          # se “perdeu” a linha, mantém direção anterior
    else:
        center = sum(w*a for w, a in zip(W, acts)) / sw
        G["last_center"] = center
        err = center                     # alvo é 0

    # derivada filtrada
    d_raw = (err - G["prev_err"]) / DT_S
    G["d_filt"] = (1.0 - D_ALPHA) * G["d_filt"] + D_ALPHA * d_raw
    G["prev_err"] = err

    # PD
    diff = KP * err + KD * G["d_filt"]

    # zona morta suave perto do centro
    if abs(err) < 500.0:
        diff *= 0.6

    pwm_left  = _clamp(BASE_PWM - diff)
    pwm_right = _clamp(BASE_PWM + diff)
    return {"pwm_left": pwm_left, "pwm_right": pwm_right}
