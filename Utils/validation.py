from __future__ import annotations

from typing import Any, Iterable


class ValidationError(Exception):
    """Erro de validação com mensagem principal e lista de detalhes amigáveis."""

    def __init__(self, message: str, errors: list[str] | None = None):
        self.message = message
        self.errors = errors or []
        details = "\n".join(f"- {e}" for e in self.errors)
        super().__init__(message if not details else f"{message}\n{details}")


def _fmt(path: str, key: str | None = None) -> str:
    return f"{path}.{key}" if key else path


def require_key(obj: dict[str, Any], key: str, path: str = "$", errors: list[str] | None = None) -> Any:
    if not isinstance(obj, dict) or key not in obj:
        msg = f"Campo obrigatório ausente: {_fmt(path, key)}"
        if errors is not None:
            errors.append(msg)
            return None
        raise ValidationError("JSON inválido.", [msg])
    return obj[key]


def as_float(value: Any, path: str, default: float | None = None, errors: list[str] | None = None) -> float:
    if value is None and default is not None:
        return float(default)
    try:
        return float(value)
    except (TypeError, ValueError):
        msg = f"{path} deve ser número. Valor recebido: {value!r}"
        if errors is not None:
            errors.append(msg)
            return float(default or 0.0)
        raise ValidationError("JSON inválido.", [msg])


def as_int(value: Any, path: str, default: int | None = None, errors: list[str] | None = None) -> int:
    if value is None and default is not None:
        return int(default)
    try:
        return int(value)
    except (TypeError, ValueError):
        msg = f"{path} deve ser inteiro. Valor recebido: {value!r}"
        if errors is not None:
            errors.append(msg)
            return int(default or 0)
        raise ValidationError("JSON inválido.", [msg])


def as_bool(value: Any, path: str, default: bool | None = None, errors: list[str] | None = None) -> bool:
    if value is None and default is not None:
        return bool(default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        v = value.strip().lower()
        if v in {"true", "1", "yes", "sim"}:
            return True
        if v in {"false", "0", "no", "nao", "não"}:
            return False
    if isinstance(value, (int, float)):
        return bool(value)
    msg = f"{path} deve ser booleano. Valor recebido: {value!r}"
    if errors is not None:
        errors.append(msg)
        return bool(default or False)
    raise ValidationError("JSON inválido.", [msg])


def validate_range(value: float, path: str, *, min_value: float | None = None, max_value: float | None = None,
                   inclusive_min: bool = True, inclusive_max: bool = True,
                   errors: list[str] | None = None) -> None:
    ok = True
    if min_value is not None:
        ok = ok and (value >= min_value if inclusive_min else value > min_value)
    if max_value is not None:
        ok = ok and (value <= max_value if inclusive_max else value < max_value)
    if ok:
        return
    left = "-∞" if min_value is None else ("[" if inclusive_min else "(") + str(min_value)
    right = "+∞" if max_value is None else str(max_value) + ("]" if inclusive_max else ")")
    msg = f"{path} fora da faixa permitida {left}, {right}. Valor: {value!r}"
    if errors is not None:
        errors.append(msg)
        return
    raise ValidationError("JSON inválido.", [msg])


def validate_enum(value: Any, path: str, allowed: Iterable[Any], errors: list[str] | None = None) -> None:
    allowed_list = list(allowed)
    if value in allowed_list:
        return
    msg = f"{path} deve ser um de {allowed_list}. Valor recebido: {value!r}"
    if errors is not None:
        errors.append(msg)
        return
    raise ValidationError("JSON inválido.", [msg])


def raise_if_errors(title: str, errors: list[str]) -> None:
    if errors:
        raise ValidationError(title, errors)
