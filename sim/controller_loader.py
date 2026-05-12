from __future__ import annotations

import importlib.util
import os
from collections.abc import Callable


class ControllerLoadError(Exception):
    """Raised when a Python controller cannot be loaded or does not expose control_step."""


def load_controller(path: str) -> Callable[[dict], dict]:
    if not path:
        raise ControllerLoadError("Nenhum arquivo de controller foi informado.")
    if not os.path.exists(path):
        raise ControllerLoadError(f"Controller não encontrado: {path}")
    if os.path.splitext(path)[1].lower() != ".py":
        raise ControllerLoadError("Somente controllers .py são suportados nesta fase.")

    try:
        module_name = f"robotrace_controller_{abs(hash(os.path.abspath(path)))}"
        spec = importlib.util.spec_from_file_location(module_name, path)
        if spec is None or spec.loader is None:
            raise ControllerLoadError(f"Não foi possível preparar importação de {path}")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    except ControllerLoadError:
        raise
    except Exception as exc:
        raise ControllerLoadError(f"Erro ao carregar controller '{path}': {exc}") from exc

    fn = getattr(mod, "control_step", None)
    if fn is None:
        raise ControllerLoadError("Controller deve definir a função control_step(state).")
    if not callable(fn):
        raise ControllerLoadError("control_step existe, mas não é callable.")
    return fn
