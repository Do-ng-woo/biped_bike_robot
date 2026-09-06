from __future__ import annotations

from dataclasses import dataclass
import importlib.util
import math
from pathlib import Path


@dataclass(frozen=True)
class ReferencePostures:
    joint_names: tuple[str, ...]
    ready: tuple[float, ...]
    kneeling: tuple[float, ...]


def load_reference_postures(cfg: dict) -> ReferencePostures:
    posture_cfg = cfg["postures"]
    source = Path(posture_cfg["source"]).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"posture source not found: {source}")
    spec = importlib.util.spec_from_file_location("biped_reference_postures", source)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot import posture source: {source}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    source_names = tuple(
        str(v) for v in getattr(module, posture_cfg["joint_names_symbol"])
    )
    source_ready = tuple(
        float(v) for v in getattr(module, posture_cfg["ready_symbol"])
    )
    source_kneeling = tuple(
        float(v) for v in getattr(module, posture_cfg["kneeling_symbol"])
    )
    if len(source_ready) != len(source_names) or len(source_kneeling) != len(source_names):
        raise ValueError("reference posture length does not match source joint names")
    expected = tuple(cfg["lower_body"]["joint_names"] + cfg["arm"]["follower_joint_names"])
    ready_by_name = dict(zip(source_names, source_ready, strict=True))
    kneeling_by_name = dict(zip(source_names, source_kneeling, strict=True))
    extra_ready = posture_cfg.get("extra_ready", {})
    extra_kneeling = posture_cfg.get("extra_kneeling", {})
    try:
        ready = tuple(
            float(ready_by_name[name] if name in ready_by_name else extra_ready[name])
            for name in expected
        )
        kneeling = tuple(
            float(
                kneeling_by_name[name]
                if name in kneeling_by_name
                else extra_kneeling[name]
            )
            for name in expected
        )
    except KeyError as exc:
        raise ValueError(f"no reference posture value for joint {exc.args[0]}") from exc
    names = expected
    if not all(math.isfinite(value) for value in ready + kneeling):
        raise ValueError("reference posture contains a non-finite value")
    return ReferencePostures(names, ready, kneeling)
