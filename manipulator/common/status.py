from __future__ import annotations

import json
from typing import Any


def encode_status(**values: Any) -> str:
    return json.dumps(values, ensure_ascii=False, separators=(",", ":"))


def decode_message(text: str) -> dict[str, Any]:
    text = text.strip()
    if not text:
        return {}
    try:
        value = json.loads(text)
    except json.JSONDecodeError:
        return {"command": text}
    return value if isinstance(value, dict) else {"value": value}

