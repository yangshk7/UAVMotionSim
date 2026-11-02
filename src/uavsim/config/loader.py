from pathlib import Path

import yaml

from .validator import Config


def load_config(path: str | Path) -> Config:
    """读取 YAML；extends 相对于当前文件合并，支持实验覆盖，检测循环。"""

    def read(p: Path, seen: set[Path]) -> dict:
        p = p.resolve()
        if p in seen:
            raise ValueError("Cyclic configuration extends")
        data = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
        base = data.pop("extends", None)
        return merge(read(p.parent / base, seen | {p}), data) if base else data

    return Config.model_validate(read(Path(path), set()))


def merge(base: dict, override: dict) -> dict:
    result = base.copy()
    for key, value in override.items():
        result[key] = (
            merge(result[key], value)
            if isinstance(value, dict) and isinstance(result.get(key), dict)
            else value
        )
    return result


def save_config(config: Config, path: str | Path) -> None:
    Path(path).write_text(yaml.safe_dump(config.model_dump(mode="json"), sort_keys=False), encoding="utf-8")
