"""Offline-only path validation, adapted from integration-agent autotune_cli."""
from pathlib import Path

def offline_path(value: str | Path) -> Path:
    text = str(value)
    if "://" in text or text.startswith(("COM", "\\\\.\\")):
        raise ValueError("M1 accepts offline files only; device/URI paths are forbidden")
    path = Path(value).expanduser().resolve()
    if path == Path("/dev") or Path("/dev") in path.parents:
        raise ValueError("M1 accepts offline files only; /dev paths are forbidden")
    if path.exists() and not (path.is_file() or path.is_dir()):
        raise ValueError("M1 accepts regular offline files/directories only; special files are forbidden")
    return path
