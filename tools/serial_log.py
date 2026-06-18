"""Small logging helpers for serial bring-up scripts."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path


def default_log_path(prefix: str) -> Path:
    repo_root = Path(__file__).resolve().parents[1]
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return repo_root / "logs" / f"{prefix}_{stamp}.txt"


class TeeLogger:
    def __init__(self, path: Path):
        self.path = path
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("w", encoding="utf-8")

    def close(self) -> None:
        self._file.close()

    def print(self, message: str = "") -> None:
        print(message)
        self._file.write(message + "\n")
        self._file.flush()
