"""Small logging helpers for serial bring-up scripts."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import time
from typing import Optional


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


def emit_line(line: str, logger: Optional[TeeLogger] = None) -> None:
    if logger is None:
        print(line)
    else:
        logger.print(line)


def wait_for_controller_ready(ser, timeout: float = 8.0, logger: Optional[TeeLogger] = None) -> bool:
    """Read startup output until the ESP32-C3 controller has printed an online line."""
    deadline = time.time() + timeout
    saw_app = False
    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode(errors="replace").rstrip()
        emit_line(line, logger)
        if "ParallelLegRobot" in line and "CAN leg controller" in line:
            saw_app = True
        if line.startswith("online:"):
            return True
    return saw_app
