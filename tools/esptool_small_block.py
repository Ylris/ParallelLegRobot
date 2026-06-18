#!/usr/bin/env python3
import sys
from pathlib import Path

tool_dir = Path(__file__).resolve().parents[1] / ".pio" / "packages" / "tool-esptoolpy"
sys.path.insert(0, str(tool_dir / "_contrib"))
sys.path.insert(0, str(tool_dir))

from esptool import _main
from esptool.targets.esp32c3 import ESP32C3ROM, ESP32C3StubLoader


ESP32C3ROM.FLASH_WRITE_SIZE = 0x100
ESP32C3StubLoader.FLASH_WRITE_SIZE = 0x100


if __name__ == "__main__":
    sys.exit(_main())
