#!/usr/bin/env python3
"""Interactive UART console for the YYT drive-side debug firmware."""

from __future__ import annotations

import argparse
import glob
import sys
import threading
import time
from pathlib import Path

import serial


def find_port() -> str:
    ports = sorted(
        glob.glob("/dev/cu.usbserial*")
        + glob.glob("/dev/cu.usbmodem*")
        + glob.glob("/dev/tty.usbserial*")
        + glob.glob("/dev/tty.usbmodem*")
    )
    if not ports:
        raise SystemExit("No USB serial port found. Connect USB-TTL to YYT USART1 PB6/PB7.")
    return ports[0]


def default_log_path() -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    path = Path("logs") / f"yyt_drive_uart_{stamp}.txt"
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="USB-TTL serial port, for example /dev/cu.usbserial-110")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--log", type=Path, default=default_log_path())
    args = parser.parse_args()

    port = args.port or find_port()
    stop = threading.Event()

    with serial.Serial(port, args.baud, timeout=0.1) as ser, args.log.open("a", encoding="utf-8") as log:
        print(f"Connected: {port} @ {args.baud}")
        print(f"Logging: {args.log}")
        print("Type commands, Ctrl-C to quit. Start with: status")

        def reader() -> None:
            while not stop.is_set():
                data = ser.read(4096)
                if not data:
                    continue
                text = data.decode("utf-8", errors="replace")
                print(text, end="", flush=True)
                log.write(text)
                log.flush()

        thread = threading.Thread(target=reader, daemon=True)
        thread.start()

        try:
            for line in sys.stdin:
                ser.write(line.encode("utf-8"))
                log.write(f">>> {line}")
                log.flush()
        except KeyboardInterrupt:
            pass
        finally:
            try:
                ser.write(b"stop\n")
                ser.write(b"disarm\n")
            finally:
                stop.set()
                thread.join(timeout=0.5)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
