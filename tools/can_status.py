#!/usr/bin/env python3
"""Query the ESP32C3 YYT CAN zero-calibrator serial status."""

from __future__ import annotations

import argparse
import glob
import sys
import time

try:
    import serial
except ImportError as exc:
    raise SystemExit("pyserial is required: python3 -m pip install pyserial") from exc


def choose_port(explicit: str | None) -> str:
    if explicit:
        return explicit
    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*"))
    if not ports:
        raise SystemExit("No USB serial port found under /dev/cu.usbmodem* or /dev/cu.usbserial*")
    return ports[0]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Serial port, for example /dev/cu.usbmodem1301")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--wait", type=float, default=3.0, help="Seconds to wait before sending status")
    parser.add_argument("--read", type=float, default=4.0, help="Seconds to read after sending status")
    args = parser.parse_args()

    port = choose_port(args.port)
    print(f"port: {port}")

    with serial.Serial(port, args.baud, timeout=0.3) as ser:
        ser.setDTR(False)
        ser.setRTS(False)
        time.sleep(args.wait)
        ser.write(b"status\n")
        end = time.time() + args.read
        while time.time() < end:
            line = ser.readline()
            if line:
                print(line.decode(errors="replace").rstrip())

    return 0


if __name__ == "__main__":
    sys.exit(main())
