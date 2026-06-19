#!/usr/bin/env python3
"""Guided serial helper for the ESP32C3 four-leg-motor bring-up test."""

from __future__ import annotations

import argparse
import glob
from pathlib import Path
import re
import sys
import time

try:
    import serial
except ImportError as exc:
    raise SystemExit("pyserial is required: python3 -m pip install pyserial") from exc

from serial_log import TeeLogger, default_log_path, wait_for_controller_ready


PULSE_RE = re.compile(r"ID(?P<id>\d+).*dq=(?P<dq>[+-]?\d+\.\d+)")


def choose_port(explicit: str | None) -> str:
    if explicit:
        return explicit
    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*"))
    if not ports:
        raise SystemExit("No USB serial port found under /dev/cu.usbmodem* or /dev/cu.usbserial*")
    return ports[0]


def read_for(ser: serial.Serial, seconds: float, logger: TeeLogger) -> list[str]:
    lines: list[str] = []
    end = time.time() + seconds
    while time.time() < end:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode(errors="replace").rstrip()
        logger.print(line)
        lines.append(line)
    return lines


def send_and_read(ser: serial.Serial, command: str, read_seconds: float, logger: TeeLogger) -> list[str]:
    logger.print(f"\n>>> {command}")
    ser.write((command + "\n").encode())
    return read_for(ser, read_seconds, logger)


def require_enter(prompt: str, assume_yes: bool) -> None:
    if assume_yes:
        return
    input(prompt)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Serial port, for example /dev/cu.usbmodem1301")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--mv", type=int, default=80)
    parser.add_argument("--ms", type=int, default=100)
    parser.add_argument("--yes", action="store_true", help="Do not pause before each motor pulse")
    parser.add_argument("--log", type=Path, help="Log file path. Defaults to logs/leg_test_*.txt")
    parser.add_argument("--no-log", action="store_true", help="Disable log file creation")
    args = parser.parse_args()

    log_path = Path("/dev/null") if args.no_log else (args.log or default_log_path("leg_test"))
    logger = TeeLogger(log_path)
    port = choose_port(args.port)
    logger.print(f"port: {port}")
    if not args.no_log:
        logger.print(f"log: {log_path}")
    logger.print("Safety: motors must be airborne, current limit conservative, hand ready to cut power.")

    dq_by_id: dict[int, float] = {}
    try:
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = args.baud
        ser.timeout = 0.25
        ser.dtr = True
        ser.rts = False
        ser.open()
        try:
            ser.dtr = True
            ser.rts = False
            if not wait_for_controller_ready(ser, logger=logger):
                logger.print("warning: controller ready marker not seen before commands")

            send_and_read(ser, "can", 0.8, logger)
            send_and_read(ser, "status", 1.2, logger)
            send_and_read(ser, "dirs", 1.2, logger)

            require_enter("\nPress Enter to ARM. Ctrl-C to stop. ", args.yes)
            send_and_read(ser, "arm", 0.8, logger)

            for motor_id in (1, 2, 5, 6):
                require_enter(
                    f"\nPress Enter to pulse ID{motor_id} at {args.mv} mV for {args.ms} ms. ",
                    args.yes,
                )
                lines = send_and_read(ser, f"test {motor_id} {args.mv} {args.ms}", 2.2, logger)
                for line in lines:
                    match = PULSE_RE.search(line)
                    if match:
                        dq_by_id[int(match.group("id"))] = float(match.group("dq"))
                        break
                send_and_read(ser, "stop", 0.4, logger)

            send_and_read(ser, "dirs", 1.2, logger)
            send_and_read(ser, "status", 1.2, logger)
        finally:
            ser.dtr = True
            ser.rts = False
            ser.close()

        logger.print("\nSummary:")
        for motor_id in (1, 2, 5, 6):
            if motor_id in dq_by_id:
                logger.print(f"  ID{motor_id}: dq={dq_by_id[motor_id]:+.4f} rad")
            else:
                logger.print(f"  ID{motor_id}: dq=missing")

        logger.print("\nSend this summary to Codex before running confirm_dirs / height 100.")
        if not args.no_log:
            logger.print(f"Saved log: {log_path}")
        return 0
    finally:
        logger.close()


if __name__ == "__main__":
    sys.exit(main())
