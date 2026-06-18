#!/usr/bin/env python3
"""Guided helper for confirming directions and monitoring height hold."""

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

from serial_log import TeeLogger, default_log_path


TESTED_RE = re.compile(r"ID(?P<id>\d+).*tested=(?P<tested>yes|no)")


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


def parse_tested(lines: list[str]) -> dict[int, bool]:
    result: dict[int, bool] = {}
    for line in lines:
        match = TESTED_RE.search(line)
        if match:
            result[int(match.group("id"))] = match.group("tested") == "yes"
    return result


def all_tested(tested: dict[int, bool]) -> bool:
    return all(tested.get(motor_id, False) for motor_id in (1, 2, 5, 6))


def stop_safely(ser: serial.Serial, logger: TeeLogger) -> None:
    for command in ("holdoff", "stop", "disarm"):
        try:
            send_and_read(ser, command, 0.4, logger)
        except Exception:
            pass


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Serial port, for example /dev/cu.usbmodem1301")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--height", type=float, default=100.0)
    parser.add_argument("--duration", type=float, default=8.0, help="Seconds to monitor height hold")
    parser.add_argument("--yes", action="store_true", help="Do not pause before confirm_dirs/height")
    parser.add_argument("--log", type=Path, help="Log file path. Defaults to logs/height_hold_*.txt")
    parser.add_argument("--no-log", action="store_true", help="Disable log file creation")
    args = parser.parse_args()

    log_path = Path("/dev/null") if args.no_log else (args.log or default_log_path("height_hold"))
    logger = TeeLogger(log_path)
    port = choose_port(args.port)
    logger.print(f"port: {port}")
    if not args.no_log:
        logger.print(f"log: {log_path}")
    logger.print("Safety: use only after four joint tests are done and directions are physically correct.")

    try:
        with serial.Serial(port, args.baud, timeout=0.25) as ser:
            ser.setDTR(True)
            ser.setRTS(False)
            time.sleep(2.0)

            send_and_read(ser, "can", 0.8, logger)
            status_lines = send_and_read(ser, "status", 1.2, logger)
            dirs_lines = send_and_read(ser, "dirs", 1.2, logger)
            tested = parse_tested(status_lines + dirs_lines)
            if not all_tested(tested):
                logger.print("\nRefusing to continue: not all ID1/2/5/6 are tested=yes.")
                logger.print("Run tools/leg_test_sequence.py first, fix directions if needed, then retry.")
                return 2

            if not args.yes:
                input("\nConfirm all four directions are correct. Press Enter to run confirm_dirs. ")
            send_and_read(ser, "confirm_dirs", 1.0, logger)

            if not args.yes:
                input(f"\nPress Enter to start height {args.height:.1f} mm. Ctrl-C to stop. ")

            try:
                send_and_read(ser, f"height {args.height:.1f}", 1.0, logger)
                end = time.time() + args.duration
                while time.time() < end:
                    send_and_read(ser, "can", 0.6, logger)
                    send_and_read(ser, "status", 1.0, logger)
                    time.sleep(0.4)
            except KeyboardInterrupt:
                logger.print("\nInterrupted by user.")
            finally:
                logger.print("\nStopping height hold and disarming.")
                stop_safely(ser, logger)

        logger.print("\nDone. Send the monitor output to Codex for review.")
        if not args.no_log:
            logger.print(f"Saved log: {log_path}")
        return 0
    finally:
        logger.close()


if __name__ == "__main__":
    sys.exit(main())
