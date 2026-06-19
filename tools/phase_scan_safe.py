#!/usr/bin/env python3
import argparse
import glob
import sys
import time

import serial

from serial_log import wait_for_controller_ready


def choose_port() -> str:
    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*"))
    if not ports:
        raise SystemExit("No USB serial port found under /dev/cu.usbmodem* or /dev/cu.usbserial*")
    return ports[0]


def read_for(ser: serial.Serial, seconds: float) -> None:
    start = time.time()
    while time.time() - start < seconds:
        line = ser.readline()
        if line:
            print(line.decode("utf-8", errors="replace").strip(), flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run YYT fixed electrical-angle phase scan pulses safely.")
    parser.add_argument("--port", help="Serial port, for example /dev/cu.usbmodem11301")
    parser.add_argument("--id", type=int, default=1, choices=(1, 2, 5, 6))
    parser.add_argument("--ms", type=int, default=120)
    parser.add_argument("--settle", type=float, default=8.0)
    parser.add_argument("--read", type=float, default=0.65)
    parser.add_argument("--negative", action="store_true", help="Also test -7000..-7005.")
    parser.add_argument("--no-reset", action="store_true",
                        help="Do not pulse EN/reset after opening the serial port.")
    args = parser.parse_args()

    port = args.port or choose_port()
    print(f"Connecting safely to: {port}", flush=True)
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 115200
    ser.timeout = 0.1
    # On this ESP32-C3 board, DTR=False can hold BOOT low and enter ROM download mode.
    ser.dtr = True
    ser.rts = False
    ser.open()
    ser.setDTR(True)
    ser.setRTS(False)

    try:
        if not args.no_reset:
            ser.setRTS(True)
            time.sleep(0.12)
            ser.setRTS(False)
        wait_for_controller_ready(ser, timeout=max(args.settle, 8.0))
        ser.write(b"status\n")
        read_for(ser, 1.0)
        ser.write(b"arm\n")
        read_for(ser, 0.2)

        commands = list(range(7000, 7006))
        if args.negative:
            commands.extend(range(-7000, -7006, -1))

        for mv in commands:
            command = f"test {args.id} {mv} {args.ms}\n"
            print(f"PHASE_SCAN {command.strip()}", flush=True)
            ser.write(command.encode("utf-8"))
            read_for(ser, args.read)
            ser.write(b"stop\n")
            read_for(ser, 0.15)
    finally:
        try:
            ser.write(b"stop\n")
            time.sleep(0.05)
            ser.write(b"disarm\n")
            print("STOP_DISARM_SENT", flush=True)
        finally:
            ser.setDTR(True)
            ser.setRTS(False)
            ser.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
