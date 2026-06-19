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
    parser = argparse.ArgumentParser(description="Hold one manual motor command long enough for ST-Link RAM sampling.")
    parser.add_argument("--port", help="Serial port, for example /dev/cu.usbmodem11301")
    parser.add_argument("--id", type=int, required=True, choices=(1, 2, 5, 6))
    parser.add_argument("--mv", type=int, required=True)
    parser.add_argument("--seconds", type=float, default=12.0)
    parser.add_argument("--settle", type=float, default=3.0)
    parser.add_argument("--status-every", type=float, default=0.0,
                        help="Seconds between status queries while holding; 0 disables.")
    parser.add_argument("--cantx-every", type=float, default=0.0,
                        help="Seconds between cantx queries while holding; 0 disables.")
    args = parser.parse_args()

    ser = serial.Serial()
    ser.port = args.port or choose_port()
    ser.baudrate = 115200
    ser.timeout = 0.1
    ser.dtr = True
    ser.rts = False

    print(f"Connecting safely to: {ser.port}", flush=True)
    ser.open()
    ser.setDTR(True)
    ser.setRTS(False)

    try:
        wait_for_controller_ready(ser, timeout=max(args.settle, 8.0))
        ser.write(b"arm\n")
        time.sleep(0.1)
        ser.write(f"v {args.id} {args.mv}\n".encode("utf-8"))
        print(f"READY_FOR_STLINK id={args.id} mv={args.mv} seconds={args.seconds}", flush=True)
        if args.status_every <= 0.0 and args.cantx_every <= 0.0:
            read_for(ser, args.seconds)
        else:
            deadline = time.time() + args.seconds
            next_status = time.time() if args.status_every > 0.0 else float("inf")
            next_cantx = time.time() if args.cantx_every > 0.0 else float("inf")
            while time.time() < deadline:
                now = time.time()
                if now >= next_status:
                    ser.write(b"status\n")
                    next_status += args.status_every
                if now >= next_cantx:
                    ser.write(b"cantx\n")
                    next_cantx += args.cantx_every
                line = ser.readline()
                if line:
                    print(line.decode("utf-8", errors="replace").strip(), flush=True)
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
