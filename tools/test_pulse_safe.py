#!/usr/bin/env python3
import serial
import time
import sys
import glob
import argparse

from serial_log import wait_for_controller_ready

def choose_port() -> str:
    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*"))
    if not ports:
        raise SystemExit("No USB serial port found under /dev/cu.usbmodem* or /dev/cu.usbserial*")
    return ports[0]

def main():
    parser = argparse.ArgumentParser(description="Send one short single-motor pulse through the ESP32C3 controller.")
    parser.add_argument("--port", help="Serial port, for example /dev/cu.usbmodem11301")
    parser.add_argument("--id", type=int, required=True, choices=(1, 2, 5, 6), help="Drive ID to pulse")
    parser.add_argument("--mv", type=int, required=True, help="Signed motor voltage command in mV")
    parser.add_argument("--ms", type=int, default=80, help="Pulse duration in ms")
    parser.add_argument("--manual", action="store_true", help="Use v <id> <mv> for ms instead of test <id> <mv> <ms>")
    parser.add_argument("--settle", type=float, default=8.0, help="Max seconds to wait for controller startup")
    parser.add_argument("--no-reset", action="store_true",
                        help="Do not pulse EN/reset after opening the serial port.")
    args = parser.parse_args()

    port = args.port or choose_port()
    print(f"Connecting safely to: {port}")
    try:
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = 115200
        ser.timeout = 0.2
        ser.dtr = True
        ser.rts = False
        ser.open()
    except Exception as e:
        print(f"Error: {e}")
        return 1

    try:
        ser.setDTR(True)
        ser.setRTS(False)
        if not args.no_reset:
            ser.setRTS(True)
            time.sleep(0.12)
            ser.setRTS(False)
        if not wait_for_controller_ready(ser, timeout=max(args.settle, 8.0)):
            print("warning: controller ready marker not seen before commands")

        ser.write(b"status\n")
        start = time.time()
        while time.time() - start < 1.2:
            line = ser.readline()
            if line:
                print(line.decode('utf-8', errors='replace').strip())

        ser.write(b"arm\n")
        time.sleep(0.1)

        if args.manual:
            command = f"v {args.id} {args.mv}\n"
            print(f"Sending manual command: {command.strip()} for {args.ms} ms")
            ser.write(command.encode('utf-8'))
            start = time.time()
            while time.time() - start < args.ms / 1000.0:
                line = ser.readline()
                if line:
                    print(line.decode('utf-8', errors='replace').strip())
            ser.write(b"stop\n")
            time.sleep(0.1)
            ser.write(b"status\n")
            read_seconds = 1.5
        else:
            command = f"test {args.id} {args.mv} {args.ms}\n"
            print(f"Sending pulse command: {command.strip()}")
            ser.write(command.encode('utf-8'))
            read_seconds = 2.0

        start = time.time()
        while time.time() - start < read_seconds:
            line = ser.readline()
            if line:
                print(line.decode('utf-8', errors='replace').strip())
    finally:
        try:
            ser.write(b"stop\n")
            time.sleep(0.05)
            ser.write(b"disarm\n")
        except Exception:
            pass
        ser.setDTR(True)
        ser.setRTS(False)
        ser.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())
