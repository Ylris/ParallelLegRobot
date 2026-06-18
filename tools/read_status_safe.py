#!/usr/bin/env python3
import argparse
import serial
import time
import sys
import glob

SAFE_COMMANDS = ("status", "can", "dirs", "stop", "holdoff", "disarm", "zero6")

def choose_port() -> str:
    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*"))
    if not ports:
        raise SystemExit("No USB serial port found under /dev/cu.usbmodem* or /dev/cu.usbserial*")
    return ports[0]

def main():
    parser = argparse.ArgumentParser(description="Safely send one whitelisted command to the ESP32C3 controller.")
    parser.add_argument("--port", help="Serial port, for example /dev/cu.usbmodem11301")
    parser.add_argument("--command", choices=SAFE_COMMANDS, default="status")
    parser.add_argument("--pre-command-delay", type=float, default=1.0,
                        help="Seconds to wait after opening serial before sending the command")
    parser.add_argument("--read", type=float, default=3.0, help="Seconds to read after sending the command")
    args = parser.parse_args()

    port = args.port or choose_port()
    print(f"Connecting safely to: {port}")
    try:
        # Open serial with control lines disabled
        ser = serial.Serial(port, 115200, timeout=0.2)
    except Exception as e:
        print(f"Error: {e}")
        return 1

    # Keep DTR and RTS low to prevent resets
    ser.dtr = False
    ser.rts = False
    time.sleep(args.pre_command_delay)
    
    # Clear buffer and query the requested whitelisted command.
    ser.reset_input_buffer()
    ser.write((args.command + "\n").encode())
    
    # Read output for the requested interval.
    start = time.time()
    while time.time() - start < args.read:
        line = ser.readline()
        if line:
            print(line.decode('utf-8', errors='replace').strip())
            
    # Explicitly ensure control lines are low before closing
    ser.dtr = False
    ser.rts = False
    ser.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())
