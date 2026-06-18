#!/usr/bin/env python3
import serial
import time
import sys
import glob

def choose_port() -> str:
    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*"))
    if not ports:
        raise SystemExit("No USB serial port found under /dev/cu.usbmodem* or /dev/cu.usbserial*")
    return ports[0]

def main():
    port = choose_port()
    print(f"Connecting safely to: {port}")
    try:
        ser = serial.Serial(port, 115200, timeout=0.2)
    except Exception as e:
        print(f"Error: {e}")
        return 1

    # Disable hardware flow lines to prevent ESP32 auto-resets
    ser.dtr = False
    ser.rts = False
    time.sleep(1.0)
    
    ser.reset_input_buffer()
    
    # 1. First confirm we are armed
    ser.write(b"arm\n")
    time.sleep(0.1)
    
    # 2. Send the pulse command
    command = "test 6 -6000 300\n"
    print(f"Sending pulse command: {command.strip()}")
    ser.write(command.encode('utf-8'))
    
    # Read response for 2 seconds
    start = time.time()
    while time.time() - start < 2.0:
        line = ser.readline()
        if line:
            print(line.decode('utf-8', errors='replace').strip())
            
    # Ensure lines stay low before closing
    ser.dtr = False
    ser.rts = False
    ser.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())
