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
    print(f"Rebooting ESP32-C3 on port: {port}")
    try:
        ser = serial.Serial(port, 115200)
    except Exception as e:
        print(f"Error: {e}")
        return 1
        
    # Reset sequence for ESP32 standard bootloader bypass:
    # RTS=1, DTR=0 (Reset)
    ser.setDTR(False)
    ser.setRTS(True)
    time.sleep(0.1)
    # RTS=0, DTR=0 (Normal boot)
    ser.setRTS(False)
    time.sleep(0.5)
    
    # Read to verify normal startup logs
    ser.timeout = 0.5
    ser.reset_input_buffer()
    start_time = time.time()
    while time.time() - start_time < 2.0:
        line = ser.readline()
        if line:
            print(line.decode('utf-8', errors='replace').strip())
            
    ser.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())
