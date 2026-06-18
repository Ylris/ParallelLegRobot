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
    print(f"Opening port: {port}")
    try:
        ser = serial.Serial(port, 115200, timeout=0.5)
    except Exception as e:
        print(f"Error opening port: {e}")
        return 1
        
    ser.setDTR(False)
    ser.setRTS(False)
    
    # Wait for serial to settle
    time.sleep(1.5)
    ser.reset_input_buffer()
    
    command = "test 6 -6000 300\n"
    print(f"Sending command: {command.strip()}")
    ser.write(command.encode('utf-8'))
    
    # Read response for 1.5 seconds
    start_time = time.time()
    while time.time() - start_time < 1.5:
        line = ser.readline()
        if line:
            print(line.decode('utf-8', errors='replace').strip())
            
    ser.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())
