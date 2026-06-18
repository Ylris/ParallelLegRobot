#!/usr/bin/env python3
import serial
import time
import sys
import glob
import argparse
import re

def choose_port() -> str:
    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*"))
    if not ports:
        raise SystemExit("No USB serial port found under /dev/cu.usbmodem* or /dev/cu.usbserial*")
    return ports[0]

def id6_is_online(line: str) -> bool:
    return "ID6=yes" in line or re.search(r"\bID6\b.*\bonline\b", line) is not None

def parse_id6_feedback(line: str):
    if "ID6 right_rear_lower" not in line:
        return None
    q_match = re.search(r"\bq=([+-]?[0-9]+(?:\.[0-9]+)?)\s+rad\b", line)
    cmd_match = re.search(r"\bcmd=([+-]?[0-9]+)\s+mV\b", line)
    online = "online" in line and "offline" not in line
    return {
        "online": online,
        "q": float(q_match.group(1)) if q_match else None,
        "cmd": int(cmd_match.group(1)) if cmd_match else None,
    }

def main():
    parser = argparse.ArgumentParser(description="Monitor ID6 status, optionally enabling zero hold once online.")
    parser.add_argument("--port", help="Serial port, for example /dev/cu.usbmodem11301")
    parser.add_argument("--auto-zero", action="store_true", help="Send zero6 automatically when ID6 appears online")
    parser.add_argument("--leave-holding", action="store_true", help="Do not send stop on Ctrl-C after auto-zero")
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to monitor after zero6. Default is unlimited")
    parser.add_argument("--tolerance-rad", type=float, default=0.05, help="Near-zero q tolerance for final report")
    args = parser.parse_args()

    port = args.port or choose_port()
    print(f"Connecting to port: {port}")
    
    try:
        ser = serial.Serial(port, 115200, timeout=0.5)
    except Exception as e:
        print(f"Failed to open port: {e}")
        return 1
        
    ser.dtr = False
    ser.rts = False
    
    # Wait for ESP32C3 boot
    time.sleep(2)
    ser.reset_input_buffer()
    
    print("Monitoring ESP32C3. Please power on the 12V power supply for the motors...")
    if args.auto_zero:
        print("Auto-zero is ENABLED: ID6 online will trigger zero6 drive-side position hold.")
    else:
        print("Auto-zero is disabled. Re-run with --auto-zero only after the motor is safely lifted.")
    
    last_status_sent = 0
    id6_activated = False
    zero_start = None
    last_id6_feedback = None
    exit_code = 0
    
    try:
        while True:
            now = time.time()
            if now - last_status_sent > 1.5:
                ser.write(b"status\n")
                last_status_sent = now
                
            line = ser.readline()
            if line:
                decoded = line.decode('utf-8', errors='replace').strip()
                print(decoded)
                
                if id6_is_online(decoded) and not id6_activated:
                    print("\n>>> ID6 motor detected ONLINE!")
                    if not args.auto_zero:
                        id6_activated = True
                        continue

                    print(">>> Commencing ID6 drive-side zero hold sequence...")
                    ser.write(b"zero6\n")
                    id6_activated = True
                    zero_start = time.time()
                    print(">>> Sent 'zero6' command to the controller.\n")
                    
                feedback = parse_id6_feedback(decoded)
                if feedback is not None:
                    last_id6_feedback = feedback

                if id6_activated and feedback is not None:
                    print(f"  [FEEDBACK] {decoded}")

                if zero_start is not None and args.duration > 0 and now - zero_start >= args.duration:
                    if last_id6_feedback is None:
                        print("FAIL: no ID6 feedback was captured during zero hold.")
                        exit_code = 2
                    elif not last_id6_feedback["online"]:
                        print("FAIL: ID6 went offline during zero hold.")
                        exit_code = 3
                    elif last_id6_feedback["cmd"] != 2345:
                        print(f"FAIL: ID6 command is {last_id6_feedback['cmd']} mV, expected 2345 mV.")
                        exit_code = 4
                    elif last_id6_feedback["q"] is None or abs(last_id6_feedback["q"]) > args.tolerance_rad:
                        print(f"FAIL: ID6 q is {last_id6_feedback['q']} rad, outside +/-{args.tolerance_rad} rad.")
                        exit_code = 5
                    else:
                        print(f"PASS: ID6 q={last_id6_feedback['q']:+.4f} rad with cmd=2345 mV.")
                    break
                    
    except KeyboardInterrupt:
        print("\nExiting monitoring script.")
    finally:
        if args.auto_zero and id6_activated and not args.leave_holding:
            try:
                print("Sending stop before closing serial port.")
                ser.write(b"stop\n")
                time.sleep(0.2)
            except Exception:
                pass
        ser.close()
    return exit_code

if __name__ == "__main__":
    sys.exit(main())
