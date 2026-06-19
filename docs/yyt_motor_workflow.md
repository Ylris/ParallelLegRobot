# YYT MiniOdrive motor workflow

This repository is the source of truth for the robot firmware and motor driver firmware.

For the latest continuation state, read `docs/current_handoff.md` first. That
file records the current flashed diagnostic firmware, last safe state, and the
next ST-Link/CAN command-path check.

## Motor ID map

Physical positions are recorded in the side-view drawing coordinates used during bring-up:

| Side-view position | Drive ID | Role |
| --- | ---: | --- |
| upper-left | 1 | leg joint motor |
| lower-left | 2 | leg joint motor |
| upper-right | 5 | leg joint motor |
| lower-right | 6 | leg joint motor |
| left-leg wheel | 3 | wheel motor |
| right-leg wheel | 4 | wheel motor |

## Flashed leg drive IDs

| Joint | Drive ID | Firmware |
| --- | ---: | --- |
| Side-view upper-left | 1 | `DriveFirmware/firmware_ids/turing_CBU6_id1_6v.elf` |
| Side-view lower-left | 2 | `DriveFirmware/firmware_ids/turing_CBU6_id2_6v.elf` |
| Side-view upper-right | 5 | `DriveFirmware/firmware_ids/turing_CBU6_id5_6v.elf` |
| Side-view lower-right | 6 | `DriveFirmware/firmware_ids/turing_CBU6_id6_6v.elf` |

## Verified drive direction

| Drive ID | Joint | Verified command direction | Main-controller `drive_sign` |
| ---: | --- | --- | ---: |
| 6 | Side-view lower-right | negative command, tested after reflashing board as ID6 | -1 |

## Build all ID firmwares

```sh
cd DriveFirmware
make ids
```

## Flash one drive

```sh
tools/flash_yyt_id.sh 1
tools/flash_yyt_id.sh 2
tools/flash_yyt_id.sh 5
tools/flash_yyt_id.sh 6
```

## Mechanical zero

The FOC electrical zero belongs to the motor driver. The robot leg mechanical zero belongs to the main controller.

Do not guess mechanical zero offsets. Put the robot in the standby pose, read each motor angle, then write those measured offsets into `config/leg_calibration.json`.

For the current ID6 bring-up, the YYT drive firmware also recognizes a temporary command sentinel:

```text
zero6
```

The ESP32-C3 main controller sends `2345 mV` in the ID6 command slot. The ID6 YYT firmware treats that sentinel as a local zero-position hold request, moves toward `DRIVE_ZERO_RAD=1.991`, then holds with its local PI position loop. Use `stop`, `holdoff`, or `disarm` to exit.

Current ID6 field-tested zero-hold build, confirmed usable on 2026-06-19:

```sh
make -C DriveFirmware id6-zero-hold-current
make -C DriveFirmware flash-id6-zero-hold-current
```

That target builds ID6 with `KP=12.0`, `KI=3.2`, `KD=0.0`, `DRIVE_AUTO_ZERO_DEADBAND_RAD=0.02f`, `DRIVE_AUTO_ZERO_HOLD_LIMIT=12.0f`, `DRIVE_AUTO_ZERO_APPROACH_VOLTAGE=12.0f`, and `FOC_MODULATION_LIMIT=1.0f`. The motor still has audible whine at hold, but this version was acceptable in field testing because it did not show the severe shaking seen with the later `KP=8.0` and `DRIVE_AUTO_ZERO_DEADBAND_RAD=0.08f` test build.

Current ID1 FOC voltage-mode experiment, tested on 2026-06-19:

```sh
make -C DriveFirmware DRIVE_ID=1 \
  MOTOR_AUTO_ELECTRIC_ZERO=0 \
  MOTOR_ZERO_ELECTRIC_ANGLE=0.576f \
  MOTOR_SENSOR_DIRECTION=1.0f \
  MOTOR_POLE_PAIRS=11.0f \
  MOTOR_SKIP_BOOT_SWEEP=1 \
  FOC_MODULATION_LIMIT=1.0f \
  CAN_MAX_COMMAND_VOLTAGE=12.0f \
  CAN_SPIN_TEST_VOLTAGE=12.0f \
  CAN_SPIN_TEST_VELOCITY=0.05f \
  all
```

This build includes the FOC negative-`Uq` phase-flip fix in `DriveFirmware/FOC/FOC.c`. It does not auto-hold or sweep on boot. The matching backup is:

```text
backups/yyt_firmware_20260619_035115/id1_foc_zero_0p576_neg_uq_fix_no_phase_scan/
```

Important status: this ID1 FOC zero is not yet validated as stable. An initial test appeared to move, but the result did not reproduce after reflashing the same backup. Treat the `0.576f` zero as an experiment, not a confirmed calibration.

Latest reproducible ID1 status:

| Command | Duration | Result |
| --- | ---: | --- |
| `test 1 12000 300` | 300 ms | `dq=0.000 rad` after retest |
| `v 1 4321` | 1000 ms | `dq=0.000 rad` after retest |
| `v 1 -4321` | 1000 ms | about `dq=-0.010 rad` after retest |

The earlier absolute phase scan result (`7004/7005`) is not currently considered reliable because later A/B testing showed it likely included mechanical settling or rebound. Keep `CAN_PHASE_SCAN_ENABLE=0` for normal builds so `7000..7005 mV` remain ordinary voltage commands.

Current ID5 direct UART debug build, flashed and verified on 2026-06-19:

```sh
make -C DriveFirmware id5-uart-debug
make -C DriveFirmware flash-id5-uart-debug
```

This target builds ID5 with `DRIVE_AUTO_ZERO_HOLD=0`, `YYT_UART_DEBUG=1`, and `FOC_MODULATION_LIMIT=1.0f`. It does not use the ESP32-C3 main controller or CAN for tuning. Connect a USB-TTL adapter directly to the YYT drive UART:

| USB-TTL | YYT drive |
| --- | --- |
| RX | PB6 / USART1_TX |
| TX | PB7 / USART1_RX |
| GND | GND |

The UART is `115200 8N1`. The debug firmware boots `disarmed`, sends `0 mV`, and requires the `arm` command before any motion. Start with low-voltage pulses:

```text
status
arm
pulse 300 50
pulse -300 50
stop
disarm
```

For local ID5 hold tuning, use small limits first:

```text
pid 1000 0 50 800 -1
hold current
```

Units are integer-only in the serial protocol: `pulse/v/limit` use mV, `target/hold` use raw single-turn mrad, and `pid` gains use mV/rad. The default hold parameters are `sign=-1`, `Kp=1000 mV/rad`, `Ki=0`, `Kd=50 mV/(rad/s)`, and `limit=800 mV`. Avoid the earlier ID5 drive-side auto-zero builds during debugging; the ID5 versions tested with high-voltage auto-hold shook severely.

ID5 copy of the ID6 field-tested zero-hold parameters, flashed and verified on 2026-06-19:

```sh
make -C DriveFirmware DRIVE_ID=5 DRIVE_AUTO_ZERO_HOLD=1 \
  DRIVE_AUTO_ZERO_OUTPUT_SIGN=1 \
  DRIVE_AUTO_ZERO_HOLD_KP=12.0f \
  DRIVE_AUTO_ZERO_HOLD_KI=3.2f \
  DRIVE_AUTO_ZERO_HOLD_KD=0.0f \
  DRIVE_AUTO_ZERO_HOLD_LIMIT=12.0f \
  DRIVE_AUTO_ZERO_APPROACH_VOLTAGE=12.0f \
  DRIVE_AUTO_ZERO_DEADBAND_RAD=0.02f \
  FOC_MODULATION_LIMIT=1.0f \
  YYT_UART_DEBUG=0 all
```

This keeps the board identity as `DRIVE_ID=5` and uses ID5's compiled zero offset `DRIVE_ZERO_RAD=0.161f`; only the ID6 zero-hold behavior and gains are copied. It boots directly into drive-side zero hold with no start delay and no hold timeout.

Recommended standby pose:

- Left and right legs are symmetric.
- Wheels/feet are under the body.
- The leg length is medium, not fully folded and not fully extended.
- All links are away from hard mechanical stops.

## ESP32C3 main-board CAN wiring

The custom ESP32C3 main board uses these internal CAN pins:

| Signal | ESP32C3 GPIO | Note |
| --- | ---: | --- |
| CAN_TX | IO6 | Internal signal from ESP32C3 to TJA1050T TXD |
| CAN_RX | IO7 | Internal signal from TJA1050T RXD to ESP32C3 |

Do not wire IO6 or IO7 directly to a YYT MiniOdrive board. IO6/IO7 are logic-level signals inside the main board. The external robot CAN bus is after the TJA1050T transceiver:

| Main board external bus | Connects to YYT MiniOdrive |
| --- | --- |
| CAN+ | CANH |
| CAN- | CANL |
| GND | GND or power negative |

USB must stay connected between the computer and the main board while calibrating. It powers and programs the ESP32C3 and provides the serial console. The YYT MiniOdrive boards still need their own motor supply. For first CAN bring-up, use one YYT board at a time with 12 V and a low current limit.

The ESP32 control-board Gerber flying-probe data shows these repeated external connector pinouts:

| Main-board connector | Pin 1 | Pin 2 | Pin 3 | Pin 4 | Pin 5 | Pin 6 |
| --- | --- | --- | --- | --- | --- | --- |
| U5 | CAN- | CAN+ | GND | VIN | GND | GND |
| U7 | CAN- | CAN+ | GND | VIN | GND | GND |
| U10 | CAN- | CAN+ | GND | VIN | GND | GND |
| U12 | CAN- | CAN+ | GND | VIN | GND | GND |
| U13 | CAN- | CAN+ | GND | VIN | GND | GND |
| U14 | CAN- | CAN+ | GND | VIN | GND | GND |

The YYT MiniOdrive V2.7 `.epro` PCB data shows the CAN connector as `CN2`:

| YYT MiniOdrive connector | Pin 1 | Pin 2 | Pin 3 | Pin 4 |
| --- | --- | --- | --- | --- |
| CN2 | CANL | CANH | not connected in PCB data | not connected in PCB data |

Therefore, for the CAN pair itself:

| Main board | YYT MiniOdrive |
| --- | --- |
| connector pin 1, CAN- | CN2 pin 1, CANL |
| connector pin 2, CAN+ | CN2 pin 2, CANH |

Do not use the main-board connector VIN pin to power the YYT MiniOdrive unless the power path has been intentionally designed and checked. During bring-up, power the YYT board from the bench supply and use the connector only for CAN-/CAN+/GND.

## CAN zero calibrator

Build and flash the safe zero-voltage CAN calibrator:

```sh
cd /Users/ylris/轮腿/ParallelLegRobot
pio run -e yyt-can-zero -t upload --upload-port /dev/cu.usbmodem1301
```

Open or query the serial console at 115200 baud:

```sh
pio device monitor --port /dev/cu.usbmodem1301 --baud 115200
```

Quick one-shot status query:

```sh
cd /Users/ylris/轮腿/ParallelLegRobot
tools/can_status.py
```

Manual serial commands:

| Command | Meaning |
| --- | --- |
| `status` | Print TWAI/CAN diagnostics and current ID1/2/5/6 angles |
| `z` | Capture current angles as mechanical-zero candidates |
| `help` | Print command list |

Normal first success with only ID1 connected:

```text
online: ID1=yes ID2=no ID5=no ID6=no
```

Normal success with all four leg drives connected:

```text
online: ID1=yes ID2=yes ID5=yes ID6=yes
```

## Current CAN fault signature

If the serial output looks like this:

```text
CAN RX GPIO7, CAN TX GPIO6, 1 Mbps
CAN state=running tx_fail=0 recoveries=10 tx_pause=yes bus_error=33505 tx_err=128 rx_err=0 tx_queue=1 rx_queue=0 ...
online: ID1=no ID2=no ID5=no ID6=no
```

the ESP32C3 firmware is running with the correct pins and is protecting itself by clearing the transmit queue, but no powered CAN node is acknowledging frames. Check hardware before changing protocol code:

1. Connect only one YYT MiniOdrive first, preferably the ID1 board.
2. Keep main-board USB connected to the computer.
3. Power the YYT board from the bench supply, starting around 12 V with a low current limit.
4. Confirm common ground: main-board GND to YYT GND or supply negative.
5. Connect main-board connector pin 1 `CAN-` to YYT `CN2 pin 1 CANL`.
6. Connect main-board connector pin 2 `CAN+` to YYT `CN2 pin 2 CANH`.
7. If still offline, swap only CANH and CANL once and query `status` again.
8. If still offline, reflash that YYT board with its intended ID firmware and try again.

After changing wiring or power, query with a delayed CAN status so the controller has time to attempt real bus traffic:

```sh
python3 -B tools/read_status_safe.py --command can --pre-command-delay 6 --read 3
```
