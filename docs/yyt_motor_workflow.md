# YYT MiniOdrive motor workflow

This repository is the source of truth for the robot firmware and motor driver firmware.

## Flashed leg drive IDs

| Joint | Drive ID | Firmware |
| --- | ---: | --- |
| Left front / upper | 1 | `DriveFirmware/firmware_ids/turing_CBU6_id1_6v.elf` |
| Left rear / lower | 2 | `DriveFirmware/firmware_ids/turing_CBU6_id2_6v.elf` |
| Right front / upper | 5 | `DriveFirmware/firmware_ids/turing_CBU6_id5_6v.elf` |
| Right rear / lower | 6 | `DriveFirmware/firmware_ids/turing_CBU6_id6_6v.elf` |

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
CAN RX GPIO7, CAN TX GPIO6, baud 1Mbps
twai: state=2 tx_error=128 rx_error=0 msgs_to_rx=0 bus_error=...
diag: tx_ok=... tx_fail=... rx_total=0 rx_yyt=0
online: ID1=no ID2=no ID5=no ID6=no
```

the ESP32C3 firmware is running with the correct pins, but no powered CAN node is acknowledging frames. Check hardware before changing protocol code:

1. Connect only one YYT MiniOdrive first, preferably the ID1 board.
2. Keep main-board USB connected to the computer.
3. Power the YYT board from the bench supply, starting around 12 V with a low current limit.
4. Confirm common ground: main-board GND to YYT GND or supply negative.
5. Connect main-board connector pin 1 `CAN-` to YYT `CN2 pin 1 CANL`.
6. Connect main-board connector pin 2 `CAN+` to YYT `CN2 pin 2 CANH`.
7. If still offline, swap only CANH and CANL once and query `status` again.
8. If still offline, reflash that YYT board with its intended ID firmware and try again.
