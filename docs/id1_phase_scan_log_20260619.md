# ID1 Phase Scan Log - 2026-06-19

This log captures the current ID1 bring-up state so a new conversation can
continue without relying on chat history.

## Safety / Connection

- User confirmed ST-Link is physically connected to the ID1 YYT drive board.
- Main controller serial port: `/dev/cu.usbmodem11301`.
- ST-Link target: STM32G431, target voltage about `3.22 V`.
- Last verified safe state after motion tests: main controller `armed=no`,
  `hold=off`, `ID1/ID2/ID5/ID6 online`.
- Important ESP32-C3 serial handling: keep `DTR=True`, `RTS=False`. On this
  board, `DTR=False` can hold BOOT low and enter ROM download mode.

## Firmware Currently On ID1

ID1 currently has the phase-scan diagnostic firmware:

```text
backups/yyt_firmware_20260619_035115/id1_phase_scan_12v_diag/turing_CBU6.bin
```

SHA256:

```text
f3814133f5d251e32cc70391682bf37cb5398917ec9bdf414f745be0fdbbf73d
```

Build settings:

```text
DRIVE_ID=1
MOTOR_SKIP_BOOT_SWEEP=1
MOTOR_ZERO_ELECTRIC_ANGLE=0.576f
MOTOR_POLE_PAIRS=11.0f
FOC_MODULATION_LIMIT=1.0f
CAN_PHASE_SCAN_ENABLE=1
CAN_PHASE_SCAN_VOLTAGE=12.0f
CAN_MAX_COMMAND_VOLTAGE=12.0f
DRIVE_AUTO_ZERO_HOLD=0
YYT_DISABLE_OUTPUT=0
```

Behavior:

- No power-on zero hold.
- No boot motor sweep.
- Normal voltage command still works.
- `test 1 7000..7005 <ms>` maps to six fixed electrical phase tests.
- `test 1 -7000..-7005 <ms>` maps to the same six phases with negative Uq.

## Command Path Evidence

No-output RX-debug diagnostic firmware previously proved the CAN command reaches
ID1 internally:

- During `v 1 12000`, ID1 RAM showed `can_voltage_cmd=12.0`, `mode=7`,
  `uq_limit=0.0`, and centered PWM because output was intentionally disabled.
- The automatic RX debug verdict printed FAIL only because later feedback frame
  `0x106` overwrote `can_last_rx_*`; do not treat that as command-path failure.

The restored output-enabled firmware was then tested during `v 1 12000` and
ST-Link RAM showed:

```text
uq_limit=12.0
mode=7.0
can_enabled=1
can_voltage_cmd=12.0
u_a=0.0
u_b=8400.0
u_c=8400.0
vbus=4.7856
zero_electric_angle=0.576
```

This means ID1 received the 12 V command and the firmware drove PWM at full
modulation. The problem is not "main voltage command too small"; it is in the
FOC electrical angle / phase / pole-pair / wiring interpretation.

## Phase Scan Results

After flashing `id1_phase_scan_12v_diag`, short phase pulses gave:

```text
test 1 7000 120  -> dq about -0.004 rad
test 1 7001 120  -> dq about  0.000 rad
test 1 7002 120  -> dq about -0.006 rad
test 1 7003 120  -> dq about -0.001 rad
test 1 7004 120  -> dq about  0.000 rad
test 1 7005 120  -> dq about +0.269 rad
```

`7005` is the first clearly effective fixed electrical phase observed on ID1.
This proves the motor/driver can produce meaningful torque at 12 V, and the
normal FOC angle used by the previous firmware is not aligned.

The attempted full positive/negative scan was interrupted when USB serial
entered ESP32-C3 ROM download mode. Scripts were then patched to keep BOOT
released by using `DTR=True`, `RTS=False`.

## Normal FOC Candidate Result

After the `7005` result, an ordinary output-enabled CAN voltage-mode firmware
was built and flashed to ID1:

```text
backups/yyt_firmware_20260619_035115/id1_foc_zero_3p294_pp11_candidate/turing_CBU6.bin
```

SHA256:

```text
39d13baf03002ebbabe9a7441aa3be702cb291da50b9f856500139daf9e91545
```

Build settings:

```text
DRIVE_ID=1
MOTOR_SKIP_BOOT_SWEEP=1
MOTOR_ZERO_ELECTRIC_ANGLE=3.294f
MOTOR_POLE_PAIRS=11.0f
FOC_MODULATION_LIMIT=1.0f
CAN_MAX_COMMAND_VOLTAGE=12.0f
DRIVE_AUTO_ZERO_HOLD=0
YYT_DISABLE_OUTPUT=0
CAN_PHASE_SCAN_ENABLE=0
```

Short-pulse result:

```text
test 1 6000 80
q_before=-1.1822
q_after=-1.4502
dq=-0.2680 rad

test 1 -6000 80
q_before=-1.0962
q_after=-1.0592
dq=+0.0370 rad
```

This is a major improvement over the previous `0.576f` zero firmware, where
even `12000 mV` barely moved ID1. The current candidate proves normal FOC
voltage mode can produce clear motion on ID1. However, the two directions are
not symmetric yet, so this should be treated as a working base, not final
calibration.

After this test, `disarm` was sent and the controller reported `armed=no`.

## Candidate Next Step

Do not tune PID yet. First verify normal CAN voltage mode is coherent in both
directions and decide the sign convention.

Immediate next test after this candidate:

```sh
Build and test the second candidate:
MOTOR_POLE_PAIRS=14
MOTOR_ZERO_ELECTRIC_ANGLE=2.193f
```

If pp14 is not better, return to the pp11/3.294 candidate and tune around it.

## PP14 Candidate Result

The second ordinary FOC candidate was built and flashed:

```text
backups/yyt_firmware_20260619_035115/id1_foc_zero_2p193_pp14_candidate/turing_CBU6.bin
```

SHA256:

```text
73e49b70a195ef12a96800b7190d4154bfd4acd77ec1c2be7b1b3a9d3f6d1d38
```

Build settings:

```text
DRIVE_ID=1
MOTOR_SKIP_BOOT_SWEEP=1
MOTOR_ZERO_ELECTRIC_ANGLE=2.193f
MOTOR_POLE_PAIRS=14.0f
FOC_MODULATION_LIMIT=1.0f
CAN_MAX_COMMAND_VOLTAGE=12.0f
DRIVE_AUTO_ZERO_HOLD=0
YYT_DISABLE_OUTPUT=0
CAN_PHASE_SCAN_ENABLE=0
```

Short-pulse result:

```text
test 1 6000 80
q_before=-1.0592
q_after=-1.6012
dq=-0.5420 rad

test 1 -6000 80
q_before=-1.3022
q_after=-1.2772
dq=+0.0250 rad
```

pp14/2.193 has a stronger positive response than pp11/3.294, but reverse
response is still weak. Current ID1 firmware is this pp14 candidate. After the
tests, `disarm` was sent and status confirmed `armed=no`, `ID1/ID2/ID5/ID6
online`.

Next technical target: fix the direction asymmetry before PID or position hold.
Likely areas are electrical zero offset around pp14/2.193, phase order, or the
negative-Uq/angle-shift handling in `FOC.c`.

For reference, the derivation used for the first candidate was:

At the local position before the strong `7005` pulse, the main controller showed:

```text
raw about -0.367 rad
q about -1.056 rad
```

The strongest fixed phase was `7005`, which corresponds to phase index 5:

```text
phase_input = 5*pi/3 = 5.23599 rad
```

Candidate electrical zero values derived from that observation:

```text
MOTOR_POLE_PAIRS=11, zero_for_input_phase ~= 3.294 rad  <-- tested; works
MOTOR_POLE_PAIRS=14, zero_for_input_phase ~= 2.193 rad
```
Keep `DRIVE_AUTO_ZERO_HOLD=0` until normal FOC voltage motion and direction are
verified.

## Useful Commands

Safe state:

```sh
python3 -B tools/read_status_safe.py --port /dev/cu.usbmodem11301 --command disarm --pre-command-delay 0.5 --read 1.0
```

Reflash the current ID1 phase-scan diagnostic, if needed:

```sh
cd /Users/ylris/轮腿/ParallelLegRobot/DriveFirmware
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "adapter speed 100; program ../backups/yyt_firmware_20260619_035115/id1_phase_scan_12v_diag/turing_CBU6.bin 0x08000000 verify reset exit"
```

Run one phase pulse:

```sh
python3 tools/test_pulse_safe.py --port /dev/cu.usbmodem11301 --id 1 --mv 7005 --ms 120
```

Test the current normal FOC candidate:

```sh
python3 tools/test_pulse_safe.py --port /dev/cu.usbmodem11301 --id 1 --mv 6000 --ms 80
python3 tools/test_pulse_safe.py --port /dev/cu.usbmodem11301 --id 1 --mv -6000 --ms 80
```

Run the safe phase scan helper:

```sh
python3 tools/phase_scan_safe.py --port /dev/cu.usbmodem11301 --id 1 --ms 120 --negative
```
