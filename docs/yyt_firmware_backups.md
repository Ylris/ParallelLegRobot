# YYT Firmware Backups

## ID1 - Before Unified Refresh

The ID1 board flash was backed up before refreshing ID1 to the unified main-controller-driven firmware.

Backup file:

```text
backups/yyt_firmware_20260619_035115/id1_before_unified/id1_board_flash_0x08000000_128k.bin
```

Restore command:

```sh
cd /Users/ylris/轮腿/ParallelLegRobot/DriveFirmware
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "adapter speed 100; program ../backups/yyt_firmware_20260619_035115/id1_before_unified/id1_board_flash_0x08000000_128k.bin 0x08000000 verify reset exit"
```

SHA256:

```text
c832d74ba2b95694cfbcc2e81a691490b01550b9779125c34cd6162136773dfb
```

## ID1 - Refreshed Unified Firmware

ID1 was refreshed with the unified main-controller-driven firmware after the raw board flash backup was created.

Build settings:

```text
DRIVE_ID=1
DRIVE_AUTO_ZERO_HOLD=0
YYT_UART_DEBUG=0
MOTOR_SKIP_BOOT_SWEEP=1
FOC_MODULATION_LIMIT=1.0f
```

Behavior:

- Does not automatically hold zero on power-up.
- Skips the power-on motor sweep.
- Waits for main-controller/CAN commands for motion and future leg-height control.

Flashed file copy:

```text
backups/yyt_firmware_20260619_035115/id1_refreshed_unified/turing_CBU6_id1_unified_no_auto_hold_skip_boot_sweep.bin
```

SHA256:

```text
8fbff9493639bcac29cf00affe1c29884fec0938b032780d8f40b262cff33dd8
```

## ID1 - No-Output CAN Slot Diagnostic

This diagnostic firmware was flashed to the current ST-Link target on
2026-06-19 while debugging why the ESP32-C3 main controller printed
`ID1 cmd=+12000 mV` but the drive-side RAM read showed `can_voltage_cmd=0.0`.

Build settings:

```text
DRIVE_ID=1
YYT_DISABLE_OUTPUT=1
MOTOR_AUTO_ELECTRIC_ZERO=0
MOTOR_ZERO_ELECTRIC_ANGLE=0.576f
MOTOR_SENSOR_DIRECTION=1.0f
MOTOR_POLE_PAIRS=11.0f
MOTOR_SKIP_BOOT_SWEEP=1
FOC_MODULATION_LIMIT=1.0f
CAN_MAX_COMMAND_VOLTAGE=12.0f
CAN_SPIN_TEST_VOLTAGE=12.0f
CAN_SPIN_TEST_VELOCITY=0.05f
```

Behavior:

- Keeps the board identity as `DRIVE_ID=1`.
- Receives and responds on CAN.
- Forces `uq_limit=0`, so it is for RAM/CAN diagnosis only and should not
  generate motor torque.

Flashed file copy:

```text
backups/yyt_firmware_20260619_035115/id1_no_output_can_slot_diag/turing_CBU6.bin
```

SHA256:

```text
df793d4d7f9019ad4723fd05b0d483e954d65ee128be19bc2f40ad664a641caa
```

## ID1 - No-Output CAN RX Debug Diagnostic

This is the newer no-output diagnostic build compiled after adding RAM-visible
CAN RX debug variables to `DriveFirmware/Core/Src/can_bridge.c`. It was flashed
to the confirmed ID1 target and used to prove that ID1 receives `v 1 12000`
internally as `can_voltage_cmd=12.0`. Confirm the ST-Link is physically on the
intended ID1 YYT board before flashing again.

Additional debug variables:

```text
can_last_rx_count
can_last_rx_std_id
can_last_rx_data[8]
can_last_rx_slot
can_last_rx_mv
can_last_rx_accepted
```

Flashed-file candidate:

```text
backups/yyt_firmware_20260619_035115/id1_no_output_can_rx_debug_pending/turing_CBU6.bin
```

SHA256:

```text
88723139101832c040a6c965f5edeed2db41a158de867a7c4e8d501d9d3bb53d
```

Use `tools/decode_yyt_ram_dump.py` with the matching ELF to decode the SRAM
dump after a `v 1 12000` command window.

Preferred wrapper after confirming the ST-Link is physically on ID1:

```sh
python3 tools/run_id1_can_rx_diag.py --port /dev/cu.usbmodem11301 --flash --confirm-stlink-id1
```

## ID1 - Phase Scan 12 V Diagnostic

This is the current ID1 firmware as of 2026-06-19 06:24 CST. It was used after
drive RAM proved that normal `v 1 12000` reaches full PWM but produces little
useful motion. The purpose is to test fixed electrical phases through CAN
commands `7000..7005`.

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

- Does not automatically hold zero on power-up.
- Skips the power-on motor sweep.
- Responds to normal CAN voltage commands.
- Treats `7000..7005` as fixed electrical phase tests.

Flashed file copy:

```text
backups/yyt_firmware_20260619_035115/id1_phase_scan_12v_diag/turing_CBU6.bin
```

SHA256:

```text
f3814133f5d251e32cc70391682bf37cb5398917ec9bdf414f745be0fdbbf73d
```

Restore command:

```sh
cd /Users/ylris/轮腿/ParallelLegRobot/DriveFirmware
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "adapter speed 100; program ../backups/yyt_firmware_20260619_035115/id1_phase_scan_12v_diag/turing_CBU6.bin 0x08000000 verify reset exit"
```

## ID1 - FOC Zero 3.294, Pole Pairs 11 Candidate

This is the current ordinary output-enabled FOC candidate as of 2026-06-19
06:28 CST. It was derived from the phase-scan result where `test 1 7005 120`
produced clear motion.

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

Verification:

```text
test 1 6000 80 -> dq=-0.2680 rad
test 1 -6000 80 -> dq=+0.0370 rad
```

The candidate is usable enough to prove normal FOC voltage motion, but the
response is asymmetric and not final.

Flashed file copy:

```text
backups/yyt_firmware_20260619_035115/id1_foc_zero_3p294_pp11_candidate/turing_CBU6.bin
```

SHA256:

```text
39d13baf03002ebbabe9a7441aa3be702cb291da50b9f856500139daf9e91545
```

Restore command:

```sh
cd /Users/ylris/轮腿/ParallelLegRobot/DriveFirmware
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "adapter speed 100; program ../backups/yyt_firmware_20260619_035115/id1_foc_zero_3p294_pp11_candidate/turing_CBU6.bin 0x08000000 verify reset exit"
```

## ID1 - FOC Zero 2.193, Pole Pairs 14 Candidate

This is the current ordinary output-enabled FOC candidate as of 2026-06-19
06:31 CST. It was tested because the original YYT MiniOdrive source uses
`pole_pairs=14`.

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

Verification:

```text
test 1 6000 80 -> dq=-0.5420 rad
test 1 -6000 80 -> dq=+0.0250 rad
```

It has the strongest positive response so far, but reverse response is still
weak. Do not treat it as final position-hold firmware.

Flashed file copy:

```text
backups/yyt_firmware_20260619_035115/id1_foc_zero_2p193_pp14_candidate/turing_CBU6.bin
```

SHA256:

```text
73e49b70a195ef12a96800b7190d4154bfd4acd77ec1c2be7b1b3a9d3f6d1d38
```

Restore command:

```sh
cd /Users/ylris/轮腿/ParallelLegRobot/DriveFirmware
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "adapter speed 100; program ../backups/yyt_firmware_20260619_035115/id1_foc_zero_2p193_pp14_candidate/turing_CBU6.bin 0x08000000 verify reset exit"
```

## ID2 - Before Unified Refresh

The ID2 board flash was backed up before refreshing ID2 to the unified main-controller-driven firmware.

Backup file:

```text
backups/yyt_firmware_20260619_035115/id2_before_unified/id2_board_flash_0x08000000_128k.bin
```

Restore command:

```sh
cd /Users/ylris/轮腿/ParallelLegRobot/DriveFirmware
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "adapter speed 100; program ../backups/yyt_firmware_20260619_035115/id2_before_unified/id2_board_flash_0x08000000_128k.bin 0x08000000 verify reset exit"
```

SHA256:

```text
e5613cd38b18c3c9b4d15a2a031831377e73995cb9945099c689df9c0e568ffc
```

## ID2 - Refreshed Unified Firmware

ID2 was refreshed with the unified main-controller-driven firmware after the raw board flash backup was created.

Build settings:

```text
DRIVE_ID=2
DRIVE_AUTO_ZERO_HOLD=0
YYT_UART_DEBUG=0
MOTOR_SKIP_BOOT_SWEEP=1
FOC_MODULATION_LIMIT=1.0f
```

Behavior:

- Does not automatically hold zero on power-up.
- Skips the power-on motor sweep.
- Waits for main-controller/CAN commands for motion and future leg-height control.

Flashed file copy:

```text
backups/yyt_firmware_20260619_035115/id2_refreshed_unified/turing_CBU6_id2_unified_no_auto_hold_skip_boot_sweep.bin
```

SHA256:

```text
1491c12358d796054f3a5868a36cf01b32030115d5534cdb2b215e422a229b2c
```

## ID5 - Refreshed Unified Firmware

ID5 was refreshed with the unified main-controller-driven firmware after the earlier UART/debug tuning builds.

Build settings:

```text
DRIVE_ID=5
DRIVE_AUTO_ZERO_HOLD=0
YYT_UART_DEBUG=0
MOTOR_SKIP_BOOT_SWEEP=1
FOC_MODULATION_LIMIT=1.0f
```

Behavior:

- Does not automatically hold zero on power-up.
- Skips the power-on motor sweep.
- Waits for main-controller/CAN commands for motion and future leg-height control.

Flashed file copy:

```text
backups/yyt_firmware_20260619_035115/id5_refreshed_unified/turing_CBU6_id5_unified_no_auto_hold_skip_boot_sweep.bin
```

SHA256:

```text
40846bbf5288b9caa349d156a39952085aaba23f4712e2f85e9078e081c14e25
```

## ID6 - 上电自动保持零位

The ID6 board firmware that worked well for power-on zero hold was backed up before refreshing ID6 to the unified main-controller-driven firmware.

Backup file:

```text
backups/yyt_firmware_20260619_035115/id6_power_on_zero_hold/id6_board_flash_0x08000000_128k.bin
```

Restore command:

```sh
cd /Users/ylris/轮腿/ParallelLegRobot/DriveFirmware
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "adapter speed 100; program ../backups/yyt_firmware_20260619_035115/id6_power_on_zero_hold/id6_board_flash_0x08000000_128k.bin 0x08000000 verify reset exit"
```

SHA256:

```text
d31dff8dd27d67948f4350c3f6bcdfb7f10df4e1824c080148e09b82c171c58b
```

## ID6 - Refreshed Unified Firmware

After backing up the `上电自动保持零位` firmware, ID6 was refreshed with the unified main-controller-driven firmware.

Build settings:

```text
DRIVE_ID=6
DRIVE_AUTO_ZERO_HOLD=0
YYT_UART_DEBUG=0
MOTOR_SKIP_BOOT_SWEEP=1
FOC_MODULATION_LIMIT=1.0f
```

Behavior:

- Does not automatically hold zero on power-up.
- Skips the power-on motor sweep.
- Waits for main-controller/CAN commands for motion and future leg-height control.

Flashed file copy:

```text
backups/yyt_firmware_20260619_035115/id6_refreshed_unified/turing_CBU6_id6_unified_no_auto_hold_skip_boot_sweep.bin
```

SHA256:

```text
ae634d15087050e1eb68057053662662e5fa62170599aab71669c9cc67a12f4d
```
