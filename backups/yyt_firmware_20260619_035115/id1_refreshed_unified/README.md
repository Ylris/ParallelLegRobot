# ID1 Refreshed Unified Firmware

This is the ID1 firmware image flashed after the raw ID1 board flash backup was created.

Flashed file:

- `turing_CBU6_id1_unified_no_auto_hold_skip_boot_sweep.bin`

Build settings:

- `DRIVE_ID=1`
- `DRIVE_AUTO_ZERO_HOLD=0`
- `YYT_UART_DEBUG=0`
- `MOTOR_SKIP_BOOT_SWEEP=1`
- `FOC_MODULATION_LIMIT=1.0f`

Behavior:

- Does not automatically hold zero on power-up.
- Skips the power-on motor sweep.
- Waits for main-controller/CAN commands.

Firmware details:

- BIN size: 58304 bytes
- SHA256: `8fbff9493639bcac29cf00affe1c29884fec0938b032780d8f40b262cff33dd8`
