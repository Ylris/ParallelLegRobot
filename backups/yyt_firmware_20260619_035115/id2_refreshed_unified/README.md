# ID2 Refreshed Unified Firmware

This is the ID2 firmware image flashed after the raw ID2 board flash backup was created.

Flashed file:

- `turing_CBU6_id2_unified_no_auto_hold_skip_boot_sweep.bin`

Build settings:

- `DRIVE_ID=2`
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
- SHA256: `1491c12358d796054f3a5868a36cf01b32030115d5534cdb2b215e422a229b2c`
