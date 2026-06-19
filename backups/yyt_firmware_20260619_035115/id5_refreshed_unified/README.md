# ID5 Refreshed Unified Firmware

This is the ID5 firmware image flashed after the earlier ID5 UART/debug tuning builds.

Flashed file:

- `turing_CBU6_id5_unified_no_auto_hold_skip_boot_sweep.bin`

Build settings:

- `DRIVE_ID=5`
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
- SHA256: `40846bbf5288b9caa349d156a39952085aaba23f4712e2f85e9078e081c14e25`
