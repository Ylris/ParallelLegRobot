# ID1 calibrating 12V no-auto-hold firmware

Purpose: diagnostic ID1 build after ID1 showed almost no positive motion with the unified skip-boot-sweep firmware.

Build flags:

- `DRIVE_ID=1`
- `DRIVE_AUTO_ZERO_HOLD=0`
- `YYT_UART_DEBUG=0`
- `MOTOR_SKIP_BOOT_SWEEP=0`
- `FOC_MODULATION_LIMIT=1.0f`

Expected behavior:

- Does not run drive-side power-on zero hold.
- Restores the startup motor sweep/electrical alignment before CAN control.
- Keeps CAN voltage command capability up to the 12 V limit used by the main controller.

Binary:

- `turing_CBU6_id1_calibrating_12v_no_auto_hold.bin`
- SHA-256: `18a352b9bff4a08cbc51402ed42d538a321d1821e62fe8a9395245694d5b9db0`
