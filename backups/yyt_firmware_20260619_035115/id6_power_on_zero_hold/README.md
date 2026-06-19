# ID6 Firmware Backup

Version name: 上电自动保持零位

This is a raw flash dump from the ID6 YYT drive board before refreshing it to the unified main-controller-driven firmware.

Backup file:

- `id6_board_flash_0x08000000_128k.bin`

Backup details:

- Board: ID6 YYT MiniOdrive drive board
- Address: `0x08000000`
- Size: `0x20000` bytes / 128 KiB
- SHA256: `d31dff8dd27d67948f4350c3f6bcdfb7f10df4e1824c080148e09b82c171c58b`
- Observed behavior before backup: power-on automatic zero hold worked well on ID6

Restore command:

```sh
cd /Users/ylris/轮腿/ParallelLegRobot/DriveFirmware
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "adapter speed 100; program ../backups/yyt_firmware_20260619_035115/id6_power_on_zero_hold/id6_board_flash_0x08000000_128k.bin 0x08000000 verify reset exit"
```
