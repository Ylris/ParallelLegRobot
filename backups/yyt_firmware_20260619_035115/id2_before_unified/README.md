# ID2 Firmware Backup

This is a raw flash dump from the ID2 YYT drive board before refreshing it to the unified main-controller-driven firmware.

Backup file:

- `id2_board_flash_0x08000000_128k.bin`

Backup details:

- Board: ID2 YYT MiniOdrive drive board
- Address: `0x08000000`
- Size: `0x20000` bytes / 128 KiB
- SHA256: `e5613cd38b18c3c9b4d15a2a031831377e73995cb9945099c689df9c0e568ffc`
- Backup time context: immediately before flashing ID2 unified firmware

Restore command:

```sh
cd /Users/ylris/轮腿/ParallelLegRobot/DriveFirmware
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "adapter speed 100; program ../backups/yyt_firmware_20260619_035115/id2_before_unified/id2_board_flash_0x08000000_128k.bin 0x08000000 verify reset exit"
```
