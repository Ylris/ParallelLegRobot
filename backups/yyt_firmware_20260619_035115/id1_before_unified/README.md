# ID1 Firmware Backup

This is a raw flash dump from the ID1 YYT drive board before refreshing it to the unified main-controller-driven firmware.

Backup file:

- `id1_board_flash_0x08000000_128k.bin`

Backup details:

- Board: ID1 YYT MiniOdrive drive board
- Address: `0x08000000`
- Size: `0x20000` bytes / 128 KiB
- SHA256: `c832d74ba2b95694cfbcc2e81a691490b01550b9779125c34cd6162136773dfb`
- Backup time context: immediately before flashing ID1 unified firmware

Restore command:

```sh
cd /Users/ylris/轮腿/ParallelLegRobot/DriveFirmware
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "adapter speed 100; program ../backups/yyt_firmware_20260619_035115/id1_before_unified/id1_board_flash_0x08000000_128k.bin 0x08000000 verify reset exit"
```
