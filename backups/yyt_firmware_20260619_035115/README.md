# YYT Firmware Backup 20260619_035115

This backup was created before moving toward a unified bring-up firmware set for the wheel-leg robot.

Contents:

- `DriveFirmware_snapshot/`: full local snapshot of the current `DriveFirmware` directory, including build outputs.
- `YYT_MiniOdrive_open_source_snapshot/`: snapshot of the referenced open-source firmware folder used during diagnosis.
- `tools_snapshot/yyt_drive_uart_debug.py`: UART debug console script.
- `worktree_tracked.diff`: tracked-file diff for the current worktree at backup time.
- `id5_board_flash_0x08000000_128k.bin`: raw 128 KiB flash dump from the currently connected ID5 drive board.
- `id1_before_unified/`: raw ID1 board flash backup made before refreshing ID1.
- `id1_refreshed_unified/`: copy of the unified ID1 firmware flashed after the ID1 backup.
- `id2_before_unified/`: raw ID2 board flash backup made before refreshing ID2.
- `id2_refreshed_unified/`: copy of the unified ID2 firmware flashed after the ID2 backup.
- `id5_refreshed_unified/`: copy of the unified ID5 firmware flashed after the earlier ID5 UART/debug tuning builds.
- `id6_power_on_zero_hold/`: raw ID6 board flash backup named `上电自动保持零位`.
- `id6_refreshed_unified/`: copy of the unified ID6 firmware flashed after the backup.

Known ID5 diagnostic state at backup time:

- Drive ID: 5
- UART debug firmware enabled
- Auto zero hold disabled
- UART/SWD debug voltage limit raised to 6 V
- Boot sweep skipped for ID5 UART debug build
- Position-loop test direction: `sign=+1`
- Last tested stable small-step direction: target angle increased and measured angle moved in the correct direction

Restore ID5 raw board flash with ST-Link:

```sh
cd /Users/ylris/轮腿/ParallelLegRobot/DriveFirmware
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "adapter speed 100; program ../backups/yyt_firmware_20260619_035115/id5_board_flash_0x08000000_128k.bin 0x08000000 verify reset exit"
```

Restore ID1 raw board flash with ST-Link:

```sh
cd /Users/ylris/轮腿/ParallelLegRobot/DriveFirmware
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "adapter speed 100; program ../backups/yyt_firmware_20260619_035115/id1_before_unified/id1_board_flash_0x08000000_128k.bin 0x08000000 verify reset exit"
```

Restore ID2 raw board flash with ST-Link:

```sh
cd /Users/ylris/轮腿/ParallelLegRobot/DriveFirmware
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "adapter speed 100; program ../backups/yyt_firmware_20260619_035115/id2_before_unified/id2_board_flash_0x08000000_128k.bin 0x08000000 verify reset exit"
```

ID1, ID2, ID5, and ID6 board flash backups have been read out in this backup set.
