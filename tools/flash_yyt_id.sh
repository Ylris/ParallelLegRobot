#!/usr/bin/env bash
set -euo pipefail

variant="normal"
if [ "$#" -eq 2 ] && [ "$1" = "--6v" ]; then
  variant="6v"
  shift
fi

if [ "$#" -ne 1 ]; then
  echo "Usage: tools/flash_yyt_id.sh [--6v] <drive_id>"
  echo "Example: tools/flash_yyt_id.sh --6v 2"
  exit 2
fi

drive_id="$1"
case "$drive_id" in
  1|2|3|5|6|7) ;;
  *)
    echo "Unsupported drive id: $drive_id"
    echo "Known ids: 1 2 3 5 6 7"
    exit 2
    ;;
esac

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
yyt_dir="$repo_root/DriveFirmware"
if [ "$variant" = "6v" ]; then
  firmware="$yyt_dir/firmware_ids/turing_CBU6_id${drive_id}_6v.elf"
else
  firmware="$yyt_dir/firmware_ids/turing_CBU6_id${drive_id}.bin"
fi

if [ ! -f "$firmware" ]; then
  echo "Missing firmware: $firmware"
  if [ "$variant" = "6v" ]; then
    echo "Build the 6V firmware bundle first."
  else
    echo "Run this first:"
    echo "  cd \"$yyt_dir\" && make ids"
  fi
  exit 1
fi

echo "About to flash DriveFirmware DRIVE_ID=$drive_id variant=$variant"
echo "Make sure ST-Link is physically connected to the ID$drive_id YYT board."
echo "Firmware: $firmware"

if [ "$variant" = "6v" ]; then
  openocd \
    -f interface/stlink.cfg \
    -f target/stm32g4x.cfg \
    -c "adapter speed 100; program $firmware verify reset exit"
else
  openocd \
    -f interface/stlink.cfg \
    -f target/stm32g4x.cfg \
    -c "adapter speed 100; program $firmware 0x08000000 verify reset exit"
fi
