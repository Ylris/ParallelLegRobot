#!/usr/bin/env python3
"""Decode YYT drive CAN debug variables from a ST-Link SRAM dump."""

from __future__ import annotations

import argparse
import shutil
import struct
import subprocess
import sys
from pathlib import Path


SYMBOLS: dict[str, tuple[str, int, bool]] = {
    "can_enabled": ("u8", 1),
    "last_recovery_us": ("u32", 4),
    "last_feedback_us": ("u32", 4),
    "last_command_us": ("u32", 4),
    "can_last_rx_count": ("u32", 4, False),
    "can_last_rx_std_id": ("u32", 4, False),
    "can_last_rx_data": ("bytes8", 8, False),
    "can_last_rx_slot": ("u8", 1, False),
    "can_last_rx_mv": ("i16", 2, False),
    "can_last_rx_accepted": ("u8", 1, False),
    "can_voltage_cmd": ("f32", 4),
    "can_spin_velocity": ("f32", 4, False),
    "mode": ("f32", 4),
    "u_a": ("f32", 4),
    "u_b": ("f32", 4),
    "u_c": ("f32", 4),
    "uq_limit": ("f32", 4),
    "zero_electric_angle_norm": ("f32", 4, False),
    "zero_electric_angle": ("f32", 4, False),
    "shaft_angle": ("f32", 4, False),
    "vbus": ("f32", 4, False),
}


def symbol_kind_size_required(name: str) -> tuple[str, int, bool]:
    spec = SYMBOLS[name]
    if len(spec) == 2:
        kind, size = spec
        return kind, size, True
    kind, size, required = spec
    return kind, size, required


def nm_tool() -> str:
    candidates = [
        str(Path.home() / ".local/arm-gnu-toolchain-15.2.rel1/bin/arm-none-eabi-nm"),
        "arm-none-eabi-nm",
        "nm",
    ]
    for candidate in candidates:
        if shutil.which(candidate) or Path(candidate).exists():
            return candidate
    raise SystemExit("No nm tool found")


def load_symbols(elf: Path) -> dict[str, int]:
    output = subprocess.check_output([nm_tool(), "-n", str(elf)], text=True)
    addresses: dict[str, int] = {}
    wanted = set(SYMBOLS)
    for line in output.splitlines():
        parts = line.split()
        if len(parts) != 3:
            continue
        addr_s, _kind, name = parts
        if name in wanted:
            addresses[name] = int(addr_s, 16)
    missing = sorted(
        name for name in wanted - set(addresses)
        if symbol_kind_size_required(name)[2]
    )
    if missing:
        raise SystemExit("Missing symbols in ELF: " + ", ".join(missing))
    return addresses


def dump_plan(addresses: dict[str, int]) -> tuple[int, int]:
    start = min(addresses.values())
    end = max(addresses[name] + symbol_kind_size_required(name)[1] for name in addresses)
    start &= ~0x3
    end = (end + 3) & ~0x3
    return start, end - start


def read_value(kind: str, raw: bytes):
    if kind == "u8":
        return raw[0]
    if kind == "u32":
        return struct.unpack("<I", raw[:4])[0]
    if kind == "i16":
        return struct.unpack("<h", raw[:2])[0]
    if kind == "f32":
        return struct.unpack("<f", raw[:4])[0]
    if kind == "bytes8":
        return " ".join(f"{b:02x}" for b in raw[:8])
    raise ValueError(kind)


def check_expectations(can_id: int, slot: int, mv: int, accepted: int, voltage_cmd: float,
                       expect_can_id: int | None, expect_slot: int | None,
                       expect_mv: int | None, expect_accepted: int | None,
                       expect_voltage_cmd: float | None) -> int:
    failures: list[str] = []
    if expect_can_id is not None and can_id != expect_can_id:
        failures.append(f"CAN ID expected 0x{expect_can_id:03x}, got 0x{can_id:03x}")
    if expect_slot is not None and slot != expect_slot:
        failures.append(f"slot expected {expect_slot}, got {slot}")
    if expect_mv is not None and mv != expect_mv:
        failures.append(f"mv expected {expect_mv}, got {mv}")
    if expect_accepted is not None and accepted != expect_accepted:
        failures.append(f"accepted expected {expect_accepted}, got {accepted}")
    if expect_voltage_cmd is not None and abs(voltage_cmd - expect_voltage_cmd) > 0.05:
        failures.append(f"can_voltage_cmd expected {expect_voltage_cmd:.3f}, got {voltage_cmd:.3f}")

    if failures:
        print("verdict: FAIL")
        for failure in failures:
            print(f"  - {failure}")
        return 1

    if any(value is not None for value in (
        expect_can_id, expect_slot, expect_mv, expect_accepted, expect_voltage_cmd
    )):
        print("verdict: PASS")
    return 0


def decode_dump(elf: Path, dump: Path, *,
                dump_base: int | None = None,
                expect_can_id: int | None = None,
                expect_slot: int | None = None,
                expect_mv: int | None = None,
                expect_accepted: int | None = None,
                expect_voltage_cmd: float | None = None) -> int:
    addresses = load_symbols(elf)
    start, length = dump_plan(addresses)
    data_base = start if dump_base is None else dump_base
    data = dump.read_bytes()
    max_end = max(
        addresses[name] + symbol_kind_size_required(name)[1]
        for name in addresses
    )
    required_len = max_end - data_base
    if data_base > min(addresses.values()):
        raise SystemExit(
            f"Dump base 0x{data_base:08x} is after the first symbol "
            f"0x{min(addresses.values()):08x}"
        )
    if len(data) < required_len:
        raise SystemExit(f"Dump too short: got {len(data)} bytes, need {required_len}")

    print(f"dump_base=0x{data_base:08x} symbol_span_base=0x{start:08x} symbol_span_len={length}")
    for name in sorted(addresses, key=lambda item: addresses[item]):
        kind, size, _required = symbol_kind_size_required(name)
        offset = addresses[name] - data_base
        raw = data[offset:offset + size]
        value = read_value(kind, raw)
        print(f"{name:24s} addr=0x{addresses[name]:08x} raw={raw.hex():16s} value={value}")

    voltage_cmd = read_value("f32", data[addresses["can_voltage_cmd"] - data_base:
                                          addresses["can_voltage_cmd"] - data_base + 4])
    optional_rx = [
        "can_last_rx_std_id",
        "can_last_rx_slot",
        "can_last_rx_mv",
        "can_last_rx_accepted",
    ]
    if all(name in addresses for name in optional_rx):
        can_id = read_value("u32", data[addresses["can_last_rx_std_id"] - data_base:
                                          addresses["can_last_rx_std_id"] - data_base + 4])
        slot = read_value("u8", data[addresses["can_last_rx_slot"] - data_base:
                                      addresses["can_last_rx_slot"] - data_base + 1])
        mv = read_value("i16", data[addresses["can_last_rx_mv"] - data_base:
                                     addresses["can_last_rx_mv"] - data_base + 2])
        accepted = read_value("u8", data[addresses["can_last_rx_accepted"] - data_base:
                                          addresses["can_last_rx_accepted"] - data_base + 1])
        print()
        print(f"summary: can_id=0x{can_id:03x} slot={slot} mv={mv} accepted={accepted}")
    else:
        can_id = slot = mv = accepted = None
        print()
        print("summary: rx debug symbols not present in this ELF")

    if can_id is None and any(value is not None for value in (
        expect_can_id, expect_slot, expect_mv, expect_accepted
    )):
        print("verdict: FAIL")
        print("  - RX debug expectations were requested, but this ELF has no RX debug symbols")
        return 1
    return check_expectations(
        can_id or 0,
        slot or 0,
        mv or 0,
        accepted or 0,
        voltage_cmd,
        expect_can_id,
        expect_slot,
        expect_mv,
        expect_accepted,
        expect_voltage_cmd,
    )


def run_openocd_dump(out: Path, start: int, length: int, cwd: Path) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    command = [
        "openocd",
        "-f", "interface/stlink.cfg",
        "-f", "target/stm32g4x.cfg",
        "-c",
        (
            f"adapter speed 100; init; halt; dump_image {out} "
            f"0x{start:08x} {length}; resume; shutdown"
        ),
    ]
    subprocess.check_call(command, cwd=str(cwd))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", type=Path, default=Path("DriveFirmware/build/turing_CBU6.elf"))
    parser.add_argument("--dump", type=Path, help="SRAM dump created by OpenOCD dump_image")
    parser.add_argument("--dump-base", type=lambda text: int(text, 0),
        help="Base address used when --dump was created; defaults to the symbol span base")
    parser.add_argument("--print-openocd", action="store_true",
                        help="Print a matching OpenOCD dump_image command and exit")
    parser.add_argument("--run-openocd", action="store_true",
                        help="Run OpenOCD to dump SRAM, then decode it")
    parser.add_argument("--out", type=Path, default=Path("/tmp/yyt_ram_debug.bin"),
                        help="Dump path for --print-openocd or --run-openocd")
    parser.add_argument("--openocd-cwd", type=Path, default=Path("DriveFirmware"),
        help="Directory where OpenOCD can find interface/target cfg files")
    parser.add_argument("--expect-can-id", type=lambda text: int(text, 0))
    parser.add_argument("--expect-slot", type=int)
    parser.add_argument("--expect-mv", type=int)
    parser.add_argument("--expect-accepted", type=int)
    parser.add_argument("--expect-voltage-cmd", type=float)
    args = parser.parse_args()

    addresses = load_symbols(args.elf)
    start, length = dump_plan(addresses)
    if args.print_openocd and args.run_openocd:
        parser.error("--print-openocd and --run-openocd are mutually exclusive")
    if args.print_openocd:
        print(
            "openocd -f interface/stlink.cfg -f target/stm32g4x.cfg "
            f"-c \"adapter speed 100; init; halt; dump_image {args.out} "
            f"0x{start:08x} {length}; resume; shutdown\""
        )
        return 0

    if args.run_openocd:
        run_openocd_dump(args.out, start, length, args.openocd_cwd)
        return decode_dump(
            args.elf,
            args.out,
            dump_base=None,
            expect_can_id=args.expect_can_id,
            expect_slot=args.expect_slot,
            expect_mv=args.expect_mv,
            expect_accepted=args.expect_accepted,
            expect_voltage_cmd=args.expect_voltage_cmd,
        )

    if args.dump is None:
        parser.error("--dump is required unless --print-openocd or --run-openocd is used")
    return decode_dump(
        args.elf,
        args.dump,
        dump_base=args.dump_base,
        expect_can_id=args.expect_can_id,
        expect_slot=args.expect_slot,
        expect_mv=args.expect_mv,
        expect_accepted=args.expect_accepted,
        expect_voltage_cmd=args.expect_voltage_cmd,
    )


if __name__ == "__main__":
    sys.exit(main())
