#!/usr/bin/env python3
"""Flash and/or run the ID1 no-output CAN RX diagnostic workflow."""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
PENDING_DIR = REPO_ROOT / "backups/yyt_firmware_20260619_035115/id1_no_output_can_rx_debug_pending"
DEFAULT_BIN = PENDING_DIR / "turing_CBU6.bin"
DEFAULT_ELF = PENDING_DIR / "turing_CBU6.elf"


def run_checked(command: list[str], *, cwd: Path | None = None) -> None:
    print("+ " + " ".join(str(part) for part in command), flush=True)
    subprocess.check_call(command, cwd=str(cwd) if cwd else None)


def safe_disarm(port: str) -> None:
    run_checked([
        "python3",
        "-B",
        str(REPO_ROOT / "tools/read_status_safe.py"),
        "--port",
        port,
        "--command",
        "disarm",
        "--pre-command-delay",
        "1.0",
        "--read",
        "0.8",
    ])


def flash_no_output_diag(firmware_bin: Path) -> None:
    run_checked([
        "openocd",
        "-f",
        "interface/stlink.cfg",
        "-f",
        "target/stm32g4x.cfg",
        "-c",
        f"adapter speed 100; program {firmware_bin} 0x08000000 verify reset exit",
    ], cwd=REPO_ROOT / "DriveFirmware")


def wait_for_ready(process: subprocess.Popen[str], timeout: float) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = process.stdout.readline() if process.stdout else ""
        if line:
            print(line.rstrip(), flush=True)
            if "READY_FOR_STLINK" in line:
                return True
        elif process.poll() is not None:
            return False
    return False


def drain_process(process: subprocess.Popen[str]) -> int:
    if process.stdout:
        for line in process.stdout:
            print(line.rstrip(), flush=True)
    return process.wait()


def run_command_window_and_decode(port: str, motor_id: int, mv: int, seconds: float,
                                  status_every: float, cantx_every: float,
                                  elf: Path, dump: Path) -> int:
    command = [
        "python3",
        str(REPO_ROOT / "tools/hold_manual_for_stlink.py"),
        "--port",
        port,
        "--id",
        str(motor_id),
        "--mv",
        str(mv),
        "--seconds",
        str(seconds),
        "--status-every",
        str(status_every),
        "--cantx-every",
        str(cantx_every),
    ]
    print("+ " + " ".join(command), flush=True)
    process = subprocess.Popen(
        command,
        cwd=str(REPO_ROOT),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    ready = wait_for_ready(process, timeout=max(10.0, seconds * 0.7))
    if not ready:
        print("ERROR: hold command did not reach READY_FOR_STLINK", flush=True)
        rc = drain_process(process)
        safe_disarm(port)
        return rc or 1

    decode_rc = 0
    try:
        run_checked([
            "python3",
            str(REPO_ROOT / "tools/decode_yyt_ram_dump.py"),
            "--elf",
            str(elf),
            "--run-openocd",
            "--out",
            str(dump),
            "--expect-can-id",
            "0x100" if motor_id <= 4 else "0x200",
            "--expect-slot",
            str((motor_id - 1) if motor_id <= 4 else (motor_id - 5)),
            "--expect-mv",
            str(mv),
            "--expect-accepted",
            "1",
            "--expect-voltage-cmd",
            "12.0",
        ], cwd=REPO_ROOT)
    except subprocess.CalledProcessError as exc:
        decode_rc = exc.returncode or 1

    hold_rc = drain_process(process)
    safe_disarm(port)
    return decode_rc or hold_rc


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/cu.usbmodem11301")
    parser.add_argument("--id", type=int, default=1, choices=(1, 2, 5, 6))
    parser.add_argument("--mv", type=int, default=12000)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--status-every", type=float, default=1.0)
    parser.add_argument("--cantx-every", type=float, default=1.0)
    parser.add_argument("--firmware-bin", type=Path, default=DEFAULT_BIN)
    parser.add_argument("--elf", type=Path, default=DEFAULT_ELF)
    parser.add_argument("--dump", type=Path, default=Path("/tmp/yyt_id1_debug_vars.bin"))
    parser.add_argument("--flash", action="store_true",
                        help="Flash the no-output diagnostic before running the command window")
    parser.add_argument("--confirm-stlink-id1", action="store_true",
                        help="Required with --flash: confirms ST-Link is physically on the intended ID1 board")
    parser.add_argument("--already-flashed-no-output", action="store_true",
                        help="Run without flashing because the no-output RX debug diagnostic is already flashed")
    args = parser.parse_args()

    if args.flash and not args.confirm_stlink_id1:
        parser.error("--flash requires --confirm-stlink-id1")
    if not args.flash and not args.already_flashed_no_output:
        parser.error("refusing to send a voltage command unless --flash or --already-flashed-no-output is set")
    if args.id != 1 and args.confirm_stlink_id1:
        parser.error("--confirm-stlink-id1 only matches --id 1")
    if not args.firmware_bin.exists():
        parser.error(f"missing firmware bin: {args.firmware_bin}")
    if not args.elf.exists():
        parser.error(f"missing ELF: {args.elf}")

    safe_disarm(args.port)
    if args.flash:
        flash_no_output_diag(args.firmware_bin.resolve())
        time.sleep(0.5)
        safe_disarm(args.port)

    return run_command_window_and_decode(
        args.port,
        args.id,
        args.mv,
        args.seconds,
        args.status_every,
        args.cantx_every,
        args.elf.resolve(),
        args.dump,
    )


if __name__ == "__main__":
    sys.exit(main())
