#!/usr/bin/env python3
"""Analyze leg test and height-hold logs for the current bring-up goal."""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys


MOTOR_IDS = (1, 2, 5, 6)
DQ_RE = re.compile(r"ID(?P<id>\d+).*dq=(?P<dq>[+-]?\d+\.\d+)")
TESTED_RE = re.compile(r"ID(?P<id>\d+).*tested=(?P<tested>yes|no)")
CAN_RE = re.compile(
    r"CAN state=(?P<state>\w+).*tx_fail=(?P<tx_fail>\d+).*bus_error=(?P<bus_error>\d+)"
)


def read_lines(path: Path) -> list[str]:
    if not path.exists():
        raise SystemExit(f"missing log: {path}")
    return path.read_text(encoding="utf-8", errors="replace").splitlines()


def analyze_leg_log(lines: list[str]) -> tuple[bool, list[str]]:
    messages: list[str] = []
    dq_by_id: dict[int, float] = {}
    tested_by_id: dict[int, bool] = {}

    for line in lines:
        dq_match = DQ_RE.search(line)
        if dq_match:
            dq_by_id[int(dq_match.group("id"))] = float(dq_match.group("dq"))

        tested_match = TESTED_RE.search(line)
        if tested_match:
            tested_by_id[int(tested_match.group("id"))] = tested_match.group("tested") == "yes"

    ok = True
    for motor_id in MOTOR_IDS:
        if motor_id not in dq_by_id:
            ok = False
            messages.append(f"FAIL leg: ID{motor_id} missing dq")
        else:
            messages.append(f"OK leg: ID{motor_id} dq={dq_by_id[motor_id]:+.4f} rad")

        if not tested_by_id.get(motor_id, False):
            ok = False
            messages.append(f"FAIL leg: ID{motor_id} not tested=yes in final log status")

    if "armed: output is enabled" not in "\n".join(lines):
        ok = False
        messages.append("FAIL leg: arm command evidence missing")

    return ok, messages


def analyze_height_log(lines: list[str]) -> tuple[bool, list[str]]:
    messages: list[str] = []
    text = "\n".join(lines)
    ok = True

    required_markers = [
        "direction check confirmed",
        "height hold enabled",
        "Stopping height hold and disarming.",
        "disarmed: all outputs forced to 0 mV",
    ]
    for marker in required_markers:
        if marker not in text:
            ok = False
            messages.append(f"FAIL height: missing marker: {marker}")
        else:
            messages.append(f"OK height: found marker: {marker}")

    if "height hold stopped:" in text and "height hold stopped: user command" not in text:
        ok = False
        messages.append("FAIL height: unexpected height hold stopped message")

    can_samples = 0
    for line in lines:
        can_match = CAN_RE.search(line)
        if not can_match:
            continue
        can_samples += 1
        state = can_match.group("state")
        tx_fail = int(can_match.group("tx_fail"))
        bus_error = int(can_match.group("bus_error"))
        if state != "running" or tx_fail != 0 or bus_error != 0:
            ok = False
            messages.append(
                f"FAIL height: CAN sample bad: state={state} tx_fail={tx_fail} bus_error={bus_error}"
            )

    if can_samples == 0:
        ok = False
        messages.append("FAIL height: no CAN status samples")
    else:
        messages.append(f"OK height: {can_samples} CAN samples parsed")

    offline_lines = [
        line for line in lines
        if " ID" in line and "offline" in line
    ]
    if offline_lines:
        ok = False
        messages.append(f"FAIL height: {len(offline_lines)} motor offline status lines")
    else:
        messages.append("OK height: no motor offline status lines found")

    if "hold=on" not in text:
        ok = False
        messages.append("FAIL height: no status sample with hold=on")
    else:
        messages.append("OK height: hold=on observed")

    return ok, messages


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--leg-log", type=Path, required=True)
    parser.add_argument("--height-log", type=Path, required=True)
    args = parser.parse_args()

    leg_ok, leg_messages = analyze_leg_log(read_lines(args.leg_log))
    height_ok, height_messages = analyze_height_log(read_lines(args.height_log))

    print("Leg test log:")
    for message in leg_messages:
        print(f"  {message}")

    print("\nHeight hold log:")
    for message in height_messages:
        print(f"  {message}")

    print("\nResult:")
    if leg_ok and height_ok:
        print("  SOFTWARE LOGS PASS. Physical current/limit/shake observations still need human confirmation.")
        return 0

    print("  NOT COMPLETE. Fix the failed items above or gather better logs.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
