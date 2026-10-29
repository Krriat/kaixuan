#!/usr/bin/env python3

import argparse
import os
import time
from pathlib import Path
from typing import Optional


STATE_DIR = Path("/tmp/ethercat_op_csp")
ARM_MASTER_IDS = {"right": (0,), "left": (1,), "both": (0, 1)}
GEAR_RATIOS = (101, 101, 81, 81, 101, 101, 101)


def process_is_direct_helper(pid: int, master_id: int) -> bool:
    try:
        command = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
    except OSError:
        return False
    args = [part.decode(errors="replace") for part in command if part]
    if not args or os.path.basename(args[0]) != "ethercat_direct_tool":
        return False
    try:
        return args[args.index("--master_id") + 1] == str(master_id)
    except (ValueError, IndexError):
        return False


def helper_pid(master_id: int) -> Optional[int]:
    try:
        pid = int((STATE_DIR / f"master{master_id}.pid").read_text().strip())
    except (OSError, ValueError):
        return None
    return pid if process_is_direct_helper(pid, master_id) else None


def latest_snapshot(master_id: int) -> dict[int, tuple[int, int, int]]:
    path = STATE_DIR / f"master{master_id}.log"
    try:
        if time.time() - path.stat().st_mtime > 5.0:
            raise RuntimeError(f"{path} has not been updated for more than 5 seconds")
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError as exc:
        raise RuntimeError(f"cannot read {path}: {exc}") from exc

    prefix = f"master={master_id} snapshot="
    for header_index in range(len(lines) - 1, -1, -1):
        if not lines[header_index].startswith(prefix):
            continue
        axes: dict[int, tuple[int, int, int]] = {}
        for line in lines[header_index + 1 :]:
            if line.startswith("master="):
                break
            parts = line.split()
            if len(parts) != 6 or not parts[0].isdigit():
                continue
            try:
                slave = int(parts[1])
                mode = int(parts[3])
                status = int(parts[4])
                position = int(parts[5])
            except ValueError:
                continue
            axes[slave] = (mode, status, position)
        if len(axes) == 7:
            return axes
    raise RuntimeError(f"no complete seven-axis snapshot found in {path}")


def counts_per_revolution(slave: int) -> int:
    return 1048576 * GEAR_RATIOS[slave]


def print_positions(master_ids: tuple[int, ...]) -> None:
    rows = []
    for master_id in master_ids:
        if helper_pid(master_id) is None:
            raise RuntimeError(
                f"master{master_id} direct helper is not running; "
                "run sudo python3 src/normal/prepare_ethercat_op_csp.py first"
            )
        axes = latest_snapshot(master_id)
        for slave in range(7):
            mode, status, counts = axes[slave]
            degrees = counts * 360.0 / counts_per_revolution(slave)
            arm = "right" if master_id == 0 else "left"
            rows.append(
                (f"{arm}_joint{slave + 1}", master_id, slave, mode, status, counts, degrees)
            )

    print(
        "joint          master slave mode status  position_counts  position_degrees",
        flush=True,
    )
    for joint, master, slave, mode, status, counts, degrees in rows:
        print(
            f"{joint:<14} {master:>6} {slave:>5} {mode:>4} "
            f"0x{status:04x} {counts:>16} {degrees:>17.4f}",
            flush=True,
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Read selected arm PDO positions from direct IgH hold loops"
    )
    parser.add_argument(
        "--active-arm",
        choices=("right", "left", "both"),
        default="both",
        help="Arm to read: right=master0, left=master1, both=both masters",
    )
    parser.add_argument("--once", action="store_true", help="Print one snapshot and exit")
    parser.add_argument(
        "--interval", type=float, default=0.2, help="Live refresh interval in seconds"
    )
    parser.add_argument(
        "--print_every",
        type=int,
        default=None,
        help="Compatibility option; helpers control their own log print interval",
    )
    args = parser.parse_args()
    master_ids = ARM_MASTER_IDS[args.active_arm]
    if args.interval <= 0.0:
        raise SystemExit("--interval must be greater than zero")

    try:
        while True:
            print_positions(master_ids)
            if args.once:
                return
            print(flush=True)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        return
    except RuntimeError as exc:
        raise SystemExit(f"read_14_positions_live: {exc}") from exc


if __name__ == "__main__":
    main()
