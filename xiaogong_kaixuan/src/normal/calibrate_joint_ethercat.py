#!/usr/bin/env python3

import argparse
import os
import shlex
import signal
import subprocess
import sys
import time
from typing import Optional


JOINT_TO_MASTER_SLAVE = {
    "right_joint1": (0, 0),
    "right_joint2": (0, 1),
    "right_joint3": (0, 2),
    "right_joint4": (0, 3),
    "right_joint5": (0, 4),
    "right_joint6": (0, 5),
    "right_joint7": (0, 6),
    "left_joint1": (1, 0),
    "left_joint2": (1, 1),
    "left_joint3": (1, 2),
    "left_joint4": (1, 3),
    "left_joint5": (1, 4),
    "left_joint6": (1, 5),
    "left_joint7": (1, 6),
}


WORKSPACE_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_BINARY = os.path.join(
    WORKSPACE_ROOT,
    "install",
    "kaixuan_can_hardware",
    "lib",
    "kaixuan_can_hardware",
    "ethercat_direct_tool",
)
STATE_DIR = "/tmp/ethercat_op_csp"
CONFLICT_PATTERN = (
    "ros2 launch kaixuan3_config moveit_bringup.launch.py|"
    "move_group|"
    "ros2_control_node|"
    "robot_state_publisher|"
    "controller_manager/spawner|"
    "ethercat_direct_tool"
)

STALL_POSITION_EPS_COUNTS = 16
TARGET_TOLERANCE_COUNTS = 32


def ensure_state_dir() -> None:
    os.makedirs(STATE_DIR, exist_ok=True)
    os.chmod(STATE_DIR, 0o1777)


def hold_pid_path(master_id: int) -> str:
    ensure_state_dir()
    return os.path.join(STATE_DIR, f"master{master_id}.pid")


def hold_log_path(master_id: int) -> str:
    ensure_state_dir()
    return os.path.join(STATE_DIR, f"master{master_id}.log")


def move_command_path(master_id: int) -> str:
    return os.path.join(STATE_DIR, f"master{master_id}.move")


def read_pidfile(master_id: int) -> Optional[int]:
    path = hold_pid_path(master_id)
    if not os.path.exists(path):
        return None
    try:
        with open(path, "r", encoding="ascii") as handle:
            return int(handle.read().strip())
    except (OSError, ValueError):
        return None


def write_pidfile(master_id: int, pid: int) -> None:
    with open(hold_pid_path(master_id), "w", encoding="ascii") as handle:
        handle.write(f"{pid}\n")


def remove_pidfile(master_id: int) -> None:
    try:
        os.remove(hold_pid_path(master_id))
    except FileNotFoundError:
        pass


def process_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def read_process_cmdline(pid: int) -> list[str]:
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as handle:
            raw_parts = handle.read().split(b"\0")
    except OSError:
        return []

    return [part.decode("utf-8", errors="replace") for part in raw_parts if part]


def is_expected_hold_helper(pid: int, master_id: int) -> bool:
    if not process_alive(pid):
        return False

    argv = read_process_cmdline(pid)
    if not argv:
        return False

    executable = os.path.basename(argv[0])
    if executable != "ethercat_direct_tool":
        return False
    if "--move_slave" in argv:
        return False

    try:
        arg_master_id = argv[argv.index("--master_id") + 1]
    except (ValueError, IndexError):
        return False

    return arg_master_id == str(master_id)


def tail_file(path: str, max_lines: int = 40) -> str:
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as handle:
            lines = handle.readlines()
    except OSError:
        return ""
    return "".join(lines[-max_lines:])


def terminate_pid(pid: Optional[int], label: str) -> None:
    if pid is None or not process_alive(pid):
        return

    try:
        os.killpg(pid, signal.SIGTERM)
    except ProcessLookupError:
        return

    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if not process_alive(pid):
            return
        time.sleep(0.1)

    print(f"forcing shutdown of {label}", flush=True)
    try:
        os.killpg(pid, signal.SIGKILL)
    except ProcessLookupError:
        return


def stop_hold_helper(master_id: int) -> None:
    pid = read_pidfile(master_id)
    if pid is not None and is_expected_hold_helper(pid, master_id):
        terminate_pid(pid, f"master{master_id} hold helper")
    remove_pidfile(master_id)


def start_hold_helper(master_id: int, binary: str, print_every: int) -> int:
    stop_hold_helper(master_id)
    log_path = hold_log_path(master_id)
    log_file = open(log_path, "w", encoding="utf-8")
    try:
        proc = subprocess.Popen(
            [binary, "--master_id", str(master_id), "--print_every", str(print_every), "--hold_csp_all"],
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
    finally:
        log_file.close()

    time.sleep(0.2)
    if proc.poll() is not None:
        raise RuntimeError(
            f"master{master_id} hold helper exited immediately with code {proc.returncode}\n"
            f"recent log:\n{tail_file(log_path)}"
        )

    write_pidfile(master_id, proc.pid)
    return proc.pid


def find_conflicts(ignore_pids: set[int]) -> list[str]:
    result = subprocess.run(
        ["bash", "-lc", f"pgrep -af {shlex.quote(CONFLICT_PATTERN)} || true"],
        capture_output=True,
        text=True,
        check=False,
    )

    conflicts = []
    for line in result.stdout.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        if "pgrep -af" in stripped:
            continue
        if "calibrate_joint_ethercat.py" in stripped:
            continue
        parts = stripped.split(maxsplit=1)
        try:
            pid = int(parts[0])
        except (ValueError, IndexError):
            pid = None
        if pid is not None and pid in ignore_pids:
            continue
        conflicts.append(stripped)

    return conflicts


def ethercat_output(args: list[str], timeout: float = 5.0) -> str:
    completed = subprocess.run(
        ["ethercat", *args],
        capture_output=True,
        text=True,
        check=False,
        timeout=timeout,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"ethercat {' '.join(args)} failed with code {completed.returncode}\n"
            f"stdout:\n{completed.stdout}\n"
            f"stderr:\n{completed.stderr}"
        )
    return completed.stdout


def read_latest_snapshot(log_path: str, master_id: int) -> Optional[tuple[int, dict[int, tuple[int, int, int]]]]:
    try:
        with open(log_path, "r", encoding="utf-8", errors="replace") as handle:
            lines = handle.readlines()
    except OSError:
        return None

    prefix = f"master={master_id} snapshot="
    for index in range(len(lines) - 1, -1, -1):
        header = lines[index].strip()
        if not header.startswith(prefix):
            continue

        try:
            snapshot = int(header.split("snapshot=", 1)[1].split()[0])
        except (IndexError, ValueError):
            continue

        axes: dict[int, tuple[int, int, int]] = {}
        for raw_line in lines[index + 1:]:
            line = raw_line.strip()
            if not line or line == "axis slave offset mode status position":
                continue
            if line.startswith("master="):
                break

            parts = line.split()
            if len(parts) != 6 or not parts[0].isdigit():
                continue

            try:
                slave_pos = int(parts[1])
                mode_display = int(parts[3])
                status_word = int(parts[4])
                actual_position = int(parts[5])
            except ValueError:
                continue

            axes[slave_pos] = (mode_display, status_word, actual_position)

        if len(axes) >= 7:
            return snapshot, axes

    return None


def collect_helper_rows(master_id: int) -> Optional[list[tuple[int, int, int, int, int]]]:
    rows: list[tuple[int, int, int, int, int]] = []
    snapshot = read_latest_snapshot(hold_log_path(master_id), master_id)
    if snapshot is None:
        return None
    _snapshot_index, axes = snapshot
    for slave_pos in range(7):
        axis_data = axes.get(slave_pos)
        if axis_data is None:
            return None
        mode_display, status_word, actual_position = axis_data
        rows.append((master_id, slave_pos, mode_display, status_word, actual_position))
    return rows


def wait_for_helper_csp_state(
    master_id: int, timeout_s: float
) -> list[tuple[int, int, int, int, int]]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rows = collect_helper_rows(master_id)
        if rows and all(row[2] == 8 and row[3] != 0 for row in rows):
            return rows
        time.sleep(0.2)

    log_text = tail_file(hold_log_path(master_id))
    raise RuntimeError(
        f"master{master_id} helper did not report all 7 axes in CSP within "
        f"{timeout_s}s\n{log_text}"
    )


def wait_for_master_operation(master_id: int, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    last_output = ""
    while time.monotonic() < deadline:
        last_output = ethercat_output(["-m", str(master_id), "master"])
        if "Phase: Operation" in last_output and "Active: yes" in last_output:
            return
        time.sleep(0.2)
    raise RuntimeError(f"master{master_id} did not enter OP within {timeout_s}s\n{last_output}")


def wait_for_slave_operation(master_id: int, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    last_output = ""
    while time.monotonic() < deadline:
        last_output = ethercat_output(["-m", str(master_id), "slaves"])
        lines = [line.strip() for line in last_output.splitlines() if line.strip()]
        if len(lines) == 7 and all(" OP " in f" {line} " for line in lines):
            return
        time.sleep(0.2)
    raise RuntimeError(f"master{master_id} slaves did not all enter OP within {timeout_s}s\n{last_output}")


def wait_for_system_operation(timeout_s: float) -> None:
    for master_id in (0, 1):
        wait_for_master_operation(master_id, timeout_s)
        wait_for_slave_operation(master_id, timeout_s)


def print_state_rows(rows: list[tuple[int, int, int, int, int]], title: str) -> None:
    print(f"\n{title}", flush=True)
    for master_id, slave_pos, mode_display, status_word, actual_position in rows:
        print(
            f"  m={master_id} s={slave_pos} 6061={mode_display} 6041=0x{status_word:04x} 6064={actual_position}",
            flush=True,
        )


def start_move_helper(command: list[str]) -> subprocess.Popen[str]:
    return subprocess.Popen(
        command,
        text=True,
        start_new_session=True,
    )


def terminate_process(proc: Optional[subprocess.Popen[str]], label: str) -> None:
    if proc is None or proc.poll() is not None:
        return

    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        return

    try:
        proc.wait(timeout=5.0)
        return
    except subprocess.TimeoutExpired:
        pass

    print(f"forcing shutdown of {label}", flush=True)
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    proc.wait(timeout=5.0)


def monitor_move(
    proc: subprocess.Popen[str],
    slave_pos: int,
    move_timeout_s: float,
    stall_timeout_s: float,
) -> None:
    if proc.stdout is None:
        raise RuntimeError("Move helper stdout is not available")

    deadline = time.monotonic() + move_timeout_s
    last_position: Optional[int] = None
    last_progress_time = time.monotonic()
    current_position: Optional[int] = None
    target_position: Optional[int] = None
    current_phase: Optional[int] = None

    while True:
        if time.monotonic() > deadline:
            raise RuntimeError(f"Move timed out after {move_timeout_s}s")

        line = proc.stdout.readline()
        if not line:
            if proc.poll() is not None:
                if proc.returncode == 0:
                    return
                raise RuntimeError(f"Move helper exited with code {proc.returncode}")
            time.sleep(0.1)
            continue

        print(line, end="", flush=True)
        stripped = line.strip()

        if stripped.startswith("move_axis_reached_target"):
            return

        parts = stripped.split()
        if len(parts) == 6 and parts[0].isdigit():
            try:
                axis_slave = int(parts[1])
                axis_position = int(parts[5])
            except ValueError:
                axis_slave = -1
                axis_position = 0

            if axis_slave == slave_pos:
                current_position = axis_position
                if last_position is None or abs(current_position - last_position) >= STALL_POSITION_EPS_COUNTS:
                    last_progress_time = time.monotonic()
                last_position = current_position
            continue

        if stripped.startswith("move_axis "):
            for token in stripped.split():
                if token.startswith("phase="):
                    current_phase = int(token.split("=", 1)[1])
                elif token.startswith("target="):
                    target_position = int(token.split("=", 1)[1])

            if (
                current_phase == 4
                and target_position is not None
                and current_position is not None
                and abs(current_position - target_position) > TARGET_TOLERANCE_COUNTS
                and time.monotonic() - last_progress_time > stall_timeout_s
            ):
                raise RuntimeError(
                    "Move appears stalled: phase=4 but actual position is not converging to target.\n"
                    f"current={current_position} target={target_position} "
                    f"stall_timeout={stall_timeout_s}s"
                )


def ensure_prepared_state(
    master_id: int, timeout_s: float
) -> list[tuple[int, int, int, int, int]]:
    pid = read_pidfile(master_id)
    if pid is None or not is_expected_hold_helper(pid, master_id):
        arm = "right" if master_id == 0 else "left"
        raise RuntimeError(
            f"master{master_id} ({arm}) hold helper is not running. "
            f"Run prepare_ethercat_op_csp.py --active-arm {arm} first."
        )

    # Helpers for either arm are expected and may run independently.
    running_pids: set[int] = {pid}
    for other_master_id in (0, 1):
        other_pid = read_pidfile(other_master_id)
        if other_pid is not None and is_expected_hold_helper(other_pid, other_master_id):
            running_pids.add(other_pid)

    conflicts = find_conflicts(running_pids)
    if conflicts:
        message = "\n".join(f"  {line}" for line in conflicts)
        raise RuntimeError(
            "Unexpected conflicting processes detected.\n"
            f"{message}"
        )

    wait_for_master_operation(master_id, timeout_s)
    wait_for_slave_operation(master_id, timeout_s)
    return wait_for_helper_csp_state(master_id, timeout_s)


def submit_move_command(
    master_id: int,
    slave_pos: int,
    degrees: float,
    speed_degrees_per_second: float,
    timeout_s: float,
) -> str:
    ensure_state_dir()
    command_path = move_command_path(master_id)
    request_id = f"{time.time_ns()}-{os.getpid()}"
    temporary_path = f"{command_path}.{request_id}.tmp"

    if os.path.exists(command_path):
        raise RuntimeError(
            f"master{master_id} already has a pending move command"
        )

    try:
        with open(temporary_path, "w", encoding="ascii") as handle:
            handle.write(
                f"{request_id} {slave_pos} {degrees} "
                f"{speed_degrees_per_second}\n"
            )
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_path, command_path)
    finally:
        try:
            os.remove(temporary_path)
        except FileNotFoundError:
            pass

    accepted_marker = f"move_command_applied request={request_id} "
    rejected_marker = f"move_command_rejected request={request_id} "
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        pid = read_pidfile(master_id)
        if pid is None or not is_expected_hold_helper(pid, master_id):
            raise RuntimeError(f"master{master_id} hold helper stopped unexpectedly")

        recent_log = tail_file(hold_log_path(master_id), max_lines=80)
        for line in recent_log.splitlines():
            if accepted_marker in line:
                return line
            if rejected_marker in line:
                raise RuntimeError(line)
        time.sleep(0.05)

    raise RuntimeError(
        f"master{master_id} hold helper did not acknowledge move command "
        f"within {timeout_s}s"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Move one EtherCAT joint after OP/CSP has already been prepared")
    parser.add_argument(
        "joint",
        nargs="?",
        choices=sorted(JOINT_TO_MASTER_SLAVE.keys()),
        help="Joint to move; omit to use the interactive selector",
    )
    parser.add_argument(
        "--degrees",
        type=float,
        help="Relative move in degrees; prompted interactively when omitted",
    )
    parser.add_argument("--pp-velocity", type=int, default=50000)
    parser.add_argument("--pp-acceleration", type=int, default=100000)
    parser.add_argument("--pp-deceleration", type=int, default=100000)
    parser.add_argument("--speed-deg-s", type=float, default=5.0)
    parser.add_argument("--print-every", type=int, default=200)
    parser.add_argument("--hold-print-every", type=int, default=1000)
    parser.add_argument("--binary", default=DEFAULT_BINARY)
    parser.add_argument("--op-timeout", type=float, default=60.0)
    parser.add_argument("--move-timeout", type=float, default=20.0)
    parser.add_argument("--stall-timeout", type=float, default=5.0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    if not os.path.exists(args.binary):
        raise SystemExit(f"Binary not found: {args.binary}")

    if args.joint is None:
        if not sys.stdin.isatty():
            raise SystemExit(
                "No joint specified and stdin is not interactive. "
                "Use: calibrate_joint_ethercat.py JOINT --degrees DEG"
            )
        joints = sorted(JOINT_TO_MASTER_SLAVE.keys())
        print("Available joints:")
        for index, joint_name in enumerate(joints, start=1):
            print(f"  {index:2d}. {joint_name}")
        selection = input("Select joint number or name: ").strip()
        if selection.isdigit() and 1 <= int(selection) <= len(joints):
            args.joint = joints[int(selection) - 1]
        elif selection in JOINT_TO_MASTER_SLAVE:
            args.joint = selection
        else:
            raise SystemExit(f"Invalid joint selection: {selection}")

    if args.degrees is None:
        if not sys.stdin.isatty():
            raise SystemExit("--degrees is required when stdin is not interactive")
        try:
            args.degrees = float(input("Relative move in degrees [-45, 45]: ").strip())
        except ValueError as exc:
            raise SystemExit("Invalid degree value") from exc

    if not -45.0 <= args.degrees <= 45.0:
        raise SystemExit("Relative move must be within [-45, 45] degrees")
    if not 0.0 < args.speed_deg_s <= 90.0:
        raise SystemExit("Speed must be within (0, 90] deg/s")

    master_id, slave_pos = JOINT_TO_MASTER_SLAVE[args.joint]
    print(f"joint: {args.joint}")
    print(f"master_id: {master_id}")
    print(f"slave_pos: {slave_pos}")
    print(f"degrees: {args.degrees}")
    print(f"speed_deg_s: {args.speed_deg_s}")
    print(f"command file: {move_command_path(master_id)}")

    if args.dry_run:
        return

    try:
        rows = ensure_prepared_state(master_id, args.op_timeout)
        print_state_rows(rows, "verified prepared OP/CSP state:")
        print("\nsending position command to running hold helper...", flush=True)
        acknowledgement = submit_move_command(
            master_id,
            slave_pos,
            args.degrees,
            args.speed_deg_s,
            args.move_timeout,
        )
        print(acknowledgement, flush=True)
        wait_for_master_operation(master_id, args.op_timeout)
        wait_for_slave_operation(master_id, args.op_timeout)
        print("move command accepted; hold helper remains in control", flush=True)
    except KeyboardInterrupt:
        print("\ninterrupted by user", flush=True)
        raise SystemExit(130)
    except Exception as exc:
        print(f"\nmove failed: {exc}", flush=True)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
