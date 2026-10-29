#!/usr/bin/env python3

import argparse
import os
import shlex
import signal
import subprocess
import sys
import time
from typing import Optional


WORKSPACE_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_BINARY = os.path.join(
    WORKSPACE_ROOT,
    "install",
    "kaixuan_can_hardware",
    "lib",
    "kaixuan_can_hardware",
    "ethercat_direct_tool",
)
CONFLICT_PATTERN = (
    "ros2 launch kaixuan3_config moveit_bringup.launch.py|"
    "move_group|"
    "ros2_control_node|"
    "robot_state_publisher|"
    "controller_manager/spawner|"
    "ethercat_direct_tool"
)
STATE_DIR = "/tmp/ethercat_op_csp"


def ensure_state_dir() -> None:
    os.makedirs(STATE_DIR, exist_ok=True)
    os.chmod(STATE_DIR, 0o1777)


def hold_pid_path(master_id: int) -> str:
    ensure_state_dir()
    return os.path.join(STATE_DIR, f"master{master_id}.pid")


def hold_log_path(master_id: int) -> str:
    ensure_state_dir()
    return os.path.join(STATE_DIR, f"master{master_id}.log")


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


def stop_all_hold_helpers() -> None:
    for master_id in (0, 1):
        stop_hold_helper(master_id)


def any_hold_helper_running(master_ids: tuple[int, ...] = (0, 1)) -> bool:
    for master_id in master_ids:
        pid = read_pidfile(master_id)
        if pid is not None and is_expected_hold_helper(pid, master_id):
            return True
    return False


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
        if "prepare_ethercat_op_csp.py" in stripped:
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


def stop_conflicts() -> None:
    subprocess.run(
        ["bash", "-lc", f"pkill -f {shlex.quote(CONFLICT_PATTERN)} || true"],
        check=False,
    )
    time.sleep(1.0)


def run_checked(command: list[str], label: str, timeout: Optional[float] = None) -> None:
    completed = subprocess.run(
        command,
        capture_output=True,
        text=True,
        check=False,
        timeout=timeout,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"{label} failed with code {completed.returncode}\n"
            f"stdout:\n{completed.stdout}\n"
            f"stderr:\n{completed.stderr}"
        )


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


def collect_helper_rows(master_ids: tuple[int, ...]) -> Optional[list[tuple[int, int, int, int, int]]]:
    rows: list[tuple[int, int, int, int, int]] = []
    for master_id in master_ids:
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
    master_ids: tuple[int, ...], timeout_s: float
) -> list[tuple[int, int, int, int, int]]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rows = collect_helper_rows(master_ids)
        if rows and all(row[2] == 8 and row[3] != 0 for row in rows):
            return rows
        time.sleep(0.2)

    log_text = "\n".join(
        f"master{master_id} log tail:\n{tail_file(hold_log_path(master_id))}"
        for master_id in master_ids
    )
    axis_count = len(master_ids) * 7
    raise RuntimeError(
        f"helpers did not report all {axis_count} selected axes in CSP within {timeout_s}s\n"
        f"{log_text}"
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


def wait_for_system_operation(master_ids: tuple[int, ...], timeout_s: float) -> None:
    for master_id in master_ids:
        wait_for_master_operation(master_id, timeout_s)
        wait_for_slave_operation(master_id, timeout_s)


def print_state_rows(rows: list[tuple[int, int, int, int, int]], title: str) -> None:
    print(f"\n{title}", flush=True)
    for master_id, slave_pos, mode_display, status_word, actual_position in rows:
        print(
            f"  m={master_id} s={slave_pos} 6061={mode_display} 6041=0x{status_word:04x} 6064={actual_position}",
            flush=True,
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Start background OP/CSP hold loops on the selected EtherCAT arm(s)"
    )
    parser.add_argument("--binary", default=DEFAULT_BINARY)
    parser.add_argument(
        "--active-arm",
        "--active_arm",
        choices=("both", "right", "left"),
        default="both",
        help="Arm(s) to prepare: right=master0, left=master1 (default: both)",
    )
    parser.add_argument("--print-every", type=int, default=1000)
    parser.add_argument("--op-timeout", type=float, default=60.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--stop-conflicts", action="store_true")
    parser.add_argument("--stop", action="store_true", help="Ask background hold helpers to send 0x6040=0x0006 and exit")
    args = parser.parse_args()
    master_ids = {
        "both": (0, 1),
        "right": (0,),
        "left": (1,),
    }[args.active_arm]

    if args.stop:
        if any_hold_helper_running(master_ids):
            print(
                f"requesting {args.active_arm} hold helper(s) to send "
                "0x6040 = 0x0006 and exit...",
                flush=True,
            )
        else:
            print(f"no running {args.active_arm} hold helpers detected", flush=True)
        for master_id in master_ids:
            stop_hold_helper(master_id)
        print(f"stopped {args.active_arm} background hold helper(s)", flush=True)
        return

    if not os.path.exists(args.binary):
        raise SystemExit(f"Binary not found: {args.binary}")

    print("binary:")
    print(f"  {args.binary}")
    print(f"active_arm: {args.active_arm} (masters: {', '.join(map(str, master_ids))})")
    print(f"print_every: {args.print_every}")

    if args.dry_run:
        return

    for master_id in master_ids:
        stop_hold_helper(master_id)

    # A helper holding the unselected arm is allowed to remain running.
    ignored_helper_pids = set()
    for master_id in (0, 1):
        if master_id in master_ids:
            continue
        pid = read_pidfile(master_id)
        if pid is not None and is_expected_hold_helper(pid, master_id):
            ignored_helper_pids.add(pid)

    conflicts = find_conflicts(ignored_helper_pids)
    if conflicts:
        print("\nconflicting processes detected:", flush=True)
        for line in conflicts:
            print(f"  {line}", flush=True)

        if args.stop_conflicts:
            print("\nstopping conflicting processes...", flush=True)
            stop_conflicts()
        else:
            print(
                "\nrefusing to run because EtherCAT masters are likely already reserved.\n"
                "Stop the current real-hardware bringup first, or rerun with --stop-conflicts.",
                flush=True,
            )
            raise SystemExit(2)

    started_masters: list[int] = []
    try:
        for master_id in master_ids:
            pid = start_hold_helper(master_id, args.binary, args.print_every)
            started_masters.append(master_id)
            print(f"started background hold helper on master {master_id} (pid={pid})", flush=True)

        axis_count = len(master_ids) * 7
        print(
            f"\nwaiting for selected master(s) and all {axis_count} selected slaves to enter OP...",
            flush=True,
        )
        wait_for_system_operation(master_ids, args.op_timeout)

        print(f"waiting for hold helpers to report all {axis_count} selected axes in CSP...", flush=True)
        rows = wait_for_helper_csp_state(master_ids, args.op_timeout)
        print_state_rows(rows, "verified OP/CSP state:")

        print(
            "\nbackground hold helpers remain running.\n"
            "stop them with:\n"
            f"  sudo python3 prepare_ethercat_op_csp.py --active-arm {args.active_arm} --stop",
            flush=True,
        )
    except KeyboardInterrupt:
        print("\ninterrupted by user", flush=True)
        for master_id in started_masters:
            stop_hold_helper(master_id)
        raise SystemExit(130)
    except Exception as exc:
        print(f"\nprepare failed: {exc}", flush=True)
        for master_id in started_masters:
            stop_hold_helper(master_id)
        raise SystemExit(1)


if __name__ == "__main__":
    main()




# #!/usr/bin/env python3

# import argparse
# import os
# import shlex
# import signal
# import subprocess
# import time
# from typing import Optional


# DEFAULT_BINARY = (
#     "/home/agxorin/cxl/kaixuan/xiaogong_kaixuan/install/kaixuan_can_hardware/"
#     "lib/kaixuan_can_hardware/ethercat_direct_tool"
# )

# CONFLICT_PATTERN = (
#     "ros2 launch kaixuan3_config moveit_bringup.launch.py|"
#     "move_group|"
#     "ros2_control_node|"
#     "robot_state_publisher|"
#     "controller_manager/spawner|"
#     "ethercat_direct_tool"
# )

# STATE_DIR = "/tmp/ethercat_op_csp"


# # ============================================================
# # 每个 Master 实际的 Slave 数量
# #
# # Master 0:
# #   slave 0 ~ 6，共 7 个
# #
# # Master 1:
# #   slave 0 ~ 5，共 6 个
# # ============================================================

# MASTER_SLAVE_COUNTS = {
#     0: 7,
#     1: 6,
# }


# # ============================================================
# # 获取不同 Master 的 CSP 启动参数
# #
# # Master 0:
# #   7 个 Slave，使用 --hold_csp_all
# #
# # Master 1:
# #   6 个 Slave，使用 --hold_csp_mask 63
# #
# # 63 = 0b0111111
# #
# # bit0 -> slave0
# # bit1 -> slave1
# # bit2 -> slave2
# # bit3 -> slave3
# # bit4 -> slave4
# # bit5 -> slave5
# # ============================================================

# def get_csp_args(master_id: int) -> list[str]:
#     if master_id == 0:
#         return [
#             "--hold_csp_all",
#         ]

#     if master_id == 1:
#         return [
#             "--hold_csp_mask",
#             "63",
#         ]

#     raise ValueError(
#         f"unsupported master_id: {master_id}"
#     )


# # ============================================================
# # 状态目录
# # ============================================================

# def ensure_state_dir() -> None:
#     os.makedirs(
#         STATE_DIR,
#         exist_ok=True,
#     )
#     os.chmod(
#         STATE_DIR,
#         0o1777,
#     )


# def hold_pid_path(master_id: int) -> str:
#     ensure_state_dir()
#     return os.path.join(
#         STATE_DIR,
#         f"master{master_id}.pid",
#     )


# def hold_log_path(master_id: int) -> str:
#     ensure_state_dir()
#     return os.path.join(
#         STATE_DIR,
#         f"master{master_id}.log",
#     )


# # ============================================================
# # PID 文件
# # ============================================================

# def read_pidfile(master_id: int) -> Optional[int]:
#     path = hold_pid_path(master_id)

#     if not os.path.exists(path):
#         return None

#     try:
#         with open(
#             path,
#             "r",
#             encoding="ascii",
#         ) as handle:
#             return int(
#                 handle.read().strip()
#             )
#     except (OSError, ValueError):
#         return None


# def write_pidfile(
#     master_id: int,
#     pid: int,
# ) -> None:
#     with open(
#         hold_pid_path(master_id),
#         "w",
#         encoding="ascii",
#     ) as handle:
#         handle.write(
#             f"{pid}\n"
#         )


# def remove_pidfile(
#     master_id: int,
# ) -> None:
#     try:
#         os.remove(
#             hold_pid_path(master_id)
#         )
#     except FileNotFoundError:
#         pass


# # ============================================================
# # 进程状态
# # ============================================================

# def process_alive(pid: int) -> bool:
#     try:
#         os.kill(
#             pid,
#             0,
#         )
#     except OSError:
#         return False

#     return True


# def read_process_cmdline(
#     pid: int,
# ) -> list[str]:
#     try:
#         with open(
#             f"/proc/{pid}/cmdline",
#             "rb",
#         ) as handle:
#             raw_parts = (
#                 handle
#                 .read()
#                 .split(b"\0")
#             )
#     except OSError:
#         return []

#     return [
#         part.decode(
#             "utf-8",
#             errors="replace",
#         )
#         for part in raw_parts
#         if part
#     ]


# def is_expected_hold_helper(
#     pid: int,
#     master_id: int,
# ) -> bool:
#     if not process_alive(pid):
#         return False

#     argv = read_process_cmdline(pid)

#     if not argv:
#         return False

#     executable = os.path.basename(
#         argv[0]
#     )

#     if executable != "ethercat_direct_tool":
#         return False

#     if "--move_slave" in argv:
#         return False

#     try:
#         arg_master_id = argv[
#             argv.index("--master_id") + 1
#         ]
#     except (ValueError, IndexError):
#         return False

#     return (
#         arg_master_id
#         == str(master_id)
#     )


# # ============================================================
# # 日志
# # ============================================================

# def tail_file(
#     path: str,
#     max_lines: int = 40,
# ) -> str:
#     try:
#         with open(
#             path,
#             "r",
#             encoding="utf-8",
#             errors="replace",
#         ) as handle:
#             lines = handle.readlines()
#     except OSError:
#         return ""

#     return "".join(
#         lines[-max_lines:]
#     )


# # ============================================================
# # 结束进程
# # ============================================================

# def terminate_pid(
#     pid: Optional[int],
#     label: str,
# ) -> None:
#     if pid is None or not process_alive(pid):
#         return

#     try:
#         os.killpg(
#             pid,
#             signal.SIGTERM,
#         )
#     except ProcessLookupError:
#         return

#     deadline = (
#         time.monotonic()
#         + 5.0
#     )

#     while time.monotonic() < deadline:
#         if not process_alive(pid):
#             return

#         time.sleep(0.1)

#     print(
#         f"forcing shutdown of {label}",
#         flush=True,
#     )

#     try:
#         os.killpg(
#             pid,
#             signal.SIGKILL,
#         )
#     except ProcessLookupError:
#         return


# def stop_hold_helper(
#     master_id: int,
# ) -> None:
#     pid = read_pidfile(
#         master_id
#     )

#     if (
#         pid is not None
#         and is_expected_hold_helper(
#             pid,
#             master_id,
#         )
#     ):
#         terminate_pid(
#             pid,
#             f"master{master_id} hold helper",
#         )

#     remove_pidfile(
#         master_id
#     )


# def stop_all_hold_helpers() -> None:
#     for master_id in (0, 1):
#         stop_hold_helper(
#             master_id
#         )


# def any_hold_helper_running(
#     master_ids: tuple[int, ...] = (0, 1),
# ) -> bool:
#     for master_id in master_ids:
#         pid = read_pidfile(
#             master_id
#         )

#         if (
#             pid is not None
#             and is_expected_hold_helper(
#                 pid,
#                 master_id,
#             )
#         ):
#             return True

#     return False


# # ============================================================
# # 启动 EtherCAT helper
# # ============================================================

# def start_hold_helper(
#     master_id: int,
#     binary: str,
#     print_every: int,
# ) -> int:

#     stop_hold_helper(
#         master_id
#     )

#     log_path = hold_log_path(
#         master_id
#     )

#     csp_args = get_csp_args(
#         master_id
#     )

#     print(
#         f"master{master_id} CSP mode: "
#         f"{' '.join(csp_args)}",
#         flush=True,
#     )

#     log_file = open(
#         log_path,
#         "w",
#         encoding="utf-8",
#     )

#     try:
#         proc = subprocess.Popen(
#             [
#                 binary,
#                 "--master_id",
#                 str(master_id),
#                 "--print_every",
#                 str(print_every),
#                 *csp_args,
#             ],
#             stdout=log_file,
#             stderr=subprocess.STDOUT,
#             text=True,
#             bufsize=1,
#             start_new_session=True,
#         )
#     finally:
#         log_file.close()

#     time.sleep(0.2)

#     if proc.poll() is not None:
#         raise RuntimeError(
#             f"master{master_id} hold helper "
#             f"exited immediately with code "
#             f"{proc.returncode}\n"
#             f"recent log:\n"
#             f"{tail_file(log_path)}"
#         )

#     write_pidfile(
#         master_id,
#         proc.pid,
#     )

#     return proc.pid


# # ============================================================
# # 查找冲突进程
# # ============================================================

# def find_conflicts(
#     ignore_pids: set[int],
# ) -> list[str]:

#     result = subprocess.run(
#         [
#             "bash",
#             "-lc",
#             f"pgrep -af "
#             f"{shlex.quote(CONFLICT_PATTERN)} "
#             f"|| true",
#         ],
#         capture_output=True,
#         text=True,
#         check=False,
#     )

#     conflicts = []

#     for line in result.stdout.splitlines():
#         stripped = line.strip()

#         if not stripped:
#             continue

#         if "pgrep -af" in stripped:
#             continue

#         if "prepare_ethercat_op_csp.py" in stripped:
#             continue

#         parts = stripped.split(
#             maxsplit=1
#         )

#         try:
#             pid = int(
#                 parts[0]
#             )
#         except (ValueError, IndexError):
#             pid = None

#         if (
#             pid is not None
#             and pid in ignore_pids
#         ):
#             continue

#         conflicts.append(
#             stripped
#         )

#     return conflicts


# def stop_conflicts() -> None:
#     subprocess.run(
#         [
#             "bash",
#             "-lc",
#             f"pkill -f "
#             f"{shlex.quote(CONFLICT_PATTERN)} "
#             f"|| true",
#         ],
#         check=False,
#     )

#     time.sleep(1.0)


# # ============================================================
# # EtherCAT 命令
# # ============================================================

# def ethercat_output(
#     args: list[str],
#     timeout: float = 5.0,
# ) -> str:

#     completed = subprocess.run(
#         [
#             "ethercat",
#             *args,
#         ],
#         capture_output=True,
#         text=True,
#         check=False,
#         timeout=timeout,
#     )

#     if completed.returncode != 0:
#         raise RuntimeError(
#             f"ethercat {' '.join(args)} "
#             f"failed with code "
#             f"{completed.returncode}\n"
#             f"stdout:\n"
#             f"{completed.stdout}\n"
#             f"stderr:\n"
#             f"{completed.stderr}"
#         )

#     return completed.stdout


# # ============================================================
# # 读取最新 snapshot
# # ============================================================

# def read_latest_snapshot(
#     log_path: str,
#     master_id: int,
# ) -> Optional[
#     tuple[
#         int,
#         dict[
#             int,
#             tuple[
#                 int,
#                 int,
#                 int,
#             ],
#         ],
#     ]
# ]:

#     try:
#         with open(
#             log_path,
#             "r",
#             encoding="utf-8",
#             errors="replace",
#         ) as handle:
#             lines = handle.readlines()
#     except OSError:
#         return None

#     prefix = (
#         f"master={master_id} "
#         f"snapshot="
#     )

#     for index in range(
#         len(lines) - 1,
#         -1,
#         -1,
#     ):

#         header = lines[index].strip()

#         if not header.startswith(
#             prefix
#         ):
#             continue

#         try:
#             snapshot = int(
#                 header
#                 .split(
#                     "snapshot=",
#                     1,
#                 )[1]
#                 .split()[0]
#             )
#         except (
#             IndexError,
#             ValueError,
#         ):
#             continue

#         axes: dict[
#             int,
#             tuple[
#                 int,
#                 int,
#                 int,
#             ],
#         ] = {}

#         for raw_line in lines[
#             index + 1:
#         ]:

#             line = raw_line.strip()

#             if (
#                 not line
#                 or line
#                 == (
#                     "axis slave offset "
#                     "mode status position"
#                 )
#             ):
#                 continue

#             if line.startswith(
#                 "master="
#             ):
#                 break

#             parts = line.split()

#             if (
#                 len(parts) != 6
#                 or not parts[0].isdigit()
#             ):
#                 continue

#             try:
#                 slave_pos = int(
#                     parts[1]
#                 )
#                 mode_display = int(
#                     parts[3]
#                 )
#                 status_word = int(
#                     parts[4]
#                 )
#                 actual_position = int(
#                     parts[5]
#                 )
#             except ValueError:
#                 continue

#             axes[
#                 slave_pos
#             ] = (
#                 mode_display,
#                 status_word,
#                 actual_position,
#             )

#         expected_count = (
#             MASTER_SLAVE_COUNTS[
#                 master_id
#             ]
#         )

#         if len(axes) >= expected_count:
#             return (
#                 snapshot,
#                 axes,
#             )

#     return None


# # ============================================================
# # 收集 helper 状态
# # ============================================================

# def collect_helper_rows(
#     master_ids: tuple[int, ...],
# ) -> Optional[
#     list[
#         tuple[
#             int,
#             int,
#             int,
#             int,
#             int,
#         ]
#     ]
# ]:

#     rows: list[
#         tuple[
#             int,
#             int,
#             int,
#             int,
#             int,
#         ]
#     ] = []

#     for master_id in master_ids:

#         snapshot = read_latest_snapshot(
#             hold_log_path(
#                 master_id
#             ),
#             master_id,
#         )

#         if snapshot is None:
#             return None

#         _snapshot_index, axes = (
#             snapshot
#         )

#         slave_count = (
#             MASTER_SLAVE_COUNTS[
#                 master_id
#             ]
#         )

#         for slave_pos in range(
#             slave_count
#         ):

#             axis_data = axes.get(
#                 slave_pos
#             )

#             if axis_data is None:
#                 return None

#             (
#                 mode_display,
#                 status_word,
#                 actual_position,
#             ) = axis_data

#             rows.append(
#                 (
#                     master_id,
#                     slave_pos,
#                     mode_display,
#                     status_word,
#                     actual_position,
#                 )
#             )

#     return rows


# # ============================================================
# # 等待 CSP
# # ============================================================

# def wait_for_helper_csp_state(
#     master_ids: tuple[int, ...],
#     timeout_s: float,
# ) -> list[
#     tuple[
#         int,
#         int,
#         int,
#         int,
#         int,
#     ]
# ]:

#     deadline = (
#         time.monotonic()
#         + timeout_s
#     )

#     while time.monotonic() < deadline:

#         rows = collect_helper_rows(
#             master_ids
#         )

#         if rows and all(
#             row[2] == 8
#             and row[3] != 0
#             for row in rows
#         ):
#             return rows

#         time.sleep(0.2)

#     # --------------------------------------------------------
#     # 超时后收集日志
#     # --------------------------------------------------------

#     log_parts = []

#     for master_id in master_ids:

#         log_path = hold_log_path(
#             master_id
#         )

#         log_tail = tail_file(
#             log_path
#         )

#         log_parts.append(
#             f"master{master_id} log tail:\n"
#             f"{log_tail}"
#         )

#     log_text = "\n".join(
#         log_parts
#     )

#     axis_count = sum(
#         MASTER_SLAVE_COUNTS[
#             master_id
#         ]
#         for master_id in master_ids
#     )

#     raise RuntimeError(
#         f"helpers did not report all "
#         f"{axis_count} selected axes "
#         f"in CSP within "
#         f"{timeout_s}s\n"
#         f"{log_text}"
#     )


# # ============================================================
# # 等待 Master 进入 OP
# # ============================================================

# def wait_for_master_operation(
#     master_id: int,
#     timeout_s: float,
# ) -> None:

#     deadline = (
#         time.monotonic()
#         + timeout_s
#     )

#     last_output = ""

#     while time.monotonic() < deadline:

#         last_output = ethercat_output(
#             [
#                 "-m",
#                 str(master_id),
#                 "master",
#             ]
#         )

#         if (
#             "Phase: Operation"
#             in last_output
#             and "Active: yes"
#             in last_output
#         ):
#             return

#         time.sleep(0.2)

#     raise RuntimeError(
#         f"master{master_id} did not "
#         f"enter OP within "
#         f"{timeout_s}s\n"
#         f"{last_output}"
#     )


# # ============================================================
# # 等待所有实际 Slave 进入 OP
# # ============================================================

# def wait_for_slave_operation(
#     master_id: int,
#     timeout_s: float,
# ) -> None:

#     deadline = (
#         time.monotonic()
#         + timeout_s
#     )

#     last_output = ""

#     expected_slave_count = (
#         MASTER_SLAVE_COUNTS[
#             master_id
#         ]
#     )

#     while time.monotonic() < deadline:

#         last_output = ethercat_output(
#             [
#                 "-m",
#                 str(master_id),
#                 "slaves",
#             ]
#         )

#         lines = [
#             line.strip()
#             for line in last_output.splitlines()
#             if line.strip()
#         ]

#         # ----------------------------------------------------
#         # ethercat slaves 通常会有 Master0/Master1 这一行，
#         # 所以这里只统计真正的 Slave 行。
#         # ----------------------------------------------------

#         slave_lines = []

#         for line in lines:

#             parts = line.split()

#             if (
#                 len(parts) >= 3
#                 and parts[0].isdigit()
#             ):
#                 slave_lines.append(
#                     line
#                 )

#         if (
#             len(slave_lines)
#             == expected_slave_count
#             and all(
#                 " OP "
#                 in f" {line} "
#                 for line in slave_lines
#             )
#         ):
#             return

#         time.sleep(0.2)

#     raise RuntimeError(
#         f"master{master_id} slaves did not "
#         f"all enter OP within "
#         f"{timeout_s}s\n"
#         f"expected "
#         f"{expected_slave_count} slaves\n"
#         f"{last_output}"
#     )


# # ============================================================
# # 等待所有 Master 进入 OP
# # ============================================================

# def wait_for_system_operation(
#     master_ids: tuple[int, ...],
#     timeout_s: float,
# ) -> None:

#     for master_id in master_ids:

#         wait_for_master_operation(
#             master_id,
#             timeout_s,
#         )

#         wait_for_slave_operation(
#             master_id,
#             timeout_s,
#         )


# # ============================================================
# # 打印最终状态
# # ============================================================

# def print_state_rows(
#     rows: list[
#         tuple[
#             int,
#             int,
#             int,
#             int,
#             int,
#         ]
#     ],
#     title: str,
# ) -> None:

#     print(
#         f"\n{title}",
#         flush=True,
#     )

#     for (
#         master_id,
#         slave_pos,
#         mode_display,
#         status_word,
#         actual_position,
#     ) in rows:

#         print(
#             f"  m={master_id} "
#             f"s={slave_pos} "
#             f"6061={mode_display} "
#             f"6041=0x{status_word:04x} "
#             f"6064={actual_position}",
#             flush=True,
#         )


# # ============================================================
# # MAIN
# # ============================================================

# def main() -> None:

#     parser = argparse.ArgumentParser(
#         description=(
#             "Start background OP/CSP "
#             "hold loops on the selected "
#             "EtherCAT arm(s)"
#         )
#     )

#     parser.add_argument(
#         "--binary",
#         default=DEFAULT_BINARY,
#     )

#     parser.add_argument(
#         "--active-arm",
#         "--active_arm",
#         choices=(
#             "both",
#             "right",
#             "left",
#         ),
#         default="both",
#         help=(
#             "Arm(s) to prepare: "
#             "right=master0, "
#             "left=master1 "
#             "(default: both)"
#         ),
#     )

#     parser.add_argument(
#         "--print-every",
#         type=int,
#         default=1000,
#     )

#     parser.add_argument(
#         "--op-timeout",
#         type=float,
#         default=60.0,
#     )

#     parser.add_argument(
#         "--dry-run",
#         action="store_true",
#     )

#     parser.add_argument(
#         "--stop-conflicts",
#         action="store_true",
#     )

#     parser.add_argument(
#         "--stop",
#         action="store_true",
#         help=(
#             "Ask background hold helpers "
#             "to send 0x6040=0x0006 "
#             "and exit"
#         ),
#     )

#     args = parser.parse_args()

#     master_ids = {
#         "both": (
#             0,
#             1,
#         ),
#         "right": (
#             0,
#         ),
#         "left": (
#             1,
#         ),
#     }[
#         args.active_arm
#     ]

#     # ========================================================
#     # STOP
#     # ========================================================

#     if args.stop:

#         if any_hold_helper_running(
#             master_ids
#         ):

#             print(
#                 f"requesting "
#                 f"{args.active_arm} "
#                 f"hold helper(s) to send "
#                 f"0x6040 = 0x0006 "
#                 f"and exit...",
#                 flush=True,
#             )

#         else:

#             print(
#                 f"no running "
#                 f"{args.active_arm} "
#                 f"hold helpers detected",
#                 flush=True,
#             )

#         for master_id in master_ids:

#             stop_hold_helper(
#                 master_id
#             )

#         print(
#             f"stopped "
#             f"{args.active_arm} "
#             f"background hold helper(s)",
#             flush=True,
#         )

#         return

#     # ========================================================
#     # 检查 binary
#     # ========================================================

#     if not os.path.exists(
#         args.binary
#     ):

#         raise SystemExit(
#             f"Binary not found: "
#             f"{args.binary}"
#         )

#     print(
#         "binary:"
#     )

#     print(
#         f"  {args.binary}"
#     )

#     print(
#         f"active_arm: "
#         f"{args.active_arm} "
#         f"(masters: "
#         f"{', '.join(map(str, master_ids))})"
#     )

#     print(
#         "slave counts:"
#     )

#     for master_id in master_ids:

#         print(
#             f"  master{master_id}: "
#             f"{MASTER_SLAVE_COUNTS[master_id]} "
#             f"slaves"
#         )

#     print(
#         "CSP modes:"
#     )

#     for master_id in master_ids:

#         csp_args = get_csp_args(
#             master_id
#         )

#         print(
#             f"  master{master_id}: "
#             f"{' '.join(csp_args)}"
#         )

#     print(
#         f"print_every: "
#         f"{args.print_every}"
#     )

#     if args.dry_run:
#         return

#     # ========================================================
#     # 停止当前选择的 helper
#     # ========================================================

#     for master_id in master_ids:

#         stop_hold_helper(
#             master_id
#         )

#     # ========================================================
#     # 未选择 Master 的 helper 可以继续运行
#     # ========================================================

#     ignored_helper_pids = set()

#     for master_id in (
#         0,
#         1,
#     ):

#         if master_id in master_ids:
#             continue

#         pid = read_pidfile(
#             master_id
#         )

#         if (
#             pid is not None
#             and is_expected_hold_helper(
#                 pid,
#                 master_id,
#             )
#         ):
#             ignored_helper_pids.add(
#                 pid
#             )

#     # ========================================================
#     # 检查冲突
#     # ========================================================

#     conflicts = find_conflicts(
#         ignored_helper_pids
#     )

#     if conflicts:

#         print(
#             "\nconflicting "
#             "processes detected:",
#             flush=True,
#         )

#         for line in conflicts:

#             print(
#                 f"  {line}",
#                 flush=True,
#             )

#         if args.stop_conflicts:

#             print(
#                 "\nstopping "
#                 "conflicting processes...",
#                 flush=True,
#             )

#             stop_conflicts()

#         else:

#             print(
#                 "\nrefusing to run because "
#                 "EtherCAT masters are likely "
#                 "already reserved.\n"
#                 "Stop the current real-hardware "
#                 "bringup first, or rerun with "
#                 "--stop-conflicts.",
#                 flush=True,
#             )

#             raise SystemExit(2)

#     # ========================================================
#     # 启动 helper
#     # ========================================================

#     started_masters: list[int] = []

#     try:

#         for master_id in master_ids:

#             pid = start_hold_helper(
#                 master_id,
#                 args.binary,
#                 args.print_every,
#             )

#             started_masters.append(
#                 master_id
#             )

#             print(
#                 f"started background "
#                 f"hold helper on "
#                 f"master {master_id} "
#                 f"(pid={pid})",
#                 flush=True,
#             )

#         # ====================================================
#         # 计算实际电机总数
#         # ====================================================

#         axis_count = sum(
#             MASTER_SLAVE_COUNTS[
#                 master_id
#             ]
#             for master_id in master_ids
#         )

#         print(
#             f"\nwaiting for selected "
#             f"master(s) and all "
#             f"{axis_count} selected "
#             f"slaves to enter OP...",
#             flush=True,
#         )

#         # ====================================================
#         # 等待 OP
#         # ====================================================

#         wait_for_system_operation(
#             master_ids,
#             args.op_timeout,
#         )

#         # ====================================================
#         # 等待 CSP
#         # ====================================================

#         print(
#             f"waiting for hold helpers "
#             f"to report all "
#             f"{axis_count} selected "
#             f"axes in CSP...",
#             flush=True,
#         )

#         rows = wait_for_helper_csp_state(
#             master_ids,
#             args.op_timeout,
#         )

#         # ====================================================
#         # 输出状态
#         # ====================================================

#         print_state_rows(
#             rows,
#             "verified OP/CSP state:",
#         )

#         print(
#             "\nbackground hold helpers "
#             "remain running.",
#             flush=True,
#         )

#         print(
#             "stop them with:",
#             flush=True,
#         )

#         print(
#             f"  sudo python3 "
#             f"prepare_ethercat_op_csp.py "
#             f"--active-arm "
#             f"{args.active_arm} "
#             f"--stop",
#             flush=True,
#         )

#     except KeyboardInterrupt:

#         print(
#             "\ninterrupted by user",
#             flush=True,
#         )

#         for master_id in started_masters:

#             stop_hold_helper(
#                 master_id
#             )

#         raise SystemExit(130)

#     except Exception as exc:

#         print(
#             f"\nprepare failed: "
#             f"{exc}",
#             flush=True,
#         )

#         for master_id in started_masters:

#             stop_hold_helper(
#                 master_id
#             )

#         raise SystemExit(1)


# if __name__ == "__main__":
#     main()
