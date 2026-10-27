#!/usr/bin/env bash

# 单个 EtherCAT 从站 OP 调试工具。
# up 会启动底层 helper 配置目标从站的 PDO/DC 并周期发帧；
# helper 始终写 0x6040=0x0006，不进入 CiA402 Operation Enabled。

set -Eeuo pipefail

MASTER=""
SLAVE=""
ACTION=""
TARGET=""
TIMEOUT_S=60
FORCE=0
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BINARY="${SCRIPT_DIR}/../../install/kaixuan_can_hardware/lib/kaixuan_can_hardware/ethercat_direct_tool"
STATE_DIR="/tmp/ethercat_single_op"

usage()
{
    cat <<'EOF'
用法：
  sudo bash joint_op.bash <up|down|cycle|status|init|safeop> <joint_name>
  sudo bash joint_op.bash -m <0|1> -s <0..6> <up|down|cycle|status|init|safeop>

操作：
  up       配置单个从站 PDO/DC，启动周期帧并进入 OP
  down     停止单从站 helper，安全释放 Master 并回到 PREOP
  cycle    PREOP -> OP -> PREOP，用于排查某个从站的状态切换
  status   只显示该从站当前状态
  init     请求单个从站进入 INIT
  safeop   请求单个从站进入 SAFEOP

选项：
  --timeout <秒>  等待状态切换的时间，默认 60 秒
  --binary <路径>  指定 ethercat_direct_tool
  --force         仅对 init/safeop：Master激活时仍执行（不推荐）

关节名：
  right_joint1 .. right_joint7  -> Master 0, Slave 0..6
  left_joint1  .. left_joint7   -> Master 1, Slave 0..6

示例：
  sudo bash joint_op.bash status left_joint6
  sudo bash joint_op.bash down left_joint6
  sudo bash joint_op.bash up left_joint6
  sudo bash joint_op.bash -m 1 -s 5 cycle

重要：
  同一 Master 一次只能运行一个单从站 helper。
  目标从站会进入 EtherCAT OP，但 helper 始终写 0x6040=0x0006，
  不使能电机，也不执行位置运动。
EOF
}

die()
{
    printf '错误: %s\n' "$*" >&2
    exit 1
}

run_ethercat()
{
    if [[ ${EUID} -eq 0 ]]; then
        ethercat "$@"
    else
        sudo ethercat "$@"
    fi
}

ensure_state_dir()
{
    mkdir -p "${STATE_DIR}"
    chmod 1777 "${STATE_DIR}"
}

pid_path()
{
    printf '%s/master%s.pid\n' "${STATE_DIR}" "${MASTER}"
}

log_path()
{
    printf '%s/master%s.log\n' "${STATE_DIR}" "${MASTER}"
}

read_helper_pid()
{
    local path
    path="$(pid_path)"
    [[ -f ${path} ]] || return 1
    local pid
    read -r pid < "${path}" || return 1
    [[ ${pid} =~ ^[1-9][0-9]*$ ]] || return 1
    process_alive "${pid}" || return 1
    printf '%s\n' "${pid}"
}

process_alive()
{
    local pid=$1
    kill -0 "${pid}" 2>/dev/null || return 1
    [[ -r /proc/${pid}/stat ]] || return 1
    [[ $(awk '{print $3}' "/proc/${pid}/stat" 2>/dev/null) != Z ]]
}

helper_matches()
{
    local pid=$1
    local command_line
    command_line="$(tr '\0' ' ' < "/proc/${pid}/cmdline" 2>/dev/null || true)"
    [[ ${command_line} == *"ethercat_direct_tool"* &&
       ${command_line} == *"--master_id ${MASTER}"* &&
       ${command_line} == *"--hold_op_slave"* ]]
}

helper_slave()
{
    local pid=$1
    local -a arguments=()
    mapfile -d '' -t arguments < "/proc/${pid}/cmdline" 2>/dev/null || return 1
    local index
    for ((index = 0; index + 1 < ${#arguments[@]}; ++index)); do
        if [[ ${arguments[index]} == "--hold_op_slave" ]]; then
            printf '%s\n' "${arguments[index + 1]}"
            return 0
        fi
    done
    return 1
}

slave_status()
{
    run_ethercat slaves -m "${MASTER}" -p "${SLAVE}"
}

current_state()
{
    local output
    output="$(slave_status 2>/dev/null || true)"
    if [[ ${output} =~ [[:space:]](INIT|PREOP|SAFEOP|OP)[[:space:]] ]]; then
        printf '%s\n' "${BASH_REMATCH[1]}"
    else
        printf 'UNKNOWN\n'
    fi
}

wait_for_state()
{
    local expected=$1
    local deadline=$((SECONDS + TIMEOUT_S))
    local state

    while (( SECONDS <= deadline )); do
        state="$(current_state)"
        if [[ ${state} == "${expected}" ]]; then
            printf '成功: Master %s Slave %s 已进入 %s\n' \
                "${MASTER}" "${SLAVE}" "${expected}"
            slave_status
            return 0
        fi
        sleep 0.2
    done

    printf '超时: Master %s Slave %s 未进入 %s，当前状态=%s\n' \
        "${MASTER}" "${SLAVE}" "${expected}" "$(current_state)" >&2
    slave_status >&2 || true
    return 1
}

master_is_active()
{
    local output
    output="$(run_ethercat master -m "${MASTER}" 2>/dev/null || true)"
    grep -Eq '^[[:space:]]*Active:[[:space:]]*yes' <<<"${output}"
}

stop_helper()
{
    ensure_state_dir
    local pid=""
    pid="$(read_helper_pid || true)"
    if [[ -z ${pid} ]]; then
        rm -f "$(pid_path)"
        return 1
    fi
    helper_matches "${pid}" || die "PID ${pid} 不是本脚本启动的单从站 helper"

    local running_slave
    running_slave="$(helper_slave "${pid}" || true)"
    [[ ${running_slave} == "${SLAVE}" ]] || \
        die "Master ${MASTER} 当前 helper 控制 Slave ${running_slave}，不是 Slave ${SLAVE}"

    printf '\n停止单从站 helper: pid=%s Master=%s Slave=%s\n' \
        "${pid}" "${MASTER}" "${SLAVE}"
    kill -TERM "${pid}"
    local deadline=$((SECONDS + 6))
    while process_alive "${pid}" && (( SECONDS <= deadline )); do
        sleep 0.1
    done
    if process_alive "${pid}"; then
        kill -KILL "${pid}"
        die "helper 未在6秒内安全退出，已强制终止"
    fi
    wait "${pid}" 2>/dev/null || true
    rm -f "$(pid_path)"
}

start_helper()
{
    ensure_state_dir
    [[ -x ${BINARY} ]] || die "找不到可执行工具: ${BINARY}，请先编译 kaixuan_can_hardware"

    local old_pid=""
    old_pid="$(read_helper_pid || true)"
    if [[ -n ${old_pid} ]]; then
        local old_slave
        old_slave="$(helper_slave "${old_pid}" || true)"
        die "Master ${MASTER} 已有单从站 helper（pid=${old_pid}, slave=${old_slave}）"
    fi
    rm -f "$(pid_path)"

    if master_is_active; then
        die "Master ${MASTER} 正被 PDO/ROS 进程占用，请先安全停止占用者"
    fi

    local log
    log="$(log_path)"
    : > "${log}"
    setsid "${BINARY}" \
        --master_id "${MASTER}" \
        --hold_op_slave "${SLAVE}" \
        --print_every 1000 \
        </dev/null >"${log}" 2>&1 &
    local pid=$!
    printf '%s\n' "${pid}" > "$(pid_path)"

    sleep 0.3
    if ! process_alive "${pid}"; then
        wait "${pid}" 2>/dev/null || true
        rm -f "$(pid_path)"
        printf '%s\n' "----- helper log -----" >&2
        tail -80 "${log}" >&2 || true
        die "单从站 helper 启动失败"
    fi
    printf 'helper已启动: pid=%s，log=%s\n' "${pid}" "${log}"
}

request_state()
{
    local state=$1
    printf '\n请求 Master %s Slave %s -> %s\n' "${MASTER}" "${SLAVE}" "${state}"
    run_ethercat states -m "${MASTER}" -p "${SLAVE}" "${state}"
    wait_for_state "${state}"
}

resolve_joint()
{
    local joint=$1
    if [[ ${joint} =~ ^right_joint([1-7])$ ]]; then
        MASTER=0
        SLAVE=$((BASH_REMATCH[1] - 1))
    elif [[ ${joint} =~ ^left_joint([1-7])$ ]]; then
        MASTER=1
        SLAVE=$((BASH_REMATCH[1] - 1))
    else
        die "未知关节名: ${joint}"
    fi
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -m|--master)
            [[ $# -ge 2 ]] || die "$1 缺少参数"
            MASTER=$2
            shift 2
            ;;
        -s|--slave)
            [[ $# -ge 2 ]] || die "$1 缺少参数"
            SLAVE=$2
            shift 2
            ;;
        --timeout)
            [[ $# -ge 2 ]] || die "$1 缺少参数"
            TIMEOUT_S=$2
            shift 2
            ;;
        --binary)
            [[ $# -ge 2 ]] || die "$1 缺少参数"
            BINARY=$2
            shift 2
            ;;
        --force)
            FORCE=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        up|down|cycle|status|init|safeop)
            [[ -z ${ACTION} ]] || die "只能指定一个操作"
            ACTION=$1
            shift
            ;;
        right_joint[1-7]|left_joint[1-7])
            [[ -z ${TARGET} ]] || die "只能指定一个关节"
            TARGET=$1
            shift
            ;;
        *)
            usage >&2
            die "未知参数: $1"
            ;;
    esac
done

[[ -n ${ACTION} ]] || { usage >&2; die "必须指定操作"; }
[[ ${TIMEOUT_S} =~ ^[1-9][0-9]*$ ]] || die "--timeout 必须是正整数"

if [[ -n ${TARGET} ]]; then
    [[ -z ${MASTER} && -z ${SLAVE} ]] || die "关节名不能和 -m/-s 同时使用"
    resolve_joint "${TARGET}"
else
    [[ -n ${MASTER} && -n ${SLAVE} ]] || die "必须指定关节名，或同时指定 -m 和 -s"
fi

[[ ${MASTER} =~ ^[01]$ ]] || die "Master 必须是 0 或 1"
[[ ${SLAVE} =~ ^[0-6]$ ]] || die "Slave 必须是 0..6"
command -v ethercat >/dev/null 2>&1 || die "找不到 ethercat 命令"
command -v setsid >/dev/null 2>&1 || die "找不到 setsid 命令"

printf 'Master %s Slave %s，当前状态: %s\n' \
    "${MASTER}" "${SLAVE}" "$(current_state)"
slave_status || die "无法读取该从站，请检查 Master 和总线扫描"

if [[ ${ACTION} == init || ${ACTION} == safeop ]]; then
    master_output="$(run_ethercat master -m "${MASTER}" 2>/dev/null || true)"
    if [[ ${FORCE} -ne 1 ]] &&
       grep -Eq '^[[:space:]]*Active:[[:space:]]*yes' <<<"${master_output}"; then
        die "Master ${MASTER} 正被周期 PDO/ROS 进程激活。请先安全停止占用者；确认风险后可用 --force。"
    fi
fi

case "${ACTION}" in
    status)
        ensure_state_dir
        helper_pid="$(read_helper_pid || true)"
        if [[ -n ${helper_pid} ]]; then
            printf 'single helper: pid=%s slave=%s log=%s\n' \
                "${helper_pid}" "$(helper_slave "${helper_pid}" || true)" "$(log_path)"
        else
            printf 'single helper: stopped\n'
        fi
        ;;
    up)
        start_helper
        if ! wait_for_state OP; then
            printf '%s\n' "----- helper log -----" >&2
            tail -80 "$(log_path)" >&2 || true
            stop_helper || true
            exit 1
        fi
        printf '%s\n' "单从站 PDO 周期正在后台运行；使用 down 安全停止。"
        ;;
    down)
        if ! stop_helper; then
            die "Master ${MASTER} 没有本脚本启动的单从站 helper"
        fi
        wait_for_state PREOP
        ;;
    init)
        request_state INIT
        ;;
    safeop)
        request_state SAFEOP
        ;;
    cycle)
        start_helper
        if ! wait_for_state OP; then
            printf '%s\n' "----- helper log -----" >&2
            tail -80 "$(log_path)" >&2 || true
            stop_helper || true
            exit 1
        fi
        sleep 1
        stop_helper
        wait_for_state PREOP
        ;;
esac
