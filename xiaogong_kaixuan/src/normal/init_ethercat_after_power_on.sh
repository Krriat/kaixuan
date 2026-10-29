#!/usr/bin/env bash

# 上电后的 EtherCAT 初始化：
#   1. 确认 IgH Master 和从站在线
#   2. 复用 prepare_ethercat_op_csp.py 完成 PDO/DC/CSP 初始化
#   3. 停止保持循环并释放 Master，供 ros2_control 后续接管
#   4. 逐轴读取 0x6064:00，验证 CoE/SDO 通信

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PREPARE_SCRIPT="${SCRIPT_DIR}/prepare_ethercat_op_csp.py"
ACTIVE_ARM="right"
HELPER_STARTED=0

usage()
{
    cat <<'EOF'
用法：
  sudo bash init_ethercat_after_power_on.sh [right|left|both]

示例：
  sudo bash init_ethercat_after_power_on.sh right
  sudo bash init_ethercat_after_power_on.sh left
  sudo bash init_ethercat_after_power_on.sh both

默认初始化 right（Master 0）。
初始化过程中所选机械臂的驱动器会短暂进入 OP/CSP，请先确认机械臂周围安全、急停可用。
EOF
}

log()
{
    printf '\n[%s] %s\n' "$(date '+%H:%M:%S')" "$*"
}

die()
{
    printf '错误: %s\n' "$*" >&2
    exit 1
}

stop_helper()
{
    python3 "${PREPARE_SCRIPT}" --active-arm "${ACTIVE_ARM}" --stop
    HELPER_STARTED=0
}

cleanup()
{
    local status=$?
    trap - EXIT INT TERM

    if [[ ${HELPER_STARTED} -eq 1 ]]; then
        printf '\n初始化未正常结束，正在停止后台 PDO 保持程序...\n' >&2
        python3 "${PREPARE_SCRIPT}" --active-arm "${ACTIVE_ARM}" --stop || true
    fi

    exit "${status}"
}

trap cleanup EXIT INT TERM

if [[ $# -gt 1 ]]; then
    usage
    exit 2
fi

if [[ $# -eq 1 ]]; then
    case "$1" in
        right|left|both)
            ACTIVE_ARM="$1"
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            usage
            die "机械臂参数必须是 right、left 或 both"
            ;;
    esac
fi

if [[ ${EUID} -ne 0 ]]; then
    die "请使用 sudo 运行：sudo bash ${0} ${ACTIVE_ARM}"
fi

command -v ethercat >/dev/null 2>&1 || die "找不到 ethercat 命令"
command -v python3 >/dev/null 2>&1 || die "找不到 python3 命令"
[[ -f "${PREPARE_SCRIPT}" ]] || die "找不到 ${PREPARE_SCRIPT}"

case "${ACTIVE_ARM}" in
    right)
        MASTER_IDS=(0)
        ;;
    left)
        MASTER_IDS=(1)
        ;;
    both)
        MASTER_IDS=(0 1)
        ;;
esac

log "上电初始化对象: ${ACTIVE_ARM}（Master ${MASTER_IDS[*]}）"
printf '%s\n' "警告: 初始化阶段驱动器会短暂使能并保持当前位置。"

missing_device=0
for master_id in "${MASTER_IDS[@]}"; do
    if [[ ! -e "/dev/EtherCAT${master_id}" ]]; then
        missing_device=1
    fi
done

if [[ ${missing_device} -eq 1 ]]; then
    command -v ethercatctl >/dev/null 2>&1 || \
        die "EtherCAT设备节点不存在，并且找不到 ethercatctl"
    log "启动 IgH EtherCAT Master"
    ethercatctl start
fi

log "检查 EtherCAT 从站"
for master_id in "${MASTER_IDS[@]}"; do
    slave_output=""
    slave_count=0
    for _attempt in {1..20}; do
        slave_output="$(ethercat slaves -m "${master_id}" 2>/dev/null || true)"
        slave_count="$(awk 'NF {count++} END {print count + 0}' <<<"${slave_output}")"
        if [[ ${slave_count} -eq 7 ]]; then
            break
        fi
        sleep 0.25
    done

    if [[ ${slave_count} -ne 7 ]]; then
        printf '%s\n' "${slave_output}" >&2
        die "Master ${master_id} 应检测到7个从站，实际检测到 ${slave_count} 个"
    fi

    printf '%s\n' "${slave_output}"
done

log "配置 PDO/DC，并将所选驱动器初始化到 OP/CSP"
HELPER_STARTED=1
python3 "${PREPARE_SCRIPT}" --active-arm "${ACTIVE_ARM}"

log "发送 0x6040=0x0006，停止 PDO 保持程序并释放 Master"
stop_helper

# 给 IgH Master 状态机一点时间完成应用释放后的切换。
sleep 0.5

log "逐轴验证 CoE/SDO（读取 0x6064:00 实际位置）"
for master_id in "${MASTER_IDS[@]}"; do
    for slave_id in {0..6}; do
        if ! position="$(
            ethercat upload -m "${master_id}" -p "${slave_id}" -t int32 0x6064 0 2>&1
        )"; then
            printf '%s\n' "${position}" >&2
            die "Master ${master_id} Slave ${slave_id} 的 SDO 验证失败"
        fi
        printf '  Master %s Slave %s  0x6064:00 = %s\n' \
            "${master_id}" "${slave_id}" "${position}"
    done
done

log "初始化完成，SDO通信正常，EtherCAT Master已释放"
printf '%s\n' "现在可以启动 ros2_control/MoveIt："
printf '%s\n' \
    "  ros2 launch kaixuan3_config moveit_bringup.launch.py use_real_hardware:=true use_rviz:=false active_arm:=${ACTIVE_ARM}"

trap - EXIT INT TERM
