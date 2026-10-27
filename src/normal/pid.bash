#!/bin/bash

set -u


MASTER=0
SLAVE=""
SPEED_P=""
SPEED_I=""
POSITION_P=""
POSITION_P_FINE=""
POSITION_FF=""
SPEED_FF=""
SAVE_EEPROM=0



run_ethercat()
{
    if [[ ${EUID:-$(id -u)} -eq 0 ]]; then
        ethercat "$@"
    else
        sudo ethercat "$@"
    fi
}



write_param()
{
    local master=$1
    local slave=$2
    local index=$3
    local value=$4
    local name=$5


    echo "写入 ${name} = ${value}"


    if run_ethercat \
        -m "${master}" \
        download \
        -p "${slave}" \
        -t uint16 \
        "${index}" \
        0x00 \
        "${value}"

    then
        echo "  ${name} 写入成功"
    else
        echo "  ${name} 写入失败"
        return 1
    fi
}



read_param()
{
    local master=$1
    local slave=$2
    local index=$3
    local name=$4


    echo -n "  ${name}: "


    if output=$(run_ethercat \
        -m "${master}" \
        upload \
        -p "${slave}" \
        -t uint16 \
        "${index}" \
        0x00 2>&1)

    then
        echo "${output}"
    else
        echo "读取失败 (${output})"
    fi
}


save_parameters()
{
    local master=$1
    local slave=$2

    echo
    echo "触发 KGU 参数非易失保存：Pn097 (0x2097:00) = 1 -> 0..."
    if ! run_ethercat \
        -m "${master}" \
        download \
        -p "${slave}" \
        -t uint16 \
        0x2097 \
        0x00 \
        1
    then
        echo "保存失败：Pn097 写 1 失败。" >&2
        return 1
    fi

    sleep 0.1

    if ! run_ethercat \
        -m "${master}" \
        download \
        -p "${slave}" \
        -t uint16 \
        0x2097 \
        0x00 \
        0
    then
        echo "保存触发后 Pn097 恢复为 0 失败，请勿断电，先手动写回 0。" >&2
        return 1
    fi

    echo "Pn097 保存脉冲执行成功"
}



while [[ $# -gt 0 ]]
do

case "$1" in


-m)
    MASTER="$2"
    shift 2
    ;;


-s)
    SLAVE="$2"
    shift 2
    ;;


--speed-p)
    SPEED_P="$2"
    shift 2
    ;;


--speed-i)
    SPEED_I="$2"
    shift 2
    ;;


--position-p)
    POSITION_P="$2"
    shift 2
    ;;


--position-p-fine)
    POSITION_P_FINE="$2"
    shift 2
    ;;


--position-ff)
    POSITION_FF="$2"
    shift 2
    ;;


--speed-ff)
    SPEED_FF="$2"
    shift 2
    ;;


--save-eeprom)
    SAVE_EEPROM=1
    shift
    ;;


*)
    echo "未知参数 $1"
    exit 1
    ;;

esac

done



if [[ -z "${SLAVE}" ]]
then
    echo "错误: 必须指定 -s slave"
    exit 1
fi




echo "====== Master ${MASTER} Slave ${SLAVE} ======"



# 写参数

if [[ -n "${SPEED_P}" ]]
then
    write_param \
    "${MASTER}" \
    "${SLAVE}" \
    0x2033 \
    "${SPEED_P}" \
    "速度环P" || exit 1
fi



if [[ -n "${SPEED_I}" ]]
then
    write_param \
    "${MASTER}" \
    "${SLAVE}" \
    0x2034 \
    "${SPEED_I}" \
    "速度环I增益" || exit 1
fi



if [[ -n "${POSITION_P}" ]]
then
    write_param \
    "${MASTER}" \
    "${SLAVE}" \
    0x2065 \
    "${POSITION_P}" \
    "位置环P粗调" || exit 1
fi


if [[ -n "${POSITION_P_FINE}" ]]
then
    write_param \
    "${MASTER}" \
    "${SLAVE}" \
    0x2063 \
    "${POSITION_P_FINE}" \
    "位置环P细调" || exit 1
fi


if [[ -n "${POSITION_FF}" ]]
then
    write_param \
    "${MASTER}" \
    "${SLAVE}" \
    0x2064 \
    "${POSITION_FF}" \
    "位置前馈" || exit 1
fi



if [[ -n "${SPEED_FF}" ]]
then
    write_param \
    "${MASTER}" \
    "${SLAVE}" \
    0x2035 \
    "${SPEED_FF}" \
    "速度前馈" || exit 1
fi



# 保存EEPROM

if [[ "${SAVE_EEPROM}" -eq 1 ]]
then
    save_parameters "${MASTER}" "${SLAVE}" || exit 1
fi




# 最后读取确认

echo
echo "====== 当前PID参数 ======"


read_param "${MASTER}" "${SLAVE}" 0x2033 "速度环P (Pn033)"
read_param "${MASTER}" "${SLAVE}" 0x2034 "速度环I增益 (Pn034)"
read_param "${MASTER}" "${SLAVE}" 0x2035 "速度前馈 (Pn035)"
read_param "${MASTER}" "${SLAVE}" 0x2064 "位置前馈 (Pn064)"
read_param "${MASTER}" "${SLAVE}" 0x2065 "位置环P粗调 (Pn065)"
read_param "${MASTER}" "${SLAVE}" 0x2063 "位置环P细调 (Pn063)"


echo "====== 完成 ======"
