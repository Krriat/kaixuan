#!/usr/bin/env bash
set -u

# Read the KDE vendor drop-frame counter. The fixed-schedule mode is intended
# for non-intrusive comparison tests: it writes 0x2002:00 only once per target.

usage() {
  cat <<'EOF'
Usage:
  read_drop_frame_count.sh [options]

Options:
  -m, --master N       Read one master; combine with --slave for one drive,
                       or with --all for all seven drives on that master
  -p, --slave N        Read one slave position (0..6)
  -a, --all            Read all 14 drives (default)
  -i, --interval SEC   Repeat every SEC seconds (legacy mode; default: one shot)
  -T, --fixed-schedule Set 0x2002:00=64 once, then sample 0x3000:00 at
                       0, 5, 10, 15, 20, 30, 45, 60, 90, 120, 180, 240,
                       300, 420, and 600 seconds. Record Lost frames at each sample and at the start/end, then exit.
  -L, --long-duration SEC
                       Set 0x2002:00=64 once, then sample 0x3000:00 from
                       T=0 at a fixed interval until SEC has elapsed. The
                       default sampling interval is 60 seconds. Also record
                       master Lost frames at every sampling point.
      --sample-interval SEC
                       Sampling interval for --long-duration (default: 60).
      --align-op        With --fixed-schedule or --long-duration, set
                       0x2002=64 first, then wait
                        for every selected drive to reach CiA402 Operation
                        Enabled (0x6041 & 0x006F = 0x0027) before starting T=0.
      --op-timeout SEC  Timeout for --align-op (default: 90 seconds).
  -o, --output FILE    Append CSV records to FILE
  -h, --help           Show this help

Examples:
  read_drop_frame_count.sh --master 0 --slave 0
  read_drop_frame_count.sh --all --interval 1 --output drop_frames.csv
  read_drop_frame_count.sh --master 0 --slave 0 --fixed-schedule \
    --output /tmp/drop_frame_test.csv
  read_drop_frame_count.sh --master 0 --slave 0 --fixed-schedule --align-op \
    --output /tmp/drop_frame_test.csv
  read_drop_frame_count.sh --master 0 --all --fixed-schedule --align-op \
    --output /tmp/right_arm_drop_frame_test.csv
  read_drop_frame_count.sh --master 0 --all --long-duration 28800 \
    --align-op --output /tmp/right_arm_8h.csv
  read_drop_frame_count.sh --all --long-duration 18000 --align-op \
    --output /tmp/both_arms_5h.csv
EOF
}

master=""
slave=""
all=1
all_on_master=0
interval=""
output=""
fixed_schedule=0
long_duration=""
sample_interval=60
align_op=0
op_timeout=90

while (($#)); do
  case "$1" in
    -m|--master) master="$2"; shift 2 ;;
    -p|--slave) slave="$2"; shift 2 ;;
    -a|--all) all=1; shift ;;
    -i|--interval) interval="$2"; shift 2 ;;
    -T|--fixed-schedule) fixed_schedule=1; shift ;;
    -L|--long-duration) long_duration="$2"; shift 2 ;;
    --sample-interval) sample_interval="$2"; shift 2 ;;
    --align-op) align_op=1; shift ;;
    --op-timeout) op_timeout="$2"; shift 2 ;;
    -o|--output) output="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -n "$master" || -n "$slave" ]]; then
  if [[ ! "$master" =~ ^[01]$ ]]; then
    echo "--master must be 0 or 1" >&2
    exit 2
  fi
  if [[ -n "$slave" && ! "$slave" =~ ^[0-6]$ ]]; then
    echo "--slave must be 0..6" >&2
    exit 2
  fi
  all=0
  [[ -n "$slave" ]] || all_on_master=1
fi

if [[ -n "$interval" ]] && ! [[ "$interval" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "--interval must be a non-negative number" >&2
  exit 2
fi

if [[ -n "$long_duration" ]] && ! [[ "$long_duration" =~ ^[1-9][0-9]*$ ]]; then
  echo "--long-duration must be a positive integer number of seconds" >&2
  exit 2
fi

if ! [[ "$sample_interval" =~ ^[1-9][0-9]*$ ]]; then
  echo "--sample-interval must be a positive integer number of seconds" >&2
  exit 2
fi

if ! [[ "$op_timeout" =~ ^[1-9][0-9]*$ ]]; then
  echo "--op-timeout must be a positive integer" >&2
  exit 2
fi

if ((fixed_schedule)) && [[ -n "$interval" || -n "$long_duration" ]]; then
  echo "--fixed-schedule cannot be combined with --interval or --long-duration" >&2
  exit 2
fi

if [[ -n "$long_duration" && -n "$interval" ]]; then
  echo "--long-duration cannot be combined with --interval" >&2
  exit 2
fi

if ((align_op)) && (( !fixed_schedule )) && [[ -z "$long_duration" ]]; then
  echo "--align-op requires --fixed-schedule or --long-duration" >&2
  exit 2
fi

write_header() {
  [[ -z "$output" || -e "$output" ]] && return
  if ((fixed_schedule)) || [[ -n "$long_duration" ]]; then
    printf 'timestamp_utc,event,sample_target_s,master,slave,write_2002,drop_frames_raw,drop_frames_decimal,lost_frames\n' > "$output"
  else
    printf 'timestamp_utc,master,slave,write_2002,drop_frames_raw,drop_frames_decimal\n' > "$output"
  fi
}

run_ethercat() {
  if ethercat "$@" 2>/dev/null; then
    return 0
  fi
  command -v sudo >/dev/null 2>&1 && sudo ethercat "$@"
}

targets() {
  local m p
  if ((all)); then
    for m in 0 1; do
      for p in 0 1 2 3 4 5 6; do
        printf '%s %s\n' "$m" "$p"
      done
    done
  elif ((all_on_master)); then
    for p in 0 1 2 3 4 5 6; do
      printf '%s %s\n' "$master" "$p"
    done
  else
    printf '%s %s\n' "$master" "$slave"
  fi
}

selected_masters() {
  if ((all)); then
    printf '0\n1\n'
  else
    printf '%s\n' "$master"
  fi
}

set_counter_selector() {
  local m="$1" p="$2" now
  if ! run_ethercat download -m "$m" -p "$p" --type int16 0x2002 0 64 >/dev/null; then
    echo "master=$m slave=$p: failed to write 0x2002:00=64" >&2
    return 1
  fi
  now="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  printf '%s master=%s slave=%s set 0x2002=64\n' "$now" "$m" "$p"
  if [[ -n "$output" && ( $fixed_schedule -eq 1 || -n "$long_duration" ) ]]; then
    printf '%s,setup_2002,,%s,%s,64,,,\n' "$now" "$m" "$p" >> "$output"
  fi
}

parse_counter() {
  local raw="$1"
  if [[ "$raw" =~ ^0x[0-9a-fA-F]+$ ]]; then
    printf '%s\n' "$((raw))"
  elif [[ "$raw" =~ ^[0-9]+$ ]]; then
    printf '%s\n' "$raw"
  else
    printf 'NA\n'
  fi
}

read_counter_legacy() {
  local m="$1" p="$2" raw decimal now
  set_counter_selector "$m" "$p" || return 1
  raw="$(run_ethercat upload -m "$m" -p "$p" --type int32 0x3000 0)" || {
    echo "master=$m slave=$p: failed to read 0x3000:00" >&2
    return 1
  }
  raw="$(printf '%s\n' "$raw" | awk 'NF {print $1; exit}')"
  decimal="$(parse_counter "$raw")"
  now="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  printf '%s master=%s slave=%s 0x2002=64 0x3000=%s (%s)\n' "$now" "$m" "$p" "$raw" "$decimal"
  [[ -z "$output" ]] || printf '%s,%s,%s,64,%s,%s\n' "$now" "$m" "$p" "$raw" "$decimal" >> "$output"
}

read_counter_sample() {
  local target_s="$1" m="$2" p="$3" raw decimal now
  raw="$(run_ethercat upload -m "$m" -p "$p" --type int32 0x3000 0)" || {
    echo "master=$m slave=$p: failed to read 0x3000:00" >&2
    return 1
  }
  raw="$(printf '%s\n' "$raw" | awk 'NF {print $1; exit}')"
  decimal="$(parse_counter "$raw")"
  now="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  printf '%s sample=%ss master=%s slave=%s 0x3000=%s (%s)\n' "$now" "$target_s" "$m" "$p" "$raw" "$decimal"
  [[ -z "$output" ]] || printf '%s,drop_frame_sample,%s,%s,%s,,%s,%s,\n' "$now" "$target_s" "$m" "$p" "$raw" "$decimal" >> "$output"
}

read_lost_frames() {
  local label="$1" m="$2" target_s="${3:-}" text lost now
  text="$(run_ethercat master -m "$m")" || {
    echo "master=$m: failed to read master statistics" >&2
    return 1
  }
  lost="$(printf '%s\n' "$text" | awk '/^[[:space:]]*Lost frames:/ {print $3; exit}')"
  if ! [[ "$lost" =~ ^[0-9]+$ ]]; then
    echo "master=$m: could not parse Lost frames" >&2
    return 1
  fi
  now="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  printf '%s master=%s %s Lost frames=%s\n' "$now" "$m" "$label" "$lost"
  [[ -z "$output" ]] || printf '%s,%s,%s,%s,,,,,%s\n' "$now" "$label" "$target_s" "$m" "$lost" >> "$output"
}

cia402_operation_enabled() {
  local m="$1" p="$2" raw
  raw="$(run_ethercat upload -m "$m" -p "$p" --type uint16 0x6041 0 2>/dev/null || true)"
  raw="$(printf '%s\n' "$raw" | awk 'NF {print $1; exit}')"
  [[ "$raw" =~ ^0x[0-9a-fA-F]+$ ]] && (( (raw & 0x006F) == 0x0027 ))
}

wait_for_operation_enabled() {
  local start_ns deadline_ns now_ns m p all_enabled now
  start_ns="$(date +%s%N)"
  deadline_ns=$((start_ns + op_timeout * 1000000000))
  printf 'waiting up to %ss for all selected drives to reach CiA402 Operation Enabled\n' \
    "$op_timeout"

  while :; do
    all_enabled=1
    while read -r m p; do
      if ! cia402_operation_enabled "$m" "$p"; then
        all_enabled=0
        break
      fi
    done < <(targets)
    if ((all_enabled)); then
      now="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
      printf '%s all selected drives reached CiA402 Operation Enabled\n' "$now"
      [[ -z "$output" ]] || printf '%s,operation_enabled,,,,,,,\n' "$now" >> "$output"
      return 0
    fi
    now_ns="$(date +%s%N)"
    if ((now_ns >= deadline_ns)); then
      echo "timed out waiting for master=$m slave=$p CiA402 Operation Enabled" >&2
      return 1
    fi
    sleep 0.2
  done
}

sleep_until() {
  local start_ns="$1" target_s="$2" now_ns target_ns wait_ns wait_s
  target_ns=$((start_ns + target_s * 1000000000))
  now_ns="$(date +%s%N)"
  if ((now_ns < target_ns)); then
    wait_ns=$((target_ns - now_ns))
    wait_s="$(awk -v ns="$wait_ns" 'BEGIN { printf "%.9f", ns / 1000000000 }')"
    sleep "$wait_s"
  fi
}

run_fixed_schedule() {
  local m p target_s start_ns
  write_header

  while read -r m p; do
    set_counter_selector "$m" "$p" || exit 1
  done < <(targets)

  if ((align_op)); then
    wait_for_operation_enabled || exit 1
  fi

  start_ns="$(date +%s%N)"
  while read -r m; do
    read_lost_frames 'master_lost_start' "$m" || exit 1
  done < <(selected_masters)

  for target_s in 0 5 10 15 20 30 45 60 90 120 180 240 300 420 600; do
    sleep_until "$start_ns" "$target_s"
    while read -r m; do
      read_lost_frames 'master_lost_sample' "$m" "$target_s" || exit 1
    done < <(selected_masters)
    while read -r m p; do
      read_counter_sample "$target_s" "$m" "$p" || exit 1
    done < <(targets)
  done

  while read -r m; do
    read_lost_frames 'master_lost_end' "$m" || exit 1
  done < <(selected_masters)
}

run_long_duration() {
  local m p target_s start_ns
  write_header

  while read -r m p; do
    set_counter_selector "$m" "$p" || exit 1
  done < <(targets)

  if ((align_op)); then
    wait_for_operation_enabled || exit 1
  fi

  start_ns="$(date +%s%N)"
  while read -r m; do
    read_lost_frames 'master_lost_start' "$m" || exit 1
  done < <(selected_masters)

  target_s=0
  while ((target_s <= long_duration)); do
    sleep_until "$start_ns" "$target_s"
    while read -r m; do
      read_lost_frames 'master_lost_sample' "$m" "$target_s" || exit 1
    done < <(selected_masters)
    while read -r m p; do
      read_counter_sample "$target_s" "$m" "$p" || exit 1
    done < <(targets)
    target_s=$((target_s + sample_interval))
  done

  # Always capture the requested endpoint, even when it is not a whole interval.
  if (((target_s - sample_interval) != long_duration)); then
    sleep_until "$start_ns" "$long_duration"
    while read -r m; do
      read_lost_frames 'master_lost_sample' "$m" "$long_duration" || exit 1
    done < <(selected_masters)
    while read -r m p; do
      read_counter_sample "$long_duration" "$m" "$p" || exit 1
    done < <(targets)
  fi

  while read -r m; do
    read_lost_frames 'master_lost_end' "$m" || exit 1
  done < <(selected_masters)
}

if ((fixed_schedule)); then
  run_fixed_schedule
  exit 0
fi

if [[ -n "$long_duration" ]]; then
  run_long_duration
  exit 0
fi

write_header
while :; do
  while read -r m p; do
    read_counter_legacy "$m" "$p" || exit 1
  done < <(targets)
  [[ -n "$interval" ]] || break
  sleep "$interval"
done