#!/usr/bin/env bash
set -u

# Set KGU object 0x2002:00 to 64, then read 0x3000:00.
# This script only performs SDO access; it does not start or enable motors.

usage() {
  cat <<'EOF'
Usage:
  read_drop_frame_count.sh [options]

Options:
  -m, --master N       Read one master (requires --slave)
  -p, --slave N        Read one slave position (0..6)
  -a, --all            Read all 14 drives (default)
  -i, --interval SEC   Repeat every SEC seconds (default: one shot)
  -o, --output FILE    Append CSV records to FILE
  -h, --help           Show this help

Examples:
  read_drop_frame_count.sh --master 0 --slave 0
  read_drop_frame_count.sh --all --output drop_frames.csv
  read_drop_frame_count.sh --all --interval 1 --output drop_frames.csv
EOF
}

master=""
slave=""
all=1
interval=""
output=""

while (($#)); do
  case "$1" in
    -m|--master) master="$2"; shift 2 ;;
    -p|--slave) slave="$2"; shift 2 ;;
    -a|--all) all=1; shift ;;
    -i|--interval) interval="$2"; shift 2 ;;
    -o|--output) output="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -n "$master" || -n "$slave" ]]; then
  if [[ ! "$master" =~ ^[01]$ || ! "$slave" =~ ^[0-6]$ ]]; then
    echo "--master must be 0 or 1 and --slave must be 0..6" >&2
    exit 2
  fi
  all=0
fi

if [[ -n "$interval" ]] && ! [[ "$interval" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "--interval must be a non-negative number" >&2
  exit 2
fi

if [[ -n "$output" && ! -e "$output" ]]; then
  printf 'timestamp_utc,master,slave,write_2002,drop_frames_raw,drop_frames_decimal\n' > "$output"
fi

run_ethercat() {
  if ethercat "$@" 2>/dev/null; then
    return 0
  fi
  command -v sudo >/dev/null 2>&1 && sudo ethercat "$@"
}

read_one() {
  local m="$1" p="$2" raw decimal now
  if ! run_ethercat download -m "$m" -p "$p" --type uint16 0x2002 0 64 >/dev/null; then
    echo "master=$m slave=$p: failed to write 0x2002:00=64" >&2
    return 1
  fi
  raw="$(run_ethercat upload -m "$m" -p "$p" 0x3000 0)" || {
    echo "master=$m slave=$p: failed to read 0x3000:00" >&2
    return 1
  }
  raw="$(printf '%s\n' "$raw" | awk 'NF {print $1; exit}')"
  if [[ "$raw" =~ ^0x[0-9a-fA-F]+$ ]]; then
    decimal=$((raw))
  elif [[ "$raw" =~ ^[0-9]+$ ]]; then
    decimal="$raw"
  else
    decimal="NA"
  fi
  now="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  printf '%s master=%s slave=%s 0x2002=64 0x3000=%s (%s)\n' "$now" "$m" "$p" "$raw" "$decimal"
  if [[ -n "$output" ]]; then
    printf '%s,%s,%s,64,%s,%s\n' "$now" "$m" "$p" "$raw" "$decimal" >> "$output"
  fi
}

while :; do
  if ((all)); then
    for m in 0 1; do
      for p in 0 1 2 3 4 5 6; do
        read_one "$m" "$p" || true
      done
    done
  else
    read_one "$master" "$slave" || exit 1
  fi
  [[ -n "$interval" ]] || break
  sleep "$interval"
done
