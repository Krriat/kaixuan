#!/usr/bin/env bash
# Extract ROS's matching Pinocchio packages into a private prefix, without sudo.
set -euo pipefail
task_root="${KAIXUAN_PINOCCHIO_ROOT:-${HOME}/.local/share/kaixuan/pinocchio}"
task_cache="${KAIXUAN_PINOCCHIO_DEB_CACHE:-}"
if [[ -z "$task_cache" ]]; then
  task_cache="$(mktemp -d)"
  trap 'rm -rf -- "$task_cache"' EXIT
  (cd "$task_cache" && apt-get download ros-humble-pinocchio ros-humble-coal ros-humble-eigenpy)
fi
mkdir -p "$task_root"
for task_package in ros-humble-pinocchio ros-humble-coal ros-humble-eigenpy; do
  task_files=("$task_cache"/"$task_package"_*.deb)
  [[ -f "${task_files[0]}" ]] || { echo "Missing package: $task_package" >&2; exit 1; }
  dpkg-deb -x "${task_files[0]}" "$task_root"
done
python3 - "$task_root" <<'PY'
import hashlib, json, pathlib, sys
root = pathlib.Path(sys.argv[1]).resolve()
prefix = root / 'opt/ros/humble'
# Debian exports use absolute paths for Python modules; make these relocatable.
for path in (prefix / 'lib').rglob('*.cmake'):
    text = path.read_text()
    changed = text.replace('/opt/ros/humble/lib/python3.10/site-packages/',
                           str(prefix / 'lib/python3.10/site-packages') + '/')
    if changed != text:
        path.write_text(changed)
print('Private Pinocchio prefix:', prefix)
print('Rebuild kaixuan_can_hardware; CMake finds this default prefix automatically.')
PY
