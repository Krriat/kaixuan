#!/usr/bin/env bash
set -eo pipefail
client_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$client_root/scripts/environment.sh"
python3 "$client_root/scripts/check_dependencies.py"
exec python3 "$client_root/scripts/check_connection.py" "$@"
