#!/usr/bin/env bash
set -eo pipefail
ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [ ! -f "$ws/install/setup.bash" ]; then
  printf '%s\n' 'Build first: source /opt/ros/jazzy/setup.bash, then colcon build from this workspace.' >&2
  exit 2
fi
source /opt/ros/jazzy/setup.bash
source "$ws/install/setup.bash"
export DUAL_SCARA_WORKSPACE="$ws"
exec /usr/bin/python3 -m dual_scara_desktop.app --workspace "$ws" "$@"
