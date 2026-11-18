#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/humble/setup.bash
source /home/misys/forza_ws/race_stack/install/setup.bash
source "$PWD/ws/install/setup.bash"
export ROS_LOCALHOST_ONLY=1
export MPLBACKEND=Agg
export QT_X11_NO_MITSHM=1
exec python3 "$PWD/tools/manual_session.py" "$@"
