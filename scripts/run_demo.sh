#!/usr/bin/env bash
# Copyright (c) 2026 00carmen
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# Run from any directory after building this workspace.
set -eo pipefail

mode="${1:-v3}"
if (( $# > 1 )); then
    printf 'Usage: bash scripts/run_demo.sh [v3|baseline|adaptive|none|waypoints|waypoints-basic]\n' >&2
    exit 2
fi
case "$mode" in
    v3|baseline|adaptive|none)
        launch_file=canyon_full.launch.py
        launch_arg="control:=$mode"
        ;;
    waypoints)
        launch_file=canyon_waypoints.launch.py
        launch_arg=algorithm:=lidar
        ;;
    waypoints-basic)
        launch_file=canyon_waypoints.launch.py
        launch_arg=algorithm:=basic
        ;;
    -h|--help)
        printf 'Usage: bash scripts/run_demo.sh [v3|baseline|adaptive|none|waypoints|waypoints-basic]\n'
        exit 0
        ;;
    *)
        printf 'Unknown mode: %s\n' "$mode" >&2
        exit 2
        ;;
esac

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    printf 'ROS 2 Humble was not found at /opt/ros/humble.\n' >&2
    exit 3
fi
if [[ ! -f "$repo_dir/install/setup.bash" ]]; then
    printf 'Build this workspace first: %s\n' "$repo_dir" >&2
    printf 'colcon build --symlink-install --packages-select canyon_control\n' >&2
    exit 3
fi

# ROS setup scripts may reference unset variables, so do not use set -u.
source /opt/ros/humble/setup.bash
source "$repo_dir/install/setup.bash"
prefix="$(ros2 pkg prefix canyon_control)"
prefix="$(realpath -- "$prefix")"
install_dir="$(realpath -- "$repo_dir/install")"
if [[ "$prefix" != "$install_dir" && "$prefix" != "$install_dir/"* ]]; then
    printf 'Wrong canyon_control overlay: %s\nExpected this workspace: %s\n' "$prefix" "$install_dir" >&2
    exit 4
fi

if pgrep -x gzserver >/dev/null || pgrep -x gzclient >/dev/null; then
    printf 'An existing Gazebo session is running. Close it normally before starting another.\n' >&2
    exit 4
fi

export TURTLEBOT3_MODEL=burger
printf 'Workspace: %s\nMode: %s\n' "$repo_dir" "$mode"
exec ros2 launch canyon_control "$launch_file" "$launch_arg"
