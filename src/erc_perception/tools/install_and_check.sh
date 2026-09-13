#!/usr/bin/env bash
# Install only erc_perception, preserving a backup, then check it in erc_sim.
set -euo pipefail

repo=${1:-"$HOME/erc_sim_2026_team"}
update=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
repo=$(cd -- "$repo" && pwd)
branch=$(git -C "$repo" branch --show-current)
if [[ "$branch" != perception ]]; then
    echo "Expected branch perception; current branch is: $branch" >&2
    echo 'Switch to your perception branch before installing this update.' >&2
    exit 1
fi
destination="$repo/src/erc_perception"
if [[ ! -d "$destination" || "$update" == "$destination" ]]; then
    echo 'Run this installer from the separately extracted update folder.' >&2
    exit 1
fi
backup=$(mktemp -d "$HOME/erc-perception-backup.XXXXXX")
tar -czf "$backup/erc_perception.tar.gz" -C "$repo/src" erc_perception
echo "Backup saved: $backup/erc_perception.tar.gz"
cp -R "$update/." "$destination/"
echo 'Updated src/erc_perception.'

if [[ $(docker inspect -f '{{.State.Running}}' erc_sim) != true ]]; then
    docker start erc_sim
fi
docker exec -i erc_sim bash -s <<'CONTAINER'
set -e
source /opt/ros/humble/setup.bash
cd /opt/erc_ws
colcon build --packages-select erc_description erc_perception
source /opt/erc_ws/install/setup.bash
ros2 launch erc_perception perception.launch.py --show-args
colcon test --packages-select erc_perception --event-handlers console_direct+
colcon test-result --test-result-base build/erc_perception --verbose
python3 src/erc_perception/tools/check_ros.py
CONTAINER
echo 'Build and automated checks passed. A live shelf-camera check remains.'
git -C "$repo" status --short
