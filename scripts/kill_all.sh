#!/usr/bin/env bash
# Stop every ROS / Gazebo process started from this checkout and clean up
# leftovers. Run with: pixi run kill-all
#
# Every process whose command line points into the .pixi env of this checkout
# OR any worktree of this repo is killed: the robot, the cameras and the DDS
# shared memory are shared hardware, so a stack left running from another
# worktree would otherwise hold the OAK-D cameras (X_LINK_DEVICE_ALREADY_IN_USE).
# Unrelated programs are left alone.
# The [x] bracket trick keeps pkill/pgrep from matching this script itself.

root="${PIXI_PROJECT_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
# The main checkout is the parent of git's common dir (worktrees share it).
main_root="$(cd "$(git -C "$root" rev-parse --git-common-dir 2>/dev/null)/.." 2>/dev/null && pwd)"
main_root="${main_root:-$root}"
env_pat="(${root}|${main_root}|${main_root}/worktrees/[^/]+)/[.]pixi/envs/"

list() {
    pgrep -af "${env_pat}|[g]z sim" | grep -v -e "kill_all" || true
}

echo "Processes to stop:"
list | cut -c1-140
echo

# Polite stop first (SIGINT lets launch files shut their children down),
# then SIGTERM, then SIGKILL for anything stubborn.
for sig in INT TERM KILL; do
    pkill -"${sig}" -f "${env_pat}" 2>/dev/null
    pkill -"${sig}" -f "[g]z sim" 2>/dev/null
    for _ in 1 2 3 4 5; do
        [ -z "$(list)" ] && break 2
        sleep 1
    done
done

# Cleanup: stale DDS shared-memory files cause the
# "RTPS_TRANSPORT_SHM Error ... open_and_lock_file failed" warnings.
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null

# The ROS 2 daemon caches the node graph and can show dead nodes.
ros2 daemon stop >/dev/null 2>&1 || true

left="$(list)"
if [ -n "$left" ]; then
    echo "Still running:"
    echo "$left" | cut -c1-140
    exit 1
fi
echo "All colman processes stopped; DDS shared memory cleaned."
