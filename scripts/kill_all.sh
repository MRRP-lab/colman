#!/usr/bin/env bash
# Stop every ROS / Gazebo process started from this checkout and clean up
# leftovers. Run with: pixi run kill-all
#
# Only processes whose command line points into THIS checkout's .pixi env are
# killed, so other worktrees and unrelated programs are left alone.
# The [x] bracket trick keeps pkill/pgrep from matching this script itself.

root="${PIXI_PROJECT_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
env_pat="${root}/[.]pixi/envs/"

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
# Skip files still held open: ROS processes from other checkouts (or anything not
# started from this one) share /dev/shm, and deleting a live participant's
# files breaks its shared-memory discovery.
for f in /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*; do
    [ -e "$f" ] || continue
    fuser -s "$f" 2>/dev/null || rm -f "$f"
done

# The ROS 2 daemon caches the node graph and can show dead nodes.
ros2 daemon stop >/dev/null 2>&1 || true

left="$(list)"
if [ -n "$left" ]; then
    echo "Still running:"
    echo "$left" | cut -c1-140
    exit 1
fi
echo "All colman processes stopped; stale DDS shared memory cleaned."
