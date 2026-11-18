"""Kill -9 a fake node under `supervise` and watch it come back (or not).

Runs a real `ros2 launch` of a throwaway launch file, so it needs the pixi env
but no robot, no DDS traffic and no simulator.
"""

import os
import signal
import subprocess
import time

import pytest

LAUNCH = """
import sys
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from colman_bringup.supervision import supervise

def make():
    return [ExecuteProcess(
        name="fakenode",
        cmd=[sys.executable, "-c", {script!r}],
        output="screen",
    )]

def generate_launch_description():
    bystander = ExecuteProcess(
        name="bystander", cmd=[sys.executable, "-c", "import time; time.sleep(600)"]
    )
    return LaunchDescription([bystander] + supervise(
        "fakenode", make, delay=0.3, max_delay=0.3, max_crashes=3, window=60.0,
    ))
"""

# Appends its pid to a file, then sleeps until killed.
SCRIPT = """
import os, time
with open({pids!r}, "a") as f:
    f.write(str(os.getpid()) + "\\n")
time.sleep(600)
"""


def pids(path):
    try:
        with open(path) as f:
            return [int(x) for x in f.read().split()]
    except FileNotFoundError:
        return []


def wait_for(cond, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


@pytest.fixture
def launched(tmp_path):
    pid_file = str(tmp_path / "pids")
    launch_file = tmp_path / "fake.launch.py"
    launch_file.write_text(LAUNCH.format(script=SCRIPT.format(pids=pid_file)))
    log_path = tmp_path / "launch.log"
    with open(log_path, "w") as log:
        proc = subprocess.Popen(
            ["ros2", "launch", str(launch_file)],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            env={**os.environ, "ROS_LOG_DIR": str(tmp_path / "ros_log")},
        )
    yield proc, pid_file, log_path
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(10)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        os.killpg(proc.pid, signal.SIGKILL)
    for pid in pids(pid_file):
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def test_killed_node_is_restarted_then_crash_loop_gives_up(launched):
    proc, pid_file, log = launched
    assert wait_for(lambda: len(pids(pid_file)) == 1, 20), "never started"

    # Crash 1: kill -9 must be followed by a fresh process within the bound.
    t0 = time.monotonic()
    os.kill(pids(pid_file)[-1], signal.SIGKILL)
    assert wait_for(lambda: len(pids(pid_file)) == 2, 5), "not restarted"
    assert time.monotonic() - t0 < 5
    assert pids(pid_file)[0] != pids(pid_file)[1]

    # Crash 2: still restarted.
    os.kill(pids(pid_file)[-1], signal.SIGKILL)
    assert wait_for(lambda: len(pids(pid_file)) == 3, 5), "not restarted again"

    # Crash 3 inside the window trips the guard: no fourth process.
    os.kill(pids(pid_file)[-1], signal.SIGKILL)
    assert wait_for(lambda: "GIVING UP" in log.read_text(), 5), "no loud give-up"
    time.sleep(1.0)
    assert len(pids(pid_file)) == 3
    assert proc.poll() is None  # launch itself stays up for the other nodes
