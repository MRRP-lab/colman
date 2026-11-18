import signal

from colman_bringup.driver_supervisor.driver_process import DriverProcess


class FakeLog:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(("info", m))

    warn = error = info


class FakeProc:
    _next_pid = 1000

    def __init__(self):
        FakeProc._next_pid += 1
        self.pid = FakeProc._next_pid
        self.returncode = None

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        return self.returncode


class Rig:
    def __init__(self):
        self.t = 0.0
        self.spawned = []
        self.log = FakeLog()
        self.d = DriverProcess(
            ["ur"], self.log, popen=self._popen, clock=lambda: self.t
        )

    def _popen(self, cmd, **kw):
        assert kw["start_new_session"]
        p = FakeProc()
        self.spawned.append(p)
        return p

    def tick(self, seconds):
        for _ in range(int(seconds)):
            self.t += 1.0
            self.d.ensure_running()


def test_starts_once_and_leaves_a_healthy_driver_alone():
    r = Rig()
    r.tick(30)
    assert len(r.spawned) == 1


def test_crashed_driver_is_restarted_with_growing_backoff(monkeypatch):
    monkeypatch.setattr("os.killpg", lambda pid, sig: None)
    r = Rig()
    r.tick(1)
    gaps = []
    for _ in range(4):
        r.spawned[-1].returncode = 1
        n = len(r.spawned)
        died_at = r.t
        while len(r.spawned) == n:
            r.tick(1)
        gaps.append(r.t - died_at)
    assert gaps == sorted(gaps) and gaps[-1] > gaps[0]
    assert gaps[0] <= 2


def test_crash_sweeps_the_dead_launchs_process_group(monkeypatch):
    killed = []
    monkeypatch.setattr("os.killpg", lambda pid, sig: killed.append((pid, sig)))
    r = Rig()
    r.tick(1)
    pid = r.spawned[0].pid
    r.spawned[0].returncode = -9
    r.tick(1)
    assert (pid, signal.SIGKILL) in killed


def test_explicit_restart_and_stop_do_not_count_as_crashes(monkeypatch):
    monkeypatch.setattr("os.killpg", lambda pid, sig: None)
    r = Rig()
    r.tick(1)
    for _ in range(5):
        r.spawned[-1].returncode = 0
        r.d.restart()
    # restart() starts immediately; no backoff accumulated
    r.d.stop()
    r.tick(1)
    assert len(r.spawned) == 7
