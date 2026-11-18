"""Owns the `ur_control.launch.py` child process for the supervisor."""

import ctypes
import os
import signal
import subprocess
import time

from colman_bringup.crash_guard import CrashLoopGuard

STOP_TIMEOUT = 15.0
PR_SET_PDEATHSIG = 1


def _die_with_parent():
    """Have the kernel SIGTERM the driver launch if the supervisor is SIGKILLed.

    The driver runs in its own session, so without this a kill -9 of the
    supervisor orphans it, and the restarted supervisor starts a second one.
    """
    try:
        ctypes.CDLL("libc.so.6", use_errno=True).prctl(PR_SET_PDEATHSIG, signal.SIGTERM)
    except OSError:
        pass


class DriverProcess:
    def __init__(self, command, logger, popen=subprocess.Popen, clock=time.monotonic):
        self._command = command
        self._log = logger
        self._popen = popen
        self._clock = clock
        self._proc = None
        # Only used for backoff; never gives up, because the arm may well come
        # back (power cycle, cable) and the supervisor is what notices.
        self._guard = CrashLoopGuard(
            delay=1.0, max_delay=30.0, max_crashes=10**9, window=120.0
        )
        self._died_unexpectedly = False
        self._retry_at = 0.0

    @property
    def running(self):
        return self._proc is not None and self._proc.poll() is None

    def ensure_running(self):
        """Start the driver, or restart it after a crash once the backoff passed.

        Returns True if a process was started now.
        """
        if self.running:
            return False
        now = self._clock()
        if self._proc is not None and not self._died_unexpectedly:
            self._died_unexpectedly = True
            # The launch leader died; sweep any children it left in its group.
            self._signal(signal.SIGKILL)
            decision = self._guard.record_crash(now)
            self._retry_at = now + decision.delay
            self._log.error(
                f"Driver exited with code {self._proc.returncode} "
                f"(crash {decision.crashes} in 120s); restarting in "
                f"{decision.delay:.0f}s"
            )
        if now < self._retry_at:
            return False
        self.start()
        return True

    def start(self):
        if self.running:
            return
        self._log.info(f"Starting driver: {' '.join(self._command)}")
        # Own process group so we can take down the whole launch tree.
        self._proc = self._popen(
            self._command, start_new_session=True, preexec_fn=_die_with_parent
        )
        self._died_unexpectedly = False

    def stop(self):
        if self._proc is None:
            return
        if self._proc.poll() is None:
            self._log.info("Stopping driver")
            self._signal(signal.SIGINT)
            try:
                self._proc.wait(STOP_TIMEOUT)
            except subprocess.TimeoutExpired:
                self._log.warn("Driver ignored SIGINT, killing it")
                self._signal(signal.SIGKILL)
                self._proc.wait()
        self._proc = None
        self._died_unexpectedly = False
        self._retry_at = 0.0

    def restart(self):
        self.stop()
        self.start()

    def _signal(self, sig):
        try:
            os.killpg(self._proc.pid, sig)
        except ProcessLookupError:
            pass
