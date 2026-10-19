"""Owns the `ur_control.launch.py` child process for the supervisor."""

import os
import signal
import subprocess

STOP_TIMEOUT = 15.0


class DriverProcess:
    def __init__(self, command, logger):
        self._command = command
        self._log = logger
        self._proc = None

    @property
    def running(self):
        return self._proc is not None and self._proc.poll() is None

    def start(self):
        if self.running:
            return
        self._log.info(f"Starting driver: {' '.join(self._command)}")
        # Own process group so we can take down the whole launch tree.
        self._proc = subprocess.Popen(self._command, start_new_session=True)

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

    def restart(self):
        self.stop()
        self.start()

    def _signal(self, sig):
        try:
            os.killpg(self._proc.pid, sig)
        except ProcessLookupError:
            pass
