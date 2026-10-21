"""Pure state machine for supervising a UR arm. No ROS imports.

The node feeds it an `Observation` every tick and carries out the `Action`s
it returns. Keeping it free of I/O means every recovery path can be unit
tested with a fake clock.
"""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

# ur_dashboard_msgs/msg/RobotMode
ROBOT_MODE_RUNNING = 7

# ur_dashboard_msgs/msg/SafetyMode
SAFETY_NORMAL = 1
SAFETY_REDUCED = 2
SAFETY_PROTECTIVE_STOP = 3
SAFETY_RECOVERY = 4
SAFETY_SAFEGUARD_STOP = 5
SAFETY_SYSTEM_EMERGENCY_STOP = 6
SAFETY_ROBOT_EMERGENCY_STOP = 7
SAFETY_VIOLATION = 8
SAFETY_FAULT = 9
SAFETY_VALIDATE_JOINT_ID = 10
SAFETY_UNDEFINED = 11
SAFETY_AUTOMATIC_MODE_SAFEGUARD_STOP = 12
SAFETY_SYSTEM_THREE_POSITION_ENABLING_STOP = 13

SAFETY_OK = {SAFETY_NORMAL, SAFETY_REDUCED}

# Nothing we can do over the network: a person has to release or inspect.
SAFETY_NEEDS_HUMAN = {
    SAFETY_RECOVERY,
    SAFETY_SYSTEM_EMERGENCY_STOP,
    SAFETY_ROBOT_EMERGENCY_STOP,
    SAFETY_VIOLATION,
    SAFETY_FAULT,
    SAFETY_SYSTEM_THREE_POSITION_ENABLING_STOP,
}

# Clears on its own (boot-time checks, light curtains); just wait.
SAFETY_WAIT = {
    SAFETY_SAFEGUARD_STOP,
    SAFETY_VALIDATE_JOINT_ID,
    SAFETY_UNDEFINED,
    SAFETY_AUTOMATIC_MODE_SAFEGUARD_STOP,
}


class State(Enum):
    WAITING_FOR_LINK = auto()
    WAITING_FOR_DRIVER = auto()
    POWERING_ON = auto()
    STARTING_PROGRAM = auto()
    ACTIVATING_CONTROLLERS = auto()
    READY = auto()
    HUMAN_REQUIRED = auto()


class Action(Enum):
    SET_MODE_RUNNING = auto()
    RESEND_PROGRAM = auto()
    ACTIVATE_CONTROLLER = auto()
    RESTART_DRIVER = auto()


@dataclass
class Observation:
    """One snapshot of the outside world. `None` means unknown or stale."""

    now: float
    link_up: bool
    driver_up: bool
    robot_mode: Optional[int] = None
    safety_mode: Optional[int] = None
    program_running: Optional[bool] = None
    controller_active: Optional[bool] = None


@dataclass
class Config:
    auto_recover_protective_stop: bool = True
    # How long a bad condition must persist before we react to it, so a
    # dropped packet doesn't tear anything down.
    link_grace: float = 5.0
    driver_grace: float = 5.0
    state_grace: float = 1.0
    # Minimum time between repeats of the same action while waiting on it.
    retry_interval: float = 5.0
    # Link is up but the driver never came good: ask for a restart.
    driver_timeout: float = 30.0


class SupervisorMachine:
    def __init__(self, config: Optional[Config] = None):
        self.config = config or Config()
        self.state = State.WAITING_FOR_LINK
        self.reason = "starting"
        self._entered = None
        self._bad_since = {}
        self._last_action = {}

    def update(self, obs: Observation) -> list:
        """Advance on `obs` and return the actions to carry out now."""
        if self._entered is None:
            self._entered = obs.now
        before = self.state
        actions = self._step(obs)
        if self.state != before:
            self._entered = obs.now
            self._bad_since.clear()
            self._last_action.clear()
        return actions

    def _go(self, state, reason):
        self.state = state
        self.reason = reason

    def _held(self, key, bad, now, duration):
        """True once `bad` has been continuously true for `duration`."""
        if not bad:
            self._bad_since.pop(key, None)
            return False
        since = self._bad_since.setdefault(key, now)
        return now - since >= duration

    def _due(self, action, now):
        last = self._last_action.get(action)
        if last is not None and now - last < self.config.retry_interval:
            return False
        self._last_action[action] = now
        return True

    def _safety_ok(self, obs):
        return obs.safety_mode in SAFETY_OK

    def _step(self, obs):
        cfg = self.config
        now = obs.now

        if self.state == State.WAITING_FOR_LINK:
            if obs.link_up:
                self._go(State.WAITING_FOR_DRIVER, "link is up")
            return []

        if self._held("link", not obs.link_up, now, cfg.link_grace):
            self._go(State.WAITING_FOR_LINK, "link to the arm lost")
            return []

        if self.state == State.WAITING_FOR_DRIVER:
            if obs.driver_up and obs.robot_mode is not None:
                self._go(State.POWERING_ON, "driver is up")
            elif now - self._entered >= cfg.driver_timeout:
                self._entered = now
                return [Action.RESTART_DRIVER]
            return []

        if self._held("driver", not obs.driver_up, now, cfg.driver_grace):
            self._go(State.WAITING_FOR_DRIVER, "driver went away")
            return []

        if self.state == State.HUMAN_REQUIRED:
            if (
                obs.safety_mode not in SAFETY_NEEDS_HUMAN
                and obs.safety_mode is not None
            ):
                if obs.safety_mode == SAFETY_PROTECTIVE_STOP and not (
                    cfg.auto_recover_protective_stop
                ):
                    return []
                self._go(State.POWERING_ON, "safety condition cleared")
            return []

        if obs.safety_mode in SAFETY_NEEDS_HUMAN or (
            obs.safety_mode == SAFETY_PROTECTIVE_STOP
            and not cfg.auto_recover_protective_stop
        ):
            self._go(
                State.HUMAN_REQUIRED, f"safety mode {obs.safety_mode} needs a person"
            )
            return []

        if self.state == State.POWERING_ON:
            if obs.safety_mode in SAFETY_WAIT:
                return []
            if obs.robot_mode == ROBOT_MODE_RUNNING and self._safety_ok(obs):
                self._go(State.STARTING_PROGRAM, "arm is powered and running")
                return []
            if self._due(Action.SET_MODE_RUNNING, now):
                return [Action.SET_MODE_RUNNING]
            return []

        # Everything from here on assumes the arm is powered and safe.
        powered = obs.robot_mode == ROBOT_MODE_RUNNING and self._safety_ok(obs)
        if self._held("powered", not powered, now, cfg.state_grace):
            self._go(State.POWERING_ON, "arm lost power or safety state")
            return []

        if self.state == State.STARTING_PROGRAM:
            if obs.program_running:
                self._go(State.ACTIVATING_CONTROLLERS, "robot program is running")
            elif self._due(Action.RESEND_PROGRAM, now):
                return [Action.RESEND_PROGRAM]
            return []

        if self._held("program", not obs.program_running, now, cfg.state_grace):
            self._go(State.STARTING_PROGRAM, "robot program stopped")
            return []

        if self.state == State.ACTIVATING_CONTROLLERS:
            if obs.controller_active:
                self._go(State.READY, "controllers are active")
            elif self._due(Action.ACTIVATE_CONTROLLER, now):
                return [Action.ACTIVATE_CONTROLLER]
            return []

        if self.state == State.READY:
            if self._held(
                "controller", not obs.controller_active, now, cfg.state_grace
            ):
                self._go(State.ACTIVATING_CONTROLLERS, "motion controller inactive")
        return []
