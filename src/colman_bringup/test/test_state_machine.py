from colman_bringup.driver_supervisor.state_machine import (
    Action,
    Config,
    Observation,
    State,
    SupervisorMachine,
)


class Rig:
    """Drives a machine with a fake clock and a healthy arm by default."""

    def __init__(self, **config):
        self.m = SupervisorMachine(Config(**config))
        self.t = 0.0
        self.link_up = True
        self.driver_up = True
        self.robot_mode = 7
        self.safety_mode = 1
        self.program_running = True
        self.controller_active = True

    def tick(self, dt=0.5):
        self.t += dt
        return self.m.update(
            Observation(
                now=self.t,
                link_up=self.link_up,
                driver_up=self.driver_up,
                robot_mode=self.robot_mode,
                safety_mode=self.safety_mode,
                program_running=self.program_running,
                controller_active=self.controller_active,
            )
        )

    def run(self, seconds, dt=0.5):
        actions = []
        for _ in range(int(seconds / dt)):
            actions += self.tick(dt)
        return actions


def ready_rig(**config):
    rig = Rig(**config)
    rig.run(5)
    assert rig.m.state == State.READY
    return rig


def test_healthy_arm_reaches_ready_with_no_actions():
    rig = Rig()
    assert rig.run(5) == []
    assert rig.m.state == State.READY


def test_starting_before_the_arm_is_attached_waits_then_recovers():
    rig = Rig()
    rig.link_up = False
    rig.run(10)
    assert rig.m.state == State.WAITING_FOR_LINK
    rig.link_up = True
    rig.run(5)
    assert rig.m.state == State.READY


def test_cold_arm_is_powered_on_then_program_started():
    rig = Rig()
    rig.robot_mode = 3
    rig.program_running = False
    rig.controller_active = False

    assert rig.run(2) == [Action.SET_MODE_RUNNING]
    assert rig.m.state == State.POWERING_ON

    rig.robot_mode = 7
    assert Action.RESEND_PROGRAM in rig.run(2)
    assert rig.m.state == State.STARTING_PROGRAM

    rig.program_running = True
    assert Action.ACTIVATE_CONTROLLER in rig.run(2)
    assert rig.m.state == State.ACTIVATING_CONTROLLERS

    rig.controller_active = True
    rig.run(2)
    assert rig.m.state == State.READY


def test_actions_are_rate_limited():
    rig = Rig(retry_interval=5.0)
    rig.robot_mode = 3
    assert rig.run(10).count(Action.SET_MODE_RUNNING) == 2


def test_short_link_blip_is_ignored():
    rig = ready_rig(link_grace=5.0)
    rig.link_up = False
    rig.run(2)
    rig.link_up = True
    rig.run(10)
    assert rig.m.state == State.READY


def test_unplug_and_replug_recovers_in_place():
    rig = ready_rig()
    rig.link_up = False
    rig.driver_up = False
    rig.program_running = None
    rig.controller_active = None
    rig.run(10)
    assert rig.m.state == State.WAITING_FOR_LINK

    rig.link_up = True
    rig.driver_up = True
    rig.program_running = False
    actions = rig.run(5)
    assert Action.RESEND_PROGRAM in actions
    rig.program_running = True
    rig.controller_active = False
    assert Action.ACTIVATE_CONTROLLER in rig.run(3)
    rig.controller_active = True
    rig.run(3)
    assert rig.m.state == State.READY


def test_program_stopping_triggers_resend():
    rig = ready_rig()
    rig.program_running = False
    rig.controller_active = False
    assert Action.RESEND_PROGRAM in rig.run(3)
    assert rig.m.state == State.STARTING_PROGRAM


def test_emergency_stop_waits_for_a_person_and_never_acts():
    rig = ready_rig()
    rig.safety_mode = 7
    rig.robot_mode = 3
    assert rig.run(20) == []
    assert rig.m.state == State.HUMAN_REQUIRED

    rig.safety_mode = 1
    assert Action.SET_MODE_RUNNING in rig.run(3)
    assert rig.m.state == State.POWERING_ON


def test_protective_stop_auto_recovers_by_default():
    rig = ready_rig()
    rig.safety_mode = 3
    assert Action.SET_MODE_RUNNING in rig.run(3)
    rig.safety_mode = 1
    rig.run(3)
    assert rig.m.state != State.HUMAN_REQUIRED


def test_protective_stop_can_require_a_person():
    rig = ready_rig(auto_recover_protective_stop=False)
    rig.safety_mode = 3
    assert rig.run(10) == []
    assert rig.m.state == State.HUMAN_REQUIRED


def test_safeguard_stop_just_waits():
    rig = ready_rig()
    rig.safety_mode = 5
    rig.robot_mode = 5
    assert rig.run(10) == []
    assert rig.m.state == State.POWERING_ON


def test_driver_that_never_comes_up_gets_restart_requests():
    rig = Rig(driver_timeout=30.0)
    rig.driver_up = False
    rig.robot_mode = None
    actions = rig.run(100)
    assert actions.count(Action.RESTART_DRIVER) >= 2
    assert rig.m.state == State.WAITING_FOR_DRIVER
