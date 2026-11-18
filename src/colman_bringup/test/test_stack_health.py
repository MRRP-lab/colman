from colman_bringup.stack_health.health import (
    ACTION,
    NODE,
    SERVICE,
    Change,
    Component,
    Event,
    HealthMonitor,
    Reaction,
    Snapshot,
    Status,
    plan_reactions,
    speech_for,
)

MOTION = Component(NODE, "/motion_server")
LLM = Component(NODE, "/llm_node")
STOP = Component(SERVICE, "/stop")
PICK = Component(ACTION, "/pick")


class FakeGraph:
    """A mutable graph; the motion server owns /stop and the /pick action."""

    def __init__(self):
        self.nodes = {"/motion_server", "/llm_node"}
        self.services = {"/stop", "/pick/_action/send_goal"}

    def kill_motion(self):
        self.nodes.discard("/motion_server")
        self.services -= {"/stop", "/pick/_action/send_goal"}

    def start_motion(self):
        self.nodes.add("/motion_server")
        self.services |= {"/stop", "/pick/_action/send_goal"}

    def snap(self):
        return Snapshot(set(self.nodes), set(self.services))


class Rig:
    def __init__(self, **kw):
        self.g = FakeGraph()
        self.m = HealthMonitor([MOTION, LLM, STOP, PICK], **kw)
        self.t = 0.0

    def run(self, seconds, dt=1.0):
        changes = []
        for _ in range(int(seconds / dt)):
            self.t += dt
            changes += self.m.update(self.g.snap(), self.t)
        return changes


def test_healthy_graph_reports_nothing_and_is_all_alive():
    r = Rig()
    assert r.run(10) == []
    assert set(r.m.status.values()) == {Status.ALIVE}
    assert r.m.summary() == ([], [])


def test_lost_after_grace_not_before():
    r = Rig(grace=3.0)
    r.run(2)
    r.g.kill_motion()
    assert r.run(2) == []  # inside the grace, flicker is ignored
    changes = r.run(3)
    assert {c.component for c in changes} == {MOTION, STOP, PICK}
    assert all(c.event == Event.LOST for c in changes)
    assert r.m.status[MOTION] == Status.MISSING
    assert "node:/motion_server" in r.m.summary()[0]


def test_flicker_shorter_than_grace_is_silent():
    r = Rig(grace=3.0)
    r.run(2)
    r.g.kill_motion()
    r.run(2)
    r.g.start_motion()
    assert r.run(10) == []
    assert r.m.status[MOTION] == Status.ALIVE


def test_loss_is_reported_once_then_recovery_once():
    r = Rig(grace=1.0)
    r.run(2)
    r.g.kill_motion()
    assert len(r.run(5)) == 3
    assert r.run(20) == []
    r.g.start_motion()
    changes = r.run(2)
    assert {c.event for c in changes} == {Event.RECOVERED}
    assert r.run(5) == []


def test_never_appeared_reported_after_startup_timeout_only():
    r = Rig(startup_timeout=30.0)
    r.g.nodes.discard("/llm_node")
    assert r.run(29) == []
    assert r.m.status[LLM] == Status.STARTING
    assert r.m.summary() == ([], ["node:/llm_node"])
    assert r.run(2) == [Change(Event.NEVER_APPEARED, LLM)]
    assert r.run(10) == []


def test_reactions_on_motion_loss_release_vacuum_and_speak():
    changes = [Change(Event.LOST, MOTION), Change(Event.LOST, STOP)]
    out = plan_reactions(changes, MOTION, STOP)
    assert (Reaction.RELEASE_VACUUM, "") in out
    says = [t for r, t in out if r == Reaction.SAY]
    assert says == ["Warning: the motion server stopped responding."]


def test_reactions_on_motion_return_stop_the_arm():
    out = plan_reactions([Change(Event.RECOVERED, STOP)], MOTION, STOP)
    assert (Reaction.STOP_MOTION, "") in out


def test_reactions_respect_switches_and_ignore_unrelated_loss():
    out = plan_reactions(
        [Change(Event.LOST, MOTION), Change(Event.RECOVERED, STOP)],
        MOTION,
        STOP,
        release_vacuum_on_loss=False,
        stop_on_motion_return=False,
    )
    assert [r for r, _ in out] == [Reaction.SAY]
    out = plan_reactions([Change(Event.LOST, LLM)], MOTION, STOP)
    assert [r for r, _ in out] == [Reaction.SAY]


def test_speech_names_nodes_only_when_a_node_died():
    text = speech_for([Change(Event.LOST, LLM), Change(Event.LOST, PICK)])
    assert text == "Warning: the language model stopped responding."
    assert speech_for([]) == ""
    assert speech_for([Change(Event.RECOVERED, LLM)]) == "the language model is back."
