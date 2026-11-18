"""Seeded random sequences of goals and /stop, /resume, cancel, with invariants.

After every operation: the server process is alive and answering, at most one goal is
active, none has been active for ages, and a goal sent while another is running or
while stopped is refused. At the end, once /resume has been called and the last goal
has ended, the server must take a goal again.

A failure prints the seed and the operations so far; rerun one with
FAULT_SEED=<n> pytest ... -k random.
"""

import os
import random
import time

import pytest
from harness import wait_until

FAST_SEEDS = [11]
FAST_STEPS = 10
SLOW_SEEDS = list(range(100, 106))
SLOW_STEPS = 40
GOAL_S = 90

DOWN = (1.0, 0.0, 0.0, 0.0)
OPS = {
    "pick": 5,
    "place_tag": 3,
    "place_position": 2,
    "move_to": 3,
    "move_pose": 2,
    "stop": 2,
    "resume": 3,
    "cancel": 3,
    "release": 1,
    "wait": 4,
    "toggle_tag": 1,
}


class Sequence:
    def __init__(self, rig, seed):
        self.rig = rig
        self.client = rig.client
        self.rng = random.Random(seed)
        self.seed = seed
        self.log = []
        self.live = None  # the last goal that was accepted
        self.stopped = False
        self.hidden = False

    def note(self, text):
        self.log.append(text)
        print(f"[seed {self.seed}] {len(self.log):3d} {text}")

    def live_running(self):
        return self.live is not None and not self.live.done

    def send(self, run):
        """Send a goal and check it was accepted or refused as it should be."""
        was_running = self.live_running()
        accepted = run.wait_accepted()
        still_running = self.live_running()
        if self.stopped:
            assert not accepted, f"goal accepted while stopped: {run}"
        elif was_running and still_running:
            assert not accepted, f"second goal accepted while one was running: {run}"
        elif not was_running and not self.stopped:
            assert accepted, f"goal refused with nothing running and not stopped: {run}"
        if accepted:
            self.live = run
        return accepted

    def finish_live(self):
        if self.live_running():
            self.live.wait_done(GOAL_S)
        self.check_result(self.live)

    def check_result(self, run):
        if run is None or not run.done or run.accepted is False:
            return
        assert run.outcome in ("succeeded", "aborted", "canceled"), run
        if run.outcome == "aborted" and run.label.startswith(("pick", "place")):
            assert run.message, f"aborted without a message: {run}"

    def step(self):
        rng, client = self.rng, self.client
        op = rng.choices(list(OPS), weights=list(OPS.values()))[0]
        tag = f"tag_{rng.randint(1, 8)}"
        if op == "pick":
            self.note(f"pick {tag}")
            self.send(client.pick(tag))
        elif op == "place_tag":
            self.note(f"place {tag}")
            self.send(client.place(tag))
        elif op == "place_position":
            xyz = (rng.uniform(0.15, 0.4), rng.uniform(-0.25, 0.25), 0.05)
            self.note(f"place at {xyz}")
            self.send(client.place(position=xyz))
        elif op == "move_to":
            name = rng.choice(["home", "Compact", "nonsense"])
            self.note(f"move_to {name}")
            self.send(client.move_to(name))
        elif op == "move_pose":
            xyz = (rng.uniform(-0.5, 0.6), rng.uniform(-0.5, 0.5), rng.uniform(0, 0.5))
            self.note(f"move_pose {xyz}")
            self.send(client.move_pose(xyz, DOWN))
        elif op == "stop":
            self.note("/stop")
            assert client.trigger("/stop").success
            self.stopped = True
        elif op == "resume":
            self.note("/resume")
            assert client.trigger("/resume").success
            self.stopped = False
        elif op == "cancel":
            self.note("cancel")
            if self.live_running():
                self.live.cancel()
        elif op == "release":
            self.note("/release")
            if not self.live_running():
                self.rig.release()
        elif op == "toggle_tag":
            self.hidden = not self.hidden
            self.note(f"tags {'hidden' if self.hidden else 'shown'}")
            client.hide_tags(*([2, 3] if self.hidden else []))
        else:
            self.note("wait for the goal")
            self.finish_live()
        self.invariants()
        # Let the goal progress a little so goals overlap with operations.
        time.sleep(rng.uniform(0.0, 1.5))

    def invariants(self):
        self.rig.check_invariants()
        self.check_result(self.live)

    def finish(self):
        """Quiesce, then the server must take a goal."""
        self.note("finish: resume, wait, move home")
        self.client.trigger("/resume")
        self.stopped = False
        self.client.reset_tags()
        self.finish_live()
        state = self.rig.settle()
        assert not state["busy"], state
        # A goal blocked by a busy flag stuck on would be refused here.
        run = self.client.move_to("home")
        assert run.wait_accepted(), f"server refuses goals after the sequence: {run}"
        run.wait_done(GOAL_S)
        assert run.outcome == "succeeded", run


def run_sequence(rig, seed, steps):
    seq = Sequence(rig, seed)
    try:
        for _ in range(steps):
            seq.step()
        seq.finish()
    except BaseException as error:
        error.add_note(f"seed={seed}; operations so far: {seq.log}")
        print(f"FAILED with seed {seed}: {seq.log}")
        raise


def seeds_from_env(default):
    wanted = os.environ.get("FAULT_SEED")
    return [int(wanted)] if wanted else default


@pytest.mark.parametrize("seed", seeds_from_env(FAST_SEEDS))
def test_random_sequence_fast(healthy, seed):
    run_sequence(healthy, seed, FAST_STEPS)


@pytest.mark.slow
@pytest.mark.parametrize("seed", seeds_from_env(SLOW_SEEDS))
def test_random_sequence_long(healthy, seed):
    run_sequence(healthy, seed, SLOW_STEPS)


def test_goals_sent_back_to_back_are_refused_not_queued(healthy):
    client = healthy.client
    first = client.move_to("Compact")
    assert first.wait_accepted()
    second = client.move_to("home")
    third = client.pick("tag_2")
    assert second.wait_accepted() is False and third.wait_accepted() is False
    first.wait_done(GOAL_S)
    wait_until(lambda: not client.active_goals(), 10, "the goals to clear")
    healthy.check_invariants()
