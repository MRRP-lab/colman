"""Offline tests for hil_check.py. Run: pixi run test-hil"""

import json
import os
import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hil_check as hc  # noqa: E402


class FakeHandle:
    def __init__(self, fake):
        self.fake = fake
        self.cancelled = False

    def accepted(self, timeout_s):
        return self.fake.accept_goal

    def result(self, timeout_s):
        if self.fake.hang_after_stop:
            return False, False, ""
        return True, self.fake.stop_move_succeeds, "stopped"

    def cancel(self):
        self.cancelled = True


class FakeBackend(hc.Backend):
    def __init__(self):
        self.calls = []
        self.fail = {}  # call name -> message
        self.missing = set()
        self.stopped = False
        self.accept_goal = True
        self.stop_move_succeeds = False
        self.hang_after_stop = False
        self.spoken = []

    def node_names(self):
        return ["a", "b"]

    def has_action(self, name):
        return name not in self.missing

    def has_service(self, name):
        return name not in self.missing

    def has_subscriber(self, topic):
        return topic not in self.missing

    def wait_message(self, topic, timeout_s):
        return topic not in self.missing

    def tag_visible(self, tag, timeout_s):
        return tag not in self.missing

    def trigger(self, service, timeout_s):
        self.calls.append(service)
        if service == hc.STOP_SERVICE:
            self.stopped = True
        if service == hc.RESUME_SERVICE:
            self.stopped = False
        if service in self.fail:
            return False, self.fail[service]
        return True, "ok"

    def _act(self, name, arg):
        self.calls.append(f"{name}:{arg}")
        if self.stopped:
            return False, "stopped"
        if name in self.fail:
            return False, self.fail[name]
        return True, "done"

    def move_to(self, pose, timeout_s):
        return self._act("move_to", pose)

    def pick(self, frame, timeout_s):
        return self._act("pick", frame)

    def place(self, location, timeout_s):
        return self._act("place", location)

    def start_move_to(self, pose):
        self.calls.append(f"start_move_to:{pose}")
        return FakeHandle(self)

    def say(self, text):
        self.spoken.append(text)


def make_runner(backend=None, **kw):
    kw.setdefault("stop_delay_s", 0.0)
    cfg = hc.Config(**kw)
    be = backend or FakeBackend()
    return be, hc.Runner(be, cfg, prompt=lambda _p: "", log=lambda *_a: None)


def test_full_cycle_passes_in_order():
    be, runner = make_runner(cycles=2)
    results = runner.run()
    assert not runner.aborted
    assert all(r.status == hc.PASS for r in results)
    assert [r.step for r in results if r.cycle == 2] == hc.CYCLE_STEPS
    assert results[0].step == "preflight"
    assert be.calls.count(hc.STOP_SERVICE) == 2  # once per cycle, from the stop test
    assert "pick:tag_2" in be.calls and "place:tag_1" in be.calls


def test_preflight_failure_aborts_before_any_motion():
    be = FakeBackend()
    be.missing = {hc.PICK_ACTION, "tag_2", hc.JOINT_STATES_TOPIC}
    _, runner = make_runner(be)
    results = runner.run()
    assert runner.aborted
    assert len(results) == 1 and results[0].status == hc.FAIL
    for part in (hc.PICK_ACTION, "tag_2", "/joint_states"):
        assert part in results[0].message
    assert not any(c.startswith(("move_to", "pick", "place")) for c in be.calls)
    assert be.calls[-1] == hc.STOP_SERVICE


def test_first_motion_failure_stops_everything():
    be = FakeBackend()
    be.fail["pick"] = "tag_2 not visible"
    _, runner = make_runner(be, cycles=3)
    results = runner.run()
    assert runner.aborted
    assert results[-1].step == "pick" and results[-1].status == hc.FAIL
    assert "tag_2 not visible" in results[-1].message
    assert be.calls[-1] == hc.STOP_SERVICE
    assert not any(r.cycle > 1 for r in results)
    assert not any(c.startswith("place") for c in be.calls)


def test_skip_steps_are_recorded_and_not_run():
    be, runner = make_runner(skip=("driver_recovery", "pick_again", "place_again"))
    results = runner.run()
    skipped = [r.step for r in results if r.status == hc.SKIP]
    assert skipped == ["pick_again", "place_again", "driver_recovery"]
    assert be.calls.count("pick:tag_2") == 1


def test_stop_test_fails_if_move_completes_despite_stop():
    be = FakeBackend()
    be.stop_move_succeeds = True
    _, runner = make_runner(be)
    results = runner.run()
    assert runner.aborted
    assert results[-1].step == "stop_mid_motion" and results[-1].status == hc.FAIL


def test_stop_test_fails_if_goal_hangs():
    be = FakeBackend()
    be.hang_after_stop = True
    _, runner = make_runner(be)
    results = runner.run()
    assert results[-1].step == "stop_mid_motion" and results[-1].status == hc.FAIL
    assert "still running" in results[-1].message


def test_stop_test_rejected_goal_fails():
    be = FakeBackend()
    be.accept_goal = False
    _, runner = make_runner(be)
    assert runner.run()[-1].status == hc.FAIL


def test_resume_failure_aborts():
    be = FakeBackend()
    be.fail[hc.RESUME_SERVICE] = "nope"
    _, runner = make_runner(be)
    results = runner.run()
    assert results[-1].step == "resume" and results[-1].status == hc.FAIL


def test_driver_recovery_operator_skip_still_passes():
    _, runner = make_runner()
    runner.prompt = lambda _p: "skip"
    last = runner.run()[-1]
    assert last.step == "driver_recovery" and last.status == hc.PASS
    assert "operator" in last.message


def test_unexpected_exception_is_a_failure_not_a_pass():
    be = FakeBackend()

    def boom(*_a):
        raise RuntimeError("lost connection")

    be.pick = boom
    _, runner = make_runner(be)
    results = runner.run()
    assert results[-1].status == hc.FAIL and "lost connection" in results[-1].message
    assert be.calls[-1] == hc.STOP_SERVICE


def test_speak_only_when_enabled():
    be, runner = make_runner(speak=True)
    runner.run()
    assert be.spoken
    be2, runner2 = make_runner()
    runner2.run()
    assert not be2.spoken


def test_summary_and_result_files(tmp_path):
    be = FakeBackend()
    be.fail["pick"] = "x"
    _, runner = make_runner(be)
    results = runner.run()
    summary = hc.summarize(results)
    assert summary["failed"] == 1
    assert summary["pass_rate"] == pytest.approx(
        summary["passed"] / summary["steps_run"]
    )
    path = hc.write_results(results, hc.Config(), runner.aborted, tmp_path / "sub")
    data = json.loads(path.read_text())
    assert data["aborted"] is True and len(data["results"]) == len(results)
    csv_lines = path.with_suffix(".csv").read_text().strip().splitlines()
    assert csv_lines[0] == "cycle,step,status,duration_s,message"
    assert len(csv_lines) == len(results) + 1
    assert "pick" in hc.format_table(results)


def test_summary_empty():
    assert hc.summarize([])["pass_rate"] is None


def never_backend():
    raise AssertionError("backend must not be created")


def test_dry_run_sends_nothing(capsys):
    assert hc.main(["--dry-run", "--cycles", "2"], backend_factory=never_backend) == 0
    out = capsys.readouterr().out
    assert "Dry run" in out and "[2] driver_recovery" in out


def test_motion_requires_confirmation_flag(capsys):
    assert hc.main(["--cycles", "1"], backend_factory=never_backend) == 2
    assert "--i-am-at-the-robot" in capsys.readouterr().err


def test_main_end_to_end_with_fake_backend(tmp_path, capsys):
    be = FakeBackend()
    code = hc.main(
        [
            "--i-am-at-the-robot",
            "--results-dir",
            str(tmp_path),
            "--skip",
            "driver_recovery",
        ],
        backend_factory=lambda: be,
    )
    assert code == 0
    assert list(tmp_path.glob("hil_*.json")) and list(tmp_path.glob("hil_*.csv"))
    assert "steps passed" in capsys.readouterr().out


def test_unknown_skip_rejected():
    with pytest.raises(SystemExit):
        hc.parse_args(["--skip", "bogus"])


def _ros_available():
    try:
        import rclpy  # noqa: F401

        import colman_interfaces.action  # noqa: F401
    except ImportError:
        return False
    return True


@pytest.mark.skipif(not _ros_available(), reason="needs the pixi ROS environment")
def test_ros_backend_against_fake_servers():
    import rclpy
    from rclpy.action import ActionServer
    from rclpy.executors import MultiThreadedExecutor
    from std_srvs.srv import Trigger

    from colman_interfaces.action import MoveTo, Pick, Place

    # Private domain so this can never reach a real motion server on the network.
    os.environ["ROS_DOMAIN_ID"] = "117"
    rclpy.init()
    server = rclpy.create_node("fake_motion")
    state = {"stopped": False}

    def make_cb(action):
        def cb(goal_handle):
            time.sleep(0.2)
            res = action.Result()
            res.success = not state["stopped"]
            if hasattr(res, "message"):
                res.message = "fake"
            if res.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return res

        return cb

    servers = [
        ActionServer(server, MoveTo, "/move_to", make_cb(MoveTo)),
        ActionServer(server, Pick, "/pick", make_cb(Pick)),
        ActionServer(server, Place, "/place", make_cb(Place)),
    ]

    def stop(_req, resp):
        state["stopped"] = True
        resp.success = True
        return resp

    def resume(_req, resp):
        state["stopped"] = False
        resp.success = True
        return resp

    server.create_service(Trigger, "/stop", stop)
    server.create_service(Trigger, "/resume", resume)
    ex = MultiThreadedExecutor()
    ex.add_node(server)
    threading.Thread(target=ex.spin, daemon=True).start()
    be = hc.RosBackend()
    try:
        assert be.has_action(hc.MOVE_TO_ACTION)
        assert be.has_service(hc.STOP_SERVICE)
        assert be.move_to("home", 10.0)[0]
        assert be.pick("tag_2", 10.0)[0]
        assert be.trigger(hc.STOP_SERVICE, 5.0)[0] and state["stopped"]
        assert not be.place("tag_1", 10.0)[0]
        assert be.trigger(hc.RESUME_SERVICE, 5.0)[0]
        assert be.place("tag_1", 10.0)[0]
    finally:
        be.close()
        ex.shutdown()
        del servers
        server.destroy_node()
        rclpy.try_shutdown()
