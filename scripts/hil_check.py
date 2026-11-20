#!/usr/bin/env python3
"""Hardware-in-the-loop reliability check for the real arm. Not run in CI.

Drives a running stack (``pixi run motion`` plus driver, detector and so on) through a
fixed sequence over the normal ROS interfaces and records pass/fail per step:

    preflight -> [home -> check_tags -> pick -> place -> stop_mid_motion -> resume
                  -> pick_again -> place_again -> driver_recovery] x N cycles

Usage (dry run first, it sends nothing):

    pixi run hil-check --dry-run
    pixi run hil-check --i-am-at-the-robot --cycles 3

See docs/hil_checklist.md. Importing this module never touches ROS or the arm.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

RESULTS_DIR = Path(__file__).resolve().parent.parent / "hil_results"

HOME_POSE = "home"
MOVE_TO_ACTION = "/move_to"
PICK_ACTION = "/pick"
PLACE_ACTION = "/place"
STOP_SERVICE = "/stop"
RESUME_SERVICE = "/resume"
GRASP_SERVICE = "/grasp"
RELEASE_SERVICE = "/release"
TTS_TOPIC = "llm_response"
JOINT_STATES_TOPIC = "/joint_states"
IMAGE_TOPICS = ["/oakd_pro/rgb/image", "/overhead/rgb/image"]
BASE_FRAME = "base_link"

DEFAULT_TAGS = ["tag_1", "tag_2"]
DEFAULT_PICK_TAG = "tag_2"
DEFAULT_PLACE_TAG = "tag_1"
DEFAULT_STOP_DELAY_S = 1.0
# Pose for the stop test: far from home so the move takes a while.
DEFAULT_STOP_TARGET = "Compact"

CYCLE_STEPS = [
    "home",
    "check_tags",
    "pick",
    "place",
    "stop_mid_motion",
    "resume",
    "pick_again",
    "place_again",
    "driver_recovery",
]
PASS, FAIL, SKIP = "pass", "fail", "skip"


@dataclass
class StepResult:
    cycle: int  # 0 is preflight
    step: str
    status: str
    duration_s: float
    message: str = ""


class SafetyAbort(Exception):
    """A step failed. The runner calls /stop and ends the run."""


# --------------------------------------------------------------------------------------
# Backend interface. The runner only uses these methods, so tests substitute a fake.
# Every method returns plain data; failures are (False, message), not exceptions.
# --------------------------------------------------------------------------------------


class Backend:
    def node_names(self) -> list[str]:
        raise NotImplementedError

    def has_action(self, name: str) -> bool:
        raise NotImplementedError

    def has_service(self, name: str) -> bool:
        raise NotImplementedError

    def has_subscriber(self, topic: str) -> bool:
        raise NotImplementedError

    def wait_message(self, topic: str, timeout_s: float) -> bool:
        """True if at least one message arrives on topic (joint_states or an image)."""
        raise NotImplementedError

    def tag_visible(self, tag: str, timeout_s: float) -> bool:
        raise NotImplementedError

    def trigger(self, service: str, timeout_s: float) -> tuple[bool, str]:
        raise NotImplementedError

    def move_to(self, pose: str, timeout_s: float) -> tuple[bool, str]:
        raise NotImplementedError

    def pick(self, frame: str, timeout_s: float) -> tuple[bool, str]:
        raise NotImplementedError

    def place(self, location: str, timeout_s: float) -> tuple[bool, str]:
        raise NotImplementedError

    def start_move_to(self, pose: str):
        """Send a MoveTo goal; return a handle with accepted/result/cancel."""
        raise NotImplementedError

    def say(self, text: str) -> None:
        raise NotImplementedError

    def close(self) -> None:
        pass


# --------------------------------------------------------------------------------------
# Runner
# --------------------------------------------------------------------------------------


@dataclass
class Config:
    cycles: int = 1
    skip: tuple = ()
    tags: tuple = tuple(DEFAULT_TAGS)
    pick_tag: str = DEFAULT_PICK_TAG
    place_tag: str = DEFAULT_PLACE_TAG
    stop_delay_s: float = DEFAULT_STOP_DELAY_S
    stop_target: str = DEFAULT_STOP_TARGET
    motion_timeout_s: float = 90.0
    service_timeout_s: float = 5.0
    wait_timeout_s: float = 5.0
    speak: bool = False
    dry_run: bool = False


def expected_plan(cfg: Config) -> list[tuple[int, str, bool]]:
    """(cycle, step, skipped) for the whole run, preflight first."""
    plan = [(0, "preflight", "preflight" in cfg.skip)]
    for cycle in range(1, cfg.cycles + 1):
        for step in CYCLE_STEPS:
            plan.append((cycle, step, step in cfg.skip))
    return plan


class Runner:
    def __init__(self, backend: Backend, cfg: Config, prompt=input, log=print):
        self.be = backend
        self.cfg = cfg
        self.prompt = prompt
        self.log = log
        self.results: list[StepResult] = []
        self.aborted = False

    # -- bookkeeping -------------------------------------------------------------

    def record(self, cycle, step, status, started, message=""):
        result = StepResult(cycle, step, status, time.monotonic() - started, message)
        self.results.append(result)
        tail = f"  {message}" if message else ""
        self.log(
            f"[{cycle}] {step:<16} {status.upper():<4} {result.duration_s:6.1f}s{tail}"
        )
        return result

    def run_step(self, cycle, step, fn):
        if step in self.cfg.skip:
            self.record(cycle, step, SKIP, time.monotonic(), "skipped by --skip")
            return
        started = time.monotonic()
        try:
            message = fn() or ""
        except SafetyAbort as err:
            self.record(cycle, step, FAIL, started, str(err))
            raise
        except Exception as err:  # a crash in a step is a failure, never a silent pass
            self.record(cycle, step, FAIL, started, f"{type(err).__name__}: {err}")
            raise SafetyAbort(str(err)) from err
        self.record(cycle, step, PASS, started, message)

    def emergency_stop(self):
        """Best effort /stop. Never raises."""
        try:
            ok, msg = self.be.trigger(STOP_SERVICE, self.cfg.service_timeout_s)
            self.log(f"/stop sent: ok={ok} {msg}")
        except Exception as err:
            self.log(f"/stop FAILED to send: {err}")

    @staticmethod
    def need(ok_msg, what):
        ok, msg = ok_msg
        if not ok:
            raise SafetyAbort(f"{what} failed: {msg}")
        return msg

    # -- steps -------------------------------------------------------------------

    def preflight(self):
        problems = []
        for action in (MOVE_TO_ACTION, PICK_ACTION, PLACE_ACTION):
            if not self.be.has_action(action):
                problems.append(f"action {action} missing")
        for srv in (STOP_SERVICE, RESUME_SERVICE, GRASP_SERVICE, RELEASE_SERVICE):
            if not self.be.has_service(srv):
                problems.append(f"service {srv} missing")
        if not self.be.has_subscriber(TTS_TOPIC):
            problems.append(f"nobody listens on {TTS_TOPIC} (TTS node down)")
        if not self.be.wait_message(JOINT_STATES_TOPIC, self.cfg.wait_timeout_s):
            problems.append("no /joint_states")
        for topic in IMAGE_TOPICS:
            if not self.be.wait_message(topic, self.cfg.wait_timeout_s):
                problems.append(f"no frames on {topic}")
        for tag in self.cfg.tags:
            if not self.be.tag_visible(tag, self.cfg.wait_timeout_s):
                problems.append(f"{tag} not visible in TF")
        if not problems:
            # Vacuum off is harmless and proves the IO path to the controller answers.
            ok, msg = self.be.trigger(RELEASE_SERVICE, self.cfg.service_timeout_s)
            if not ok:
                problems.append(f"vacuum {RELEASE_SERVICE} failed: {msg}")
        if problems:
            raise SafetyAbort("; ".join(problems))
        return f"nodes seen: {len(self.be.node_names())}"

    def step_home(self):
        return self.need(self.be.move_to(HOME_POSE, self.cfg.motion_timeout_s), "home")

    def step_check_tags(self):
        missing = [
            t
            for t in self.cfg.tags
            if not self.be.tag_visible(t, self.cfg.wait_timeout_s)
        ]
        if missing:
            raise SafetyAbort(f"tags not visible after home: {', '.join(missing)}")
        return f"saw {', '.join(self.cfg.tags)}"

    def step_pick(self):
        return self.need(
            self.be.pick(self.cfg.pick_tag, self.cfg.motion_timeout_s), "pick"
        )

    def step_place(self):
        return self.need(
            self.be.place(self.cfg.place_tag, self.cfg.motion_timeout_s), "place"
        )

    def step_stop_mid_motion(self):
        handle = self.be.start_move_to(self.cfg.stop_target)
        if not handle.accepted(self.cfg.service_timeout_s):
            raise SafetyAbort("move_to goal for the stop test was rejected")
        time.sleep(self.cfg.stop_delay_s)
        ok, msg = self.be.trigger(STOP_SERVICE, self.cfg.service_timeout_s)
        if not ok:
            raise SafetyAbort(f"/stop call failed: {msg}")
        finished, success, rmsg = handle.result(self.cfg.motion_timeout_s)
        if not finished:
            handle.cancel()
            raise SafetyAbort("goal still running after /stop")
        if success:
            raise SafetyAbort(
                "move completed despite /stop (stop came too late or was ignored)"
            )
        # While stopped the server must reject new goals.
        ok, msg = self.be.move_to(HOME_POSE, self.cfg.service_timeout_s)
        if ok:
            raise SafetyAbort("a goal was accepted and succeeded while stopped")
        return f"halted ({rmsg or 'aborted'}); new goal refused"

    def step_resume(self):
        ok, msg = self.be.trigger(RESUME_SERVICE, self.cfg.service_timeout_s)
        if not ok:
            raise SafetyAbort(f"/resume failed: {msg}")
        self.need(
            self.be.move_to(HOME_POSE, self.cfg.motion_timeout_s), "move after resume"
        )
        return "moved home after resume"

    def step_driver_recovery(self):
        answer = self.prompt(
            "DRIVER RECOVERY: kill the UR driver process now, wait for the "
            "supervisor to recover it,\nthen press Enter (type 'skip' to skip): "
        )
        if answer.strip().lower() == "skip":
            return "skipped by operator"
        if not self.be.wait_message(JOINT_STATES_TOPIC, 30.0):
            raise SafetyAbort("no /joint_states after driver recovery")
        self.need(
            self.be.move_to(HOME_POSE, self.cfg.motion_timeout_s), "move after recovery"
        )
        return "joint_states back, home move ok"

    # -- driver ------------------------------------------------------------------

    def say(self, text):
        if self.cfg.speak:
            try:
                self.be.say(text)
            except Exception as err:
                self.log(f"(tts failed: {err})")

    def run(self) -> list[StepResult]:
        steps = {
            "home": self.step_home,
            "check_tags": self.step_check_tags,
            "pick": self.step_pick,
            "place": self.step_place,
            "stop_mid_motion": self.step_stop_mid_motion,
            "resume": self.step_resume,
            "pick_again": self.step_pick,
            "place_again": self.step_place,
            "driver_recovery": self.step_driver_recovery,
        }
        try:
            self.run_step(0, "preflight", self.preflight)
            for cycle in range(1, self.cfg.cycles + 1):
                self.say(f"Cycle {cycle} of {self.cfg.cycles}")
                for name in CYCLE_STEPS:
                    self.run_step(cycle, name, steps[name])
        except SafetyAbort:
            self.aborted = True
            self.emergency_stop()
            self.say("Hardware check failed. Stopped.")
        except KeyboardInterrupt:
            self.aborted = True
            self.log("interrupted")
            self.emergency_stop()
        else:
            self.say("Hardware check finished.")
        return self.results


# --------------------------------------------------------------------------------------
# Reporting
# --------------------------------------------------------------------------------------


def summarize(results: list[StepResult]) -> dict:
    ran = [r for r in results if r.status != SKIP]
    passed = [r for r in ran if r.status == PASS]
    per_step: dict[str, dict] = {}
    for r in ran:
        entry = per_step.setdefault(r.step, {"pass": 0, "fail": 0, "total_s": 0.0})
        entry[r.status] += 1
        entry["total_s"] += r.duration_s
    return {
        "steps_run": len(ran),
        "passed": len(passed),
        "failed": len(ran) - len(passed),
        "pass_rate": (len(passed) / len(ran)) if ran else None,
        "per_step": per_step,
    }


def format_table(results: list[StepResult]) -> str:
    rows = [("cycle", "step", "status", "secs", "message")]
    rows += [
        (str(r.cycle), r.step, r.status, f"{r.duration_s:.1f}", r.message)
        for r in results
    ]
    widths = [max(len(row[i]) for row in rows) for i in range(4)]
    lines = []
    for row in rows:
        head = "  ".join(cell.ljust(w) for cell, w in zip(row[:4], widths))
        lines.append(f"{head}  {row[4]}".rstrip())
    return "\n".join(lines)


def write_results(results, cfg: Config, aborted: bool, out_dir: Path, now=None) -> Path:
    """Write <stamp>.json and <stamp>.csv; return the JSON path."""
    now = now or datetime.now()
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = now.strftime("%Y%m%d_%H%M%S")
    json_path = out_dir / f"hil_{stamp}.json"
    payload = {
        "timestamp": now.isoformat(timespec="seconds"),
        "aborted": aborted,
        "config": {
            k: list(v) if isinstance(v, tuple) else v for k, v in asdict(cfg).items()
        },
        "summary": summarize(results),
        "results": [asdict(r) for r in results],
    }
    json_path.write_text(json.dumps(payload, indent=2))
    with open(json_path.with_suffix(".csv"), "w", newline="") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=["cycle", "step", "status", "duration_s", "message"]
        )
        writer.writeheader()
        for r in results:
            writer.writerow(asdict(r))
    return json_path


# --------------------------------------------------------------------------------------
# ROS backend (rclpy is imported only when this is constructed)
# --------------------------------------------------------------------------------------


class _Handle:
    def __init__(self, backend, goal_future):
        self.be = backend
        self.goal_future = goal_future
        self.goal_handle = None

    def accepted(self, timeout_s):
        if not self.be.wait_future(self.goal_future, timeout_s):
            return False
        self.goal_handle = self.goal_future.result()
        return bool(self.goal_handle.accepted)

    def result(self, timeout_s):
        """(finished, success, message)"""
        fut = self.goal_handle.get_result_async()
        if not self.be.wait_future(fut, timeout_s):
            return False, False, ""
        res = fut.result().result
        return True, bool(res.success), getattr(res, "message", "")

    def cancel(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()


class RosBackend(Backend):
    def __init__(self):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, JointState
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from tf2_ros import Buffer, TransformListener

        from colman_interfaces.action import MoveTo, Pick, Place

        self._rclpy = rclpy
        self._Trigger = Trigger
        self._types = {"Image": Image, "JointState": JointState}
        self._qos = qos_profile_sensor_data
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init()
        self.node = rclpy.create_node("hil_check")
        self.exec = MultiThreadedExecutor()
        self.exec.add_node(self.node)
        self.thread = threading.Thread(target=self.exec.spin, daemon=True)
        self.thread.start()
        self.tts_pub = self.node.create_publisher(String, TTS_TOPIC, 10)
        self._String = String
        self.clients = {
            MOVE_TO_ACTION: ActionClient(self.node, MoveTo, MOVE_TO_ACTION),
            PICK_ACTION: ActionClient(self.node, Pick, PICK_ACTION),
            PLACE_ACTION: ActionClient(self.node, Place, PLACE_ACTION),
        }
        self.MoveTo, self.Pick, self.Place = MoveTo, Pick, Place
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self.node)
        self.trigger_clients = {}

    def close(self):
        self.exec.shutdown()
        self.node.destroy_node()
        if self._owns_context:
            self._rclpy.shutdown()

    @staticmethod
    def wait_future(future, timeout_s):
        deadline = time.monotonic() + timeout_s
        while not future.done():
            if time.monotonic() > deadline:
                return False
            time.sleep(0.02)
        return True

    def node_names(self):
        return self.node.get_node_names()

    def has_action(self, name):
        return self.clients[name].wait_for_server(timeout_sec=3.0)

    def _trigger_client(self, name):
        if name not in self.trigger_clients:
            self.trigger_clients[name] = self.node.create_client(self._Trigger, name)
        return self.trigger_clients[name]

    def has_service(self, name):
        return self._trigger_client(name).wait_for_service(timeout_sec=3.0)

    def has_subscriber(self, topic):
        return self.node.count_subscribers(topic) > 0

    def wait_message(self, topic, timeout_s):
        msg_type = self._types["JointState" if topic == JOINT_STATES_TOPIC else "Image"]
        got = threading.Event()
        sub = self.node.create_subscription(
            msg_type, topic, lambda _m: got.set(), self._qos
        )
        try:
            return got.wait(timeout_s)
        finally:
            self.node.destroy_subscription(sub)

    def tag_visible(self, tag, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.tf.can_transform(BASE_FRAME, tag, self._rclpy.time.Time()):
                return True
            time.sleep(0.1)
        return False

    def trigger(self, service, timeout_s):
        client = self._trigger_client(service)
        if not client.wait_for_service(timeout_sec=timeout_s):
            return False, f"{service} not available"
        fut = client.call_async(self._Trigger.Request())
        if not self.wait_future(fut, timeout_s):
            return False, f"{service} timed out"
        res = fut.result()
        return bool(res.success), res.message

    def _send(self, name, goal, timeout_s):
        client = self.clients[name]
        fut = client.send_goal_async(goal)
        handle = _Handle(self, fut)
        if not handle.accepted(timeout_s if timeout_s < 10 else 10.0):
            return False, f"{name} goal rejected"
        finished, success, msg = handle.result(timeout_s)
        if not finished:
            handle.cancel()
            return False, f"{name} timed out after {timeout_s:.0f}s"
        return success, msg

    def move_to(self, pose, timeout_s):
        goal = self.MoveTo.Goal()
        goal.pose_name = pose
        return self._send(MOVE_TO_ACTION, goal, timeout_s)

    def pick(self, frame, timeout_s):
        goal = self.Pick.Goal()
        goal.frame = frame
        return self._send(PICK_ACTION, goal, timeout_s)

    def place(self, location, timeout_s):
        goal = self.Place.Goal()
        goal.location = location
        return self._send(PLACE_ACTION, goal, timeout_s)

    def start_move_to(self, pose):
        goal = self.MoveTo.Goal()
        goal.pose_name = pose
        return _Handle(self, self.clients[MOVE_TO_ACTION].send_goal_async(goal))

    def say(self, text):
        self.tts_pub.publish(self._String(data=text))


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument(
        "--i-am-at-the-robot",
        action="store_true",
        dest="confirmed",
        help="required for anything that moves the arm; you hold the e-stop",
    )
    p.add_argument(
        "--dry-run", action="store_true", help="print the plan, send nothing"
    )
    p.add_argument("--cycles", type=int, default=1)
    p.add_argument(
        "--skip",
        action="append",
        default=[],
        metavar="STEP",
        help=f"skip a step (repeatable). One of: preflight, {', '.join(CYCLE_STEPS)}",
    )
    p.add_argument(
        "--tags", default=",".join(DEFAULT_TAGS), help="tags that must be visible"
    )
    p.add_argument("--pick-tag", default=DEFAULT_PICK_TAG)
    p.add_argument("--place-tag", default=DEFAULT_PLACE_TAG)
    p.add_argument(
        "--stop-delay",
        type=float,
        default=DEFAULT_STOP_DELAY_S,
        help="seconds into the stop-test move before /stop",
    )
    p.add_argument(
        "--stop-target",
        default=DEFAULT_STOP_TARGET,
        help="named pose for the stop-test move",
    )
    p.add_argument("--motion-timeout", type=float, default=90.0)
    p.add_argument(
        "--speak", action="store_true", help="announce progress on llm_response"
    )
    p.add_argument("--results-dir", type=Path, default=RESULTS_DIR)
    args = p.parse_args(argv)
    valid = {"preflight", *CYCLE_STEPS}
    unknown = [s for s in args.skip if s not in valid]
    if unknown:
        p.error(f"unknown step(s) to skip: {', '.join(unknown)}")
    if args.cycles < 1:
        p.error("--cycles must be >= 1")
    return args


def config_from_args(args) -> Config:
    return Config(
        cycles=args.cycles,
        skip=tuple(args.skip),
        tags=tuple(t for t in args.tags.split(",") if t),
        pick_tag=args.pick_tag,
        place_tag=args.place_tag,
        stop_delay_s=args.stop_delay,
        stop_target=args.stop_target,
        motion_timeout_s=args.motion_timeout,
        speak=args.speak,
        dry_run=args.dry_run,
    )


def print_plan(cfg: Config, log=print):
    log(
        f"HIL plan: {cfg.cycles} cycle(s), tags {list(cfg.tags)}, "
        f"pick {cfg.pick_tag}, place {cfg.place_tag}"
    )
    for cycle, step, skipped in expected_plan(cfg):
        log(f"  [{cycle}] {step}{'  (skipped)' if skipped else ''}")
    log("Dry run: nothing was sent.")


def main(argv=None, backend_factory=RosBackend, prompt=input) -> int:
    args = parse_args(argv)
    cfg = config_from_args(args)
    if cfg.dry_run:
        print_plan(cfg)
        return 0
    if not args.confirmed:
        print(
            "Refusing to move the arm: pass --i-am-at-the-robot (or use --dry-run).",
            file=sys.stderr,
        )
        return 2
    backend = backend_factory()
    try:
        runner = Runner(backend, cfg, prompt=prompt)
        results = runner.run()
    finally:
        backend.close()
    print()
    print(format_table(results))
    summary = summarize(results)
    rate = summary["pass_rate"]
    print(
        f"\n{summary['passed']}/{summary['steps_run']} steps passed"
        + (f" ({rate:.0%})" if rate is not None else "")
        + ("  ABORTED" if runner.aborted else "")
    )
    path = write_results(results, cfg, runner.aborted, args.results_dir)
    print(f"results: {path} (+ .csv)")
    failed = runner.aborted or summary["failed"] > 0
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
