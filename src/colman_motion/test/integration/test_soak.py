"""Soak: repeated pick and place cycles, with latency and resource numbers.

Slow, so it only runs on demand (pixi run test-motion-faults). SOAK_CYCLES sets the
number of cycles (default 10). Logs the pass rate, the time spent in each stage and
how the server's thread count and memory move over the run, and fails if cycles fail
or the thread count keeps growing.
"""

import json
import os
import statistics

import pytest

pytestmark = pytest.mark.slow

CYCLES = int(os.environ.get("SOAK_CYCLES", "10"))
MIN_PASS_RATE = 0.95
# Threads may settle as pools warm up; they must not climb with every cycle.
MAX_THREAD_GROWTH = 6
MAX_RSS_GROWTH_MB = 300
# pick/place from tag_a onto tag_b, so the cubes move around.
PAIRS = [("tag_2", "tag_1"), ("tag_3", "tag_5"), ("tag_4", "tag_6"), ("tag_7", "tag_8")]


def os_threads(pid):
    return len(os.listdir(f"/proc/{pid}/task"))


def rss_mb(pid):
    with open(f"/proc/{pid}/status") as status:
        for line in status:
            if line.startswith("VmRSS"):
                return int(line.split()[1]) / 1024
    return 0.0


def stage_durations(run):
    """Seconds spent in each feedback stage of a goal, and in total."""
    times = [(t, s) for t, s in run.stages] + [(run.done_at, "end")]
    out = {}
    for (start, stage), (end, _) in zip(times, times[1:]):
        out[stage] = out.get(stage, 0.0) + (end - start)
    return out


def test_pick_place_soak(healthy, capsys):
    client = healthy.client
    pid = client.server_state()["pid"]
    samples = []
    stages = {}
    passed = 0
    for cycle in range(CYCLES):
        source, target = PAIRS[cycle % len(PAIRS)]
        results = [client.pick(source).wait_done()]
        if results[0].outcome == "succeeded":
            results.append(client.place(target).wait_done())
        ok = len(results) == 2 and all(r.outcome == "succeeded" for r in results)
        if not ok:
            # Let go and go home so one failure does not fail every later cycle.
            healthy.recover()
        passed += ok
        for run in results:
            for stage, seconds in stage_durations(run).items():
                stages.setdefault(f"{run.label.split('{')[0]}:{stage}", []).append(
                    seconds
                )
        state = client.server_state()
        samples.append(
            {
                "cycle": cycle,
                "ok": ok,
                "python_threads": state["threads"],
                "os_threads": os_threads(pid),
                "rss_mb": round(rss_mb(pid), 1),
                "messages": [r.message for r in results if r.outcome != "succeeded"],
            }
        )
        print(json.dumps(samples[-1]))
    summary = {
        "cycles": CYCLES,
        "pass_rate": passed / CYCLES,
        "stage_seconds_median": {
            k: round(statistics.median(v), 2) for k, v in sorted(stages.items())
        },
        "stage_seconds_max": {k: round(max(v), 2) for k, v in sorted(stages.items())},
        "os_threads_first_last": [samples[0]["os_threads"], samples[-1]["os_threads"]],
        "python_threads_first_last": [
            samples[0]["python_threads"],
            samples[-1]["python_threads"],
        ],
        "rss_mb_first_last": [samples[0]["rss_mb"], samples[-1]["rss_mb"]],
    }
    with capsys.disabled():
        print("\nSOAK SUMMARY " + json.dumps(summary, indent=2))
    assert summary["pass_rate"] >= MIN_PASS_RATE, summary
    assert samples[-1]["os_threads"] - samples[0]["os_threads"] <= MAX_THREAD_GROWTH, (
        summary
    )
    assert samples[-1]["rss_mb"] - samples[0]["rss_mb"] <= MAX_RSS_GROWTH_MB, summary
