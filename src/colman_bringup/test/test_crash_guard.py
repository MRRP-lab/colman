from colman_bringup.crash_guard import CrashLoopGuard


def test_first_crash_restarts_after_base_delay():
    d = CrashLoopGuard(delay=2.0).record_crash(0.0)
    assert d.restart and d.delay == 2.0 and d.crashes == 1


def test_delay_doubles_and_is_capped():
    g = CrashLoopGuard(delay=2.0, max_delay=7.0, max_crashes=10, window=100)
    assert [g.record_crash(t).delay for t in range(5)] == [2.0, 4.0, 7.0, 7.0, 7.0]


def test_gives_up_after_max_crashes_in_window_and_stays_given_up():
    g = CrashLoopGuard(max_crashes=3, window=60)
    assert g.record_crash(0).restart
    assert g.record_crash(1).restart
    assert not g.record_crash(2).restart
    assert g.gave_up
    assert not g.record_crash(1000).restart


def test_spread_out_crashes_never_give_up_and_reset_backoff():
    g = CrashLoopGuard(delay=2.0, max_crashes=3, window=60)
    for i in range(20):
        d = g.record_crash(i * 100.0)
        assert d.restart and d.delay == 2.0
