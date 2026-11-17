import pytest
from colman_perception.camera_names import load_cameras, mxid_for, name_for


def test_wrist_and_overhead_are_named_and_distinct():
    cameras = load_cameras()
    assert {"wrist", "overhead"} <= set(cameras)
    assert mxid_for("wrist") != mxid_for("overhead")


def test_name_round_trips():
    for name in ("wrist", "overhead"):
        assert name_for(mxid_for(name)) == name


def test_unknown_serial_has_no_name():
    assert name_for("NOT-A-REAL-SERIAL") is None


def test_unknown_name_says_what_is_known():
    with pytest.raises(ValueError, match="wrist"):
        mxid_for("ceiling")
