"""Names for the OAK cameras, so a node can ask for "wrist" or "overhead".

The serials live in config/cameras.yaml. Run `ros2 run colman_perception
list_cameras` to see which connected camera has which name.
"""

from pathlib import Path

import yaml

CONFIG = Path(__file__).parent / "config" / "cameras.yaml"


def load_cameras(path=CONFIG):
    """{name: {"mxid": ..., "mount": ...}} from the config file."""
    with open(path) as f:
        return yaml.safe_load(f)["cameras"]


def mxid_for(name, cameras=None):
    cameras = load_cameras() if cameras is None else cameras
    if name not in cameras:
        raise ValueError(
            f"unknown camera '{name}', known cameras: {', '.join(cameras)}"
        )
    return cameras[name]["mxid"]


def name_for(mxid, cameras=None):
    """The name for a serial, or None if it is not in the config."""
    cameras = load_cameras() if cameras is None else cameras
    for name, entry in cameras.items():
        if entry["mxid"] == mxid:
            return name
    return None


def device_info(name):
    """The depthai DeviceInfo for a named camera, so dai.Device opens that one."""
    import depthai as dai

    return dai.DeviceInfo(mxid_for(name))


def main():
    """Print every camera in the config and every camera that is plugged in."""
    import depthai as dai

    cameras = load_cameras()
    connected = {d.getDeviceId() for d in dai.Device.getAllAvailableDevices()}
    for name, entry in cameras.items():
        state = "connected" if entry["mxid"] in connected else "NOT connected"
        print(f"{name:10s} {entry['mxid']}  {state}  ({entry['mount']})")
    for mxid in sorted(connected):
        if name_for(mxid, cameras) is None:
            print(f"{'unnamed':10s} {mxid}  connected  (add it to config/cameras.yaml)")


if __name__ == "__main__":
    main()
