"""
Captures one still, undistorted color frame from a specific OAK camera, with
its intrinsics, for grid calibration.

Usage:
    python capture_grid.py <device id> <output stem> [manual focus 0-255 or -1] \
        [width height]

Writes <stem>_rgb.png and <stem>.npz (rgb_k, device_id, focus). The default size
is 1920x1080; the OAK-D Pro's IMX378 goes up to 4056x3040. Takes several
frames and keeps the sharpest by Laplacian variance. The overhead camera
autofocuses badly at 1.8 m, so give it a manual focus position (118 worked).
Don't run this while camera_publisher holds the camera.
"""

import sys
import time

import cv2
import depthai as dai
import numpy as np

WIDTH = 1920
HEIGHT = 1080
FRAMES = 15
SETTLE_S = 2.0


def sharpness(frame):
    return cv2.Laplacian(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()


def main():
    device_id, stem = sys.argv[1:3]
    focus = int(sys.argv[3]) if len(sys.argv) > 3 else -1
    focus = None if focus < 0 else focus
    width, height = (
        (int(sys.argv[4]), int(sys.argv[5])) if len(sys.argv) > 5 else (WIDTH, HEIGHT)
    )

    device = dai.Device(dai.DeviceInfo(device_id))
    with dai.Pipeline(device) as pipeline:
        cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        out = cam.requestOutput((width, height), enableUndistortion=True, fps=15.0)
        queue = out.createOutputQueue(maxSize=4, blocking=False)
        control = cam.inputControl.createInputQueue()
        pipeline.start()

        if focus is not None:
            command = dai.CameraControl()
            command.setManualFocus(focus)
            control.send(command)
        time.sleep(SETTLE_S)

        best, best_score, k = None, -1.0, None
        for _ in range(FRAMES):
            packet = queue.get()
            frame = packet.getCvFrame()
            score = sharpness(frame)
            if score > best_score:
                best, best_score = frame, score
                k = np.array(packet.getTransformation().getIntrinsicMatrix())

    cv2.imwrite(f"{stem}_rgb.png", best)
    np.savez(
        f"{stem}.npz",
        rgb_k=k,
        device_id=device_id,
        focus=-1 if focus is None else focus,
    )
    print(f"saved {stem}_rgb.png and {stem}.npz, sharpness {best_score:.1f}")
    print("rgb_k =", k.tolist())


if __name__ == "__main__":
    main()
