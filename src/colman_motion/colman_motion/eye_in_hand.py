import sys
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformException, TransformListener

BASE_FRAME = "base_link"
EE_FRAME = "tool0"
CAMERA_FRAME = "oakd_pro_camera_rgb_optical_frame_wrist"
TAG_FRAME = "tag_20"
EXPORT_PATH = "calib_samples.npz"


class EyeInHandCalibration(Node):
    def __init__(self):
        super().__init__("eye_in_hand_calibration")

        self.arm_rotations = []
        self.arm_translations = []
        self.tag_rotations = []
        self.tag_translations = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        threading.Thread(target=self.control, daemon=True).start()

        self.get_logger().info("enter to save pose, q to quit")

    def control(self):
        for line in sys.stdin:
            key = line.strip().lower()
            if key == "":
                self.capture()
            elif key == "q":
                self.save()
                return

    def capture(self):
        try:
            ee = self.tf_buffer.lookup_transform(BASE_FRAME, EE_FRAME, Time())
            tag = self.tf_buffer.lookup_transform(CAMERA_FRAME, TAG_FRAME, Time())
        except TransformException as e:
            self.get_logger().warn(f"skipped frame: {e}")
            return

        R, t = get_rotation_translation(ee)
        self.arm_rotations.append(R)
        self.arm_translations.append(t)

        R, t = get_rotation_translation(tag)
        self.tag_rotations.append(R)
        self.tag_translations.append(t)

        self.get_logger().info(f"captured sample {len(self.arm_rotations)}")

    def save(self):
        np.savez(
            EXPORT_PATH,
            arm_rotations=self.arm_rotations,
            arm_translations=self.arm_translations,
            tag_rotations=self.tag_rotations,
            tag_translations=self.tag_translations,
        )
        self.get_logger().info(
            f"saved {len(self.arm_rotations)} samples to {EXPORT_PATH}"
        )
        rclpy.shutdown()


def get_rotation_translation(tf):
    """
    Seperates out the transformation and rotation for opencv calibration
    """
    transform = tf.transform.translation
    quat = tf.transform.rotation
    rotation = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_matrix()
    return rotation, np.array([[transform.x], [transform.y], [transform.z]])


def main():
    rclpy.init()
    node = EyeInHandCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
