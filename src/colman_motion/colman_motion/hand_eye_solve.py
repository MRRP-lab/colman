import cv2
import numpy as np
from scipy.spatial.transform import Rotation

# We use all methods to make sure they give similar outputs
METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}

PATH = "calib_samples.npz"

def solve(path):
    data = np.load(path, allow_pickle=True)
    arm_rotations = list(data["arm_rotations"])
    arm_translations = list(data["arm_translations"])
    tag_rotations = list(data["tag_rotations"])
    tag_translations = list(data["tag_translations"])

    for name, method in METHODS.items():
        cam_gripper_rotation, cam_gripper_translation = cv2.calibrateHandEye(
            arm_rotations,
            arm_translations,
            tag_rotations,
            tag_translations,
            method=method,
        )

        # invert transform for urdf
        gripper_cam_rotation = cam_gripper_rotation.T
        gripper_cam_translation = -gripper_cam_rotation @ cam_gripper_translation

        # remove the fixed optical frame rotation
        optical_rotation = Rotation.from_euler("xyz", [-np.pi/2, 0, -np.pi/2]).as_matrix()
        gripper_cam_rotation = gripper_cam_rotation @ optical_rotation.T

        xyz = gripper_cam_translation.flatten()
        rpy = Rotation.from_matrix(gripper_cam_rotation).as_euler("xyz")

        print(name)
        print(
            f'<origin xyz="{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}" '
            f'rpy="{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}" />'
        )
        print()


if __name__ == "__main__":
    solve(PATH)
