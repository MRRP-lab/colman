import os
from glob import glob

from setuptools import find_packages, setup

package_name = "colman_perception"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/config",
            glob(os.path.join(package_name, "config", "*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pierce",
    maintainer_email="dev@piercecoyle.com",
    description="OAK-D, AprilTag and YOLO perception for the Colman robot",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "detection = colman_perception.detection:main",
            "camera_info_republisher = colman_perception.camera_info_republisher:main",
            "camera_publisher = colman_perception.camera_publisher:main",
            "list_cameras = colman_perception.camera_names:main",
            "print_tags = colman_perception.print_tags:main",
            "world_state = colman_perception.world_state:main",
            # camera calibration, see calibration/README.md
            "capture_grid = colman_perception.capture_grid:main",
            "calibrate_camera = colman_perception.calibrate_camera:main",
            "calibrate_overhead = colman_perception.calibrate_overhead:main",
            "grid_check_overhead = colman_perception.grid_check_overhead:main",
            "grid_fit_overhead = colman_perception.grid_fit_overhead:main",
            "grid_fit_wrist = colman_perception.grid_fit_wrist:main",
            "merge_overhead = colman_perception.merge_overhead:main",
            "touch_off_pick = colman_perception.touch_off_pick:main",
            "touch_off_record = colman_perception.touch_off_record:main",
            "touch_off_record_once = colman_perception.touch_off_record_once:main",
            "touch_off_check = colman_perception.touch_off_check:main",
            "touch_off_fit = colman_perception.touch_off_fit:main",
            "wrist_circle_score = colman_perception.wrist_circle_score:main",
        ],
    },
)
