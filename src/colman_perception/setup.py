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
    description="OAK-D, AprilTag and optional YOLO perception for the Colman robot",
    license="Apache-2.0",
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
            # fitting where the wrist and overhead cameras are,
            # see camera_pose_calibration/README.md
            "capture_undistorted_camera_still = "
            "colman_perception.capture_undistorted_camera_still:main",
            "fit_wrist_mount_to_table_edges = "
            "colman_perception.fit_wrist_mount_to_table_edges:main",
            "fit_overhead_pose_to_table_edges = "
            "colman_perception.fit_overhead_pose_to_table_edges:main",
            "check_overhead_pose_with_grid = "
            "colman_perception.check_overhead_pose_with_grid:main",
            "fit_overhead_tilt_and_height_to_grids = "
            "colman_perception.fit_overhead_tilt_and_height_to_grids:main",
            "fit_wrist_mount_to_grid_views = "
            "colman_perception.fit_wrist_mount_to_grid_views:main",
            "merge_overhead_pose_fits = "
            "colman_perception.merge_overhead_pose_fits:main",
            "click_touch_off_marks = colman_perception.click_touch_off_marks:main",
            "record_touched_points = colman_perception.record_touched_points:main",
            "record_touched_point_once = "
            "colman_perception.record_touched_point_once:main",
            "check_overhead_pose_against_touched_points = "
            "colman_perception.check_overhead_pose_against_touched_points:main",
            "fit_overhead_pose_to_touched_points = "
            "colman_perception.fit_overhead_pose_to_touched_points:main",
            "score_wrist_mounts_against_touched_circles = "
            "colman_perception.score_wrist_mounts_against_touched_circles:main",
        ],
    },
)
