from setuptools import find_packages, setup

package_name = "colman_motion"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "colman_motion/motion_server.launch.py",
                "colman_motion/demo.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pierce",
    maintainer_email="dev@piercecoyle.com",
    description="MoveIt arm, scene, AprilTag and vacuum control for the Colman robot",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "pick_and_place = colman_motion.pick_and_place:main",
            "motion_server = colman_motion.motion_server:main",
            "demo_turns = colman_motion.demo_turns:main",
            "demo_choose = colman_motion.demo_choose:main",
            "demo_circle = colman_motion.demo_circle:main",
            "demo_obstacle = colman_motion.demo_obstacle:main",
            "record_wrist_hand_eye_samples = "
            "colman_motion.record_wrist_hand_eye_samples:main",
            # fitting where the wrist and overhead cameras are,
            # see camera_pose_calibration/README.md
            "capture_wrist_table_edge_views = "
            "colman_motion.capture_wrist_table_edge_views:main",
            "capture_wrist_grid_views = colman_motion.capture_wrist_grid_views:main",
            "capture_wrist_views_of_touched_circles = "
            "colman_motion.capture_wrist_views_of_touched_circles:main",
        ],
    },
)
