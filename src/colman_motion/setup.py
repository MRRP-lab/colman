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
    license="TODO: License declaration",
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
            "demo_camera = colman_motion.demo_camera:main",
            "hand_up = colman_motion.hand_up:main",
            "look_around = colman_motion.look_around:main",
            "click_move = colman_motion.click_move:main",
            "calib_sweep = colman_motion.calib_sweep:main",
        ],
    },
)
