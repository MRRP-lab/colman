import os
from glob import glob

from setuptools import find_packages, setup

package_name = "colman_gazebo"


def model_files():
    """Map each directory under models/ to its files, keeping the tree layout."""
    entries = {}
    models = os.path.join(package_name, "models")
    for path in glob(os.path.join(models, "**", "*"), recursive=True):
        if os.path.isfile(path):
            subdir = os.path.relpath(os.path.dirname(path), models)
            target = os.path.join("share", package_name, "models", subdir)
            entries.setdefault(target, []).append(path)
    return list(entries.items())

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/worlds",
            glob(os.path.join(package_name, "worlds", "*")),
        ),
        (
            "share/" + package_name + "/config",
            glob(os.path.join(package_name, "config", "*")),
        ),
        (
            "share/" + package_name + "/urdf",
            glob(os.path.join(package_name, "urdf", "*")),
        ),
    ]
    + model_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pierce",
    maintainer_email="dev@piercecoyle.com",
    description="Gazebo worlds, models and simulation support for the Colman workcell",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [],
    },
)
