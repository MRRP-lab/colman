from setuptools import find_packages, setup

package_name = "colman_audio_output"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="neil",
    maintainer_email="neil.macneale.v@gmail.com",
    description="Speech output (text to speech) nodes for Colman",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "eleven_labs = colman_audio_output.eleven_labs:main",
        ],
    },
)
