from setuptools import find_packages, setup

package_name = "colman_audio_input"

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
    maintainer="pierce",
    maintainer_email="dev@piercecoyle.com",
    description="Speech input (microphone to text) nodes for Colman",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "speech_to_text = colman_audio_input.speech_to_text_node:main",
        ],
    },
)
