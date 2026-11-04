from setuptools import find_packages, setup

package_name = "colman_llm"

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
    description="Voice-driven LLM node for Colman",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "llm_node = colman_llm.llm_node:main",
        ],
    },
)
