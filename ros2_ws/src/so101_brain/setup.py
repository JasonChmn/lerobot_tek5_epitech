from setuptools import setup
from glob import glob

package_name = "so101_brain"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/brain.launch.py"]),
        ("share/" + package_name + "/srv", glob("srv/*.srv")),
    ],
    entry_points={
        "console_scripts": [
            "brain = so101_brain.brain_node:main",
        ],
    },
)
