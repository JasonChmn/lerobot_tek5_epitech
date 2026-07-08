from glob import glob

from setuptools import setup

package_name = "so101_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/srv", glob("srv/*.srv")),
    ],
    entry_points={
        "console_scripts": [
            "driver = so101_driver.driver_node:main",
        ],
    },
)
