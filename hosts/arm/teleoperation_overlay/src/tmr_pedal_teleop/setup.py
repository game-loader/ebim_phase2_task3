import os
from glob import glob

from setuptools import setup

package_name = "tmr_pedal_teleop"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="demo",
    maintainer_email="osmallfrogo.hchs@gmail.com",
    description="Foot-pedal bridge to the TMR mobile base and Franka spine.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "base_bridge = tmr_pedal_teleop.base_bridge:main",
            "spine_bridge = tmr_pedal_teleop.spine_bridge:main",
        ],
    },
)
