import os
from glob import glob

from setuptools import setup

package_name = "simple_ros2_app"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*launch.[pxy][yma]*"),
        ),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.world")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.sh")),
        (os.path.join("share", package_name, "models"), glob("models/*.sdf")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="vt",
    maintainer_email="andrew.hrytsenko@outlook.com",
    description="Labs for ROS2 course at Kyiv Polytechnic Institute",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "px4_control = simple_ros2_app.nodes.px4_companion_node:main",
            "ground_control = simple_ros2_app.nodes.ground_control_node:main",
            # "mission = simple_ros2_app.nodes.simple_mission_node:main",
            "swarm_control = simple_ros2_app.nodes.swarm_control_node:main",
            "ardupilot_control = simple_ros2_app.nodes.ardupilot_companion_node:main",
            "optical_flow = simple_ros2_app.nodes.optical_flow_node:main",
        ],
    },
)
