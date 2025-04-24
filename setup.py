from setuptools import setup
import os
from glob import glob

package_name = "obstacle_perception"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "msg"), glob("msg/*.msg")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),

    ],
    install_requires=["setuptools","robot_msgs"],
    zip_safe=True,
    maintainer="Pito Salas",  # Change this to your name
    maintainer_email="pitosalas@gmail.com",
    description="Obstacle perception using costmap and beam checking.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perception_node = obstacle_perception.perception_node:main",
            "costmap_client = obstacle_perception.costmap_client:main",
            "costmap_subscriber = obstacle_perception.costmap_subscriber:main",
            "beam_checker = obstacle_perception.beam_checker:main",
            "explorer_node = obstacle_perception.explorer_node:main",
            "explore_purposeful = obstacle_perception.explore_purposeful:main",
        ],
    },
)
