from setuptools import find_packages, setup
import os
from glob import glob

package_name = "vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "camera_node = vision.camera_node:main",
            "yolo_node = vision.yolo_node:main",
            "pose_node = vision.pose_node:main",
            "tracker_node = vision.tracker_node:main",
            "context_builder = vision.context_builder:main",
        ],
    },
)
