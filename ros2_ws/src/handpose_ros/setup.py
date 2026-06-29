from setuptools import find_packages, setup

package_name = "handpose_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="KAIROS7-team3",
    maintainer_email="seojunsoo312@gmail.com",
    description="MediaPipe 손 감지 ROS2 노드",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mediapipe_hands_node = handpose_ros.mediapipe_hands_node:main",
        ],
    },
)
