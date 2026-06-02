from setuptools import find_packages, setup

package_name = "db"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]) + ["db_core"],
    package_dir={"db_core": "../../../db_core"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="KAIROS7-team3",
    maintainer_email="gold73201-collab@users.noreply.github.com",
    description="ROS2 DB service package for Track A/B.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "db_service_node = db.db_service_node:main",
            "fod_monitor_node = db.fod_monitor_node:main",
            "intent_status_simulator_node = db.intent_status_simulator_node:main",
        ],
    },
)
