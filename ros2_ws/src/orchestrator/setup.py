import os
from glob import glob
from setuptools import find_packages, setup

package_name = "orchestrator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="KAIROS7-team3",
    maintainer_email="team@example.com",
    description="Behavior Tree orchestrator for Track A/B.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "orchestrator_node = orchestrator.orchestrator_node:main",
        ],
    },
)
