from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = "voice"
repo_root = Path(__file__).resolve().parents[3]
toolbox_path = repo_root / "config" / "toolbox.yaml"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    package_data={package_name: ["gemma_prompt.txt"]},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}", ["voice/gemma_prompt.txt"]),
        (
            f"share/{package_name}/config",
            glob("config/*.yaml"),
        ),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="KAIROS7-team3",
    maintainer_email="team@example.com",
    description="Whisper STT and intent classification package for Track A/B.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "whisper_node = voice.whisper_node:main",
            "gemma_intent_node = voice.gemma_intent_node:main",
            "rule_intent_node = voice.rule_intent_node:main",
        ],
    },
)
