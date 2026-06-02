from setuptools import find_packages, setup

package_name = "voice"

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
    maintainer_email="team@example.com",
    description="Whisper STT and intent classification package for Track A/B.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "list_audio_devices = voice.audio_input:list_audio_devices_main",
            "whisper_node = voice.whisper_node:main",
            "rule_intent_node = voice.rule_intent_node:main",
        ],
    },
)
