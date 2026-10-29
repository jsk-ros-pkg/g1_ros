from glob import glob

from setuptools import find_packages, setup


package_name = "g1_pico_teleop"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"lib/{package_name}", glob("scripts/pico_xrt_source.py")),
        (f"share/{package_name}/config", glob("config/*")),
        (f"share/{package_name}/patches", glob("patches/*.patch")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="G1 Pico Teleop maintainers",
    maintainer_email="maintainer@example.com",
    description="PICO optical-hand adapter for the G1 ROS 2 stack",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pico_hand_adapter = g1_pico_teleop.hand.hand_ros_adapter_node:main",
        ],
    },
)
