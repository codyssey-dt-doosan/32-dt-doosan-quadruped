from setuptools import find_packages, setup
from glob import glob
import os

package_name = "fall_recovery"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="도훈",
    maintainer_email="team@doosan.local",
    description="IMU로 전도를 감지하고 일어서기 시퀀스를 수행한다.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fall_recovery_node = fall_recovery.fall_recovery_node:main",
        ],
    },
)
