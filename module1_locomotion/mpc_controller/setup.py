from setuptools import find_packages, setup
from glob import glob
import os

package_name = "mpc_controller"

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
    description="elevation map 전방 헤딩 스캔으로 장애물 피해 /patrol/goal로 간다. 진짜 MPC 아님(향후).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mpc_controller_node = mpc_controller.mpc_controller_node:main",
        ],
    },
)
