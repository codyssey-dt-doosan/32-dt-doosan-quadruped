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
    description="CoM/접지력 MPC로 cmd_vel 또는 관절 명령을 낸다.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mpc_controller_node = mpc_controller.mpc_controller_node:main",
        ],
    },
)
