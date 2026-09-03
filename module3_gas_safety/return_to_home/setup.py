from setuptools import find_packages, setup
from glob import glob
import os

package_name = "return_to_home"

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
    maintainer="채현",
    maintainer_email="team@doosan.local",
    description="알람/배터리/임무 종료 시 홈 포즈로 복귀한다.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "return_to_home_node = return_to_home.return_to_home_node:main",
        ],
    },
)
