from setuptools import find_packages, setup
from glob import glob
import os

package_name = "patrol_path"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="태우",
    maintainer_email="team@doosan.local",
    description="복도/공장 웨이포인트와 점검 정차점을 순회한다.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "patrol_path_node = patrol_path.patrol_path_node:main",
        ],
    },
)
