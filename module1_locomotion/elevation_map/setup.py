from setuptools import find_packages, setup
from glob import glob
import os

package_name = "elevation_map"

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
    description="LiDAR/깊이 포인트로 지형 elevation grid를 갱신한다.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "elevation_map_node = elevation_map.elevation_map_node:main",
        ],
    },
)
