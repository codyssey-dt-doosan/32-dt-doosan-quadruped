from setuptools import find_packages, setup
from glob import glob
import os

package_name = "thermal_fusion"

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
    maintainer="운학",
    maintainer_email="team@doosan.local",
    description="RGB와 열화상을 정렬·융합해 과열 영역을 표시한다.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "thermal_fusion_node = thermal_fusion.thermal_fusion_node:main",
        ],
    },
)
