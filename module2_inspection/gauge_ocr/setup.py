from setuptools import find_packages, setup
from glob import glob
import os

package_name = "gauge_ocr"

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
    description="카메라 영상에서 게이지 ROI를 잡고 지침/숫자를 판독한다.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gauge_ocr_node = gauge_ocr.gauge_ocr_node:main",
        ],
    },
)
