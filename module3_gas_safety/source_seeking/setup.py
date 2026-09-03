from setuptools import find_packages, setup
from glob import glob
import os

package_name = "source_seeking"

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
    description="농도 구배로 누출원을 추적한다.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "source_seeking_node = source_seeking.source_seeking_node:main",
        ],
    },
)
