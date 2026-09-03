from setuptools import find_packages, setup
from glob import glob
import os

package_name = "plume_sim"

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
    description="탱크 누출원의 농도장을 가상 플랜트에 올린다.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "plume_sim_node = plume_sim.plume_sim_node:main",
        ],
    },
)
