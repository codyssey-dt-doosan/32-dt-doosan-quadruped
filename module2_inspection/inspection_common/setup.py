from setuptools import find_packages, setup

package_name = "inspection_common"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="운학",
    maintainer_email="team@doosan.local",
    description="module2 점검 공용: 맵(SDF) 읽기, 기하, 점검 위치, 상태 문자열",
    license="Apache-2.0",
    tests_require=["pytest"],
)
