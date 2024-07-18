from setuptools import find_packages, setup
from glob import glob

package_name = "ur_rtde_receiver"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"], include=["rtde_interfaces"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="siderdax",
    maintainer_email="headwaving@gmail.com",
    description="ROS2 Universal Robot RTDE Receiver",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["receiver = ur_rtde_receiver.rtde_receiver:main"],
    },
)
