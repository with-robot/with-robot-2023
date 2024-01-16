from setuptools import setup
import os, glob

package_name = "robot_tools"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="michael kwon",
    maintainer_email="c4now@naver.com",
    description="로봇서비스 패키지",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "starter = robot_tools.robot_starter:main",
            "camera = robot_tools.camera_sensor_pub:main",
            "wheel = robot_tools.wheel_control_srv:main",
            "kbd = teleop_twist_keyboard.teleop_twist_keyboard",
        ],
    },
)
