from setuptools import find_packages, setup
import os
from glob import glob

package_name = "client_nodes"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Michael Kwon",
    maintainer_email="c4now@naver.com",
    description="user nodes using robot tools",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera = client_nodes.camera_image_sub:main",
            "wheel = client_nodes.wheel_control_client:main",
        ],
    },
)
