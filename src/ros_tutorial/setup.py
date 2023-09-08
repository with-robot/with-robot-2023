import os
from glob import glob
from setuptools import setup

package_name = 'ros_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlesim_circle = ros_tutorial.turtlesim_circle:main',
            'turtlesim_echo = ros_tutorial.turtlesim_echo:main',
            'turtlesim_abs_client = ros_tutorial.turtlesim_abs_client:main',
            'turtlesim_abs_server = ros_tutorial.turtlesim_abs_server:main',
            'turtlesim_rot_client = ros_tutorial.turtlesim_rot_client:main',
            'turtlesim_rot_server = ros_tutorial.turtlesim_rot_server:main'
        ],
    },
)
