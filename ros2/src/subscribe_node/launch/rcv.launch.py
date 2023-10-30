import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# import xacro


def generate_launch_description():
    # package_name = "img_receive"

    # robot_state_publisher
    # pkg_path = os.path.join(get_package_share_directory(package_name))
    # xacro_file = os.path.join(pkg_path, 'urdf', 'car.xacro')
    # robot_description = xacro.process_file(xacro_file)
    # params = {'robot_description': robot_description.toxml(), 'use_sim_time': False}

    # fake driver
    # driver = Node(
    #     package="tcp_agent",
    #     executable="car_driver",
    #     output="screen",
    #     parameters=[],
    # )

    agent = Node(
        package="tcp_agent",
        executable="tcp_agent",
        output="screen",
        parameters=[],
    )

    # rviz2
    img_receiver = Node(
        package="subscribe_node",
        executable="capture",
        output="screen",
        # arguments=['-d', 'src/car_tutorial/config/car.rviz'],
    )

    return LaunchDescription(
        [
            # driver,
            agent,
            img_receiver,
        ]
    )
