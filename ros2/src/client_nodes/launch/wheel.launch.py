from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 로봇 바퀴구동 서비스
    robot_wheel = Node(
        package="robot_tools",
        executable="wheel",
        output="screen",
    )

    # 서비스
    client_wheel = Node(
        package="client_nodes",
        executable="wheel",
        output="screen",
    )

    return LaunchDescription([robot_wheel, client_wheel])
