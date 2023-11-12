from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 로봇 드라이버 로드
    robot_camera = Node(
        package="robot_tools",
        executable="camera",
        output="screen",
    )

    # 로봇 카메라이미지 구독
    client_service = Node(
        package="client_nodes",
        executable="camera",
        output="screen",
    )

    return LaunchDescription([robot_camera, client_service])
