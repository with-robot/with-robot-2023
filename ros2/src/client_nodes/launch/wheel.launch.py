from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 로봇 드라이버 로드
    robot_tools = Node(
        package="robot_tools",
        executable="camera",
        output="screen",
    )
    # 로봇 카메라이미지 구독
    camera_service = Node(
        package="robot_tools",
        executable="wheel",
        output="screen",
    )

    # 서비스
    camera_client = Node(
        package="client_nodes",
        executable="wheel",
        output="screen",
    )

    return LaunchDescription([camera_service, camera_client])
