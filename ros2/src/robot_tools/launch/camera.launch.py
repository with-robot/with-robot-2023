from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # package_name = "img_receive"

    # robot_state_publisher
    # pkg_path = os.path.join(get_package_share_directory(package_name))
    # xacro_file = os.path.join(pkg_path, 'urdf', 'car.xacro')
    # robot_description = xacro.process_file(xacro_file)
    # params = {'robot_description': robot_description.toxml(), 'use_sim_time': False}

    # camera image
    camera_sensor_pub = Node(
        package="robot_tools",
        executable="camera",
        output="screen",
        parameters=[],
    )

    return LaunchDescription(
        [
            camera_sensor_pub,
        ]
    )
