import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    filename = os.path.join(get_package_share_directory('isaac_ros_dope'), 'resources','0002_rgb.jpg')
    return LaunchDescription([

        launch_ros.actions.Node(
            package='image_publisher', executable='image_publisher_node', output='screen',
            arguments=[filename],
            remappings=[('image_raw', '/image'),
                        ('camera_info', '/camera_info')]),
    ])
