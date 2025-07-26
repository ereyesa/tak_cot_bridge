from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('tak_cot_bridge'),
        'config',
        'tak_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='tak_cot_bridge',
            executable='cot_publisher_node',
            name='cot_publisher_node',
            parameters=[config_file],
            output='screen',
            # env={'PYTHONPATH': os.getenv('PYTHONPATH', '')}
        ),
    ])