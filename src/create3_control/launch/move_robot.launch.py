import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('create3_control'),
        'config',
        'target_position.yaml'
        )
        
    node=Node(
        package = 'create3_control',
        name = 'move_robot',
        executable = 'move_robot',
        output='screen',
        parameters = [config]
    )
    ld.add_action(node)
    return ld