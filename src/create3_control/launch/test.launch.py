import launch
from launch import LaunchDescription
from launch_ros.actions import Node  # Ensure launch_ros is imported here

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='create3_control',  # The name of your ROS 2 package
            executable='test',          # The name of the Python script (without .py extension)
            output='screen',            # Print the output to the screen
            parameters=[{
                # Add your parameters here if needed
            }]
        ),
    ])
