from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robotic_sol',            
            executable='sensor_node',
            output='screen',
            emulate_tty=True,            
        ),
        Node(
            package='robotic_sol',            
            executable='sensor_mgr_node',
            output='screen',
            emulate_tty=True,            
        ),
        Node(
            package='robotic_sol',
            executable='sensor_client_node',            
            output='screen',
            emulate_tty=True,
        ),
    ])