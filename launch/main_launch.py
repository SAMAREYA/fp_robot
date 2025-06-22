from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fp_robot', 
            executable='udp_node',
            name='udp_node_instance',

        ),
        Node(
            package='fp_robot',
            executable='direction_pub',
            name='direction_publisher_instance',

        ),
        Node(
            package='fp_robot',
            executable='maze_node',
            name='maze_solver_instance',

        )
    ])