# path_planner_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'trajectory',
            default_value='circle',
            description='Choose the desired trajectory: circle, square, infinity, star'
        ),

        # Start Turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),

        # Start the path planner node
        Node(
            package='turtlesim_path_planner',
            executable='path_planner',
            name='path_planner',
            output='screen',
            parameters=[{'trajectory': LaunchConfiguration('trajectory')}]
        )
    ])

