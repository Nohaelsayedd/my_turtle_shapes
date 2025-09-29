from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_turtle_shapes',
            executable='shape_node',
            name='shape_node',
            output='screen'
        ),
        Node(
            package='my_turtle_shapes',
            executable='turtle_commander',
            name='turtle_commander',
            output='screen'
        ),
        # Optionally add turtlesim (uncomment if you want a full launch)
        # Node(
        #     package='turtlesim',
        #     executable='turtlesim_node',
        #     name='turtlesim',
        #     output='screen'
        # ),
    ])