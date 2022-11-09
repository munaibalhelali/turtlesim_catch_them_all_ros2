from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='main_node')

    spawn_turtle = Node(
        package='turtlesim_catch_them_all',
        executable='turtle_spawn_client',
    )

    control_turtle = Node(
        package='turtlesim_catch_them_all',
        executable='turtle_controller'
    )
    ld.add_action(turtlesim_node)
    ld.add_action(spawn_turtle)
    ld.add_action(control_turtle)
    
    return ld
