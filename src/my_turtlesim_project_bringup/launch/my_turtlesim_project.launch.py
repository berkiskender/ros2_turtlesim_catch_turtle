from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
        
    turtle_spawn_node = Node(
        package='my_turtlesim_project',
        executable='turtle_spawn_node',
        name='turtle_spawn_node',
        parameters=[
            {'turtle_spawn_period': 1.0},
            {'alive_turtle_publish_period': 0.1}
        ]
    )
        
    turtle_controller_node = Node(
        package='my_turtlesim_project',
        executable='turtle_controller_node',
        name='turtle_controller_node',
        parameters=[
            {'master_cmdvel_period': 0.1},
            {'catch_dist_threshold': 0.5}
        ]
    )
    
    ld.add_action(turtle_spawn_node)
    ld.add_action(turtle_controller_node)
    
    return ld
