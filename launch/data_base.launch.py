import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    record_node = Node(
        package='ur3_traj',
        executable='record_F',
        name='record_node',
        output='screen'
    )

    move_action_node = Node(
        package='ur3_traj',
        executable='ur3_moveit_action',
        name='ur3_moveit_action',
        output='screen'
    )

    return LaunchDescription([
        record_node,
        move_action_node
    ])