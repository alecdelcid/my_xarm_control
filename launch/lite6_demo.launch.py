#!/usr/bin/env python3
"""
Launch file for xArm Lite 6 control demos
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    demo_type = LaunchConfiguration('demo_type').perform(context)
    
    if demo_type == 'basic':
        demo_node = Node(
            package='my_xarm_control',
            executable='basic_movement',
            name='basic_movement_demo',
            output='screen'
        )
    elif demo_type == 'moveit':
        demo_node = Node(
            package='my_xarm_control',
            executable='moveit_example',
            name='moveit_demo',
            output='screen'
        )
    elif demo_type == 'gripper':
        demo_node = Node(
            package='my_xarm_control',
            executable='gripper_control',
            name='gripper_demo',
            output='screen'
        )
    elif demo_type == 'vacuum':
        demo_node = Node(
            package='my_xarm_control',
            executable='vacuum_gripper_demo',
            name='vacuum_gripper_demo',
            output='screen'
        )
    else:
        # Default to basic
        demo_node = Node(
            package='my_xarm_control',
            executable='basic_movement',
            name='basic_movement_demo',
            output='screen'
        )
    
    return [demo_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'demo_type',
            default_value='basic',
            description='Type of demo to run: basic, moveit, gripper, or vacuum'
        ),
        OpaqueFunction(function=launch_setup)
    ])