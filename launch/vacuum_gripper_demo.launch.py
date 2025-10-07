#!/usr/bin/env python3
"""
Launch file for vacuum gripper demonstration
Launches xarm6 with vacuum gripper in Gazebo MoveIt simulation and runs the demo
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    demo_type_arg = DeclareLaunchArgument(
        'demo_type',
        default_value='full',
        description='Type of demo to run: full (pick and place) or simple (vacuum test only)'
    )
    
    # Vacuum gripper demo node - delayed to allow simulation to start
    vacuum_demo_node = TimerAction(
        period=5.0,  # Wait 5 seconds for simulation to start
        actions=[
            Node(
                package='my_xarm_control',
                executable='vacuum_gripper_demo',
                name='vacuum_gripper_demo',
                output='screen',
                arguments=['--simple-test'] if LaunchConfiguration('demo_type') == 'simple' else []
            )
        ]
    )
    
    return LaunchDescription([
        demo_type_arg,
        
        # Launch xarm6 with vacuum gripper in Gazebo MoveIt
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'xarm_moveit_config', 'xarm6_moveit_gazebo.launch.py',
                'add_vacuum_gripper:=true'
            ],
            output='screen'
        ),
        
        # Launch the vacuum gripper demo after a delay
        vacuum_demo_node,
    ])