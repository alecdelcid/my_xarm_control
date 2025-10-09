#!/usr/bin/env python3
"""
Launch file for Digital Twin Controller demonstration
This launch file can work with simulation only, real robot only, or both
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='simulation',
        description='Mode to run: simulation, real, or both'
    )
    
    # Digital twin controller node - delayed to allow simulation to start
    digital_twin_node = TimerAction(
        period=3.0,  # Wait 3 seconds for simulation to start
        actions=[
            Node(
                package='my_xarm_control',
                executable='digital_twin_controller',
                name='digital_twin_controller',
                output='screen'
            )
        ]
    )
    
    # Conditional launch of simulation based on mode
    simulation_launch = TimerAction(
        period=0.1,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    'if [ "$(echo $MODE)" != "real" ]; then '
                    'ros2 launch xarm_moveit_config lite6_moveit_gazebo.launch.py add_gripper:=true; '
                    'fi'
                ],
                output='screen',
                env={'MODE': LaunchConfiguration('mode')}
            )
        ]
    )
    
    return LaunchDescription([
        mode_arg,
        
        # Launch simulation if not real-only mode
        simulation_launch,
        
        # Launch the digital twin controller after a delay
        digital_twin_node,
    ])