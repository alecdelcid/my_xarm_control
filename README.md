# My xArm Control Package

This package contains custom Python scripts for controlling the xArm robot using ROS2.

## Scripts Included

### 1. basic_movement.py
- Uses ROS2 services directly to control the robot
- Demonstrates basic initialization, joint movements, and cartesian movements
- Good for understanding the low-level service interface

### 2. moveit_example.py
- Uses MoveIt Python API for motion planning
- Includes joint space planning, cartesian space planning, and path planning
- Supports gripper control if available
- More sophisticated motion planning capabilities

### 3. gripper_control.py
- Dedicated gripper control script
- Supports both programmatic and interactive control
- Can set precise gripper positions or use presets

## Usage

### Build the package:
```bash
cd ~/dev_ws
colcon build --packages-select my_xarm_control
source install/setup.bash
```

### Run individual scripts:
```bash
# Basic movement demo
ros2 run my_xarm_control basic_movement

# MoveIt demo
ros2 run my_xarm_control moveit_example

# Gripper control demo
ros2 run my_xarm_control gripper_control

# Interactive gripper control
ros2 run my_xarm_control gripper_control --interactive
```

### Use launch file:
```bash
# Basic demo
ros2 launch my_xarm_control my_xarm_demo.launch.py demo_type:=basic

# MoveIt demo
ros2 launch my_xarm_control my_xarm_demo.launch.py demo_type:=moveit

# Gripper demo
ros2 launch my_xarm_control my_xarm_demo.launch.py demo_type:=gripper
```

## Prerequisites

Make sure you have the xArm simulation running:
```bash
ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py add_gripper:=true
```

## Customization

You can modify these scripts or create new ones based on your specific needs. The examples cover the most common control patterns for the xArm robot.