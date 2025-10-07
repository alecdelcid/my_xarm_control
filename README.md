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

### 4. vacuum_gripper_demo.py
- **NEW**: Vacuum gripper control for MoveIt simulation only
- Demonstrates complete pick and place operations using vacuum gripper
- Includes up/down movements with vacuum activation/deactivation
- Supports both full demo and simple vacuum test modes

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
python3 install/my_xarm_control/bin/basice_movement

# Vacuum gripper demo (requires vacuum gripper simulation)
ros2 run my_xarm_control vacuum_gripper_demo
python3 install/my_xarm_control/bin/vacuum_gripper_demo

# Simple vacuum test only
ros2 run my_xarm_control vacuum_gripper_demo --simple-test
```

### Use launch file:
```bash
# Basic demo
ros2 launch my_xarm_control my_xarm_demo.launch.py demo_type:=basic


# Vacuum gripper demo (launches simulation and demo automatically)
ros2 launch my_xarm_control vacuum_gripper_demo.launch.py

# Vacuum gripper simple test only
ros2 launch my_xarm_control vacuum_gripper_demo.launch.py demo_type:=simple

```

## Prerequisites

Make sure you have the xArm simulation running:

**For basic scripts (basic_movement):**
```bash
ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py add_gripper:=true
```

**For vacuum gripper demo:**
```bash
ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py add_vacuum_gripper:=true

```

**Note:** The vacuum gripper demo script is designed specifically for MoveIt simulation and requires the vacuum gripper to be enabled in the simulation.

## Customization

You can modify these scripts or create new ones based on your specific needs. The examples cover the most common control patterns for the xArm robot.