# My xArm Lite 6 Control Package

This package contains custom Python scripts for controlling the xArm Lite 6 robot using ROS2.

## Scripts Included

### 1. basic_movement.py
- Uses ROS2 services directly to control the robot
- Demonstrates basic initialization, joint movements, and cartesian movements
- Good for understanding the low-level service interface
- **Lite 6 ONLY**: Uses `lite6_traj_controller`

### 2. vacuum_gripper_demo.py
- Vacuum gripper control for MoveIt simulation only
- Demonstrates complete pick and place operations using vacuum gripper
- Includes up/down movements with vacuum activation/deactivation
- Supports both full demo and simple vacuum test modes
- **Lite 6 ONLY**: Uses `lite6_traj_controller`

### 3. digital_twin_controller.py
- **NEW**: Digital twin controller for both simulation and real robot
- Auto-detects available interfaces (simulation MoveIt and/or real xArm API)
- Can sync between simulation and real robot
- Demonstrates coordinated movement between digital twin and physical robot
- **Supports**: Both `lite6_traj_controller` (simulation) and xArm API (real robot)

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

# Vacuum gripper demo (requires vacuum gripper simulation)
ros2 run my_xarm_control vacuum_gripper_demo

# Digital twin controller (auto-detects simulation and/or real robot)
ros2 run my_xarm_control digital_twin_controller

# Simple vacuum test only
ros2 run my_xarm_control vacuum_gripper_demo --simple-test
```

### Use launch files:
```bash
# Basic demo
ros2 launch my_xarm_control lite6_demo.launch.py demo_type:=basic

# Vacuum gripper simple test only
ros2 launch my_xarm_control vacuum_gripper_demo.launch.py demo_type:=simple

# Dedicated Lite 6 vacuum gripper demo
ros2 launch my_xarm_control lite6_vacuum_gripper_demo.launch.py

# Digital twin demo (simulation only)
ros2 launch my_xarm_control digital_twin_demo.launch.py mode:=simulation

# Digital twin demo (real robot only - requires real robot connected)
ros2 launch my_xarm_control digital_twin_demo.launch.py mode:=real

# Digital twin demo (both simulation and real robot)
ros2 launch my_xarm_control digital_twin_demo.launch.py mode:=both
```

## Prerequisites

Make sure you have the xArm Lite 6 simulation running:

**For basic scripts:**
```bash
ros2 launch xarm_moveit_config lite6_moveit_gazebo.launch.py add_gripper:=true
```

**For vacuum gripper demo:**
```bash
ros2 launch xarm_moveit_config lite6_moveit_gazebo.launch.py add_vacuum_gripper:=true
```

**For digital twin demo:**
```bash
# Simulation only
ros2 launch xarm_moveit_config lite6_moveit_gazebo.launch.py add_gripper:=true

# Real robot only (requires real xArm Lite 6 connected)
ros2 launch xarm_api lite6_driver.launch.py robot_ip:=<your_robot_ip>

# Both simulation and real robot (run both commands above)
```

## xArm Lite 6 Specific Features

This package is designed exclusively for xArm Lite 6:

- **Controller**: Uses `lite6_traj_controller/follow_joint_trajectory`
- **Joint Names**: joint1, joint2, joint3, joint4, joint5, joint6
- **Launch Files**: Uses `lite6_moveit_gazebo.launch.py`
- **Gripper Support**: Supports both standard gripper and vacuum gripper

## Digital Twin Capabilities

The `digital_twin_controller.py` provides advanced functionality:

- **Auto-Detection**: Automatically detects simulation and/or real robot interfaces
- **Dual Control**: Can control simulation (MoveIt) and real robot (xArm API) simultaneously
- **Synchronization**: Sync positions between digital twin and physical robot
- **Joint State Monitoring**: Real-time joint state feedback from both systems
- **Safety**: Moves simulation first (planning), then real robot (execution)

### Digital Twin Usage Modes:
- **Simulation Only**: Use for planning and testing without real robot
- **Real Robot Only**: Direct control of physical robot
- **Both**: Full digital twin with simulation and real robot synchronization

### Quick Start:
```bash
# 1. Launch Lite 6 simulation
ros2 launch xarm_moveit_config lite6_moveit_gazebo.launch.py add_gripper:=true

# 2. Run basic movement demo
ros2 run my_xarm_control basic_movement

# 3. Or use launch file for complete automation
ros2 launch my_xarm_control lite6_demo.launch.py demo_type:=basic
```

## Customization

You can modify these scripts or create new ones based on your specific needs. The examples cover the most common control patterns for the xArm Lite 6 robot.