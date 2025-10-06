#!/usr/bin/env python3
"""
Basic movement example - works with both simulation and real robot
This script detects if xArm API services are available (real robot) or uses MoveIt (simulation)
"""

import rclpy
from rclpy.node import Node
from xarm_msgs.srv import SetInt16, MoveCartesian, MoveJoint
import time


class BasicXArmController(Node):
    def __init__(self):
        super().__init__('basic_xarm_controller')
        
        self.use_moveit = False
        self.move_group = None
        
        # First, check what services are available
        if self.check_xarm_services_available():
            self.get_logger().info("xArm API services detected - using direct service calls")
            self.setup_service_clients()
        else:
            self.get_logger().info("xArm API services not found - switching to MoveIt mode")
            self.use_moveit = True
            self.setup_moveit()
    
    def check_xarm_services_available(self):
        """Check if xArm API services are available"""
        required_services = ['/xarm/motion_enable', '/xarm/set_mode', '/xarm/set_state']
        available_services = self.get_service_names_and_types()
        service_names = [name for name, _ in available_services]
        
        for service in required_services:
            if service not in service_names:
                return False
        return True
    
    def setup_service_clients(self):
        """Setup service clients for direct xArm API"""
        self.motion_enable_client = self.create_client(SetInt16, '/xarm/motion_enable')
        self.set_mode_client = self.create_client(SetInt16, '/xarm/set_mode')
        self.set_state_client = self.create_client(SetInt16, '/xarm/set_state')
        self.set_position_client = self.create_client(MoveCartesian, '/xarm/set_position')
        self.set_servo_angle_client = self.create_client(MoveJoint, '/xarm/set_servo_angle')
        
        # Wait for services to be available
        self.wait_for_services()
    
    def setup_moveit(self):
        """Setup trajectory action client for simulation control"""
        try:
            from control_msgs.action import FollowJointTrajectory
            from rclpy.action import ActionClient
            
            # Create action client for joint trajectory
            self.trajectory_client = ActionClient(
                self, 
                FollowJointTrajectory, 
                '/xarm6_traj_controller/follow_joint_trajectory'
            )
            
            # Wait for action server
            if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Trajectory action server not available')
                raise Exception('Trajectory action server not available')
            
            self.get_logger().info("Trajectory action client setup complete")
            
        except Exception as e:
            self.get_logger().error(f"Failed to setup trajectory control: {str(e)}")
            raise
    
    def wait_for_services(self):
        """Wait for all required services to be available"""
        services = [
            (self.motion_enable_client, '/xarm/motion_enable'),
            (self.set_mode_client, '/xarm/set_mode'),
            (self.set_state_client, '/xarm/set_state'),
            (self.set_position_client, '/xarm/set_position'),
            (self.set_servo_angle_client, '/xarm/set_servo_angle')
        ]
        
        for client, service_name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for service {service_name}...')
        
        self.get_logger().info('All services are available!')
    
    def call_service(self, client, request):
        """Generic service call function"""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def initialize_robot(self):
        """Initialize robot for movement"""
        if self.use_moveit:
            self.get_logger().info('Robot ready (MoveIt mode)')
            return
            
        self.get_logger().info('Initializing robot...')
        
        # Enable motion
        req = SetInt16.Request()
        req.data = 1
        result = self.call_service(self.motion_enable_client, req)
        self.get_logger().info(f'Motion enable result: {result.ret}')
        
        # Set mode (0 = position mode)
        req = SetInt16.Request()
        req.data = 0
        result = self.call_service(self.set_mode_client, req)
        self.get_logger().info(f'Set mode result: {result.ret}')
        
        # Set state (0 = ready)
        req = SetInt16.Request()
        req.data = 0
        result = self.call_service(self.set_state_client, req)
        self.get_logger().info(f'Set state result: {result.ret}')
        
        time.sleep(1)  # Wait for robot to be ready
    
    def move_to_cartesian_position(self, x, y, z, roll, pitch, yaw, speed=50, acc=500):
        """Move robot to cartesian position"""
        if self.use_moveit:
            return self.moveit_move_to_pose(x, y, z, roll, pitch, yaw)
        else:
            return self.service_move_to_position(x, y, z, roll, pitch, yaw, speed, acc)
    
    def service_move_to_position(self, x, y, z, roll, pitch, yaw, speed, acc):
        """Move using service calls (real robot)"""
        req = MoveCartesian.Request()
        req.pose = [float(x), float(y), float(z), float(roll), float(pitch), float(yaw)]
        req.speed = float(speed)
        req.acc = float(acc)
        req.mvtime = 0.0
        
        self.get_logger().info(f'Moving to position: {req.pose}')
        result = self.call_service(self.set_position_client, req)
        self.get_logger().info(f'Move result: {result.ret}')
        
        return result.ret == 0
    
    def moveit_move_to_pose(self, x, y, z, roll, pitch, yaw):
        """Move using trajectory actions (simulation) - simplified version"""
        # For now, just move to a reasonable joint configuration
        # In a real application, you'd use inverse kinematics
        self.get_logger().info(f'Moving to approximate pose: x={x}, y={y}, z={z}')
        
        # Use some predefined joint configurations for different positions
        if abs(x - 300) < 50 and abs(y) < 50:  # Position 1
            target_joints = [0.0, -0.5, 0.0, 1.0, 0.0, 0.5]
        elif abs(y - 200) > 100:  # Position 2 
            target_joints = [0.5, -0.3, 0.0, 0.8, 0.0, 0.3]
        elif abs(x - 500) < 50:  # Position 3
            target_joints = [0.8, -0.2, 0.0, 0.6, 0.0, 0.2]
        else:  # Default position
            target_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        return self.moveit_move_joints(target_joints)
    
    def move_joints(self, joint_angles, speed=0.35, acc=10.0):
        """Move robot joints to specified angles (in radians)"""
        if self.use_moveit:
            return self.moveit_move_joints(joint_angles)
        else:
            return self.service_move_joints(joint_angles, speed, acc)
    
    def service_move_joints(self, joint_angles, speed, acc):
        """Move joints using service calls (real robot)"""
        req = MoveJoint.Request()
        req.angles = [float(angle) for angle in joint_angles]
        req.speed = float(speed)
        req.acc = float(acc)
        req.mvtime = 0.0
        
        self.get_logger().info(f'Moving joints to: {req.angles}')
        result = self.call_service(self.set_servo_angle_client, req)
        self.get_logger().info(f'Joint move result: {result.ret}')
        
        return result.ret == 0
    
    def moveit_move_joints(self, joint_angles):
        """Move joints using trajectory actions (simulation)"""
        try:
            from control_msgs.action import FollowJointTrajectory
            from trajectory_msgs.msg import JointTrajectoryPoint
            from builtin_interfaces.msg import Duration
            import rclpy
            
            goal_msg = FollowJointTrajectory.Goal()
            
            # Set joint names (must match your robot configuration)
            goal_msg.trajectory.joint_names = [
                'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
            ]
            
            # Create trajectory point
            point = JointTrajectoryPoint()
            point.positions = [float(angle) for angle in joint_angles[:6]]  # Ensure 6 joints
            point.time_from_start = Duration(sec=3, nanosec=0)  # 3 seconds to reach target
            
            goal_msg.trajectory.points = [point]
            
            self.get_logger().info(f'Sending joint trajectory: {point.positions}')
            
            # Send goal and wait for result
            send_goal_future = self.trajectory_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future)
            
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected')
                return False
            
            self.get_logger().info('Goal accepted, waiting for result...')
            
            # Wait for result
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=10.0)
            
            if get_result_future.done():
                result = get_result_future.result()
                if result.result.error_code == 0:
                    self.get_logger().info("Joint trajectory completed successfully")
                    return True
                else:
                    self.get_logger().error(f"Joint trajectory failed with error code: {result.result.error_code}")
                    return False
            else:
                self.get_logger().error("Joint trajectory timed out")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Joint trajectory failed: {str(e)}")
            return False
    
    def demo_sequence(self):
        """Run a demo sequence of movements"""
        mode_str = "MoveIt (simulation)" if self.use_moveit else "xArm API (real robot)"
        self.get_logger().info(f'Starting demo sequence using {mode_str}...')
        
        # Initialize robot
        self.initialize_robot()
        
        # Home position (joint movement)
        self.get_logger().info('=== Moving to home position ===')
        home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # All joints at 0
        if not self.move_joints(home_joints):
            self.get_logger().error('Failed to move to home position')
            return
        
        time.sleep(2)
        
        # Move to a cartesian position
        self.get_logger().info('=== Moving to cartesian position 1 ===')
        if not self.move_to_cartesian_position(300, 0, 300, 3.14, 0, 0):
            self.get_logger().error('Failed to move to position 1')
            return
        
        time.sleep(2)
        
        # Move to another position
        self.get_logger().info('=== Moving to cartesian position 2 ===')
        if not self.move_to_cartesian_position(300, 200, 300, 3.14, 0, 0):
            self.get_logger().error('Failed to move to position 2')
            return
        
        time.sleep(2)
        
        # Move to position 3
        self.get_logger().info('=== Moving to cartesian position 3 ===')
        if not self.move_to_cartesian_position(500, 200, 300, 3.14, 0, 0):
            self.get_logger().error('Failed to move to position 3')
            return
        
        time.sleep(2)
        
        # Return to home
        self.get_logger().info('=== Returning to home ===')
        self.move_joints(home_joints)
        
        self.get_logger().info('=== Demo sequence completed! ===')
    
    def cleanup(self):
        """Cleanup resources"""
        if self.use_moveit and hasattr(self, 'trajectory_client'):
            try:
                self.trajectory_client.destroy()
            except:
                pass


def main(args=None):
    rclpy.init(args=args)
    
    controller = BasicXArmController()
    
    try:
        controller.demo_sequence()
    except KeyboardInterrupt:
        controller.get_logger().info('Demo interrupted by user')
    except Exception as e:
        controller.get_logger().error(f'Demo failed: {str(e)}')
    finally:
        controller.cleanup()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()