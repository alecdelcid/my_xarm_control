#!/usr/bin/env python3
"""
Vacuum gripper demo for xArm in MoveIt simulation
This script controls an xArm with vacuum gripper to demonstrate pick and place operations
"""

import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from std_msgs.msg import Bool
import time


class VacuumGripperDemo(Node):
    def __init__(self):
        super().__init__('vacuum_gripper_demo')
        
        # Create action client for joint trajectory
        self.trajectory_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/xarm6_traj_controller/follow_joint_trajectory'
        )
        
        # Create publisher for vacuum gripper control
        self.vacuum_publisher = self.create_publisher(
            Bool, 
            '/vacuum_gripper/suction_on', 
            10
        )
        
        # Wait for action server
        if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Trajectory action server not available after 10 seconds')
            raise Exception('Trajectory action server not available')
        
        self.get_logger().info("Vacuum gripper demo initialized successfully")
        
        # Define key positions
        self.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pickup_up_position = [0.0, -0.3, 0.2, 0.5, 0.0, 0.0]  # Above pickup location
        self.pickup_down_position = [0.0, -0.5, 0.4, 0.8, 0.0, 0.0]  # At pickup location
        self.dropoff_up_position = [0.8, -0.3, 0.2, 0.5, 0.0, 0.0]  # Above dropoff location
        self.dropoff_down_position = [0.8, -0.5, 0.4, 0.8, 0.0, 0.0]  # At dropoff location
    
    def move_to_joint_position(self, joint_angles, duration_sec=3.0):
        """Move robot to specified joint position"""
        goal_msg = FollowJointTrajectory.Goal()
        
        # Set joint names (must match your robot configuration)
        goal_msg.trajectory.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
        ]
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = [float(angle) for angle in joint_angles[:6]]
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info(f'Moving to joint position: {[f"{pos:.2f}" for pos in point.positions]}')
        
        # Send goal and wait for result
        send_goal_future = self.trajectory_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False
        
        # Wait for result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=duration_sec + 2.0)
        
        if get_result_future.done():
            result = get_result_future.result()
            if result.result.error_code == 0:
                self.get_logger().info("Movement completed successfully")
                return True
            else:
                self.get_logger().error(f"Movement failed with error code: {result.result.error_code}")
                return False
        else:
            self.get_logger().error("Movement timed out")
            return False
    
    def activate_vacuum(self, activate=True):
        """Activate or deactivate vacuum gripper"""
        msg = Bool()
        msg.data = activate
        self.vacuum_publisher.publish(msg)
        
        action = "Activating" if activate else "Deactivating"
        self.get_logger().info(f'{action} vacuum gripper')
        time.sleep(0.5)  # Give time for the command to process
    
    def pick_and_place_demo(self):
        """Execute a complete pick and place sequence"""
        self.get_logger().info("=== Starting Vacuum Gripper Pick and Place Demo ===")
        
        try:
            # Step 1: Move to home position
            self.get_logger().info("Step 1: Moving to home position")
            if not self.move_to_joint_position(self.home_position):
                return False
            time.sleep(1)
            
            # Step 2: Move above pickup location
            self.get_logger().info("Step 2: Moving above pickup location")
            if not self.move_to_joint_position(self.pickup_up_position):
                return False
            time.sleep(1)
            
            # Step 3: Move down to pickup location
            self.get_logger().info("Step 3: Moving down to pickup location")
            if not self.move_to_joint_position(self.pickup_down_position, duration_sec=2.0):
                return False
            time.sleep(1)
            
            # Step 4: Activate vacuum gripper
            self.get_logger().info("Step 4: Activating vacuum gripper")
            self.activate_vacuum(True)
            time.sleep(2)  # Wait for vacuum to engage
            
            # Step 5: Move up with object
            self.get_logger().info("Step 5: Moving up with object")
            if not self.move_to_joint_position(self.pickup_up_position):
                return False
            time.sleep(1)
            
            # Step 6: Move above dropoff location
            self.get_logger().info("Step 6: Moving above dropoff location")
            if not self.move_to_joint_position(self.dropoff_up_position):
                return False
            time.sleep(1)
            
            # Step 7: Move down to dropoff location
            self.get_logger().info("Step 7: Moving down to dropoff location")
            if not self.move_to_joint_position(self.dropoff_down_position, duration_sec=2.0):
                return False
            time.sleep(1)
            
            # Step 8: Deactivate vacuum gripper
            self.get_logger().info("Step 8: Deactivating vacuum gripper")
            self.activate_vacuum(False)
            time.sleep(2)  # Wait for object to release
            
            # Step 9: Move up after dropoff
            self.get_logger().info("Step 9: Moving up after dropoff")
            if not self.move_to_joint_position(self.dropoff_up_position):
                return False
            time.sleep(1)
            
            # Step 10: Return to home position
            self.get_logger().info("Step 10: Returning to home position")
            if not self.move_to_joint_position(self.home_position):
                return False
            
            self.get_logger().info("=== Pick and Place Demo Completed Successfully! ===")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Demo failed with exception: {str(e)}")
            return False
    
    def simple_vacuum_test(self):
        """Simple test of vacuum gripper functionality"""
        self.get_logger().info("=== Starting Simple Vacuum Test ===")
        
        try:
            # Move to a test position
            self.get_logger().info("Moving to test position")
            test_position = [0.0, -0.4, 0.3, 0.6, 0.0, 0.0]
            if not self.move_to_joint_position(test_position):
                return False
            
            # Test vacuum on/off cycle
            for i in range(3):
                self.get_logger().info(f"Vacuum test cycle {i+1}/3")
                
                # Activate vacuum
                self.activate_vacuum(True)
                time.sleep(2)
                
                # Deactivate vacuum
                self.activate_vacuum(False)
                time.sleep(2)
            
            # Return home
            self.get_logger().info("Returning to home position")
            if not self.move_to_joint_position(self.home_position):
                return False
            
            self.get_logger().info("=== Simple Vacuum Test Completed! ===")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Vacuum test failed: {str(e)}")
            return False
    
    def cleanup(self):
        """Cleanup resources"""
        # Make sure vacuum is off
        self.activate_vacuum(False)
        if hasattr(self, 'trajectory_client'):
            try:
                self.trajectory_client.destroy()
            except:
                pass


def main(args=None):
    rclpy.init(args=args)
    
    demo = VacuumGripperDemo()
    
    try:
        # Check command line arguments for demo type
        import sys
        if len(sys.argv) > 1 and sys.argv[1] == '--simple-test':
            demo.simple_vacuum_test()
        else:
            demo.pick_and_place_demo()
            
    except KeyboardInterrupt:
        demo.get_logger().info('Demo interrupted by user')
    except Exception as e:
        demo.get_logger().error(f'Demo failed: {str(e)}')
    finally:
        demo.cleanup()
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()