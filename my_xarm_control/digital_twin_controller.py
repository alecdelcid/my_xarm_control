#!/usr/bin/env python3
"""
Digital Twin Controller - Basic movement between simulated and real xArm Lite 6
This script can control both simulation (MoveIt) and real robot (xArm API) and sync between them
"""

import rclpy
from rclpy.node import Node
from xarm_msgs.srv import SetInt16, MoveCartesian, MoveJoint
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
import time
import threading


class DigitalTwinController(Node):
    def __init__(self):
        super().__init__('digital_twin_controller')
        
        self.use_simulation = False
        self.use_real_robot = False
        self.current_joint_states = None
        self.joint_state_lock = threading.Lock()
        
        # Check what's available
        self.detect_available_interfaces()
        
        if not self.use_simulation and not self.use_real_robot:
            self.get_logger().error("No robot interface available (neither simulation nor real robot)")
            raise Exception("No robot interface available")
        
        # Subscribe to joint states for synchronization
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.get_logger().info(f"Digital Twin Controller initialized - Simulation: {self.use_simulation}, Real Robot: {self.use_real_robot}")
    
    def detect_available_interfaces(self):
        """Detect which interfaces are available"""
        # Check for simulation (lite6_traj_controller)
        try:
            from rclpy.action import ActionClient
            
            self.trajectory_client = ActionClient(
                self, 
                FollowJointTrajectory, 
                '/lite6_traj_controller/follow_joint_trajectory'
            )
            
            if self.trajectory_client.wait_for_server(timeout_sec=2.0):
                self.use_simulation = True
                self.get_logger().info("✓ Simulation interface detected (lite6_traj_controller)")
            else:
                self.trajectory_client.destroy()
                self.get_logger().info("✗ Simulation interface not available")
        except Exception as e:
            self.get_logger().info(f"✗ Simulation interface error: {e}")
        
        # Check for real robot (xArm API services)
        required_services = ['/xarm/motion_enable', '/xarm/set_mode', '/xarm/set_state']
        available_services = self.get_service_names_and_types()
        service_names = [name for name, _ in available_services]
        
        if all(service in service_names for service in required_services):
            self.use_real_robot = True
            self.setup_real_robot_clients()
            self.get_logger().info("✓ Real robot interface detected (xArm API)")
        else:
            self.get_logger().info("✗ Real robot interface not available")
    
    def setup_real_robot_clients(self):
        """Setup service clients for real robot"""
        self.motion_enable_client = self.create_client(SetInt16, '/xarm/motion_enable')
        self.set_mode_client = self.create_client(SetInt16, '/xarm/set_mode')
        self.set_state_client = self.create_client(SetInt16, '/xarm/set_state')
        self.set_position_client = self.create_client(MoveCartesian, '/xarm/set_position')
        self.set_servo_angle_client = self.create_client(MoveJoint, '/xarm/set_servo_angle')
        
        # Wait for services
        self.wait_for_real_robot_services()
    
    def wait_for_real_robot_services(self):
        """Wait for real robot services to be available"""
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
        
        self.get_logger().info('All real robot services are available!')
    
    def joint_state_callback(self, msg):
        """Store current joint states for synchronization"""
        with self.joint_state_lock:
            if len(msg.position) >= 6:  # Ensure we have at least 6 joints
                self.current_joint_states = list(msg.position[:6])
    
    def get_current_joint_states(self):
        """Get current joint states safely"""
        with self.joint_state_lock:
            return self.current_joint_states.copy() if self.current_joint_states else None
    
    def call_service(self, client, request):
        """Generic service call function"""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def initialize_real_robot(self):
        """Initialize real robot for movement"""
        if not self.use_real_robot:
            return True
            
        self.get_logger().info('Initializing real robot...')
        
        try:
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
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to initialize real robot: {e}')
            return False
    
    def move_simulation_joints(self, joint_angles, duration_sec=3.0):
        """Move simulation robot joints"""
        if not self.use_simulation:
            return False
            
        try:
            goal_msg = FollowJointTrajectory.Goal()
            
            # Set joint names
            goal_msg.trajectory.joint_names = [
                'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
            ]
            
            # Create trajectory point
            point = JointTrajectoryPoint()
            point.positions = [float(angle) for angle in joint_angles[:6]]
            point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
            
            goal_msg.trajectory.points = [point]
            
            self.get_logger().info(f'Moving simulation to joint angles: {[f"{pos:.2f}" for pos in point.positions]}')
            
            # Send goal and wait for result
            send_goal_future = self.trajectory_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future)
            
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Simulation goal rejected')
                return False
            
            # Wait for result
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=duration_sec + 2.0)
            
            if get_result_future.done():
                result = get_result_future.result()
                if result.result.error_code == 0:
                    self.get_logger().info("Simulation movement completed successfully")
                    return True
                else:
                    self.get_logger().error(f"Simulation movement failed with error code: {result.result.error_code}")
                    return False
            else:
                self.get_logger().error("Simulation movement timed out")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Simulation movement failed: {str(e)}")
            return False
    
    def move_real_robot_joints(self, joint_angles, speed=0.35, acc=10.0):
        """Move real robot joints"""
        if not self.use_real_robot:
            return False
            
        try:
            req = MoveJoint.Request()
            req.angles = [float(angle) for angle in joint_angles[:6]]
            req.speed = float(speed)
            req.acc = float(acc)
            req.mvtime = 0.0
            
            self.get_logger().info(f'Moving real robot to joint angles: {[f"{angle:.2f}" for angle in req.angles]}')
            result = self.call_service(self.set_servo_angle_client, req)
            self.get_logger().info(f'Real robot joint move result: {result.ret}')
            
            return result.ret == 0
        except Exception as e:
            self.get_logger().error(f"Real robot movement failed: {str(e)}")
            return False
    
    def move_real_robot_cartesian(self, x, y, z, roll, pitch, yaw, speed=50, acc=500):
        """Move real robot to cartesian position"""
        if not self.use_real_robot:
            return False
            
        try:
            req = MoveCartesian.Request()
            req.pose = [float(x), float(y), float(z), float(roll), float(pitch), float(yaw)]
            req.speed = float(speed)
            req.acc = float(acc)
            req.mvtime = 0.0
            
            self.get_logger().info(f'Moving real robot to cartesian position: {req.pose}')
            result = self.call_service(self.set_position_client, req)
            self.get_logger().info(f'Real robot cartesian move result: {result.ret}')
            
            return result.ret == 0
        except Exception as e:
            self.get_logger().error(f"Real robot cartesian movement failed: {str(e)}")
            return False
    
    def sync_real_to_simulation(self):
        """Sync real robot position to simulation"""
        if not (self.use_real_robot and self.use_simulation):
            self.get_logger().warn("Cannot sync - both interfaces not available")
            return False
        
        current_joints = self.get_current_joint_states()
        if current_joints is None:
            self.get_logger().warn("No joint states available for synchronization")
            return False
        
        self.get_logger().info("Syncing real robot position to simulation...")
        return self.move_simulation_joints(current_joints)
    
    def sync_simulation_to_real(self, joint_angles):
        """Sync simulation planned position to real robot"""
        if not (self.use_real_robot and self.use_simulation):
            self.get_logger().warn("Cannot sync - both interfaces not available")
            return False
        
        self.get_logger().info("Syncing simulation plan to real robot...")
        return self.move_real_robot_joints(joint_angles)
    
    def digital_twin_demo(self):
        """Demonstrate digital twin functionality"""
        self.get_logger().info("=== Starting Digital Twin Demo ===")
        
        # Initialize real robot if available
        if self.use_real_robot:
            if not self.initialize_real_robot():
                return False
        
        # Wait for initial joint states
        self.get_logger().info("Waiting for joint states...")
        for i in range(10):
            if self.get_current_joint_states() is not None:
                break
            time.sleep(0.5)
        
        # Demo movements
        demo_positions = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Home
            [0.5, -0.3, 0.2, 0.5, 0.0, 0.0],  # Position 1
            [0.0, -0.5, 0.4, 0.8, 0.0, 0.0],  # Position 2
            [-0.5, -0.3, 0.2, 0.5, 0.0, 0.0], # Position 3
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Return home
        ]
        
        for i, position in enumerate(demo_positions):
            self.get_logger().info(f'=== Demo Movement {i+1}/{len(demo_positions)} ===')
            
            # Move simulation first (planning)
            if self.use_simulation:
                self.get_logger().info("Moving simulation (digital twin)...")
                if not self.move_simulation_joints(position):
                    self.get_logger().error(f"Failed to move simulation to position {i+1}")
                    continue
                time.sleep(1)
            
            # Then move real robot (execution)
            if self.use_real_robot:
                self.get_logger().info("Moving real robot...")
                if not self.move_real_robot_joints(position):
                    self.get_logger().error(f"Failed to move real robot to position {i+1}")
                    continue
                time.sleep(2)
            
            # Show synchronization status
            current_joints = self.get_current_joint_states()
            if current_joints:
                self.get_logger().info(f"Current joint states: {[f'{j:.2f}' for j in current_joints]}")
        
        self.get_logger().info("=== Digital Twin Demo Completed! ===")
        return True
    
    def cleanup(self):
        """Cleanup resources"""
        if hasattr(self, 'trajectory_client'):
            try:
                self.trajectory_client.destroy()
            except:
                pass


def main(args=None):
    rclpy.init(args=args)
    
    controller = DigitalTwinController()
    
    try:
        controller.digital_twin_demo()
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