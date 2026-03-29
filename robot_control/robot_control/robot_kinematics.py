import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ros2_igtl_bridge.msg import Transform 
import math

class RobotKinematicsNode(Node):
    def __init__(self):
        super().__init__('robot_kinematics_node')
        
        # 1. Subscriber: Listens to the OpenIGTLink Bridge 
        self.subscription = self.create_subscription(
            Transform, 
            '/IGTL_TRANSFORM_IN', 
            self.trajectory_callback,
            10)
            
        # 2. Publisher: Controls the URDF model in RViz via joint_states 
        self.joint_publisher = self.create_publisher(
            JointState, 
            '/joint_states', 
            10)
            
        # 3. NEW: State Storage. Keeps track of the last known angles (starts at zero)
        self.current_angles = [0.0, 0.0]
        
        # 4. NEW: Timer. Runs the timer_callback 10 times a second (10Hz) to keep RViz alive
        self.timer = self.create_timer(0.1, self.timer_callback)
            
        self.get_logger().info("Kinematics Node Initialized. Waiting for trajectory from 3D Slicer...")

    def trajectory_callback(self, msg):
        """
        Triggered when a trajectory arrives from Slicer over TCP/IP.
        """
        if msg.name != 'Trajectory_ROS_Trans':
            return
            
        self.get_logger().info(f"Trajectory '{msg.name}' received! Calculating kinematics...")
        
        # Scale Conversion: Convert Slicer millimeters to ROS meters 
        target_x = msg.transform.translation.x / 1000.0
        target_y = msg.transform.translation.y / 1000.0
        target_z = msg.transform.translation.z / 1000.0
        
        # Compute joint positions
        joint_angles = self.compute_inverse_kinematics(target_x, target_y, target_z)
        
        # Update the stored angles so the timer can continuously publish them
        if joint_angles:
            self.current_angles = joint_angles
            self.get_logger().info(f"Target angles updated: Pan={joint_angles[0]:.2f}, Tilt={joint_angles[1]:.2f}")

    def timer_callback(self):
        """
        NEW: This function runs continuously at 10Hz. 
        It prevents the "Transform Expiration" fade-out in RViz.
        """
        self.publish_joint_angles(self.current_angles)

    def compute_inverse_kinematics(self, x, y, z):
        """
        Calculates joint angles and verifies reachability constraints.
        """
        distance_to_target = math.sqrt(x**2 + y**2 + z**2)
        max_reach = 0.90  # Combined link lengths defined in the URDF 
        
        if distance_to_target > max_reach:
            self.get_logger().error(f"Position unachievable! Distance {distance_to_target:.2f}m exceeds max reach.")
            return None

        base_pan = math.atan2(y, x)
        xy_distance = math.sqrt(x**2 + y**2)
        base_tilt = math.atan2(z, xy_distance)
        
        return [base_pan, base_tilt]

    def publish_joint_angles(self, angles):
        """
        Formats and sends the JointState message to move the URDF model.
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # These names MUST match the joint names in your URDF file 
        msg.name = ['pan_joint', 'tilt_joint'] 
        msg.position = angles
        
        self.joint_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotKinematicsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()