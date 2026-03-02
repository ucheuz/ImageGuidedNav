import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import math

class RobotKinematicsNode(Node):
    def __init__(self):
        super().__init__('robot_kinematics_node')
        
        # 1. Subscriber: Listens to the OpenIGTLink Bridge
        # Note: 'IGTL_Transform_Topic' should match the exact output of your ROS2_IGTLink_Bridge
        self.subscription = self.create_subscription(
            TransformStamped,
            '/IGTL_Transform_Topic', 
            self.trajectory_callback,
            10)
            
        # 2. Publisher: Sends commands to the robot's joints for simulation in RViz
        self.joint_publisher = self.create_publisher(
            JointState, 
            '/joint_states', 
            10)
            
        self.get_logger().info("Kinematics Node Initialized. Waiting for trajectory from 3D Slicer...")

    def trajectory_callback(self, msg):
        """
        Triggered when a trajectory arrives from Slicer over TCP/IP.
        """
        self.get_logger().info("Trajectory received! Calculating kinematics...")
        
        # Extract the Cartesian target (in the ROS coordinate frame)
        target_x = msg.transform.translation.x
        target_y = msg.transform.translation.y
        target_z = msg.transform.translation.z
        
        # Calculate Inverse Kinematics
        joint_angles = self.compute_inverse_kinematics(target_x, target_y, target_z)
        
        # Execute the movement
        self.publish_joint_angles(joint_angles)

    def compute_inverse_kinematics(self, x, y, z):
        """
        A simplified IK solver for demonstration. 
        Calculates joint angles to point the robot toward the target.
        """
        # Example calculation for a base pan/tilt mechanism
        base_pan = math.atan2(y, x)
        
        xy_distance = math.sqrt(x**2 + y**2)
        base_tilt = math.atan2(z, xy_distance)
        
        # Assuming a simple 3-joint simulated robot for the report
        # [Base Rotation, Shoulder Tilt, Tool Extension]
        return [base_pan, base_tilt, 0.0]

    def publish_joint_angles(self, angles):
        """
        Formats and sends the JointState message to update the robot simulation.
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # These names MUST match the joint names in your URDF file!
        msg.name = ['joint_base_pan', 'joint_shoulder_tilt', 'joint_tool_extension'] 
        msg.position = angles
        
        self.joint_publisher.publish(msg)
        self.get_logger().info(f"Published joint commands: {angles}")

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