# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import TransformStamped
# from sensor_msgs.msg import JointState
# import math

# class RobotKinematicsNode(Node):
#     def __init__(self):
#         super().__init__('robot_kinematics_node')
        
#         # 1. UPDATED: Correct topic name based on your network wiretap
#         self.subscription = self.create_subscription(
#             TransformStamped, # Note: if this throws an import error later, change to 'Transform' from ros2_igtl_bridge.msg
#             '/IGTL_TRANSFORM_IN', 
#             self.trajectory_callback,
#             10)
            
#         self.joint_publisher = self.create_publisher(
#             JointState, 
#             '/joint_states', 
#             10)
            
#         self.get_logger().info("Kinematics Node Initialized. Waiting for trajectory from 3D Slicer...")

#     def trajectory_callback(self, msg):
#         """
#         Triggered when a trajectory arrives from Slicer over TCP/IP.
#         """
#         # We check for 'name' or 'child_frame_id' depending on the exact bridge message type
#         msg_name = getattr(msg, 'name', getattr(msg, 'child_frame_id', ''))
        
#         # 2. UPDATED: The 20-character limit filter
#         if msg_name != 'Trajectory_ROS_Trans':
#             return # Ignore background noise from Slicer
            
#         self.get_logger().info("Trajectory received! Calculating kinematics...")
        
#         # 3. UPDATED: Convert Slicer Millimeters to ROS Meters
#         target_x = msg.transform.translation.x / 1000.0
#         target_y = msg.transform.translation.y / 1000.0
#         target_z = msg.transform.translation.z / 1000.0
        
#         joint_angles = self.compute_inverse_kinematics(target_x, target_y, target_z)
#         self.publish_joint_angles(joint_angles)

#     def compute_inverse_kinematics(self, x, y, z):
#         """
#         Calculates joint angles to point the robot toward the target.
#         """
#         distance_to_target = math.sqrt(x**2 + y**2 + z**2)
#         max_reach = 0.90  
        
#         if distance_to_target > max_reach:
#             self.get_logger().error(f"Target out of reach! Distance: {distance_to_target:.2f}m. Max is {max_reach}m.")
#             return [0.0, 0.0] 

#         base_pan = math.atan2(y, x)
#         xy_distance = math.sqrt(x**2 + y**2)
#         base_tilt = math.atan2(z, xy_distance)
        
#         # UPDATED: Return only 2 joints, matching our 2-DOF URDF
#         return [base_pan, base_tilt]

#     def publish_joint_angles(self, angles):
#         """
#         Formats and sends the JointState message to update the robot simulation.
#         """
#         msg = JointState()
#         msg.header.stamp = self.get_clock().now().to_msg()
        
#         # 4. UPDATED: Exact joint names from your custom Ellipse URDF
#         msg.name = ['pan_joint', 'tilt_joint'] 
#         msg.position = angles
        
#         self.joint_publisher.publish(msg)
#         self.get_logger().info(f"Published joint commands: Pan={angles[0]:.2f}, Tilt={angles[1]:.2f}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = RobotKinematicsNode()
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ros2_igtl_bridge.msg import Transform 
import math

class RobotKinematicsNode(Node):
    def __init__(self):
        super().__init__('robot_kinematics_node')
        
        # 1. Subscriber: Listens to the OpenIGTLink Bridge 
        # Uses the specific 'Transform' type required by the bridge 
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
            
        self.get_logger().info("Kinematics Node Initialized. Waiting for trajectory from 3D Slicer...")

    def trajectory_callback(self, msg):
        """
        Triggered when a trajectory arrives from Slicer over TCP/IP.
        """
        # Filter for the specific truncated name from OpenIGTLink 
        if msg.name != 'Trajectory_ROS_Trans':
            return
            
        self.get_logger().info(f"Trajectory '{msg.name}' received! Calculating kinematics...")
        
        # 3. Scale Conversion: Convert Slicer millimeters to ROS meters 
        # This is critical for accurate coordinate frame alignment 
        target_x = msg.transform.translation.x / 1000.0
        target_y = msg.transform.translation.y / 1000.0
        target_z = msg.transform.translation.z / 1000.0
        
        # Compute joint positions based on the received pose
        joint_angles = self.compute_inverse_kinematics(target_x, target_y, target_z)
        
        # Publish to the simulation 
        if joint_angles:
            self.publish_joint_angles(joint_angles)

    def compute_inverse_kinematics(self, x, y, z):
        """
        Calculates joint angles and verifies reachability constraints.
        """
        # Safety Check: Verify if the point is achievable 
        distance_to_target = math.sqrt(x**2 + y**2 + z**2)
        max_reach = 0.90  # Combined link lengths defined in the URDF 
        
        if distance_to_target > max_reach:
            self.get_logger().error(f"Position unachievable! Distance {distance_to_target:.2f}m exceeds max reach.") [cite: 18]
            return None

        # Kinematic Equations for a 2-Joint (Pan/Tilt) Robot 
        base_pan = math.atan2(y, x)
        xy_distance = math.sqrt(x**2 + y**2)
        base_tilt = math.atan2(z, xy_distance)
        
        return [base_pan, base_tilt]

    def publish_joint_angles(self, angles):
        """
        Formats and sends the JointState message to move the URDF model[cite: 15, 16].
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # These names MUST match the joint names in your URDF file 
        msg.name = ['pan_joint', 'tilt_joint'] 
        msg.position = angles
        
        self.joint_publisher.publish(msg)
        self.get_logger().info(f"Published joint commands: Pan={angles[0]:.2f}, Tilt={angles[1]:.2f}")

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