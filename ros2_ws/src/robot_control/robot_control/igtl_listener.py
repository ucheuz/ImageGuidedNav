import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, JointConstraint
from shape_msgs.msg import SolidPrimitive
import pyigtl 
import numpy as np

class SlicerToMoveItListener(Node):
    """
    ROS 2 Action Client Node.
    Acts as a bridge between 3D Slicer (OpenIGTLink) and MoveIt 2.
    Translates incoming 4x4 transformation matrices into constrained kinematic goals.
    """
    def __init__(self):
        super().__init__('slicer_moveit_listener')
        
        # Initialise MoveIt Action Client for the arm/gripper
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        # Establish OpenIGTLink connection to Slicer
        self.client = pyigtl.OpenIGTLinkClient(host="127.0.0.1", port=18945)
        self.client.start()
        
        self.timer = self.create_timer(0.1, self.listen_for_slicer_data)
        self.get_logger().info("Listener Active. Bridge connected")
        # Execute safety protocol before accepting commands
        self.close_gripper()

        self.timer = self.create_timer(0.5, self.listen_for_slicer_data)
    
    def get_quaternion_from_matrix(self, matrix):
        """
        Converts a 4x4 rotation matrix into a ROS-compatible quaternion (x, y, z, w).
        MoveIt 2 strictly requires quaternions for OrientationConstraints.
        """
        m = matrix[:3, :3]
        t = np.trace(m)
        if t > 0.0:
            s = np.sqrt(t + 1.0) * 2.0
            w = 0.25 * s
            x = (m[2, 1] - m[1, 2]) / s
            y = (m[0, 2] - m[2, 0]) / s
            z = (m[1, 0] - m[0, 1]) / s
        elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
            s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            w = (m[2, 1] - m[1, 2]) / s
            x = 0.25 * s
            y = (m[0, 1] + m[1, 0]) / s
            z = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            w = (m[0, 2] - m[2, 0]) / s
            x = (m[0, 1] + m[1, 0]) / s
            y = 0.25 * s
            z = (m[1, 2] + m[2, 1]) / s
        else:
            s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            w = (m[1, 0] - m[0, 1]) / s
            x = (m[0, 2] + m[2, 0]) / s
            y = (m[1, 2] + m[2, 1]) / s
            z = 0.25 * s
        return float(x), float(y), float(z), float(w)

    def listen_for_slicer_data(self):
        """Polls the OpenIGTLink buffer for valid trajectory matrices."""
        messages = self.client.get_latest_messages()
        
        if not messages:
            return 
            
        # Debug: Print the device_names of the messages we just received
        names = [msg.device_name for msg in messages]
        self.get_logger().info(f"Received {len(messages)} message(s) from Slicer: {names}")
        
        for message in messages:
            # Match the transform name broadcasted by PathPlanner.py
            if message.device_name == "Trajectory_ROS_Trans":
                self.get_logger().info("Target Trajectory found! Extracting coordinates...")
                
                # Matrix extraction and scaling (mm to meters)
                matrix = message.matrix
                
                qx, qy, qz, qw = self.get_quaternion_from_matrix(matrix)
                
                self.send_goal(matrix[0][3]/1000.0, matrix[1][3]/1000.0, matrix[2][3]/1000.0, qx, qy, qz, qw)
                
    def close_gripper(self):
        """Pre-flight safety function to ensure tool is secured before motion."""
        self.get_logger().info("Executing Pre-Flight Check: Closing gripper for safe insertion...")

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "gripper" 
        goal_msg.request.num_planning_attempts = 1
        goal_msg.request.allowed_planning_time = 2.0

        c = Constraints()

        # Finger 1
        jc1 = JointConstraint()
        jc1.joint_name = "Revolute 8"
        jc1.position = 0.4  
        jc1.tolerance_above = 0.05
        jc1.tolerance_below = 0.05
        jc1.weight = 1.0
        c.joint_constraints.append(jc1)

        # Finger 2
        jc2 = JointConstraint()
        jc2.joint_name = "Revolute 9"
        jc2.position = 0.4  
        jc2.tolerance_above = 0.05
        jc2.tolerance_below = 0.05
        jc2.weight = 1.0
        c.joint_constraints.append(jc2)

        goal_msg.request.goal_constraints.append(c)

        self._action_client.wait_for_server()
        self.get_logger().info("Gripper Action Server found. Actuating joints...")
        

        self._action_client.send_goal_async(goal_msg)

    def send_goal(self, x, y, z, qx, qy, qz, qw):
        """Packages the Cartesian coordinates into a MoveIt action request."""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.x = qx
        target_pose.orientation.y = qy
        target_pose.orientation.z = qz
        target_pose.orientation.w = qw
        
        # Position Tolerance: Bound the end-effector within a 5mm cubic volume
        pc = PositionConstraint()
        pc.header.frame_id = "world"
        pc.link_name = "Component7_1"
        s = SolidPrimitive()
        s.type = SolidPrimitive.BOX
        s.dimensions = [0.005, 0.005, 0.005] # 5mm tolerance
        pc.constraint_region.primitives.append(s)
        pc.constraint_region.primitive_poses.append(target_pose)
        pc.weight = 1.0

        # Orientation Tolerance: Restrict angular deviation to 0.1 radians
        oc = OrientationConstraint()
        oc.header.frame_id = "world"
        oc.link_name = "Component7_1"
        oc.orientation = target_pose.orientation
        oc.absolute_x_axis_tolerance = 0.1 # Radians
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0

        c = Constraints()
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        goal_msg.request.goal_constraints.append(c)

        self.get_logger().info(f"Moving robot to: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Evaluates immediate rejection by the Inverse Kinematics solver."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Target unachievable. Position rejected by MoveIt instantly.")
            error_msg = pyigtl.StringMessage(
                "ERROR: Target Position Unachievable. Position rejected by IK solver.", 
                device_name="ROS2_Feedback"
            )
            self.client.send_message(error_msg)
            return
            
        self.get_logger().info("Target accepted by MoveIt. Planning and executing...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Evaluates the outcome of the physical execution phase."""
        status = future.result().status
        
        # STATUS_SUCCEEDED is 4 in the actionlib GoalStatus enumerator
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal Reached Successfully.")

            # Bidirectional Feedback: Transmit success back to the Slicer UI
            success_msg = pyigtl.StringMessage(
                "SUCCESS: Robot has achieved the approach pose.", 
                device_name="ROS2_Feedback"
            )
            self.client.send_message(success_msg)
        else:
            self.get_logger().error(f"Execution failed! MoveIt Status Code: {status}")
            error_msg = pyigtl.StringMessage(
                "ERROR: Robot failed to reach the target. Coordinate is out of bounds or violates floor limits.", 
                device_name="ROS2_Feedback"
            )
            self.client.send_message(error_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SlicerToMoveItListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.client.stop()
        rclpy.shutdown()
