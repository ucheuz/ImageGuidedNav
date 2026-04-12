#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import subprocess
from typing import Tuple

import ikpy.chain
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ros2_igtl_bridge.msg import Transform


def _quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Unit quaternion (x,y,z,w) to 3x3 rotation matrix (row-vector convention)."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-10:
        return np.eye(3, dtype=float)
    x, y, z, w = x / n, y / n, z / n, w / n
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ],
        dtype=float,
    )


class RobotKinematicsNode(Node):
    """Subscribes to OpenIGTLink transforms and publishes /joint_states for RViz."""

    # OpenIGTLink often truncates MRML node names; accept common variants.
    _TRAJECTORY_PREFIX = "Trajectory_ROS_Trans"

    def __init__(self):
        super().__init__("robot_kinematics_node")

        pkg_share = get_package_share_directory("robot_control")
        xacro_path = os.path.join(pkg_share, "urdf", "Robotic.xacro")

        try:
            urdf_xml = subprocess.check_output(["xacro", xacro_path]).decode("utf-8")
            urdf_xml = urdf_xml.replace('type="continuous"', 'type="revolute"')
            processed = "/tmp/robot_processed.urdf"
            with open(processed, "w", encoding="utf-8") as f:
                f.write(urdf_xml)

            self.my_chain = ikpy.chain.Chain.from_urdf_file(processed)
            self.get_logger().info(f"Chain loaded with {len(self.my_chain.links)} links.")
        except Exception as e:
            self.get_logger().error(f"URDF / xacro load failed: {e!s}")
            self.my_chain = None
            return

        self.subscription = self.create_subscription(
            Transform, "/IGTL_TRANSFORM_IN", self.trajectory_callback, 10
        )
        self.joint_publisher = self.create_publisher(JointState, "/joint_states", 10)

        self.joint_map = {
            "Revolute 2": 0.0,
            "Revolute 3": 0.0,
            "Revolute 4": 0.0,
            "Revolute 5": 0.0,
            "Revolute 6": 0.0,
            "Revolute 7": 0.0,
            "Revolute 9": 0.0,
        }

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(
            "Kinematics node ready. Expecting full pose in OpenIGTLink transform "
            "(same basis as PathPlanner Trajectory_ROS_Transform)."
        )

    def _trajectory_name_ok(self, name: str) -> bool:
        if not name:
            return False
        return name == self._TRAJECTORY_PREFIX or name.startswith(self._TRAJECTORY_PREFIX)

    def _igtl_to_ros_frame(self, msg: Transform) -> Tuple[np.ndarray, np.ndarray]:
        """
        Map Slicer/OpenIGT translation + quaternion into the ROS basis used in PathPlanner
        (swap X/Y with a sign flip on the incoming X axis).
        """
        t = msg.transform.translation
        pos_mm = np.array([t.x, t.y, t.z], dtype=float)
        p_ros = np.array([pos_mm[1], -pos_mm[0], pos_mm[2]], dtype=float) / 1000.0

        q = msg.transform.rotation
        r_msg = _quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
        basis_change = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]], dtype=float)
        r_ros = basis_change @ r_msg @ basis_change.T
        return p_ros, r_ros

    def trajectory_callback(self, msg: Transform):
        if self.my_chain is None:
            return
        if not self._trajectory_name_ok(msg.name):
            return

        pos_ros, rot_ros = self._igtl_to_ros_frame(msg)
        self.get_logger().info(
            f"IK target position (m): X={pos_ros[0]:.3f}, Y={pos_ros[1]:.3f}, Z={pos_ros[2]:.3f}"
        )

        target = np.eye(4, dtype=float)
        target[:3, :3] = rot_ros
        target[:3, 3] = pos_ros

        try:
            ik_results = self.my_chain.inverse_kinematics_frame(target)
        except Exception as e:
            self.get_logger().error(f"Full-pose IK failed: {e!s}; trying translation-only.")
            try:
                ik_results = self.my_chain.inverse_kinematics(pos_ros)
            except TypeError:
                ik_results = self.my_chain.inverse_kinematics(target_position=pos_ros)
            except Exception as e2:
                self.get_logger().error(f"Robot cannot achieve pose: {e2!s}")
                return

        for i, link in enumerate(self.my_chain.links):
            if link.name in self.joint_map:
                self.joint_map[link.name] = float(ik_results[i])

        self.get_logger().info("IK solution applied to joint map.")

    def timer_callback(self):
        if self.my_chain is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_map.keys())
        msg.position = list(self.joint_map.values())
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


if __name__ == "__main__":
    main()
