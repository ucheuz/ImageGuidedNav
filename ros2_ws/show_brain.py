import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

class AnatomyPublisher(Node):
    def __init__(self):
        super().__init__('anatomy_publisher')
        # Changed to MarkerArray to handle multiple meshes
        self.publisher_ = self.create_publisher(MarkerArray, 'registered_anatomy_v2', 10)
        self.timer = self.create_timer(1.0, self.publish_meshes)
        self.get_logger().info("Publishing Fusion-Registered Brain and Target Hippocampus...")

    def publish_meshes(self):
        marker_array = MarkerArray()

        # ==========================================
        # 1. THE CORTEX (Glass Green)
        # ==========================================
        brain = Marker()
        brain.header.frame_id = "base_link" 
        brain.header.stamp = self.get_clock().now().to_msg()
        brain.ns = "anatomy"
        brain.id = 0
        brain.type = Marker.MESH_RESOURCE
        brain.action = Marker.ADD
        brain.mesh_resource = "file:///home/ros2box/ros2_ws/Registered_Brain.stl"
        
        brain.scale.x = 1.0
        brain.scale.y = 1.0
        brain.scale.z = 1.0
        
        # Ghostly Green so you can see inside
        brain.color.r = 0.0
        brain.color.g = 1.0
        brain.color.b = 0.5
        brain.color.a = 0.15 
        
        # YOUR FUSION COORDINATES
        brain.pose.position.x = 0.127
        brain.pose.position.y = -0.207
        brain.pose.position.z = 0.130
        brain.pose.orientation.w = 1.0

        # ==========================================
        # 2. THE HIPPOCAMPUS (Solid Crimson Red)
        # ==========================================
        hippo = Marker()
        hippo.header.frame_id = "base_link"
        hippo.header.stamp = self.get_clock().now().to_msg()
        hippo.ns = "anatomy"
        hippo.id = 1  # Must be different from the brain ID!
        hippo.type = Marker.SPHERE
        hippo.action = Marker.ADD
        hippo.mesh_resource = "file:///home/ros2box/ros2_ws/Segment_1.stl"
        
        hippo.scale.x = 0.02
        hippo.scale.y = 0.02
        hippo.scale.z = 0.02
        
        # High Visibility Red
        hippo.color.r = 1.0
        hippo.color.g = 0.0
        hippo.color.b = 0.15
        hippo.color.a = 0.95 
        
        # MUST MATCH THE BRAIN COORDINATES EXACTLY
        hippo.pose.position.x = 0.127
        hippo.pose.position.y = -0.207
        hippo.pose.position.z = 0.130
        hippo.pose.orientation.w = 1.0

        # Add both to the array and publish
        marker_array.markers.append(brain)
        marker_array.markers.append(hippo)
        self.publisher_.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = AnatomyPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
