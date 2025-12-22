#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String
import math

class EnvironmentPublisher(Node):
    def __init__(self):
        super().__init__('environment_publisher')
        self.marker_pub = self.create_publisher(MarkerArray, 'environment_markers', 10)
        
        # Subscriber for box state (from demo movement)
        self.state_sub = self.create_subscription(
            String,
            '/box_state',
            self.state_callback,
            10
        )
        
        # Box state: 'source', 'gripped', 'target'
        self.box_state = 'source'
        
        self.timer = self.create_timer(0.1, self.publish_markers)
    
    def state_callback(self, msg):
        """Handle box state changes from demo movement"""
        new_state = msg.data
        if new_state != self.box_state:
            self.box_state = new_state
            self.get_logger().info(f'Box state: {self.box_state}')
        
    def publish_markers(self):
        marker_array = MarkerArray()
        
        # Source stage (blue platform on left)
        source_stage = Marker()
        source_stage.header.frame_id = "base_link"
        source_stage.header.stamp = self.get_clock().now().to_msg()
        source_stage.ns = "environment"
        source_stage.id = 0
        source_stage.type = Marker.CUBE
        source_stage.action = Marker.ADD
        source_stage.pose.position.x = 0.25
        source_stage.pose.position.y = 0.18
        source_stage.pose.position.z = 0.01
        source_stage.pose.orientation.w = 1.0
        source_stage.scale.x = 0.3
        source_stage.scale.y = 0.3
        source_stage.scale.z = 0.02
        source_stage.color.r = 0.3
        source_stage.color.g = 0.3
        source_stage.color.b = 0.8
        source_stage.color.a = 0.8
        marker_array.markers.append(source_stage)
        
        # Target stage (blue platform on right, rotated)
        target_stage = Marker()
        target_stage.header.frame_id = "base_link"
        target_stage.header.stamp = self.get_clock().now().to_msg()
        target_stage.ns = "environment"
        target_stage.id = 1
        target_stage.type = Marker.CUBE
        target_stage.action = Marker.ADD
        target_stage.pose.position.x = 0.25
        target_stage.pose.position.y = -0.18
        target_stage.pose.position.z = 0.01
        # Rotation 90 degrees around Z
        target_stage.pose.orientation.z = math.sin(1.5708/2)
        target_stage.pose.orientation.w = math.cos(1.5708/2)
        target_stage.scale.x = 0.3
        target_stage.scale.y = 0.3
        target_stage.scale.z = 0.02
        target_stage.color.r = 0.3
        target_stage.color.g = 0.3
        target_stage.color.b = 0.8
        target_stage.color.a = 0.8
        marker_array.markers.append(target_stage)
        
        # Pickup box (red cube) - position changes based on state
        pickup_box = Marker()
        pickup_box.ns = "environment"
        pickup_box.id = 2
        pickup_box.type = Marker.CUBE
        pickup_box.action = Marker.ADD
        
        if self.box_state == 'source':
            # Box on source stage
            pickup_box.header.frame_id = "base_link"
            pickup_box.header.stamp = self.get_clock().now().to_msg()
            pickup_box.pose.position.x = 0.25
            pickup_box.pose.position.y = 0.18
            pickup_box.pose.position.z = 0.045
        elif self.box_state == 'gripped':
            # Box attached to gripper (end effector link)
            pickup_box.header.frame_id = "Empty_Link6"
            pickup_box.header.stamp = self.get_clock().now().to_msg()
            pickup_box.pose.position.x = 0.12
            pickup_box.pose.position.y = 0.0
            pickup_box.pose.position.z = 0.0
        elif self.box_state == 'target':
            # Box on target stage
            pickup_box.header.frame_id = "base_link"
            pickup_box.header.stamp = self.get_clock().now().to_msg()
            pickup_box.pose.position.x = 0.25
            pickup_box.pose.position.y = -0.18
            pickup_box.pose.position.z = 0.045
        
        pickup_box.pose.orientation.w = 1.0
        pickup_box.scale.x = 0.05
        pickup_box.scale.y = 0.05
        pickup_box.scale.z = 0.05
        pickup_box.color.r = 0.8
        pickup_box.color.g = 0.2
        pickup_box.color.b = 0.2
        pickup_box.color.a = 1.0
        marker_array.markers.append(pickup_box)
        
        # Ground plane
        ground = Marker()
        ground.header.frame_id = "base_link"
        ground.header.stamp = self.get_clock().now().to_msg()
        ground.ns = "environment"
        ground.id = 3
        ground.type = Marker.CUBE
        ground.action = Marker.ADD
        ground.pose.position.x = 0.0
        ground.pose.position.y = 0.0
        ground.pose.position.z = -0.01
        ground.pose.orientation.w = 1.0
        ground.scale.x = 2.0
        ground.scale.y = 2.0
        ground.scale.z = 0.01
        ground.color.r = 0.5
        ground.color.g = 0.5
        ground.color.b = 0.5
        ground.color.a = 0.3
        marker_array.markers.append(ground)
        
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
