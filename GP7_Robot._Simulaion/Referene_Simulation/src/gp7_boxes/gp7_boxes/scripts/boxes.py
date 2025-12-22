#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration


class BoxPublisher(Node):
    def __init__(self):
        super().__init__('box_publisher')

        # Publishers for box A and box B
        self.pub_a = self.create_publisher(Marker, 'box_a', 10)
        self.pub_b = self.create_publisher(Marker, 'box_b', 10)

        # Timer to continuously publish
        self.timer = self.create_timer(0.1, self.publish_boxes)

    def publish_boxes(self):
        # ===============================
        # BOX A — BIG RED BOX (STATIC)
        # ===============================
        box_a = Marker()
        box_a.header.frame_id = "base_link"
        box_a.header.stamp = self.get_clock().now().to_msg()

        box_a.ns = "box_a"
        box_a.id = 0
        box_a.type = Marker.CUBE
        box_a.action = Marker.ADD

        # Position 0.30, 0.40, 0.10 meters
        box_a.pose.position.x = 0.30
        box_a.pose.position.y = 0.40
        box_a.pose.position.z = 0.10
        box_a.pose.orientation.w = 1.0

        # Size: 30cm x 30cm x 20cm
        box_a.scale.x = 0.30
        box_a.scale.y = 0.30
        box_a.scale.z = 0.20

        # Red color
        box_a.color.r = 1.0
        box_a.color.g = 0.0
        box_a.color.b = 0.0
        box_a.color.a = 0.8

        box_a.lifetime = Duration(sec=0)

        self.pub_a.publish(box_a)

        # ===============================
        # BOX B — SMALL BLUE BOX (TCP)
        # ===============================
        box_b = Marker()
        box_b.header.frame_id = "Link_6"      # <<-- YOUR TCP FRAME NAME
        box_b.header.stamp = self.get_clock().now().to_msg()

        box_b.ns = "box_b"
        box_b.id = 1
        box_b.type = Marker.CUBE
        box_b.action = Marker.ADD

        # ---- Position relative to Link_6 ----
        # RED AXIS = +X direction
        # so place the cube in front of the EE
        box_b.pose.position.x = 0.05   # ≈ 12 cm in front of Link_6
        box_b.pose.position.y = 0.0
        box_b.pose.position.z = 0.0
        box_b.pose.orientation.w = 1.0

        # Size 5cm x 5cm x 5cm
        box_b.scale.x = 0.05
        box_b.scale.y = 0.05
        box_b.scale.z = 0.05

        # Blue color
        box_b.color.r = 0.0
        box_b.color.g = 0.0
        box_b.color.b = 1.0
        box_b.color.a = 0.9

        box_b.lifetime = Duration(sec=0)

        self.pub_b.publish(box_b)


def main(args=None):
    rclpy.init(args=args)
    node = BoxPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
