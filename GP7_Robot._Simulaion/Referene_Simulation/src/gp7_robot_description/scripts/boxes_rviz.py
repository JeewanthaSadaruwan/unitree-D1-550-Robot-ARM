#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration

class BoxPublisher(Node):
    def __init__(self):
        super().__init__('box_publisher')
        self.pub = self.create_publisher(Marker, 'boxes', 10)
        timer_period = 0.2  
        self.timer = self.create_timer(timer_period, self.publish_boxes)

    def publish_boxes(self):
        # Box A
        box_a = Marker()
        box_a.header.frame_id = "base_link"
        box_a.ns = "task"
        box_a.id = 1
        box_a.type = Marker.CUBE
        box_a.action = Marker.ADD
        box_a.scale.x = 0.30
        box_a.scale.y = 0.30
        box_a.scale.z = 0.30
        box_a.pose.position.x = 0.30
        box_a.pose.position.y = 0.40
        box_a.pose.position.z = 0.15
        box_a.color.r = 0.0
        box_a.color.g = 0.0
        box_a.color.b = 1.0
        box_a.color.a = 0.7

        box_a.lifetime = Duration(sec=0)

        # Box B
        box_b = Marker()
        box_b.header.frame_id = "base_link"
        box_b.ns = "task"
        box_b.id = 2
        box_b.type = Marker.CUBE
        box_b.action = Marker.ADD
        box_b.scale.x = 0.10
        box_b.scale.y = 0.10
        box_b.scale.z = 0.10
        box_b.pose.position.x = 0.30
        box_b.pose.position.y = 0.40
        box_b.pose.position.z = 0.20
        box_b.color.r = 1.0
        box_b.color.g = 0.0
        box_b.color.b = 0.0
        box_b.color.a = 0.7

        box_b.lifetime = Duration(sec=0)

        # publish both
        self.pub.publish(box_a)
        self.pub.publish(box_b)


def main(args=None):
    rclpy.init(args=args)
    node = BoxPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
