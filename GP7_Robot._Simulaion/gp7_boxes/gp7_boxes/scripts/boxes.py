#!/usr/bin/env python3
"""
Box Publisher for GP7 Robot - Grab and Place Simulation

Publishes two RViz markers:
  - RED box: Big target box (static) - where green box gets placed ON TOP
  - GREEN box: Small box that moves (floor -> gripped -> placed)
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from builtin_interfaces.msg import Duration


class BoxPublisher(Node):
    def __init__(self):
        super().__init__('box_publisher')

        # Publishers for boxes
        self.pub_red = self.create_publisher(Marker, 'box_red', 10)
        self.pub_green = self.create_publisher(Marker, 'box_green', 10)
        
        # Subscriber for box state (from movement demo)
        self.state_sub = self.create_subscription(
            String,
            '/box_state',
            self.state_callback,
            10
        )
        
        # Box state: 'floor', 'gripped', 'placed'
        self.box_state = 'floor'
        
        # POSITIONS - All coordinates in base_link frame
        # Robot reaches forward in +X direction when joint_1 = 0
        # Robot rotates toward +Y when joint_1 > 0 (positive rotation)
        
        # GREEN box pickup location: In front of robot
        self.green_pickup = {'x': -0.45, 'y': 0.0, 'z': 0.04}
        
        # RED target box: To the side (positive Y)
        self.red_box = {'x': -0.45, 'y': -0.35, 'z': 0.10}
        
        # GREEN placement: On top of red box
        self.green_placed = {'x': -0.45, 'y': -0.35, 'z': 0.24}

        # Timer to continuously publish (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_boxes)
        
        self.get_logger().info('Box Publisher Started')
        self.get_logger().info(f'GREEN pickup: {self.green_pickup}')
        self.get_logger().info(f'RED box: {self.red_box}')

    def state_callback(self, msg):
        new_state = msg.data
        if new_state != self.box_state:
            self.box_state = new_state
            self.get_logger().info(f'Box state: {self.box_state}')

    def publish_boxes(self):
        # RED BOX - BIG TARGET (STATIC)
        red = Marker()
        red.header.frame_id = "base_link"
        red.header.stamp = self.get_clock().now().to_msg()
        red.ns = "red_box"
        red.id = 0
        red.type = Marker.CUBE
        red.action = Marker.ADD
        
        red.pose.position.x = self.red_box['x']
        red.pose.position.y = self.red_box['y']
        red.pose.position.z = self.red_box['z']
        red.pose.orientation.w = 1.0
        
        red.scale.x = 0.20
        red.scale.y = 0.20
        red.scale.z = 0.20
        
        red.color.r = 1.0
        red.color.g = 0.0
        red.color.b = 0.0
        red.color.a = 0.9
        
        red.lifetime = Duration(sec=0)
        self.pub_red.publish(red)

        # GREEN BOX - SMALL, MOVABLE
        green = Marker()
        green.header.stamp = self.get_clock().now().to_msg()
        green.ns = "green_box"
        green.id = 1
        green.type = Marker.CUBE
        green.action = Marker.ADD

        if self.box_state == 'floor':
            green.header.frame_id = "base_link"
            green.pose.position.x = self.green_pickup['x']
            green.pose.position.y = self.green_pickup['y']
            green.pose.position.z = self.green_pickup['z']
            
        elif self.box_state == 'gripped':
            green.header.frame_id = "link_8"
            green.pose.position.x = 0.0
            green.pose.position.y = 0.0
            green.pose.position.z = 0.05
            
        elif self.box_state == 'placed':
            green.header.frame_id = "base_link"
            green.pose.position.x = self.green_placed['x']
            green.pose.position.y = self.green_placed['y']
            green.pose.position.z = self.green_placed['z']

        green.pose.orientation.w = 1.0

        green.scale.x = 0.08
        green.scale.y = 0.08
        green.scale.z = 0.08

        green.color.r = 0.0
        green.color.g = 1.0
        green.color.b = 0.0
        green.color.a = 0.9

        green.lifetime = Duration(sec=0)
        self.pub_green.publish(green)


def main(args=None):
    rclpy.init(args=args)
    node = BoxPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
