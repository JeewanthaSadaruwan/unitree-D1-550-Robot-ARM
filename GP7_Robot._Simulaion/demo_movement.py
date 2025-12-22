#!/usr/bin/env python3
"""
Automated Pick and Place Demo for GP7 Robot

This script coordinates with the box publisher to demonstrate
picking up a green box from the floor and placing it on a red box.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import time
import math


class PickPlaceDemo(Node):
    def __init__(self):
        super().__init__('pick_place_demo')
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Publisher for box state
        self.box_state_pub = self.create_publisher(String, '/box_state', 10)
        
        # Joint names for GP7 robot
        # Joints 1-6: revolute (arm joints)
        # Joints 7-8: prismatic (gripper fingers)
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6',
            'joint_7', 'joint_8'
        ]
        
        # IMPORTANT: Publish initial joint state immediately to render robot!
        self.publish_joint_state([0.0] * 8)
        
        # Movement sequence
        # Format: (name, [j1, j2, j3, j4, j5, j6, j7, j8], box_state, duration)
        # 
        # Robot structure based on URDF:
        # - joint_1: base rotation (around Z) - POSITIVE = toward red box at Y=+0.35
        # - joint_2: shoulder (pitch)
        # - joint_3: elbow (pitch) 
        # - joint_4: wrist rotation
        # - joint_5: wrist pitch
        # - joint_6: wrist roll
        # - joint_7, joint_8: gripper fingers (prismatic) - 0.04 for 8cm box
        
        self.sequence = [
            # HOME - Robot upright, show box on floor for 3 seconds
            ("HOME - Showing green box on floor",
             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
             "floor", 3.0),
            
            # Prepare - Slight arm movement to get ready
            ("PREPARE - Getting ready",
             [0.0, 0.3, -0.3, 0.0, 0.0, 0.0, 0.0, 0.0],
             "floor", 1.5),
            
            # Reach forward and down toward green box
            ("REACH - Moving toward green box",
             [0.0, 0.8, -0.6, 0.0, -0.5, 0.0, 0.0, 0.0],
             "floor", 2.0),
            
            # Lower more to reach the box on floor
            ("LOWER - Descending to green box",
             [0.0, 1.0, -0.4, 0.0, -0.8, 0.0, 0.0, 0.0],
             "floor", 2.0),
            
            # Close gripper - box becomes gripped
            # Gripper values: negative = close inward, positive = open outward
            # For 8cm box (0.08m), grippers close to touch at 4cm from center
            ("GRASP - Grabbing the green box",
             [0.0, 1.0, -0.4, 0.0, -0.8, 0.0, 0.04, -0.04],
             "gripped", 1.0),
            
            # Lift up with box
            ("LIFT - Raising box from floor",
             [0.0, 0.5, -0.5, 0.0, -0.3, 0.0, -0.04, -0.04],
             "gripped", 2.0),
            
            # Rotate toward red box (POSITIVE joint_1 = toward red box at Y=+0.35)
            ("ROTATE - Turning toward red box",
             [0.8, 0.5, -0.5, 0.0, -0.3, 0.0, 0.04, -0.04],
             "gripped", 2.0),
            
            # Position above red box
            ("POSITION - Above red box",
             [0.8, 0.6, -0.4, 0.0, -0.4, 0.0, 0.04, -0.04],
             "gripped", 2.0),
            
            # Lower to place on red box
            ("LOWER - Placing on red box",
             [0.8, 0.8, -0.3, 0.0, -0.6, 0.0, 0.04, -0.04],
             "gripped", 2.0),
            
            # Release - open gripper, box stays on red box
            ("RELEASE - Releasing green box",
             [0.8, 0.8, -0.3, 0.0, -0.6, 0.0, 0.0, 0.0],
             "placed", 1.0),
            
            # Retreat upward
            ("RETREAT - Moving away",
             [0.8, 0.4, -0.5, 0.0, -0.2, 0.0, 0.0, 0.0],
             "placed", 1.5),
            
            # Return to home
            ("HOME - Returning to start",
             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
             "placed", 2.0),
        ]
        
        self.current_step = 0
        self.interpolation_t = 0.0
        
        # CRITICAL: Track where each step STARTS from (not current animated position)
        self.step_start_positions = [0.0] * 8
        
        self.get_logger().info('='*60)
        self.get_logger().info('  GP7 Robot - Pick and Place Demo')
        self.get_logger().info('='*60)
        self.get_logger().info('  GREEN box: starts on floor')
        self.get_logger().info('  RED box: target location')
        self.get_logger().info('='*60)
        
        # Publish initial box state
        self.publish_box_state("floor")
        
        # High frequency timer for smooth animation
        self.timer = self.create_timer(0.02, self.update)
        
    def publish_joint_state(self, positions):
        """Publish joint states to move robot"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = positions
        self.joint_pub.publish(msg)
        
    def publish_box_state(self, state):
        """Publish box state to control box position"""
        msg = String()
        msg.data = state
        self.box_state_pub.publish(msg)
        
    def update(self):
        # Check if demo complete
        if self.current_step >= len(self.sequence):
            self.get_logger().info('='*60)
            self.get_logger().info('  Demo Complete! Restarting...')
            self.get_logger().info('='*60)
            time.sleep(2.0)
            self.current_step = 0
            self.interpolation_t = 0.0
            self.step_start_positions = [0.0] * 8
            self.publish_box_state("floor")
            return
        
        step_name, target_pos, box_state, duration = self.sequence[self.current_step]
        steps_for_move = max(1, int(duration / 0.02))
        
        # Calculate smooth interpolation factor
        t = min(1.0, self.interpolation_t / steps_for_move)
        
        # Smooth ease-in-out
        if t < 0.5:
            smooth_t = 4 * t * t * t
        else:
            smooth_t = 1 - pow(-2 * t + 2, 3) / 2
        
        # Interpolate from step START position to target (not current animated position!)
        interpolated = []
        for i in range(8):
            pos = self.step_start_positions[i] + (target_pos[i] - self.step_start_positions[i]) * smooth_t
            interpolated.append(pos)
        
        # Publish joint state
        self.publish_joint_state(interpolated)
        
        # Publish box state
        self.publish_box_state(box_state)
        
        self.interpolation_t += 1
        
        # Step complete - move to next step
        if self.interpolation_t >= steps_for_move:
            # Save target as start position for NEXT step
            self.step_start_positions = list(target_pos)
            self.interpolation_t = 0.0
            self.current_step += 1
            
            if self.current_step < len(self.sequence):
                self.get_logger().info(f'Step {self.current_step}: {self.sequence[self.current_step][0]}')


def main(args=None):
    rclpy.init(args=args)
    
    print("\n" + "="*60)
    print("  GP7 Robot - Pick and Place Demo")
    print("="*60)
    print("\n  Sequence:")
    print("    1. GREEN box appears on floor (3 sec)")
    print("    2. Robot reaches and picks up box")
    print("    3. Robot moves box to RED box")
    print("    4. Robot places GREEN box on RED box")
    print("\n  Press Ctrl+C to stop")
    print("="*60 + "\n")
    
    node = PickPlaceDemo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
