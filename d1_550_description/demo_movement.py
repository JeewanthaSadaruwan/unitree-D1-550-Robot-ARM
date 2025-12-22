#!/usr/bin/env python3
"""
Automated Pick and Place Demo for D1-550 Robot

This script coordinates with the environment publisher to demonstrate
picking up a red box from the source stage and placing it on the target stage.
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
        
        # Joint names for D1-550 robot
        # Joints 1-6: revolute (arm joints)
        # Joint_L, Joint_R: prismatic (gripper fingers)
        self.joint_names = [
            'Joint1', 'Joint2', 'Joint3',
            'Joint4', 'Joint5', 'Joint6',
            'Joint_L', 'Joint_R'
        ]
        
        # IMPORTANT: Publish initial joint state immediately to render robot!
        self.publish_joint_state([0.0] * 8)
        
        # Movement sequence
        # Format: (name, [j1, j2, j3, j4, j5, j6, jL, jR], box_state, duration)
        # 
        # Robot structure based on URDF:
        # - Joint1: base rotation (around Z) - Positive = counterclockwise from top view
        # - Joint2: shoulder pitch (around Y) - Positive = forward/down
        # - Joint3: elbow pitch (around Y) - Positive = forward/down  
        # - Joint4: wrist roll (around X)
        # - Joint5: wrist pitch (around Y)
        # - Joint6: wrist roll (around X)
        # - Joint_L, Joint_R: gripper fingers (0=closed, 0.033=open)
        #
        # Link lengths from URDF:
        # - Base to Joint1: 0.0738m (Z)
        # - Joint1 to Joint2: 0.0578m (Z)
        # - Joint2 to Joint3: 0.27m (Z) - MAIN ARM
        # - Joint3 to Joint4: 0.05m (X)
        # - Joint4 to Joint5: 0.15468m (X) - FOREARM
        # - Joint5 to Joint6: 0.0777m (X)
        # - Joint6 to gripper: ~0.072m (X)
        # Total reach: ~0.57m horizontal when extended
        #
        # Environment:
        # - Source stage: (0.25, 0.18, 0.075) - right side from robot view  
        # - Target stage: (0.25, -0.18, 0.075) - left side from robot view
        # - Box height: 0.05m, center at z=0.045
        # - Robot base at origin (0, 0, 0)
        # - Robot height to gripper when J2=J3=0: ~0.4m
        
        self.sequence = [
            # HOME - Robot upright, gripper open, clearly visible
            ("HOME - Starting position",
             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.033, 0.033],
             "source", 3.0),
            
            # Rotate toward source stage (Y=+0.18, Joint1 axis is -Z, so NEGATIVE rotation)
            ("ROTATE - Turning toward source stage",
             [-0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.033, 0.033],
             "source", 2.0),
            
            # Reach forward and start lowering toward box, orient gripper downward
            ("REACH - Extending toward pickup box",
             [-0.6, 0.7, -0.6, 0.0, 0.8, 0.0, 0.033, 0.033],
             "source", 2.5),
            
            # Lower more to reach box, gripper pointing down with high positive Joint5
            ("LOWER - Positioning at box",
             [-0.6, 1.1, -0.9, 0.0, 1.2, 0.0, 0.033, 0.033],
             "source", 2.0),
            
            # Close gripper - box becomes gripped
            ("GRASP - Closing gripper on box",
             [-0.6, 1.1, -0.9, 0.0, 1.2, 0.0, 0.0, 0.0],
             "gripped", 1.5),
            
            # Lift box up, keep gripper oriented
            ("LIFT - Raising box from source stage",
             [-0.6, 0.6, -0.6, 0.0, 0.6, 0.0, 0.0, 0.0],
             "gripped", 2.0),
            
            # Rotate toward center, adjust wrist to keep box level
            ("ROTATE - Moving to center",
             [0.0, 0.5, -0.5, 0.0, 0.4, 0.0, 0.0, 0.0],
             "gripped", 2.5),
            
            # Rotate toward target stage (Y=-0.18, so POSITIVE Joint1)
            ("ROTATE - Turning toward target stage",
             [0.6, 0.5, -0.5, 0.0, 0.4, 0.0, 0.0, 0.0],
             "gripped", 2.0),
            
            # Extend and lower to target stage, gripper pointing down
            ("POSITION - Extending to target stage",
             [0.6, 1.1, -0.9, 0.0, 1.2, 0.0, 0.0, 0.0],
             "gripped", 2.5),
            
            # Release - open gripper, box stays on target stage
            ("RELEASE - Releasing box on target stage",
             [0.6, 1.1, -0.9, 0.0, 1.2, 0.0, 0.033, 0.033],
             "target", 1.5),
            
            # Retreat upward
            ("RETREAT - Moving away from target",
             [0.6, 0.6, -0.6, 0.0, 0.6, 0.0, 0.033, 0.033],
             "target", 2.0),
            
            # Return to home
            ("HOME - Returning to start position",
             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.033, 0.033],
             "target", 3.0),
        ]
        
        self.current_step = 0
        self.interpolation_t = 0.0
        
        # CRITICAL: Track where each step STARTS from (not current animated position)
        self.step_start_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.033, 0.033]
        
        self.get_logger().info('='*60)
        self.get_logger().info('  D1-550 Robot - Pick and Place Demo')
        self.get_logger().info('='*60)
        self.get_logger().info('  RED box: starts on source stage (blue)')
        self.get_logger().info('  Target: target stage (blue)')
        self.get_logger().info('='*60)
        
        # Publish initial box state
        self.publish_box_state("source")
        
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
            self.step_start_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.033, 0.033]
            self.publish_box_state("source")
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
    print("  D1-550 Robot - Pick and Place Demo")
    print("="*60)
    print("\n  Sequence:")
    print("    1. RED box appears on source stage (blue)")
    print("    2. Robot rotates and reaches toward box")
    print("    3. Robot picks up box from source stage")
    print("    4. Robot moves box to target stage")
    print("    5. Robot places box on target stage")
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
