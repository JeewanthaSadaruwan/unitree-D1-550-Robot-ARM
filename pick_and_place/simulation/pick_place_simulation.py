#!/usr/bin/env python3
"""
Pick and Place Simulation for D1-550 Robot (RViz-friendly, no blinking)

What this version fixes compared to your current script:
1) No blocking time.sleep() inside the motion loop (ROS2 timers are used instead).
2) Continuous, fixed-rate publishing of /joint_states (stable visualization in RViz).
3) Quintic (5th-order) time-scaling per segment (smooth position/velocity/acceleration).
4) “Hold” is implemented as an explicit segment while still publishing continuously.

Run:
  # start your robot_state_publisher / rviz / simulation as you already do
  ros2 run <your_pkg> pick_place_simulation.py
"""

import math
import os
from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import ikpy.chain


@dataclass
class Segment:
    description: str
    start: List[float]    # [j1..j6, gripper] in degrees for revolute joints, meters for gripper
    goal: List[float]     # same format
    duration: float       # seconds (must be > 0)
    hold: bool = False    # if True, treat as hold (start==goal usually)


def compute_forward_kinematics(joint_angles_deg: List[float], urdf_path: str) -> Tuple[float, float, float]:
    """
    Compute end-effector position (X, Y, Z) in meters from joint angles using ikpy.
    Reads D1-550 URDF and computes FK automatically.
    
    Args:
        joint_angles_deg: List of 6 joint angles in degrees [j1, j2, j3, j4, j5, j6]
        urdf_path: Path to the URDF file
    
    Returns:
        Tuple of (x, y, z) in meters
    """
    # Load the kinematic chain from URDF
    chain = ikpy.chain.Chain.from_urdf_file(urdf_path, base_elements=["base_link"])
    
    # ikpy needs joint positions in radians for all active joints in the chain
    # The chain includes: base + 6 arm joints + 2 gripper joints = 8 links total
    # We'll use: base(0) + j1-j6 (radians) + gripper(0)
    joint_positions = [0] + [math.radians(a) for a in joint_angles_deg[:6]] + [0]
    
    # Compute forward kinematics (returns 4x4 transformation matrix)
    transform_matrix = chain.forward_kinematics(joint_positions)
    
    # Extract X, Y, Z position from transformation matrix
    x = transform_matrix[0, 3]
    y = transform_matrix[1, 3]
    z = transform_matrix[2, 3]
    
    return (x, y, z)


class Quintic:
    """
    Quintic trajectory for one DoF with zero boundary velocity and acceleration:
      q(t) = q0 + dq*(10*s^3 - 15*s^4 + 6*s^5), where s=t/T in [0,1]
    This is C2-continuous and typically removes most vibration/flicker artifacts in visualization.
    """
    def __init__(self, q0: float, q1: float, T: float):
        self.q0 = q0
        self.q1 = q1
        self.T = max(T, 1e-6)
        self.dq = (q1 - q0)

    def pos(self, t: float) -> float:
        if t <= 0.0:
            return self.q0
        if t >= self.T:
            return self.q1
        s = t / self.T
        s2 = s * s
        s3 = s2 * s
        s4 = s3 * s
        s5 = s4 * s
        blend = (10.0 * s3) - (15.0 * s4) + (6.0 * s5)
        return self.q0 + self.dq * blend


class PickPlaceSimulation(Node):
    def __init__(self):
        super().__init__('pick_place_simulation')

        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Joint names for D1-550 robot
        self.joint_names = [
            'Joint1', 'Joint2', 'Joint3',
            'Joint4', 'Joint5', 'Joint6',
            'Joint_L', 'Joint_R'
        ]
        
        # Path to URDF file
        self.urdf_path = os.path.expanduser('/home/jeewanthas/Desktop/arm/d1_550_description/urdf/d1_550_description.urdf')

        # Publish rate (Hz): higher is usually smoother in RViz
        # You can increase this: 100, 200, 500, or even 1000 Hz
        self.publish_hz = 200.0  # Increased from 100 Hz for smoother motion
        self.dt = 1.0 / self.publish_hz

        # Sequence (angles in degrees, gripper in meters)
        # Note: For "open/close gripper" steps, we treat them as a smooth segment too.
        # Duration controls speed - increase duration = slower motion
        self.sequence = [
            ("Initial/Safe position (folded near base)",
             [2.4, -86.3, 88.5, -16.6, -7.8, -164, 0.033], 3.0, True),

            ("Step 1: Moving to first position (gripper closed)",
             [67.9, 73.2, -22.6, 11.6, 35.7, 59.7, 0.001], 5.0, False),

            ("Step 3: Moving to pick position",
             [70, 88.3, -30.2, 11.7, 31.9, 59.5, 0.001], 4.0, False),

            ("Step 4: Closing gripper - GRAB!",
             [70, 88.3, -30.2, 11.7, 31.9, 59.5, 0.033], 2.0, False),

            ("Step 5: Moving to transport position",
             [70.6, 54.7, -14, 74.5, -70.2, 133.8, 0.033], 5.0, False),

            ("Step 6: Moving to intermediate position",
             [-57.1, 62.1, -15.2, -10.6, 37.6, -164.1, 0.033], 6.0, False),

            ("Step 7: Moving to place position",
             [-59.3, 82.3, -20.5, -10.4, 29.1, -164.1, 0.033], 4.0, False),

            ("Step 8: Opening gripper - RELEASE!",
             [-59.3, 82.3, -20.5, -10.4, 29.1, -164.1, 0.001], 2.0, False),

            ("Step 9: Returning to safe position",
             [2.4, -86.3, 88.5, -16.6, -7.8, -164, 0.001], 6.0, False),

            ("Hold before repeating...",
             [2.4, -86.3, 88.5, -16.6, -7.8, -164, 0.033], 3.0, True),
        ]

        # Build segments with explicit starts
        self.segments: List[Segment] = []
        self._build_segments()

        # Segment state
        self.seg_index = 0
        self.seg_t = 0.0
        self.active_traj = None  # list[Quintic] per DOF

        # Current commanded position (degrees for joints, meters for gripper)
        self.current_cmd = self.segments[0].start.copy()

        # Logging
        self.get_logger().info("=" * 60)
        self.get_logger().info("Pick and Place Simulation Starting (Timer-based, Quintic)")
        self.get_logger().info(f"Publishing /joint_states at {self.publish_hz:.1f} Hz")
        self.get_logger().info("=" * 60)

        # Initialize first segment trajectories
        self._start_segment(0)

        # Timer: drives the entire sequence and publishing
        self.timer = self.create_timer(self.dt, self._on_timer)

    def _build_segments(self):
        # Starting pose is zero
        current = [0, 0, 0, 0, 0, 0, 0.033]

        for desc, target, duration, is_hold in self.sequence:
            target = target.copy()

            # If it's a hold step, keep current==target (or accept provided target)
            if is_hold:
                # ensure continuity: hold uses current -> current unless target differs
                seg_start = current.copy()
                seg_goal = target.copy()
                # If user passed a different target for hold, we still move there smoothly first
            else:
                seg_start = current.copy()
                seg_goal = target.copy()

            self.segments.append(Segment(
                description=desc,
                start=seg_start,
                goal=seg_goal,
                duration=max(float(duration), 1e-3),
                hold=bool(is_hold),
            ))

            current = seg_goal.copy()

    def _start_segment(self, idx: int):
        idx = max(0, min(idx, len(self.segments) - 1))
        seg = self.segments[idx]

        self.seg_index = idx
        self.seg_t = 0.0

        # Create quintic trajectories per DOF (7 DOF: 6 joints + gripper)
        self.active_traj = [Quintic(seg.start[i], seg.goal[i], seg.duration) for i in range(7)]

        self.get_logger().info("")
        self.get_logger().info(seg.description)
        self.get_logger().info(
            "  Goal: "
            f"J1={seg.goal[0]:.1f}° J2={seg.goal[1]:.1f}° J3={seg.goal[2]:.1f}° "
            f"J4={seg.goal[3]:.1f}° J5={seg.goal[4]:.1f}° J6={seg.goal[5]:.1f}° "
            f"Gripper={seg.goal[6]:.3f}m "
            f"(T={seg.duration:.2f}s)"
        )
        
        # Compute and display end-effector position using ikpy + URDF
        try:
            x, y, z = compute_forward_kinematics(seg.goal, self.urdf_path)
            distance = math.sqrt(x**2 + y**2 + z**2)
            self.get_logger().info(
                f"  📍 End-Effector Position (ikpy from URDF): "
                f"X={x*1000:.1f}mm, Y={y*1000:.1f}mm, Z={z*1000:.1f}mm "
                f"(Distance from base: {distance*1000:.1f}mm)"
            )
        except Exception as e:
            self.get_logger().warn(f"  ⚠ FK computation failed: {e}")

    def _publish_joint_state(self, angles_deg_and_gripper_m: List[float]):
        """
        Publish joint state:
          - Revolute joints: degrees -> radians
          - Gripper: meters (published to Joint_L and Joint_R equally)
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        j1, j2, j3, j4, j5, j6, grip = angles_deg_and_gripper_m

        msg.position = [
            math.radians(j1),
            math.radians(j2),
            math.radians(j3),
            math.radians(j4),
            math.radians(j5),
            math.radians(j6),
            float(grip),
            float(grip),
        ]

        self.joint_pub.publish(msg)

    def _on_timer(self):
        # Always publish at a fixed rate to keep RViz stable
        seg = self.segments[self.seg_index]

        # Evaluate current segment trajectories
        if self.active_traj is not None:
            self.current_cmd = [self.active_traj[i].pos(self.seg_t) for i in range(7)]

        self._publish_joint_state(self.current_cmd)

        # Advance time and segment logic
        self.seg_t += self.dt

        # Move to next segment when done, or loop back to start
        if self.seg_t >= seg.duration:
            # Clamp to goal exactly at segment end
            self.current_cmd = seg.goal.copy()
            self._publish_joint_state(self.current_cmd)

            # Advance to next segment
            if self.seg_index < (len(self.segments) - 1):
                self._start_segment(self.seg_index + 1)
            else:
                # Reached end of sequence - loop back to beginning
                self.get_logger().info("\n🔄 Restarting pick and place sequence...\n")
                self._start_segment(0)


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceSimulation()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
