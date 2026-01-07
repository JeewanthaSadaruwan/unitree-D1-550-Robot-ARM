import math
import os
import sys

import numpy as np
from ikpy.chain import Chain

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
except Exception:
    rclpy = None

WORKSPACE_HINTS = {
    "x": (-0.2, 0.4),
    "y": (-0.4, 0.4),
    "z": (0.0, 0.6),
}


def load_chain():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "urdf", "d1_550_description.urdf")
    if not os.path.exists(urdf_path):
        print(f"URDF not found at: {urdf_path}")
        sys.exit(1)
    chain = Chain.from_urdf_file(urdf_path, base_elements=["base_link"])
    return chain


def print_workspace():
    print("\n=== D1-550 Arm Workspace (approximate) ===")
    print(f"X: {WORKSPACE_HINTS['x'][0]}m to {WORKSPACE_HINTS['x'][1]}m")
    print(f"Y: {WORKSPACE_HINTS['y'][0]}m to {WORKSPACE_HINTS['y'][1]}m")
    print(f"Z: {WORKSPACE_HINTS['z'][0]}m to {WORKSPACE_HINTS['z'][1]}m")
    print("(Values outside this range may be unreachable or unsafe)\n")


def prompt_target():
    x = float(input("Enter target X (meters, e.g. 0.2): "))
    y = float(input("Enter target Y (meters, e.g. 0.0): "))
    z = float(input("Enter target Z (meters, e.g. 0.3): "))
    return x, y, z


def configure_active_joints(chain):
    arm_joint_names = {"Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"}
    chain.active_links_mask = [link.name in arm_joint_names for link in chain.links]
    return arm_joint_names


def format_joint_outputs(chain, joint_angles, arm_joint_names):
    active_links = [
        (i, link.name)
        for i, link in enumerate(chain.links)
        if link.name in arm_joint_names
    ]

    print("\nComputed joint angles:")
    for idx, name in active_links:
        angle = joint_angles[idx]
        print(f"  {name}: {angle:.3f} rad ({math.degrees(angle):.1f} deg)")


class IKPublisher(Node):
    def __init__(self, joint_names, joint_positions):
        super().__init__("d1_ik_demo_publisher")
        self.joint_names = joint_names
        self.joint_positions = joint_positions
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(0.05, self._publish)

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = list(self.joint_positions)
        self.pub.publish(msg)


def publish_to_rviz(joint_names, joint_positions, seconds=3.0):
    if rclpy is None:
        print("ROS2 (rclpy) not available; cannot publish to RViz.")
        return

    rclpy.init()
    node = IKPublisher(joint_names, joint_positions)
    end_time = node.get_clock().now().nanoseconds + int(seconds * 1e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


def main():
    chain = load_chain()
    arm_joint_names = configure_active_joints(chain)

    print_workspace()
    x, y, z = prompt_target()

    target_position = np.array([x, y, z], dtype=float)

    initial_positions = [0.0] * len(chain.links)
    joint_angles = chain.inverse_kinematics(target_position, initial_position=initial_positions)

    format_joint_outputs(chain, joint_angles, arm_joint_names)

    effector_pose = chain.forward_kinematics(joint_angles)
    print("\nResulting end-effector position (FK check):")
    print(f"  X: {effector_pose[0,3]:.3f} m")
    print(f"  Y: {effector_pose[1,3]:.3f} m")
    print(f"  Z: {effector_pose[2,3]:.3f} m")

    error = math.sqrt(
        (effector_pose[0, 3] - x) ** 2
        + (effector_pose[1, 3] - y) ** 2
        + (effector_pose[2, 3] - z) ** 2
    )
    print(f"  Position error: {error:.4f} m")

    joint_names = []
    joint_positions = []
    for idx, link in enumerate(chain.links):
        if link.name in arm_joint_names:
            joint_names.append(link.name)
            joint_positions.append(float(joint_angles[idx]))

    publish_to_rviz(joint_names, joint_positions)


if __name__ == "__main__":
    main()
