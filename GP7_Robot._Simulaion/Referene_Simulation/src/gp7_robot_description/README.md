# GP7 Robot Description Package

ROS 2 Jazzy-compatible URDF description package for the GP7 Robot (6-DOF industrial manipulator).

## Package Information

- **Package Name**: `gp7_robot_description`
- **ROS Version**: ROS 2 Jazzy Jalisco
- **Build System**: ament_cmake
- **License**: BSD

## Package Structure

```
gp7_robot_description/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package manifest
├── README.md                   # This file
├── config/
│   └── joint_names_gp7_robot.yaml  # Joint configuration
├── launch/
│   ├── display.launch.py       # Launch file for RViz visualization
│   └── gazebo.launch.py        # Launch file for Gazebo simulation
├── meshes/
│   ├── base_link.STL           # 3D mesh files (to be added)
│   ├── Link_1.STL
│   ├── Link_2.STL
│   ├── Link_3.STL
│   ├── Link_4.STL
│   ├── Link_5.STL
│   └── Link_6.STL
├── rviz/
│   └── display.rviz            # RViz configuration
└── urdf/
    └── gp7_robot.urdf          # Robot description
```

## Robot Specifications

### Joints

The GP7 Robot has 6 revolute joints:

| Joint   | Type     | Range (rad)        | Parent   | Child    |
|---------|----------|--------------------|----------|----------|
| Joint_0 | Revolute | -2.967 to 2.967    | base_link| Link_1   |
| Joint_1 | Revolute | -1.135 to 2.094    | Link_1   | Link_2   |
| Joint_2 | Revolute | -1.222 to 3.316    | Link_2   | Link_3   |
| Joint_3 | Revolute | -3.316 to 3.316    | Link_3   | Link_4   |
| Joint_4 | Revolute | -2.356 to 2.356    | Link_4   | Link_5   |
| Joint_5 | Revolute | -6.281 to 6.281    | Link_5   | Link_6   |

### Links

- **base_link**: Robot base (mass: 5.22 kg)
- **Link_1**: First link (mass: 2.94 kg)
- **Link_2**: Second link (mass: 6.24 kg)
- **Link_3**: Third link (mass: 2.85 kg)
- **Link_4**: Fourth link (mass: 2.76 kg)
- **Link_5**: Fifth link (mass: 0.25 kg)
- **Link_6**: End effector link (mass: 0.024 kg)

## Dependencies

### Build Dependencies
- `ament_cmake`

### Runtime Dependencies
- `robot_state_publisher`
- `joint_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`
- `xacro`
- `gazebo_ros`

## Installation

### Prerequisites

Ensure you have ROS 2 Jazzy installed on your system.

### Building the Package

1. Create a workspace (if you don't have one):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Copy this package to your workspace:
```bash
cp -r gp7_robot_description ~/ros2_ws/src/
```

3. Copy the STL mesh files to the meshes directory:
```bash
# Copy your STL files from the original export location
cp /path/to/meshes/*.STL ~/ros2_ws/src/gp7_robot_description/meshes/
```

4. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select gp7_robot_description
```

5. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

### Visualize in RViz2

Launch the robot visualization in RViz2 with the joint state publisher GUI:

```bash
ros2 launch gp7_robot_description display.launch.py
```

This will open:
- RViz2 with the robot model
- Joint State Publisher GUI to control joint angles

### Launch Options

You can customize the launch with arguments:

```bash
# Launch without GUI (uses joint_state_publisher instead)
ros2 launch gp7_robot_description display.launch.py gui:=false

# Use simulation time
ros2 launch gp7_robot_description display.launch.py use_sim_time:=true
```

### Gazebo Simulation

To spawn the robot in Gazebo:

```bash
ros2 launch gp7_robot_description gazebo.launch.py
```

With custom spawn position:

```bash
ros2 launch gp7_robot_description gazebo.launch.py x_pose:=1.0 y_pose:=2.0 z_pose:=0.5
```

### Accessing Robot Description

To access the robot description in other nodes:

```bash
# View the robot description topic
ros2 topic echo /robot_description

# Use in Python
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor

# Get robot description parameter
robot_desc = self.get_parameter('robot_description').value
```

### TF Tree

View the transform tree:

```bash
# Install tf2_tools if not already installed
sudo apt install ros-jazzy-tf2-tools

# View TF tree
ros2 run tf2_tools view_frames

# View specific transforms
ros2 run tf2_ros tf2_echo base_link Link_6
```

### Joint State Control

Publish joint states manually:

```bash
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
  name: ['Joint_0', 'Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5'],
  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  velocity: [],
  effort: []
}"
```

## Integration Examples

### Using with MoveIt 2

This package can be integrated with MoveIt 2 for motion planning:

```bash
# Install MoveIt 2 (if not already installed)
sudo apt install ros-jazzy-moveit

# You'll need to generate a MoveIt configuration package
ros2 run moveit_setup_assistant moveit_setup_assistant
```

### Using with ros2_control

For hardware control, you can integrate with ros2_control:

1. Add ros2_control tags to the URDF
2. Define hardware interfaces
3. Configure controllers

## Troubleshooting

### Meshes Not Found

If you see warnings about missing meshes:

```
[WARN] [robot_state_publisher]: Could not load mesh: package://gp7_robot_description/meshes/base_link.STL
```

Ensure STL files are copied to the `meshes/` directory and the package is built and sourced.

### Joint Limits Violated

If joints exceed their limits, check the joint configuration in:
- `config/joint_names_gp7_robot.yaml`
- Joint limits in the URDF file

### RViz2 Not Showing Robot

1. Check that `/robot_description` topic is publishing:
```bash
ros2 topic list | grep robot_description
```

2. Verify TF frames:
```bash
ros2 run tf2_ros tf2_echo base_link Link_1
```

3. Check RViz2 Fixed Frame setting (should be `base_link`)

## Contributing

When making changes:

1. Update the URDF file in `urdf/gp7_robot.urdf`
2. Rebuild the package: `colcon build --packages-select gp7_robot_description`
3. Test with: `ros2 launch gp7_robot_description display.launch.py`

## References

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [URDF Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html)
- [ROS 2 Launch Files](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html)

## License

This package is licensed under the BSD License.

## Author

Original SolidWorks model exported using SW2URDF exporter.
ROS 2 Jazzy adaptation by: TODO

## Contact

Maintainer: TODO@email.com
