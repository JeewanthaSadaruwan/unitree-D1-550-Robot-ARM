# GP7 Robot - Quick Start Guide

## Installation (5 minutes)

### 1. Prerequisites
```bash
# Ensure ROS 2 Jazzy is installed
source /opt/ros/jazzy/setup.bash

# Install dependencies
sudo apt update
sudo apt install \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-rviz2 \
    ros-jazzy-gazebo-ros-pkgs \
    ros-jazzy-xacro
```

### 2. Setup Workspace
```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Copy this package
cp -r /path/to/gp7_robot_description .

# **IMPORTANT**: Copy mesh files
cp /path/to/your/meshes/*.STL ~/ros2_ws/src/gp7_robot_description/meshes/

# Build
cd ~/ros2_ws
colcon build --packages-select gp7_robot_description

# Source workspace
source install/setup.bash
```

## Quick Commands

### Visualize in RViz2
```bash
ros2 launch gp7_robot_description display.launch.py
```

### Launch in Gazebo
```bash
ros2 launch gp7_robot_description gazebo.launch.py
```

### Validate URDF
```bash
cd ~/ros2_ws/src/gp7_robot_description
./validate_urdf.py
```

### Check Robot Description
```bash
# View robot description
ros2 topic echo /robot_description

# List all topics
ros2 topic list

# View TF tree
ros2 run tf2_tools view_frames
```

### Control Joints Manually
```bash
# Publish to joint_states topic
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{
  name: ['Joint_0', 'Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5'],
  position: [0.5, 0.5, 0.5, 0.0, 0.0, 0.0]
}"
```

## Common Issues

### Issue: Meshes not loading
**Solution**: Ensure STL files are in the `meshes/` directory and rebuild:
```bash
cd ~/ros2_ws
colcon build --packages-select gp7_robot_description
source install/setup.bash
```

### Issue: Package not found
**Solution**: Source your workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

### Issue: RViz shows no robot
**Solution**: 
1. Check Fixed Frame is set to `base_link`
2. Verify robot_description topic: `ros2 topic list`
3. Check for errors: `ros2 topic echo /rosout`

## File Structure Reference

```
gp7_robot_description/
├── launch/
│   ├── display.launch.py    # RViz visualization
│   └── gazebo.launch.py     # Gazebo simulation
├── urdf/
│   └── gp7_robot.urdf       # Robot model
├── meshes/
│   └── *.STL                # Visual/collision meshes
├── config/
│   └── joint_names_*.yaml   # Joint configuration
└── rviz/
    └── display.rviz         # RViz settings
```

## Next Steps

1. **Motion Planning**: Integrate with MoveIt 2
2. **Control**: Add ros2_control configuration
3. **Simulation**: Enhance Gazebo model with sensors
4. **Hardware**: Connect to real robot controller

## Resources

- [ROS 2 Jazzy Docs](https://docs.ros.org/en/jazzy/)
- [URDF Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html)
- [MoveIt 2](https://moveit.picknik.ai/main/index.html)

## Support

For issues or questions, check the README.md file in the package root.
