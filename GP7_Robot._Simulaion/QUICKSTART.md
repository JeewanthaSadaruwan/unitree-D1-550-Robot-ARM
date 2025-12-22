# GP7 Robot - Quick Start Guide

## Box Grab and Place Simulation

This simulation demonstrates a GP7 robotic manipulator with 8 links (including end effector) performing a box grab and place task visualization in RViz.

---

## What's Included

✅ **GP7 Robot Model** - Full 8-link robotic arm with end effector  
✅ **Target Box (Red)** - Static box representing the target location  
✅ **Gripped Box (Blue)** - Box attached to end effector (simulates gripping)  
✅ **Joint Control** - GUI to manually control all robot joints  
✅ **RViz Visualization** - Real-time 3D visualization  

---

## Quick Start (3 Steps)

### 1. Build the Simulation
```bash
./build_simulation.sh
```

### 2. Launch the Simulation
```bash
./run_simulation.sh
```

### 3. Use the Simulation
- A window will open with the robot in RViz
- Use the **Joint State Publisher GUI** sliders to move robot joints
- Watch the **blue box** (gripped) follow the end effector
- Move the end effector to the **red box** (target) to simulate pick and place

---

## Manual Commands

If you prefer to run commands manually:

```bash
# Source ROS2
source /opt/ros/jazzy/setup.bash

# Source workspace
source install/setup.bash

# Launch
ros2 launch gp7_robot display.launch.py
```

---

## What You'll See

### In RViz:
- **Robot Model**: 3D visualization of GP7 robot (8 links)
- **Red Box**: Target position (static, 30x30x20 cm)
- **Blue Box**: Gripped object (attached to link_8, 10x10x10 cm)
- **TF Frames**: Coordinate frames for all links

### Joint State Publisher GUI:
- 8 sliders controlling each joint
- Real-time position feedback
- Move sliders to animate the robot

---

## Box Positions

### Target Box (Red)
- **Position**: (0.30m, 0.40m, 0.10m) from base
- **Size**: 30cm × 30cm × 20cm
- **Frame**: base_link

### Gripped Box (Blue)
- **Position**: 5cm in front of link_8
- **Size**: 10cm × 10cm × 10cm  
- **Frame**: link_8 (end effector)

---

## Simulation Tips

1. **Move to Target**: Adjust joints to position the blue box over the red box
2. **Explore Workspace**: Use sliders to understand robot's reach
3. **Check Collisions**: Visually verify robot doesn't self-collide
4. **Test Trajectories**: Plan paths between different configurations

---

## Customization

### Change Box Positions
Edit: `gp7_boxes/gp7_boxes/scripts/boxes.py`

```python
# Target box position
box_a.pose.position.x = 0.30
box_a.pose.position.y = 0.40
box_a.pose.position.z = 0.10

# Gripped box position (relative to link_8)
box_b.pose.position.x = 0.05
```

### Change Colors
```python
# RGB + Alpha (0.0 to 1.0)
box_a.color.r = 1.0  # Red
box_a.color.g = 0.0  # Green
box_a.color.b = 0.0  # Blue
box_a.color.a = 0.8  # Transparency
```

After changes, rebuild:
```bash
./build_simulation.sh
```

---

## Troubleshooting

### Boxes not visible in RViz
1. Check "Marker" displays are added in RViz
2. Topics should be `/box_a` and `/box_b`
3. Verify nodes are running: `ros2 node list`

### Blue box not following robot
- Ensure frame is "link_8" (check boxes.py)
- Verify TF tree: `ros2 run tf2_tools view_frames`

### Build fails
```bash
# Install dependencies
sudo apt update
sudo apt install ros-jazzy-joint-state-publisher-gui
sudo apt install ros-jazzy-xacro
```

### Launch fails
- Make sure ROS2 is sourced: `source /opt/ros/jazzy/setup.bash`
- Verify workspace is built: `ls install/`

---

## Package Structure

```
GP7_Robot._Simulaion/
├── gp7_boxes/              # Box visualization package
│   └── gp7_boxes/scripts/
│       └── boxes.py        # Box publisher node
├── urdf/                   # Robot URDF files
├── meshes/                 # 3D meshes (link_1 to link_8)
├── launch/
│   └── display.launch.py   # Main launch file
├── build_simulation.sh     # Build script
└── run_simulation.sh       # Launch script
```

---

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/box_a` | Marker | Target box visualization |
| `/box_b` | Marker | Gripped box visualization |
| `/joint_states` | JointState | Robot joint positions |
| `/robot_description` | String | URDF model |
| `/tf` | TFMessage | Transform tree |

---

## Next Steps

- **Trajectory Planning**: Add MoveIt integration for path planning
- **Gazebo**: Add physics simulation with Gazebo
- **Controllers**: Implement position/velocity controllers
- **Sensors**: Add camera or depth sensor simulation
- **Automated Sequence**: Create autonomous pick-and-place routine

---

## Support

For issues or questions, refer to:
- `README_BOX_SIMULATION.md` - Detailed documentation
- ROS2 Jazzy docs: https://docs.ros.org/en/jazzy/

---

**Version**: 1.0.0  
**ROS2 Distribution**: Jazzy  
**Python**: 3.10+  
