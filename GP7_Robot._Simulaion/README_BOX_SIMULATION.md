# GP7 Robot - Box Grab and Place Simulation

This package contains a complete ROS2 simulation for the GP7 Robot with end effector (8 links), including box grab and place visualization.

## Overview

The simulation includes:
- **GP7 Robot**: 8-link robotic manipulator with end effector
- **Box A (Red)**: Static target box at a fixed position
- **Box B (Blue)**: Gripped box attached to the end effector (link_8)

## Package Structure

```
GP7_Robot._Simulaion/
├── gp7_boxes/                    # Box visualization package
│   ├── gp7_boxes/
│   │   ├── scripts/
│   │   │   └── boxes.py         # Box publisher node
│   │   └── __init__.py
│   ├── resource/
│   ├── setup.py
│   ├── setup.cfg
│   └── package.xml
├── urdf/                         # Robot description files
├── meshes/                       # 3D meshes (link_1 to link_8)
├── launch/                       # Launch files
│   └── display.launch.py        # Main launch file with boxes
└── config/                       # Configuration files
```

## Installation and Build

### Prerequisites
- ROS2 (Humble, Iron, or Jazzy)
- Python 3
- RViz2

### Build Instructions

1. **Navigate to your workspace**:
   ```bash
   cd "/media/rusula-oshadha/New Volume/Oshadha/University of Moratuwa/Fifth Semester/Robotics/GP7_Robot._Simulaion"
   ```

2. **Build the packages**:
   ```bash
   colcon build --packages-select gp7_robot gp7_boxes
   ```

3. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

## Running the Simulation

### Launch the complete simulation:
```bash
ros2 launch gp7_robot display.launch.py
```

This will start:
- Robot State Publisher
- Joint State Publisher GUI (to control robot joints)
- RViz2 (visualization)
- Box Publisher (showing target and gripped boxes)

### Launch only the boxes (if robot is already running):
```bash
ros2 run gp7_boxes boxes
```

## Box Configuration

### Box A - Target Box (Red)
- **Color**: Red (semi-transparent)
- **Size**: 30cm x 30cm x 20cm
- **Position**: (0.30, 0.40, 0.10) relative to base_link
- **Purpose**: Represents the target location/object

### Box B - Gripped Box (Blue)
- **Color**: Blue (semi-transparent)
- **Size**: 10cm x 10cm x 10cm
- **Frame**: Attached to link_8 (end effector)
- **Position**: 5cm in front of link_8
- **Purpose**: Simulates an object held by the gripper

## Customization

### Adjusting Box Positions

Edit `/gp7_boxes/gp7_boxes/scripts/boxes.py`:

**For Box A (target):**
```python
box_a.pose.position.x = 0.30  # Change X position
box_a.pose.position.y = 0.40  # Change Y position
box_a.pose.position.z = 0.10  # Change Z position
```

**For Box B (gripped):**
```python
box_b.pose.position.x = 0.05  # Distance from link_8
box_b.pose.position.y = 0.0
box_b.pose.position.z = 0.0
```

### Changing Box Colors or Sizes

```python
# Color (RGBA)
box_a.color.r = 1.0  # Red component (0.0 to 1.0)
box_a.color.g = 0.0  # Green component
box_a.color.b = 0.0  # Blue component
box_a.color.a = 0.8  # Alpha (transparency)

# Size (meters)
box_a.scale.x = 0.30  # Length
box_a.scale.y = 0.30  # Width
box_a.scale.z = 0.20  # Height
```

## RViz Configuration

To view the boxes in RViz:

1. Make sure RViz is running
2. Add Marker display:
   - Click "Add" button
   - Select "Marker"
   - Set topic to `/box_a` for target box
   - Add another Marker for `/box_b` for gripped box

3. The boxes should now be visible and the blue box will follow the robot's end effector motion.

## Using the Joint State Publisher GUI

1. Use the sliders in the Joint State Publisher GUI to move the robot joints
2. Observe how:
   - The red box (Box A) remains stationary
   - The blue box (Box B) follows the end effector (link_8)
3. Move the end effector to the target box position to simulate a grab operation

## Troubleshooting

### Boxes not visible in RViz
- Check that the box publisher node is running: `ros2 node list`
- Verify marker topics: `ros2 topic list | grep box`
- Ensure RViz has Marker displays added for `/box_a` and `/box_b`

### Box B not following end effector
- Verify link_8 exists in the robot model
- Check TF frames: `ros2 run tf2_tools view_frames`
- The frame should be "link_8" (not "Link_8" or "link_7")

### Build errors
- Make sure all dependencies are installed:
  ```bash
  rosdep install --from-paths . --ignore-src -r -y
  ```

## Topics

- `/box_a` - Marker for target box
- `/box_b` - Marker for gripped box
- `/joint_states` - Robot joint states
- `/robot_description` - Robot URDF

## Nodes

- `box_publisher` - Publishes box markers to RViz
- `robot_state_publisher` - Publishes robot TF transforms
- `joint_state_publisher_gui` - GUI for controlling robot joints

## Future Enhancements

- Add trajectory planning for automatic grab and place
- Implement collision detection
- Add physics simulation with Gazebo
- Create automated pick-and-place sequences

## License

BSD

## Maintainer

GP7 Team - gp7@example.com
