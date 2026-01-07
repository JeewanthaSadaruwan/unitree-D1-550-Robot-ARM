# Simulation - Pick and Place

## Overview
RViz simulation for testing pick and place operations before deploying to the physical robot.

## Files
- `run_simulation.sh` - Launch RViz with robot visualization ⭐
- `pick_place_simulation.py` - Pick and place simulation script
- `environment_publisher.py` - Shows boxes/platforms in RViz
- `demo_movement.py` - Demo robot movements
- `inverse_kinematics_demo.py` - Test IK solver
- `build_simulation.sh` - Build simulation files
- `run_demo.sh` - Run demo movements
- `urdf.rviz` - RViz configuration
- `launch/` - ROS2 launch files
- `config/` - Joint configuration files

## How to Run

### Method 1: Quick Start (Recommended)
```bash
cd /home/jeewanthas/Desktop/arm/pick_and_place/simulation
./run_simulation.sh
```

This will launch:
- Robot State Publisher
- Joint State Publisher GUI (sliders to control joints)
- RViz2 with robot visualization

### Method 2: Manual Launch
```bash
# Terminal 1: Launch RViz
cd /home/jeewanthas/Desktop/arm
source install/setup.bash
ros2 launch d1_550_description display.launch.py

# Terminal 2: Run pick and place simulation
cd pick_and_place/simulation
python3 pick_place_simulation.py
```

## What It Does
- Publishes joint states to move the robot arm
- Visualizes the robot motion in RViz
- Tests pick and place positions
- Validates trajectory paths

## Modifying Pick/Place Positions

Edit `pick_place_simulation.py` to change target positions:

```python
# Example: Modify pick position
pick_position = {
    'Joint1': 0.0,
    'Joint2': -1.2,
    'Joint3': 1.5,
    'Joint4': 0.0,
    'Joint5': -0.3,
    'Joint6': 0.0
}
```

## Testing Workflow

1. **Launch RViz** - See the robot visualization
2. **Run simulation** - Watch the motion
3. **Verify positions** - Check if pick/place locations are correct
4. **Adjust as needed** - Modify joint angles
5. **Repeat** - Until motion is satisfactory
6. **Deploy to hardware** - Use same positions in implementation

## Tips

- Start with small movements to test
- Use Joint State Publisher GUI to find good positions manually
- Record successful joint angles for implementation
- Check for collisions in RViz

## Troubleshooting

### Robot not visible in RViz
- Check if `robot_state_publisher` is running
- Ensure `/joint_states` topic is being published

### Simulation not moving
- Verify ROS2 nodes are running: `ros2 node list`
- Check topic: `ros2 topic echo /joint_states`

## Next Steps

Once simulation works:
1. Note down successful pick/place joint angles
2. Transfer to `implementation/pick_and_place.cpp`
3. Test on physical robot
