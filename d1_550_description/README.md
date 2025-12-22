# D1-550 Robot - ROS2 Humble

6-DOF robotic arm simulation with pick-and-place demonstration.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select d1_550_description --symlink-install
source install/setup.bash
```

## Run

### Interactive Mode (with joint GUI)
```bash
cd ~/Desktop/d1_550_description
./run_simulation.sh
```
Or:
```bash
ros2 launch d1_550_description display.launch.py
```

### Automated Pick-and-Place Demo
```bash
cd ~/Desktop/d1_550_description
./run_demo.sh
```
Or:
```bash
ros2 launch d1_550_description demo.launch.py
python3 demo_movement.py
```

## Features

- **Robot**: D1-550 6-DOF arm + 2-finger gripper
- **Visualization**: RViz2 with robot model and environment markers
- **Demo**: Automated pick-and-place between two stages
- **Smooth Motion**: Cubic ease-in-out interpolation at 50Hz

## Demo Sequence

1. Robot rotates toward source stage (blue platform)
2. Reaches and lowers to pick up red box
3. Grasps box with gripper
4. Lifts and transfers to target stage
5. Places box and returns to home position
6. Loops continuously

## Stop Demo

Press `Ctrl+C` in the terminal running the demo.
