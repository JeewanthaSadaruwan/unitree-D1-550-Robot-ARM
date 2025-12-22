#!/bin/bash
# Run the complete demonstration with automated movement

echo "=========================================="
echo "GP7 Robot - Automated Pick and Place Demo"
echo "=========================================="
echo ""
echo "Starting simulation..."
echo "- RViz with boxes"
echo "- Automated smooth movement"
echo "- NO joint_state_publisher_gui (to avoid conflicts)"
echo ""

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Source ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Kill any existing processes
killall -9 rviz2 robot_state_publisher joint_state_publisher_gui boxes 2>/dev/null
sleep 1

# Launch display without joint_state_publisher_gui
echo "Launching RViz and robot..."
ros2 launch gp7_robot display_auto.launch.py &
LAUNCH_PID=$!

# Wait for everything to start
sleep 5

echo ""
echo "Starting automated movement..."
echo "Watch the robot perform pick and place!"
echo ""
echo "Press Ctrl+C to stop"
echo "=========================================="
echo ""

# Run the movement script
python3 demo_movement.py

# Cleanup on exit
kill $LAUNCH_PID 2>/dev/null
