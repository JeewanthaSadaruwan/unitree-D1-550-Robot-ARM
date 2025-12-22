#!/bin/bash
# Complete launch script - launches everything in correct order

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

echo "=========================================="
echo "GP7 Robot - Box Grab & Place Simulation"
echo "=========================================="
echo ""

# Source ROS2
echo "Sourcing ROS2..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Clean up any existing processes
echo "Cleaning up old processes..."
killall -9 rviz2 robot_state_publisher boxes python3 2>/dev/null
sleep 1

echo ""
echo "Starting simulation components..."
echo ""

# 1. Start robot_state_publisher
echo "[1/3] Starting robot state publisher..."
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat install/gp7_robot/share/gp7_robot/urdf/GP7\ Robot.SLDASM.urdf)" &
RSP_PID=$!
sleep 2

# 2. Start box publisher
echo "[2/3] Starting box publisher..."
ros2 run gp7_boxes boxes &
BOXES_PID=$!
sleep 1

# 3. Start RViz with config
echo "[3/3] Starting RViz..."
ros2 run rviz2 rviz2 -d install/gp7_robot/share/gp7_robot/config/urdf.rviz &
RVIZ_PID=$!
sleep 3

echo ""
echo "=========================================="
echo "✓ All components started!"
echo "=========================================="
echo ""
echo "RViz should now be open."
echo ""
echo "To see smooth automated movement, open a NEW terminal and run:"
echo "  cd \"$SCRIPT_DIR\""
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source install/setup.bash"
echo "  python3 demo_movement.py"
echo ""
echo "Press Ctrl+C to stop all processes"
echo "=========================================="
echo ""

# Wait for interrupt
wait $RSP_PID $BOXES_PID $RVIZ_PID
