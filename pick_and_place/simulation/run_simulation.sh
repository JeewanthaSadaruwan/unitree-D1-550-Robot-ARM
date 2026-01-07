#!/bin/bash
# Launch script for D1-550 Robot Simulation

echo "=========================================="
echo "Launching D1-550 Robot Simulation"
echo "=========================================="

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo ""
echo "Sourcing ROS2 Humble..."
source /opt/ros/humble/setup.bash

echo "Sourcing workspace..."
cd /home/jeewanthas/Desktop/arm
source install/setup.bash

echo ""
echo "Launching simulation..."
echo "- Robot State Publisher"
echo "- Joint State Publisher GUI"
echo "- RViz2 with robot visualization"
echo "- Environment markers (pickup box and stages)"
echo ""
echo "Press Ctrl+C to stop"
echo "=========================================="
echo ""

# Use the launch file from original location
ros2 launch d1_550_description display.launch.py
