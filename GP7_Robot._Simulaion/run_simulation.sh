#!/bin/bash
# Launch script for GP7 Robot with Box Simulation

echo "=========================================="
echo "Launching GP7 Robot with Box Simulation"
echo "=========================================="

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Navigate to the workspace
cd "$SCRIPT_DIR"

echo ""
echo "Sourcing ROS2 Jazzy..."
source /opt/ros/jazzy/setup.bash

echo "Sourcing workspace..."
source install/setup.bash

echo ""
echo "Launching simulation..."
echo "- Robot State Publisher"
echo "- Joint State Publisher GUI"
echo "- RViz2 with robot visualization"
echo "- Box markers (target box and gripped box)"
echo ""

ros2 launch gp7_robot display.launch.py
