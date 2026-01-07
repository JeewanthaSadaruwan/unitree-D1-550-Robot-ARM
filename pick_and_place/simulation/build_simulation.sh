#!/bin/bash
# Build script for D1-550 Robot Simulation

echo "=========================================="
echo "Building D1-550 Robot Simulation"
echo "=========================================="

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo ""
echo "Step 1: Sourcing ROS2 Humble..."
source /opt/ros/humble/setup.bash

echo ""
echo "Step 2: Building d1_550_description package..."
cd ~/ros2_ws
colcon build --packages-select d1_550_description --symlink-install

if [ $? -ne 0 ]; then
    echo "✗ Failed to build d1_550_description"
    exit 1
fi

echo ""
echo "✓ Build successful!"
echo ""
echo "=========================================="
echo "Build completed successfully!"
echo "=========================================="
echo ""
echo "To launch the simulation, run:"
echo "  ./run_simulation.sh"
echo ""
echo "Or manually:"
echo "  cd ~/ros2_ws"
echo "  source install/setup.bash"
echo "  ros2 launch d1_550_description display.launch.py"
echo ""
