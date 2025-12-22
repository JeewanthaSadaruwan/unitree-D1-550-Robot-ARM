#!/bin/bash
# Build script for GP7 Robot with Box Simulation

echo "=========================================="
echo "Building GP7 Robot with Box Simulation"
echo "=========================================="

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo ""
echo "Step 1: Sourcing ROS2 Jazzy..."
source /opt/ros/jazzy/setup.bash

echo ""
echo "Step 2: Building gp7_robot package..."
cd "$SCRIPT_DIR"
colcon build --packages-select gp7_robot

if [ $? -ne 0 ]; then
    echo "✗ Failed to build gp7_robot"
    exit 1
fi

echo ""
echo "Step 3: Building gp7_boxes package..."
cd "$SCRIPT_DIR/gp7_boxes"
colcon build --base-paths .

if [ $? -ne 0 ]; then
    echo "✗ Failed to build gp7_boxes"
    exit 1
fi

echo ""
echo "Step 4: Installing gp7_boxes to workspace..."
cp -r "$SCRIPT_DIR/gp7_boxes/install/gp7_boxes" "$SCRIPT_DIR/install/"

if [ $? -eq 0 ]; then
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
    echo "  source install/setup.bash"
    echo "  ros2 launch gp7_robot display.launch.py"
    echo ""
else
    echo ""
    echo "✗ Build failed. Please check the error messages above."
    exit 1
fi
