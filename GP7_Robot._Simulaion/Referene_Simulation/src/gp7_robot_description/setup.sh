#!/bin/bash
# Setup script for GP7 Robot Description package

set -e

echo "==================================="
echo "GP7 Robot Description Setup Script"
echo "==================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if ROS 2 Jazzy is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}ERROR: ROS 2 is not sourced. Please source ROS 2 Jazzy first:${NC}"
    echo "  source /opt/ros/jazzy/setup.bash"
    exit 1
fi

if [ "$ROS_DISTRO" != "jazzy" ]; then
    echo -e "${YELLOW}WARNING: ROS_DISTRO is set to '$ROS_DISTRO', but this package is designed for 'jazzy'${NC}"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo -e "${GREEN}✓ ROS 2 Jazzy detected${NC}"
echo ""

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PACKAGE_NAME="gp7_robot_description"

echo "Package location: $SCRIPT_DIR"
echo ""

# Check if workspace exists
if [ -d "$HOME/ros2_ws" ]; then
    WORKSPACE="$HOME/ros2_ws"
    echo -e "${GREEN}✓ Found workspace at: $WORKSPACE${NC}"
else
    echo -e "${YELLOW}No workspace found at ~/ros2_ws${NC}"
    read -p "Create workspace at ~/ros2_ws? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        mkdir -p "$HOME/ros2_ws/src"
        WORKSPACE="$HOME/ros2_ws"
        echo -e "${GREEN}✓ Created workspace at: $WORKSPACE${NC}"
    else
        read -p "Enter workspace path: " WORKSPACE
        if [ ! -d "$WORKSPACE" ]; then
            echo -e "${RED}ERROR: Workspace directory does not exist${NC}"
            exit 1
        fi
    fi
fi

echo ""

# Copy package to workspace if not already there
if [ "$SCRIPT_DIR" != "$WORKSPACE/src/$PACKAGE_NAME" ]; then
    echo "Copying package to workspace..."
    cp -r "$SCRIPT_DIR" "$WORKSPACE/src/"
    echo -e "${GREEN}✓ Package copied to $WORKSPACE/src/$PACKAGE_NAME${NC}"
else
    echo -e "${GREEN}✓ Package already in workspace${NC}"
fi

echo ""

# Check for mesh files
MESH_DIR="$WORKSPACE/src/$PACKAGE_NAME/meshes"
STL_COUNT=$(find "$MESH_DIR" -name "*.STL" 2>/dev/null | wc -l)

if [ "$STL_COUNT" -eq 0 ]; then
    echo -e "${YELLOW}WARNING: No STL mesh files found in $MESH_DIR${NC}"
    echo "You need to copy the mesh files from your SolidWorks export:"
    echo "  cp /path/to/meshes/*.STL $MESH_DIR/"
    echo ""
    read -p "Do you want to continue without mesh files? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
else
    echo -e "${GREEN}✓ Found $STL_COUNT STL mesh files${NC}"
fi

echo ""

# Install dependencies
echo "Checking dependencies..."
DEPS_MISSING=0

check_package() {
    if ! ros2 pkg list | grep -q "^$1$"; then
        echo -e "${RED}✗ Missing: $1${NC}"
        DEPS_MISSING=1
    else
        echo -e "${GREEN}✓ Found: $1${NC}"
    fi
}

check_package "robot_state_publisher"
check_package "joint_state_publisher"
check_package "joint_state_publisher_gui"
check_package "rviz2"

if [ $DEPS_MISSING -eq 1 ]; then
    echo ""
    echo -e "${YELLOW}Some dependencies are missing. Install them with:${NC}"
    echo "  sudo apt update"
    echo "  sudo apt install ros-jazzy-robot-state-publisher \\"
    echo "                   ros-jazzy-joint-state-publisher \\"
    echo "                   ros-jazzy-joint-state-publisher-gui \\"
    echo "                   ros-jazzy-rviz2 \\"
    echo "                   ros-jazzy-gazebo-ros-pkgs"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo ""

# Build the package
echo "Building package..."
cd "$WORKSPACE"

if colcon build --packages-select $PACKAGE_NAME; then
    echo -e "${GREEN}✓ Package built successfully${NC}"
else
    echo -e "${RED}ERROR: Build failed${NC}"
    exit 1
fi

echo ""

# Source the workspace
echo "Sourcing workspace..."
source "$WORKSPACE/install/setup.bash"
echo -e "${GREEN}✓ Workspace sourced${NC}"

echo ""
echo "==================================="
echo -e "${GREEN}Setup completed successfully!${NC}"
echo "==================================="
echo ""
echo "To use the package:"
echo "  1. Source the workspace in each new terminal:"
echo "     source $WORKSPACE/install/setup.bash"
echo ""
echo "  2. Launch the robot visualization:"
echo "     ros2 launch $PACKAGE_NAME display.launch.py"
echo ""
echo "  3. Or launch in Gazebo:"
echo "     ros2 launch $PACKAGE_NAME gazebo.launch.py"
echo ""
echo "Add this to your ~/.bashrc to automatically source the workspace:"
echo "  echo 'source $WORKSPACE/install/setup.bash' >> ~/.bashrc"
echo ""
