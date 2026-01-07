#!/bin/bash
# Configure network for Unitree D1-550 Robot Arm
# Sets static IP to communicate with robot at 192.168.123.100

echo "=========================================="
echo "Unitree D1-550 Network Configuration"
echo "=========================================="
echo ""

# Find the ethernet interface connected to robot
# Usually it's a USB ethernet adapter (enx...)
INTERFACE=$(ip link show | grep -E "enx[0-9a-f]+" | awk -F': ' '{print $2}')

if [ -z "$INTERFACE" ]; then
    echo "❌ ERROR: No USB ethernet interface found!"
    echo ""
    echo "Available interfaces:"
    ip link show | grep -v "lo:" | grep "state UP"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Check if robot ethernet cable is connected"
    echo "  2. Check 'ip link show' for available interfaces"
    echo "  3. Manually specify interface: sudo $0 <interface_name>"
    exit 1
fi

# Allow manual interface specification
if [ ! -z "$1" ]; then
    INTERFACE=$1
    echo "Using manually specified interface: $INTERFACE"
fi

echo "Detected interface: $INTERFACE"
echo ""

# Configuration
ROBOT_IP="192.168.123.100"
MY_IP="192.168.123.10"
NETMASK="255.255.255.0"
NETWORK="192.168.123.0/24"

echo "Configuration:"
echo "  Interface:  $INTERFACE"
echo "  Your IP:    $MY_IP"
echo "  Robot IP:   $ROBOT_IP"
echo "  Netmask:    $NETMASK"
echo ""

# Check if we need sudo
if [ "$EUID" -ne 0 ]; then
    echo "This script requires sudo privileges to configure network."
    echo "Re-running with sudo..."
    echo ""
    sudo "$0" "$@"
    exit $?
fi

echo "Step 1: Bringing interface down..."
ip link set $INTERFACE down
sleep 1

echo "Step 2: Bringing interface up..."
ip link set $INTERFACE up
sleep 1

echo "Step 3: Removing old IP addresses..."
ip addr flush dev $INTERFACE

echo "Step 4: Assigning static IP $MY_IP..."
ip addr add $MY_IP/24 dev $INTERFACE

echo "Step 5: Verifying configuration..."
echo ""
ip addr show $INTERFACE | grep "inet "

echo ""
echo "Step 6: Testing connection to robot..."
echo "Pinging $ROBOT_IP (please wait)..."
if ping -c 3 -W 2 $ROBOT_IP > /dev/null 2>&1; then
    echo "✓ SUCCESS: Robot is reachable at $ROBOT_IP"
    echo ""
    echo "=========================================="
    echo "Network configuration complete!"
    echo "=========================================="
    echo ""
    echo "You can now run robot control programs:"
    echo "  cd build"
    echo "  ./pick_and_place"
    echo "  ./pap_with_trajectory_planning"
    echo ""
else
    echo "⚠ WARNING: Cannot ping robot at $ROBOT_IP"
    echo ""
    echo "Possible issues:"
    echo "  1. Robot is not powered on"
    echo "  2. Ethernet cable not connected properly"
    echo "  3. Robot IP might be different"
    echo "  4. Robot is still booting (wait 30 seconds and try again)"
    echo ""
    echo "Network is configured correctly. Try:"
    echo "  ping $ROBOT_IP"
    echo ""
fi

echo "To see current network status:"
echo "  ip addr show $INTERFACE"
echo "  ping $ROBOT_IP"
echo ""
