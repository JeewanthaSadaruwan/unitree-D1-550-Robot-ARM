# Implementation - Physical Robot Pick and Place

## Overview
C++ implementation for running pick and place on the physical Unitree D1-550 robot using the Unitree SDK.

## Files
- `pick_and_place.cpp` - Main pick and place program
- `pap_with_trajectory_planning.cpp` - Pick and place with trajectory planning
- `move_to_safe.cpp` - Move robot to safe position
- `test_single_move.cpp` - Test single movements
- `examples/` - SDK example programs
- `msg/` - DDS message definitions
- `build/` - Compiled executables

## Build

The code is already built, but if you modify any source files:

```bash
cd /home/jeewanthas/Desktop/arm/pick_and_place/implementation/build
cmake ..
make -j4
```

## Available Executables (in build/)

### Main Programs
- `pick_and_place` - Full pick and place routine
- `pap_with_trajectory_planning` - Advanced with trajectory planning
- `move_to_safe` - Move to safe position
- `test_single_move` - Test individual movements

### SDK Examples
- `joint_angle_control` - Control single joint
- `multiple_joint_angle_control` - Control multiple joints
- `joint_enable_control` - Enable/disable joints
- `arm_zero_control` - Zero/calibrate arm
- `get_arm_joint_angle` - Read current angles

## Prerequisites

### 1. Network Setup (Required for hardware)
```bash
cd /home/jeewanthas/Desktop/arm/d1_sdk
./setup_network.sh
```

### 2. Environment Variables (Required for hardware)
```bash
cd /home/jeewanthas/Desktop/arm/d1_sdk
source run_with_dds.sh
# OR manually:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$(pwd)/cyclonedds.xml
```

### 3. Build (if modified)
```bash
cd ../../d1_sdk/build
cmake ..
make pick_and_place
```

## How to Run

```bash
cd /home/jeewanthas/Desktop/arm/pick_and_place/implementation
./pick_and_place
```

## Code Structure

The `pick_and_place.cpp` file contains:
- Robot initialization
- Pick position joint angles
- Place position joint angles
- Trajectory execution
- Gripper control

## Modifying Positions

Edit `pick_and_place.cpp`:

```cpp
// Example: Modify pick position
std::vector<float> pick_position = {
    0.0f,   // Joint1
    -1.2f,  // Joint2
    1.5f,   // Joint3
    0.0f,   // Joint4
    -0.3f,  // Joint5
    0.0f    // Joint6
};
```

After editing, rebuild:
```bash
cd ../../d1_sdk/build
make pick_and_place
```

## Safety Guidelines

⚠️ **IMPORTANT:**

1. **Always start with safe position**
   ```bash
   cd ../../d1_sdk/build
   ./move_to_safe
   ```

2. **Test movements incrementally**
   - Start with small motions
   - Verify each position before full routine

3. **Monitor the robot**
   - Keep emergency stop ready
   - Watch for unexpected movements
   - Check joint limits

4. **Workspace awareness**
   - Keep area clear of obstacles
   - Ensure proper lighting
   - Have clear escape route

## Joint Limits (Radians)

| Joint | Min | Max |
|-------|-----|-----|
| Joint1 | -2.36 | +2.36 |
| Joint2 | -1.57 | +1.57 |
| Joint3 | -1.57 | +1.57 |
| Joint4 | -2.36 | +2.36 |
| Joint5 | -1.57 | +1.57 |
| Joint6 | -2.36 | +2.36 |

## Common Commands

```bash
# Get current joint angles
cd ../../d1_sdk/build
./get_arm_joint_angle

# Move to safe position
./move_to_safe

# Control single joint
./joint_angle_control

# Run pick and place
cd ../../pick_and_place/implementation
./pick_and_place
```

## Debugging

### Check robot connection
```bash
ping <robot_ip>
```

### Verify DDS is working
```bash
echo $RMW_IMPLEMENTATION
echo $CYCLONEDDS_URI
```

### Check if another program is controlling
```bash
ps aux | grep -i d1
```

## Development Workflow

1. **Test in simulation first** (`../simulation/`)
2. **Get working joint angles from simulation**
3. **Update pick_and_place.cpp with those angles**
4. **Rebuild the executable**
5. **Move robot to safe position**
6. **Test one movement at a time**
7. **Run full pick and place routine**

## Integration with Simulation

Use the same joint angles tested in simulation:

**From simulation** (`pick_place_simulation.py`):
```python
pick_position = {
    'Joint1': 0.0,
    'Joint2': -1.2,
    'Joint3': 1.5,
    ...
}
```

**To implementation** (`pick_and_place.cpp`):
```cpp
std::vector<float> pick_position = {0.0f, -1.2f, 1.5f, ...};
```

## Troubleshooting

### Robot not responding
- Check network connection
- Verify DDS configuration
- Restart robot controller

### Unexpected movements
- Verify angle units (radians)
- Check joint limits
- Review trajectory planning

### Build errors
```bash
cd ../../d1_sdk/build
rm -rf *
cmake ..
make
```

## Next Steps

1. Test basic movements
2. Calibrate pick position
3. Calibrate place position
4. Test gripper operation
5. Add error handling
6. Integrate camera feedback
