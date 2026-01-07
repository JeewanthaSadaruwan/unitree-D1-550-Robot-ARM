# Pick and Place Project

## Overview
Complete pick and place implementation for the Unitree D1-550 robot arm.

## Project Structure

```
pick_and_place/
├── README.md                       # This file
├── simulation/                     # RViz simulation files
│   ├── run_simulation.sh          # ⭐ Launch RViz (quick start)
│   ├── pick_place_simulation.py   # Pick and place demo
│   ├── demo_movement.py           # Movement demonstrations
│   ├── inverse_kinematics_demo.py # IK solver testing
│   ├── run_demo.sh                # Run demo script
│   ├── build_simulation.sh        # Build script
│   ├── urdf.rviz                  # RViz configuration
│   ├── launch/                    # ROS2 launch files
│   ├── config/                    # Joint configurations
│   └── README.md                  # Simulation guide
└── implementation/                 # Physical robot code
    ├── pick_and_place.cpp         # Source code
    ├── pick_and_place             # Compiled executable
    └── README.md                  # Hardware guide
```

## Quick Start

### 1. Test in Simulation (⭐ Start Here)
```bash
cd simulation/
./run_simulation.sh
```

This launches RViz with:
- Robot visualization
- Joint control sliders
- TF frame display

### 2. Run Pick and Place Simulation
```bash
# In another terminal
cd simulation/
python3 pick_place_simulation.py
```

## Workflow

1. **Develop & test in simulation** (`simulation/`)
   - Visualize movements in RViz
   - Test pick and place positions
   - Validate trajectories

2. **Deploy to physical robot** (`implementation/`)
   - Use tested positions from simulation
   - Run on real hardware with d1_sdk
   - Monitor and adjust as needed

## Robot Specifications

- **Model:** Unitree D1-550
- **DOF:** 6 axis + gripper
- **Max Reach:** ~725 mm
- **URDF:** `../d1_550_description/urdf/d1_550_description.urdf`
- **SDK:** `../d1_sdk/`

## Dependencies

### Simulation
- ROS2 Humble
- RViz2
- Python 3

### Physical Robot
- d1_sdk (Unitree SDK)
- Cyclone DDS
- C++ compiler

## Next Steps

1. [ ] Test pick position in simulation
2. [ ] Test place position in simulation
3. [ ] Validate trajectory smoothness
4. [ ] Deploy to physical robot
5. [ ] Add camera integration
6. [ ] Implement object detection

## Related Files

- URDF: `../d1_550_description/urdf/`
- Launch files: `../d1_550_description/launch/`
- SDK source: `../d1_sdk/src/`
- SDK build: `../d1_sdk/build/`
