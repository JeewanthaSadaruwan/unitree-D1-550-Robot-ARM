# GP7 Robot Box Grab & Place Simulation - WORKING GUIDE

## Quick Start - 2 Terminals Needed

### Terminal 1: Launch RViz and Robot
```bash
cd "/media/rusula-oshadha/New Volume/Oshadha/University of Moratuwa/Fifth Semester/Robotics/GP7_Robot._Simulaion"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch gp7_robot display.launch.py
```

**What you'll see:**
- Joint State Publisher GUI window (with sliders)
- RViz window (3D visualization)
- Red target box visible
- Robot visible when you move sliders

### Terminal 2: Run Automated Movement (Optional)
```bash
cd "/media/rusula-oshadha/New Volume/Oshadha/University of Moratuwa/Fifth Semester/Robotics/GP7_Robot._Simulaion"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Close the Joint State Publisher GUI window first!
# Then run:
python3 demo_movement.py
```

**Important:** Close the Joint State Publisher GUI before running the automated movement!

---

## What's Working

✅ **Red Box (Target)** - Visible at position (0.3, 0.4, 0.1)m  
✅ **Blue Box (Gripped)** - Attached to link_8 (end effector)  
✅ **Robot Model** - 8-link GP7 robot  
✅ **Manual Control** - Use Joint State Publisher GUI sliders  
✅ **Automated Movement** - Smooth pick-and-place sequence  

---

## How to See the Blue Box in RViz

The blue box should appear automatically, but if not:

1. In RViz, look at the left panel "Displays"
2. You should see "Box_A_Target" and "Box_B_Gripped" in the list
3. Make sure both have checkmarks (enabled)
4. If you don't see them, click "Add" button:
   - Select "Marker"
   - Click OK
   - Set Topic to `/box_b`
   - The blue box will appear!

---

## Mode 1: Manual Control (Using GUI)

1. Launch with the command from Terminal 1
2. Use the **Joint State Publisher GUI** sliders to move each joint
3. Watch:
   - Robot moves as you adjust sliders
   - Blue box follows the end effector (link_8)
   - Red box stays in place

---

## Mode 2: Automated Movement

1. Launch with Terminal 1 command
2. **IMPORTANT:** Close the Joint State Publisher GUI window
3. Run Terminal 2 commands
4. Watch the robot perform automatic pick-and-place!

The robot will move through 9 positions in a smooth sequence:
- Home → Reach → Approach → Grab → Lift → Move → Place → Release → Home

---

## Troubleshooting

### Robot not visible or jumbled
**Cause:** No joint states being published  
**Solution:** Make sure Joint State Publisher GUI is running OR run demo_movement.py

### Robot jumps back to starting position
**Cause:** Joint State Publisher GUI and demo_movement.py both running (conflict)  
**Solution:** Close Joint State Publisher GUI before running demo_movement.py

### Blue box not visible
**Check:**
```bash
ros2 topic echo /box_b --once
```
If you see data, the box is publishing. Add Marker display in RViz for `/box_b`

### Red box not visible
**Check:**
```bash
ros2 topic echo /box_a --once
```
If you see data, the box is publishing. Should be visible automatically.

### RViz warnings about link_8
**Status:** Normal! These are info messages, not errors. The blue box is publishing fine.

---

## Files Created

- `gp7_boxes/` - Complete Python package for box visualization
- `demo_movement.py` - Automated smooth movement script
- `launch/display_auto.launch.py` - Launch without GUI (for automated mode)
- `config/urdf.rviz` - Updated with box markers
- `build_simulation.sh` - Build script
- `run_simulation.sh` - Launch with GUI
- `launch_all.sh` - Alternative launcher
- Documentation files

---

## Understanding the Simulation

### Red Box (box_a)
- **Frame:** base_link (world frame)
- **Position:** 30cm forward, 40cm right, 10cm up
- **Purpose:** Target location for pick-and-place
- **Behavior:** Static (never moves)

### Blue Box (box_b)
- **Frame:** link_8 (end effector)
- **Position:** 5cm in front of link_8
- **Purpose:** Simulates object held by gripper
- **Behavior:** Moves with robot's end effector

### Why This Works
The blue box is attached to link_8's coordinate frame. When the robot moves, link_8 moves, and the blue box automatically follows because it's defined relative to link_8.

---

## Next Steps

Want to customize?

### Change Box Positions
Edit: `gp7_boxes/gp7_boxes/scripts/boxes.py`
- Lines 46-48: Red box position
- Lines 84-86: Blue box position (relative to link_8)

### Change Movement Sequence
Edit: `demo_movement.py`
- Lines 35-57: Define your own joint angle sequences

After changes:
```bash
cd "/media/rusula-oshadha/New Volume/Oshadha/University of Moratuwa/Fifth Semester/Robotics/GP7_Robot._Simulaion"
./build_simulation.sh
```

---

## Summary

**Working Command (easiest):**
```bash
ros2 launch gp7_robot display.launch.py
```

This gives you:
- RViz with robot and boxes
- GUI to control robot manually
- Everything you need!

**For automated demo:**
- Launch as above
- Close GUI window
- Run `python3 demo_movement.py`

That's it! 🎉
