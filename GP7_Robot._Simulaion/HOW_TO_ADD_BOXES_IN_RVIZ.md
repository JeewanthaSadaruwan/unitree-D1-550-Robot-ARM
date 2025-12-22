# How to Add Boxes to RViz - Step by Step

## The simulation is running, but you need to add the Marker displays in RViz to see the boxes.

### Steps to Add the Boxes:

## 1. Add Target Box (Red Box)

1. In the RViz window, look at the left panel under "Displays"
2. Click the **"Add"** button at the bottom of the Displays panel
3. In the popup window, select **"By display type"** tab
4. Scroll down and select **"Marker"**
5. Click **"OK"**
6. A new "Marker" item will appear in the Displays list
7. Click on the small arrow next to "Marker" to expand it
8. Find the **"Topic"** field
9. Click on the topic dropdown and select **/box_a**
10. You should now see a **RED BOX** appear in the 3D view!

## 2. Add Gripped Box (Blue Box)

1. Click the **"Add"** button again
2. Select **"Marker"** again
3. Click **"OK"**
4. Expand the new Marker display
5. Change its **"Topic"** to **/box_b**
6. You should now see a **BLUE BOX** attached to the robot's end effector!

## 3. Use the Joint State Publisher GUI

1. Look for a separate window called **"Joint State Publisher"**
2. It should have 8 sliders (joint_1 through joint_8)
3. Move any slider and watch:
   - The robot moves
   - The **BLUE BOX** follows the end effector
   - The **RED BOX** stays in place

## Troubleshooting:

### If boxes don't appear:
```bash
# Check if topics are being published
ros2 topic list | grep box

# Should show:
# /box_a
# /box_b

# Check if messages are being sent
ros2 topic echo /box_a --once
```

### If Joint State Publisher GUI is missing:
- Look for it in your taskbar/window switcher
- Or restart the simulation: `./run_simulation.sh`

## Quick Visual Guide:

```
RViz Window Layout:
┌──────────────────────────────────────┐
│ Displays Panel (Left)                │
│ ├─ Global Options                    │
│ ├─ Grid                              │
│ ├─ RobotModel    ← Robot visible    │
│ ├─ TF                                │
│ ├─ Marker        ← Add this for box_a│
│ └─ Marker        ← Add this for box_b│
│                                       │
│ [Add] [Duplicate] [Remove] [Rename]  │
└──────────────────────────────────────┘
```

## What Each Box Represents:

- **RED BOX (box_a)**: Target location for pick & place
  - Position: (0.30, 0.40, 0.10) meters from base
  - Size: 30cm × 30cm × 20cm
  - Frame: base_link (static)

- **BLUE BOX (box_b)**: Object "held" by gripper
  - Position: 5cm in front of end effector
  - Size: 10cm × 10cm × 10cm
  - Frame: link_8 (moves with robot)

## Try This:

1. Move joint sliders to position the robot
2. Try to move the blue box (end effector) to touch the red box
3. This simulates a pick-and-place operation!

Enjoy! 🤖📦
