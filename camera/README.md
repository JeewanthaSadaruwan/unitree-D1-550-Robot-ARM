# Camera Scripts for Unitree D1-550 Robot Arm

This folder contains scripts for working with USB cameras connected to the robot system.

## Prerequisites

Install OpenCV for Python:
```bash
pip install opencv-python
# or
sudo apt-get install python3-opencv
```

## Scripts

### test_camera.py

Test and view USB camera feed.

**Features:**
- Auto-detects all available cameras
- Shows camera resolution and FPS
- Live camera preview
- Save snapshots with 's' key
- Easy to use interface

**Usage:**

```bash
# Make executable
chmod +x test_camera.py

# Run with auto-detection (uses first available camera)
python3 test_camera.py

# Or specify camera ID
python3 test_camera.py 0    # Use /dev/video0
python3 test_camera.py 4    # Use /dev/video4
```

**Keyboard Controls:**
- `q` - Quit
- `s` - Save snapshot (saved as camera_X_snapshot_N.jpg)

## Troubleshooting

### No cameras detected

1. **Check if camera is plugged in:**
   ```bash
   ls -la /dev/video*
   ```

2. **Check permissions:**
   ```bash
   # Add your user to video group
   sudo usermod -a -G video $USER
   # Log out and log back in for changes to take effect
   ```

3. **Test with v4l2:**
   ```bash
   sudo apt-get install v4l-utils
   v4l2-ctl --list-devices
   ```

### Camera opens but shows black screen

- Camera might be in use by another application
- Try different camera indices (0, 1, 2, etc.)
- Some /dev/video devices are metadata channels, not actual cameras

### Poor performance

- Reduce resolution in script (change CAP_PROP_FRAME_WIDTH/HEIGHT)
- Lower FPS setting
- Check CPU usage

## Device Information

Connected cameras are typically at:
- `/dev/video0` - Primary camera
- `/dev/video1` - Secondary camera or metadata
- `/dev/video2-9` - Additional cameras or streams

Some cameras create multiple devices (raw, metadata, etc.). The script automatically tests which ones actually provide video frames.

## Integration with Robot Arm

For vision-based pick and place:
1. Use this script to verify camera is working
2. Integrate OpenCV with robot control code
3. Implement object detection/tracking
4. Convert pixel coordinates to robot workspace coordinates
5. Plan trajectories to detected objects

## Next Steps

Potential enhancements:
- Object detection (YOLO, MediaPipe, etc.)
- ArUco marker detection for calibration
- Hand-eye calibration
- Real-time object tracking
- Save video recordings
- Multi-camera support
