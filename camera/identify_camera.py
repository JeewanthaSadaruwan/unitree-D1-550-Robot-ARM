#!/usr/bin/env python3
"""
USB Camera Identifier
Helps distinguish between built-in laptop camera and USB cameras
"""

import cv2
import sys
import subprocess

def get_camera_info():
    """
    Get detailed information about all video devices
    """
    print("Detecting video devices...")
    print("=" * 70)
    
    try:
        # Try to get device names using v4l2-ctl
        result = subprocess.run(['v4l2-ctl', '--list-devices'], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print("Device Information (from v4l2-ctl):")
            print(result.stdout)
            print("-" * 70)
    except (subprocess.TimeoutExpired, FileNotFoundError):
        print("Note: v4l2-ctl not available. Install with: sudo apt-get install v4l-utils")
        print("-" * 70)

def test_cameras_detailed(max_cameras=10):
    """
    Test cameras and provide detailed information
    """
    cameras_info = []
    
    for i in range(max_cameras):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                # Get camera properties
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = cap.get(cv2.CAP_PROP_FPS)
                backend = cap.getBackendName()
                
                info = {
                    'id': i,
                    'device': f'/dev/video{i}',
                    'width': width,
                    'height': height,
                    'fps': fps,
                    'backend': backend,
                    'working': True
                }
                cameras_info.append(info)
            cap.release()
    
    return cameras_info

def display_cameras(cameras_info):
    """
    Display camera information in a user-friendly way
    """
    if not cameras_info:
        print("\n❌ No working cameras found!")
        return
    
    print(f"\n✓ Found {len(cameras_info)} working camera(s):\n")
    
    for idx, cam in enumerate(cameras_info):
        print(f"[{idx}] Camera ID: {cam['id']} ({cam['device']})")
        print(f"    Resolution: {cam['width']}x{cam['height']}")
        print(f"    FPS: {cam['fps']}")
        print(f"    Backend: {cam['backend']}")
        print()

def select_camera(cameras_info):
    """
    Let user select which camera to open
    """
    if not cameras_info:
        return None
    
    if len(cameras_info) == 1:
        print(f"Only one camera found. Using Camera ID {cameras_info[0]['id']}")
        return cameras_info[0]['id']
    
    print("Multiple cameras detected!")
    print("\nTips for identifying USB camera:")
    print("  - USB cameras often have higher resolution than laptop webcams")
    print("  - Recently plugged USB cameras usually have higher device numbers")
    print("  - Laptop webcam is typically /dev/video0 or /dev/video1")
    print()
    
    while True:
        try:
            choice = input(f"Select camera [0-{len(cameras_info)-1}] or 'q' to quit: ").strip()
            
            if choice.lower() == 'q':
                return None
            
            choice_idx = int(choice)
            if 0 <= choice_idx < len(cameras_info):
                return cameras_info[choice_idx]['id']
            else:
                print(f"Invalid choice. Please enter 0-{len(cameras_info)-1}")
        except ValueError:
            print("Invalid input. Please enter a number.")
        except KeyboardInterrupt:
            print("\nCancelled by user")
            return None

def open_camera_preview(camera_id):
    """
    Open camera preview with controls
    """
    print(f"\n{'='*70}")
    print(f"Opening Camera {camera_id} Preview")
    print(f"{'='*70}")
    print("\nControls:")
    print("  'q' - Quit")
    print("  's' - Save snapshot")
    print("  'i' - Show camera info")
    print("  '+' - Increase brightness")
    print("  '-' - Decrease brightness")
    print()
    
    cap = cv2.VideoCapture(camera_id)
    
    if not cap.isOpened():
        print(f"❌ ERROR: Cannot open camera {camera_id}")
        return False
    
    # Try to set reasonable defaults
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    snapshot_count = 0
    show_info = True
    brightness = cap.get(cv2.CAP_PROP_BRIGHTNESS)
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            print("❌ ERROR: Can't receive frame")
            break
        
        # Display info overlay
        if show_info:
            height, width = frame.shape[:2]
            cv2.putText(frame, f"Camera {camera_id} | {width}x{height}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Press 'q' to quit | 's' to save | 'i' to toggle info", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        cv2.imshow(f'Camera {camera_id} - USB Camera Test', frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            print("\n✓ Closing camera...")
            break
        elif key == ord('s'):
            snapshot_count += 1
            filename = f"camera_{camera_id}_snapshot_{snapshot_count}.jpg"
            cv2.imwrite(filename, frame)
            print(f"✓ Saved: {filename}")
        elif key == ord('i'):
            show_info = not show_info
        elif key == ord('+'):
            brightness += 10
            cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness)
            print(f"Brightness: {brightness}")
        elif key == ord('-'):
            brightness -= 10
            cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness)
            print(f"Brightness: {brightness}")
    
    cap.release()
    cv2.destroyAllWindows()
    return True

def main():
    print("\n" + "="*70)
    print("USB Camera Identifier and Viewer")
    print("="*70 + "\n")
    
    # Get detailed camera info
    get_camera_info()
    
    # Test all cameras
    print("Testing cameras...")
    cameras_info = test_cameras_detailed()
    
    # Display findings
    display_cameras(cameras_info)
    
    if not cameras_info:
        print("\n💡 Troubleshooting:")
        print("  1. Check USB camera is plugged in")
        print("  2. Try: ls -la /dev/video*")
        print("  3. Check permissions: sudo usermod -a -G video $USER")
        print("  4. Install v4l-utils: sudo apt-get install v4l-utils")
        sys.exit(1)
    
    # Check for command line argument
    if len(sys.argv) > 1:
        try:
            camera_id = int(sys.argv[1])
        except ValueError:
            print("❌ ERROR: Camera ID must be a number")
            sys.exit(1)
    else:
        # Let user select
        camera_id = select_camera(cameras_info)
        if camera_id is None:
            print("\n✓ Exiting...")
            sys.exit(0)
    
    # Open selected camera
    open_camera_preview(camera_id)

if __name__ == "__main__":
    main()
