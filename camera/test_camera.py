#!/usr/bin/env python3
"""
USB Camera Test Script
Tests available cameras and opens the first working one
"""

import cv2
import sys
import os

def find_available_cameras(max_cameras=10):
    """
    Test which camera devices are actually working
    Returns list of available camera indices
    """
    available_cameras = []
    print("Scanning for available cameras...")
    print("-" * 50)
    
    for i in range(max_cameras):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                # Get camera properties
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = cap.get(cv2.CAP_PROP_FPS)
                
                print(f"✓ Camera {i}: Working")
                print(f"  Device: /dev/video{i}")
                print(f"  Resolution: {width}x{height}")
                print(f"  FPS: {fps}")
                print()
                
                available_cameras.append(i)
            cap.release()
    
    return available_cameras

def open_camera(camera_id=0):
    """
    Open camera feed and display in window
    """
    print(f"\nOpening camera {camera_id}...")
    print("Press 'q' to quit")
    print("Press 's' to save a snapshot")
    print("-" * 50)
    
    cap = cv2.VideoCapture(camera_id)
    
    if not cap.isOpened():
        print(f"ERROR: Cannot open camera {camera_id}")
        return False
    
    # Set camera properties (optional - adjust as needed)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    snapshot_count = 0
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            print("ERROR: Can't receive frame. Exiting...")
            break
        
        # Display frame info
        height, width = frame.shape[:2]
        info_text = f"Camera {camera_id} | {width}x{height} | Press 'q' to quit, 's' to save"
        cv2.putText(frame, info_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Show the frame
        cv2.imshow(f'USB Camera {camera_id}', frame)
        
        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            print("\nQuitting...")
            break
        elif key == ord('s'):
            snapshot_count += 1
            filename = f"camera_{camera_id}_snapshot_{snapshot_count}.jpg"
            cv2.imwrite(filename, frame)
            print(f"Saved: {filename}")
    
    cap.release()
    cv2.destroyAllWindows()
    return True

def main():
    print("=" * 50)
    print("USB Camera Detection and Viewer")
    print("=" * 50)
    print()
    
    # Find available cameras
    available_cameras = find_available_cameras()
    
    if not available_cameras:
        print("ERROR: No working cameras found!")
        print("\nTroubleshooting:")
        print("1. Check if camera is plugged in")
        print("2. Check permissions: sudo usermod -a -G video $USER")
        print("3. Try: ls -la /dev/video*")
        sys.exit(1)
    
    print(f"Found {len(available_cameras)} working camera(s)")
    print()
    
    # Use first available camera or specified camera
    if len(sys.argv) > 1:
        try:
            camera_id = int(sys.argv[1])
            if camera_id not in available_cameras:
                print(f"WARNING: Camera {camera_id} not in available list")
                print(f"Available cameras: {available_cameras}")
                response = input(f"Try anyway? (y/n): ")
                if response.lower() != 'y':
                    sys.exit(0)
        except ValueError:
            print("ERROR: Camera ID must be a number")
            sys.exit(1)
    else:
        camera_id = available_cameras[0]
        print(f"Using camera {camera_id} (first available)")
        if len(available_cameras) > 1:
            print(f"To use a different camera, run: python3 {sys.argv[0]} <camera_id>")
            print(f"Available cameras: {available_cameras}")
    
    # Open the camera
    open_camera(camera_id)

if __name__ == "__main__":
    main()
