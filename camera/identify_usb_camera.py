#!/usr/bin/env python3
"""
Camera Visual Identifier
Opens each camera one by one so you can see which is which
"""

import cv2
import sys

def preview_camera(camera_id, camera_info):
    """
    Show a single camera preview
    """
    print(f"\n{'='*70}")
    print(f"Camera {camera_id} Preview - {camera_info}")
    print(f"{'='*70}")
    print("\nLook at the camera preview window!")
    print("Controls:")
    print("  SPACE - This is my USB camera (save and exit)")
    print("  'n' - Next camera")
    print("  'q' - Quit without selecting")
    print()
    
    cap = cv2.VideoCapture(camera_id)
    
    if not cap.isOpened():
        print(f"❌ Cannot open camera {camera_id}")
        return None
    
    # Set resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    selected = None
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            print("❌ Cannot read frame")
            break
        
        # Add large text overlay
        height, width = frame.shape[:2]
        
        # Add semi-transparent background for text
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (width, 120), (0, 0, 0), -1)
        frame = cv2.addWeighted(overlay, 0.6, frame, 0.4, 0)
        
        # Add text
        cv2.putText(frame, f"CAMERA {camera_id} ({camera_info})", 
                   (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
        cv2.putText(frame, "Is this your USB camera?", 
                   (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(frame, "SPACE=Yes | N=Next | Q=Quit", 
                   (10, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        cv2.imshow('Camera Identifier - Wave at the camera!', frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord(' '):  # Space bar
            selected = camera_id
            print(f"\n✓ Selected Camera {camera_id} as USB camera")
            break
        elif key == ord('n') or key == ord('N'):
            print(f"Skipping Camera {camera_id}...")
            selected = 'next'
            break
        elif key == ord('q') or key == ord('Q'):
            print("\nQuitting...")
            selected = 'quit'
            break
    
    cap.release()
    cv2.destroyAllWindows()
    return selected

def main():
    # Camera list from your detection
    cameras = [
        (0, '/dev/video0', '640x480'),
        (2, '/dev/video2', '640x360'),
        (6, '/dev/video6', '640x480'),
        (8, '/dev/video8', '640x480'),
    ]
    
    print("\n" + "="*70)
    print("Visual Camera Identifier")
    print("="*70)
    print("\nThis script will open each camera one by one.")
    print("Wave your hand or cover the camera to see which one responds.")
    print()
    print("📹 Detected cameras:")
    for cam_id, dev, res in cameras:
        print(f"   Camera {cam_id}: {dev} ({res})")
    print()
    
    input("Press ENTER to start testing cameras...")
    
    for cam_id, dev, res in cameras:
        result = preview_camera(cam_id, f"{dev} - {res}")
        
        if result == cam_id:
            # User selected this camera
            print("\n" + "="*70)
            print(f"✓ USB Camera identified: Camera {cam_id} ({dev})")
            print("="*70)
            print(f"\nTo use this camera in your scripts, use:")
            print(f"  camera_id = {cam_id}")
            print(f"  # or")
            print(f"  python3 test_camera.py {cam_id}")
            print()
            
            # Save to config file
            with open('camera_config.txt', 'w') as f:
                f.write(f"USB_CAMERA_ID={cam_id}\n")
                f.write(f"USB_CAMERA_DEVICE={dev}\n")
                f.write(f"USB_CAMERA_RESOLUTION={res}\n")
            print(f"✓ Saved configuration to camera_config.txt")
            break
        elif result == 'quit':
            print("\n✓ Exiting...")
            break
        elif result == 'next':
            continue
        else:
            print(f"⚠ Could not open camera {cam_id}, skipping...")
            continue
    else:
        print("\n⚠ No camera selected")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n✓ Interrupted by user")
        cv2.destroyAllWindows()
