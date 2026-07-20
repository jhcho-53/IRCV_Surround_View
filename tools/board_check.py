#!/usr/bin/env python3
"""Board check: detect Charuco board and report horizontality across images."""
import _boot
import cv2, numpy as np, glob, os, sys, config
from core import cameras as camlib, board as boardlib, geometry

if len(sys.argv) < 3:
    print("Usage: board_check.py <camera_name> <folder>")
    sys.exit(1)

camera_name = sys.argv[1]
folder = sys.argv[2]

# Find camera index
try:
    cam_idx = config.ORDER.index(camera_name)
except ValueError:
    print(f"Camera '{camera_name}' not found in ORDER: {config.ORDER}")
    sys.exit(1)

# Load camera
cams = camlib.load_cameras()
cam = cams[cam_idx]

# Load ground plane normal
ground = geometry.load_ground()
ground_normal = ground["n"]

# Collect all frames with detected board
angles = []
detections = 0

for img_path in sorted(glob.glob(f"{folder}/*.jpg")):
    img = cv2.imread(img_path)
    if img is None:
        continue

    # Detect board
    cc, ids = boardlib.detect(img)

    # Check if enough corners detected
    if ids is None or len(ids) < 8:
        continue

    detections += 1

    # Solve PnP for board pose
    Rt = boardlib.pnp_fisheye(cc, ids, cam, min_corners=8)
    if Rt is None:
        continue

    R_board_cam, t_board_cam = Rt

    # Get board normal in reference frame
    board_normal = boardlib.board_normal_ref(R_board_cam, cam)

    # Compute angle between board normal and ground normal
    # The angle should be close to 0 if the board is lying on the ground
    cos_angle = np.clip(abs(board_normal @ ground_normal), -1, 1)
    angle_deg = np.degrees(np.arccos(cos_angle))

    angles.append(angle_deg)

angles = np.array(angles)

# Report statistics
print(f"{camera_name}: board detected {detections} frames")
if len(angles) > 0:
    print(f"angle(board-normal vs ground-normal): median={np.median(angles):.1f} deg  (0=flat on ground, 90=vertical)")
    print(f"  min={angles.min():.1f} deg, max={angles.max():.1f} deg, mean={angles.mean():.1f} deg")
    # Show a few example angles
    for angle in angles[:5]:
        print(f"    {angle:.1f} deg")
else:
    print("No board detections with sufficient corners")
