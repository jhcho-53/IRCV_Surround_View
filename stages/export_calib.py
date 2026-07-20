#!/usr/bin/env python3
"""Export per-camera intrinsics + extrinsics (in the VEHICLE frame) to
config.EXPORT_DIR/<camera>.yml  (OpenCV FileStorage readable)."""
import _boot
import os, cv2, numpy as np
import config
from core.cameras import load_cameras, load_center
from core import geometry

os.makedirs(config.EXPORT_DIR, exist_ok=True)

cams = load_cameras()
vf = geometry.load_vehicle()

for cam in cams:
    T_veh_cam = geometry.T_vehicle_from_camera(cam, vf)            # camera -> VEHICLE
    pos = T_veh_cam[:3,3]; rvec,_ = cv2.Rodrigues(T_veh_cam[:3,:3])
    f = cv2.FileStorage(f"{config.EXPORT_DIR}/{cam.name}.yml", cv2.FILE_STORAGE_WRITE)
    f.write("camera_name", cam.name)
    f.write("image_width", cam.width); f.write("image_height", cam.height)
    f.write("distortion_model", "fisheye_kannala_brandt (OpenCV cv::fisheye, 4 coeffs k1..k4)")
    f.write("camera_matrix", cam.K)
    f.write("distortion_coefficients", cam.D.reshape(1,-1))
    f.write("vehicle_frame", "X=forward, Y=left, Z=up; origin = rear-axle centre projected on the ground (Z=0)")
    f.write("T_vehicle_from_camera", T_veh_cam)                    # 4x4: X_vehicle = T * X_camera
    f.write("position_xyz_m", pos.reshape(1,3))
    f.write("rotation_rodrigues", rvec.reshape(1,3))
    f.release()
    print(f"wrote {config.EXPORT_DIR}/{cam.name}.yml   pos=({pos[0]:+.2f},{pos[1]:+.2f},{pos[2]:+.2f})")

# center: intrinsics only (was never seen with the board in the extrinsic run)
cc = load_center()
f = cv2.FileStorage(f"{config.EXPORT_DIR}/center.yml", cv2.FILE_STORAGE_WRITE)
f.write("camera_name", "center")
f.write("image_width", cc.width); f.write("image_height", cc.height)
f.write("distortion_model", "fisheye_kannala_brandt (OpenCV cv::fisheye, 4 coeffs k1..k4)")
f.write("camera_matrix", cc.K)
f.write("distortion_coefficients", cc.D.reshape(1,-1))
f.write("extrinsic_status", "NOT CALIBRATED - this camera never observed the board in the extrinsic recording")
f.release()
print(f"wrote {config.EXPORT_DIR}/center.yml   (intrinsics only, no extrinsic)")
