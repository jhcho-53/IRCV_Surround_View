"""Camera calibration I/O + fisheye projection (replaces the 11x duplicated loader)."""
from dataclasses import dataclass
import os, sys
import numpy as np, cv2, config

@dataclass
class Camera:
    name: str
    K: np.ndarray
    D: np.ndarray            # fisheye 4 coeffs
    pose: np.ndarray         # 4x4 camera -> reference(front_left)
    width: int
    height: int

def _read(node):
    K = node.getNode("camera_matrix").mat()
    D = node.getNode("distortion_vector").mat().reshape(-1)[:4].astype(np.float64)
    w = int(node.getNode("img_width").real()); h = int(node.getNode("img_height").real())
    return K, D, w, h

def _resolve(primary, golden, label):
    """Use the external calibration file when present, else the bundled golden copy."""
    if os.path.exists(primary):
        return primary
    if golden and os.path.exists(golden):
        print(f"[cameras] {label} not found at {primary} -> using bundled golden copy",
              file=sys.stderr)
        return golden
    return primary   # let cv2.FileStorage raise with the original path

def load_cameras(yaml_path=None, order=config.ORDER):
    yaml_path = _resolve(yaml_path or config.CALIB_YAML, config.CALIB_YAML_GOLDEN, "CALIB_YAML")
    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
    cams = []
    for i, name in enumerate(order):
        nd = fs.getNode(f"camera_{i}")
        K, D, w, h = _read(nd)
        pose = nd.getNode("camera_pose_matrix").mat()
        cams.append(Camera(name, K, D, pose, w, h))
    fs.release()
    return cams

def load_center(yaml_path=None):
    yaml_path = _resolve(yaml_path or config.INTRINSICS_YAML, config.INTRINSICS_YAML_GOLDEN, "INTRINSICS_YAML")
    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
    nd = fs.getNode("camera_7"); K, D, w, h = _read(nd); fs.release()
    return Camera("center", K, D, None, w, h)

def project(points_ref, cam):
    """points_ref: (...,3) in reference frame -> (px (...,2), valid (...) bool)."""
    P = np.asarray(points_ref, np.float64)
    shp = P.shape[:-1]
    R = cam.pose[:3,:3]; t = cam.pose[:3,3]
    Pc = (P.reshape(-1,3) - t) @ R                       # ref -> cam
    z = Pc[:,2]
    px, _ = cv2.fisheye.projectPoints(Pc.reshape(-1,1,3), np.zeros(3), np.zeros(3), cam.K, cam.D)
    px = px.reshape(-1,2)
    valid = (z > 0.2) & (px[:,0] >= 0) & (px[:,0] < cam.width) & (px[:,1] >= 0) & (px[:,1] < cam.height)
    return px.reshape(*shp,2), valid.reshape(*shp)
