import _env, os, subprocess, tempfile, cv2, numpy as np, config
# run the export into a temp dir by monkey-patching EXPORT_DIR
import core.geometry, core.cameras
from core import geometry, cameras
vf = geometry.load_vehicle(); cams = cameras.load_cameras()
for cam in cams:
    T = geometry.T_vehicle_from_camera(cam, vf)
    fs = cv2.FileStorage(f"{config.ARTIFACTS}/golden/calib/{cam.name}.yml", cv2.FILE_STORAGE_READ)
    Tg = fs.getNode("T_vehicle_from_camera").mat(); fs.release()
    assert np.allclose(T, Tg, atol=1e-6), (cam.name, np.abs(T-Tg).max())
print("OK export matches golden")
