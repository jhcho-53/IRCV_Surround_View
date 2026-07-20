import _env, glob, cv2, config
from core.cameras import load_cameras
from core import board
_env.require(f"{config.FLOOR_BOARD_DIR}/Cam_001", "floor-board recording")
cams = load_cameras()
# side_left_1 is Cam_001 in the floor recording, calibrated camera index 2
files = sorted(glob.glob(f"{config.FLOOR_BOARD_DIR}/Cam_001/*.jpg"))
if len(files) <= 70:
    _env.skip(f"floor-board recording has too few frames ({len(files)})")
f = files[70]
img = cv2.imread(f)
cc, ids = board.detect(img)
assert ids is not None and len(ids) >= 15, None if ids is None else len(ids)
Rt = board.pnp_fisheye(cc, ids, cams[2])
assert Rt is not None and Rt[0].shape == (3,3)
print("OK test_board", len(ids))
