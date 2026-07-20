"""Charuco board definition, detection, and fisheye PnP."""
import numpy as np, cv2, config

_DICT = getattr(cv2.aruco, config.ARUCO_DICT)

def make_board():
    d = cv2.aruco.getPredefinedDictionary(_DICT)
    return cv2.aruco.CharucoBoard((config.SQUARES_X, config.SQUARES_Y),
                                  config.SQUARE_LEN_M, config.MARKER_LEN_M, d)

BOARD = make_board()
DETECTOR = cv2.aruco.CharucoDetector(BOARD)
_CHESS = BOARD.getChessboardCorners().astype(np.float64)

def detect(image):
    cc, ids, _, _ = DETECTOR.detectBoard(image)
    return cc, ids

def pnp_fisheye(corners, ids, cam, min_corners=12):
    if ids is None or len(ids) < min_corners:
        return None
    obj = _CHESS[ids.reshape(-1)]
    und = cv2.fisheye.undistortPoints(corners.reshape(-1,1,2).astype(np.float64), cam.K, cam.D)
    ok, rvec, tvec = cv2.solvePnP(obj, und.reshape(-1,1,2), np.eye(3), None)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.reshape(3)

def board_normal_ref(R_board_cam, cam):
    return cam.pose[:3,:3] @ (R_board_cam @ np.array([0,0,1.0]))
