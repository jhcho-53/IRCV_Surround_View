"""Ground-plane fit (floor board) and vehicle-frame fit (measured horizontal positions)."""
import glob, numpy as np, cv2, config
from core import board

def fit_ground_plane(cams, floor_dir=config.FLOOR_BOARD_DIR):
    chess = board.BOARD.getChessboardCorners().astype(np.float64)
    pts = []
    for folder, idx in [("Cam_001", 2), ("Cam_002", 3)]:      # side_left_1=cam2, side_left_2=cam3
        cam = cams[idx]
        for f in sorted(glob.glob(f"{floor_dir}/{folder}/*.jpg")):
            cc, ids = board.detect(cv2.imread(f))
            Rt = board.pnp_fisheye(cc, ids, cam, min_corners=15)
            if Rt is None: continue
            R, t = Rt
            Pc = (R @ chess[ids.reshape(-1)].T).T + t          # board corners in cam frame
            Pref = (cam.pose[:3,:3] @ Pc.T).T + cam.pose[:3,3]
            pts.append(Pref)
    pts = np.vstack(pts); c = pts.mean(0)
    _, _, Vt = np.linalg.svd(pts - c); n = Vt[-1] / np.linalg.norm(Vt[-1])
    if n[1] < 0: n = -n
    return dict(n=n, c=c)

def fit_vehicle_frame(cams, ground, measured_xy=config.MEASURED_XY):
    n, c = ground["n"], ground["c"]
    P = np.array([cm.pose[:3,3] for cm in cams])
    if ((P - c) @ n).mean() < 0: n = -n
    e1 = np.array([1.0,0,0]); e1 -= (e1 @ n) * n
    if np.linalg.norm(e1) < 1e-6: e1 = np.array([0,1.0,0]) - (np.array([0,1.0,0]) @ n) * n
    e1 /= np.linalg.norm(e1); e2 = np.cross(n, e1)
    Pp = P - ((P - c) @ n)[:,None] * n
    Q = np.stack([(Pp - c) @ e1, (Pp - c) @ e2], 1)
    M = np.array([measured_xy[cm.name] for cm in cams])
    mc, qc = M.mean(0), Q.mean(0); H = (M - mc).T @ (Q - qc)
    U, S, Vt = np.linalg.svd(H); d = np.sign(np.linalg.det(Vt.T @ U.T))
    R2 = Vt.T @ np.diag([1, d]) @ U.T; t2 = qc - R2 @ mc
    pred = (R2 @ M.T).T + t2; err = np.linalg.norm(pred - Q, axis=1)
    Xv = R2[0,0]*e1 + R2[1,0]*e2; Xv /= np.linalg.norm(Xv)
    Yv = np.cross(n, Xv); origin = c + t2[0]*e1 + t2[1]*e2
    R_ref_veh = np.column_stack([Xv, Yv, n])
    return dict(R_ref_veh=R_ref_veh, origin_ref=origin,
                rms_cm=float(np.sqrt((err**2).mean())*100),
                residuals={cm.name: float(err[i]*100) for i,cm in enumerate(cams)})

def save_ground(g, path=config.GROUND_NPZ):  np.savez(path, n=g["n"], c=g["c"])
def load_ground(path=config.GROUND_NPZ):
    z = np.load(path); return dict(n=z["n"], c=z["c"])
def save_vehicle(vf, path=config.VEHICLE_NPZ):
    np.savez(path, R_ref_veh=vf["R_ref_veh"], t_ref_veh=vf["origin_ref"])
def load_vehicle(path=config.VEHICLE_NPZ):
    z = np.load(path); return dict(R_ref_veh=z["R_ref_veh"], origin_ref=z["t_ref_veh"])

def to_vehicle(points_ref, vf):
    P = np.asarray(points_ref, np.float64); shp = P.shape[:-1]
    out = (vf["R_ref_veh"].T @ (P.reshape(-1,3) - vf["origin_ref"]).T).T
    return out.reshape(*shp,3)

def T_vehicle_from_camera(cam, vf):
    T_veh_ref = np.eye(4); T_veh_ref[:3,:3] = vf["R_ref_veh"].T
    T_veh_ref[:3,3] = -vf["R_ref_veh"].T @ vf["origin_ref"]
    return T_veh_ref @ cam.pose
