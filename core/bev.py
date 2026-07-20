"""Vehicle-frame ground BEV: precompute the ground<->camera pixel mapping once, then
composite each synchronized frame set (best-incidence per pixel) + inpaint blind zone."""
import numpy as np, cv2, config
from core import cameras as camlib, geometry

class BevRenderer:
    def __init__(self, cams, vf, ground, extent=config.BEV_EXTENT, ppm=config.BEV_PPM):
        self.cams = cams; self.vf = vf; self.ppm = ppm
        self.Xmin, self.Xmax = extent["Xmin"], extent["Xmax"]
        self.Ymin, self.Ymax = extent["Ymin"], extent["Ymax"]
        self.H = int((self.Xmax - self.Xmin) * ppm); self.W = int((self.Ymax - self.Ymin) * ppm)
        Xg = self.Xmax - np.arange(self.H)/ppm; Yg = self.Ymax - np.arange(self.W)/ppm
        YY, XX = np.meshgrid(Yg, Xg)
        Pveh = np.stack([XX, YY, np.zeros_like(XX)], -1)              # ground plane Z=0
        Pref = (vf["R_ref_veh"] @ Pveh.reshape(-1,3).T).T + vf["origin_ref"]
        Pref = Pref.reshape(self.H, self.W, 3)
        self.who = np.full((self.H,self.W), -1, np.int16)
        self.UI = []; self.VI = []; best = np.full((self.H,self.W), -1.0, np.float32)
        for i, cam in enumerate(cams):
            R = cam.pose[:3,:3]; t = cam.pose[:3,3]
            Pc = (Pref - t) @ R; z = Pc[...,2]
            score = z / (np.linalg.norm(Pc, axis=2) + 1e-9)
            px, _ = cv2.fisheye.projectPoints(Pc.reshape(-1,1,3).astype(np.float64),
                                              np.zeros(3), np.zeros(3), cam.K, cam.D)
            px = px.reshape(self.H,self.W,2); u = px[...,0]; v = px[...,1]
            inb = (z>0.2)&(u>=0)&(u<cam.width)&(v>=0)&(v<cam.height)
            self.UI.append(np.clip(u,0,cam.width-1).astype(np.int32))
            self.VI.append(np.clip(v,0,cam.height-1).astype(np.int32))
            sel = inb & (score > best); self.who[sel] = i; best[sel] = score[sel]
        self.sel = [(self.who == i) for i in range(len(cams))]
        nocov = (self.who < 0).astype(np.uint8)
        self.nocov = cv2.morphologyEx(nocov, cv2.MORPH_CLOSE, np.ones((9,9), np.uint8))

    def px_of(self, X, Y):
        return int((self.Ymax - Y)*self.ppm), int((self.Xmax - X)*self.ppm)

    def render(self, images, inpaint=True):
        out = np.zeros((self.H,self.W,3), np.uint8)
        for i in range(len(self.cams)):
            im = images[i]
            if im is None: continue
            m = self.sel[i]; out[m] = im[self.VI[i][m], self.UI[i][m]]
        if inpaint:
            out = cv2.inpaint(out, self.nocov, 7, cv2.INPAINT_TELEA)
        return out

def load_default():
    return BevRenderer(camlib.load_cameras(), geometry.load_vehicle(), geometry.load_ground())
