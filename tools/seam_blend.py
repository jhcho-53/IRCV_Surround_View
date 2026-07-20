#!/usr/bin/env python3
"""Seam blend: overlay 2 cameras on ground plane to check extrinsic alignment."""
import _boot
import cv2, numpy as np, sys, config
from core import cameras as camlib, geometry

if len(sys.argv) < 4:
    print("Usage: seam_blend.py <camA> <camB> <setidx>")
    sys.exit(1)

camA, camB, setidx = sys.argv[1], sys.argv[2], int(sys.argv[3])

# Map camera names to indices
try:
    ia = config.ORDER.index(camA)
    ib = config.ORDER.index(camB)
except ValueError as e:
    print(f"Camera name not found: {e}")
    sys.exit(1)

# Load cameras and ground plane
cams = camlib.load_cameras()
ground = geometry.load_ground()
n = ground["n"]
c = ground["c"]

# Ensure normal points up (cameras above ground)
centers = np.array([cam.pose[:3,3] for cam in cams])
if ((centers - c) @ n).mean() < 0:
    n = -n

# Compute ground-plane coordinates (local 2D frame on ground)
# Offset ground plane to ensure it's below camera centers
off = (centers @ n).min() - 0.5
P0 = n * off

# Build orthonormal basis on ground plane
ex = centers[1] - centers[0]
ex -= (ex @ n) * n
ex /= np.linalg.norm(ex)
ey = np.cross(n, ex)

# Project camera positions into ground-plane coordinates
cu = centers[[ia, ib]]
a = (cu - P0) @ ex
b = (cu - P0) @ ey
amin, amax = a.min() - 3.5, a.max() + 3.5
bmin, bmax = b.min() - 3.5, b.max() + 3.5

# BEV projection parameters
ppm = 95
W = int((amax - amin) * ppm)
H = int((bmax - bmin) * ppm)

# Create grid of ground points in reference frame
GX, GY = np.meshgrid(np.arange(W)/ppm + amin, np.arange(H)/ppm + bmin)
Pref = P0[None,None] + GX[...,None] * ex[None,None] + GY[...,None] * ey[None,None]

# Composite the two camera images via averaging
acc = np.zeros((H, W, 3), np.float32)
cnt = np.zeros((H, W), np.float32)

for i in (ia, ib):
    cam = cams[i]
    R = cam.pose[:3,:3]
    t = cam.pose[:3,3]

    # Project ground points into camera frame
    Pc = (Pref - t) @ R
    z = Pc[..., 2]
    valid = z > 0.2

    # Fisheye projection
    px, _ = cv2.fisheye.projectPoints(Pc.reshape(-1,1,3).astype(np.float64),
                                      np.zeros(3), np.zeros(3), cam.K, cam.D)
    px = px.reshape(H, W, 2)
    u = px[..., 0]
    v = px[..., 1]

    # Load image
    im = cv2.imread(f"{config.FRAMES_DIR}/camera_{config.ORDER[i]}/{setidx:04d}.jpg")
    if im is None:
        print(f"Warning: could not load image for {config.ORDER[i]} frame {setidx:04d}")
        continue

    hh, ww = im.shape[:2]
    inb = valid & (u >= 0) & (u < ww) & (v >= 0) & (v < hh)

    # Accumulate
    ui = np.clip(u, 0, ww-1).astype(np.int32)
    vi = np.clip(v, 0, hh-1).astype(np.int32)
    s = im[vi, ui].astype(np.float32)
    acc[inb] += s[inb]
    cnt[inb] += 1

# Average and convert to uint8
out = np.where(cnt[...,None] > 0, acc / np.maximum(cnt[...,None], 1), 0).astype(np.uint8)

# Add label
overlap_px = (cnt >= 2).sum()
cv2.putText(out, f"{camA} + {camB}  set {setidx}  overlap={int(overlap_px)}px  (floor lines single=extrinsic OK)",
            (8, H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

# Save result
output_path = f"{config.RESULTS_DIR}/seam_{camA}_{camB}_{setidx}.jpg"
cv2.imwrite(output_path, out)
print(f"wrote {output_path}")
