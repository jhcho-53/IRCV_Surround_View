#!/usr/bin/env python3
"""Coverage map: which camera sees each ground pixel."""
import _boot
import cv2, numpy as np, config
from core import bev

# Colors for each camera (same as bev_surround_ground.py)
COL = [
    (80,160,255),    # front_left
    (255,160,80),    # front_right
    (80,255,160),    # side_left_1
    (160,80,255),    # side_left_2
    (80,220,220),    # side_right_1
    (220,80,220),    # side_right_2
    (200,200,80),    # rear
]

# Load BEV renderer (reuses core loaders)
renderer = bev.load_default()

# Color each pixel by which camera has the best view
out = np.zeros((renderer.H, renderer.W, 3), np.uint8)
for i in range(len(renderer.cams)):
    sel = renderer.who == i
    out[sel] = COL[i]

# Black for blind zones (no camera coverage)
nocov = renderer.who < 0
out[nocov] = [0, 0, 0]

# Draw the Ioniq-5 footprint rectangle (from config)
# Vehicle frame origin = rear-axle centre on ground
# X fwd, Y left
def px_of(X, Y):
    return int((renderer.Ymax - Y) * renderer.ppm), int((renderer.Xmax - X) * renderer.ppm)

# Draw footprint rectangle
cv2.rectangle(out, px_of(config.CAR_XREAR, config.CAR_HALF_W),
              px_of(config.CAR_XFRONT, -config.CAR_HALF_W),
              (255, 255, 255), 2)

# Add statistics and label
nocov_pct = 100 * nocov.astype(np.float32).mean()
cv2.putText(out, f"coverage: colour=source camera, BLACK=no camera ({nocov_pct:.1f}%)",
            (8, renderer.H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

# Save result
output_path = f"{config.RESULTS_DIR}/coverage_map.jpg"
cv2.imwrite(output_path, out)
print(f"wrote {output_path}  {renderer.W}x{renderer.H}  (ppm={renderer.ppm})")
