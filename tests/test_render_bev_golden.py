import _env, subprocess, sys, cv2, numpy as np, config, tempfile, os
_env.require(f"{config.FRAMES_DIR}/camera_rear/0261.jpg", "sync frames (frame 0261)")
out = tempfile.mktemp(suffix=".jpg", dir=config.ARTIFACTS)
subprocess.check_call([sys.executable,"stages/render_bev.py","261","--out",out], cwd=config.PIPELINE)
new = cv2.imread(out); gold = cv2.imread(f"{config.ARTIFACTS}/golden/bev_vehicle_261.jpg")
assert new.shape == gold.shape, (new.shape, gold.shape)
mad = np.abs(new.astype(int) - gold.astype(int)).mean()
assert mad < 2.0, mad          # within JPEG re-encode noise
os.remove(out); print("OK render_bev golden  MAD=%.3f" % mad)
