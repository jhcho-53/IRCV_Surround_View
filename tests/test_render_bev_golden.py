import _env, subprocess, cv2, numpy as np, config, tempfile, os
out = tempfile.mktemp(suffix=".jpg", dir=config.ARTIFACTS)
subprocess.check_call(["python","stages/render_bev.py","261","--out",out], cwd=config.PIPELINE)
new = cv2.imread(out); gold = cv2.imread(f"{config.ARTIFACTS}/golden/bev_vehicle_261.jpg")
assert new.shape == gold.shape, (new.shape, gold.shape)
mad = np.abs(new.astype(int) - gold.astype(int)).mean()
assert mad < 2.0, mad          # within JPEG re-encode noise
os.remove(out); print("OK render_bev golden  MAD=%.3f" % mad)
