import _env, subprocess, tempfile, os, glob, config
d = tempfile.mkdtemp(dir=config.ARTIFACTS)
subprocess.check_call(["python","stages/extract_frames.py",
    "/home/jaehyeon/MC-Calib/ext/calib_20260716_151806_0.db3", d, "--step","150"], cwd=config.PIPELINE)
n = len(glob.glob(f"{d}/camera_rear/*.jpg"))
assert n >= 3, n
assert os.path.exists(f"{d}/timestamps.csv")
import shutil; shutil.rmtree(d); print("OK extract_frames", n)
