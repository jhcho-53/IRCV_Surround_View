import _env, subprocess, sys, tempfile, os, glob, config
BAG = os.environ.get("AVM_TEST_BAG")
if not BAG or not os.path.exists(BAG):
    _env.skip("set AVM_TEST_BAG to a rosbag2 .db3 to run this test")
d = tempfile.mkdtemp(dir=config.ARTIFACTS)
subprocess.check_call([sys.executable,"stages/extract_frames.py",
    BAG, d, "--step","150"], cwd=config.PIPELINE)
n = len(glob.glob(f"{d}/camera_rear/*.jpg"))
assert n >= 3, n
assert os.path.exists(f"{d}/timestamps.csv")
import shutil; shutil.rmtree(d); print("OK extract_frames", n)
