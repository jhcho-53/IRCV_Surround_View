import _env
from core import frames
BAG = "/home/jaehyeon/MC-Calib/ext/calib_20260716_151806_0.db3"
tops = frames.read_image_topics(BAG)
assert len([k for k in tops if k.startswith("camera_")]) == 8, sorted(tops)
sets = frames.extract_synced(BAG, step=200)          # sparse: every 200th ref frame
assert len(sets) >= 2 and len(sets[0]["images"]) == 8
assert sets[0]["images"]["camera_rear"][:2] == b"\xff\xd8"
print("OK test_frames", len(sets))
