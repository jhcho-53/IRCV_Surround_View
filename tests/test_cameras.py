import _env, numpy as np, config
from core.cameras import load_cameras, project, load_center

cams = load_cameras()
assert [c.name for c in cams] == config.ORDER, [c.name for c in cams]
c = cams[0]
assert c.K.shape == (3,3) and c.D.shape == (4,) and c.pose.shape == (4,4)
assert c.width == 1920 and c.height == 1200
# a point 3 m in front of front_left, on the ground-ish, projects inside the image
p = c.pose[:3,3] + c.pose[:3,:3] @ np.array([0,0,3.0])   # 3 m along its optical axis, in ref frame
px, valid = project(p.reshape(1,3), c)
assert valid[0] and 0 <= px[0,0] < 1920 and 0 <= px[0,1] < 1200, (px, valid)
# non-identity-pose projection check (catches R vs R^T transform bug)
c2 = cams[2]                                          # side_left_1 (non-identity pose)
p2 = c2.pose[:3,3] + c2.pose[:3,:3] @ np.array([0,0,3.0])
px2, valid2 = project(p2.reshape(1,3), c2)
assert valid2[0] and 0 <= px2[0,0] < c2.width and 0 <= px2[0,1] < c2.height, (px2, valid2)
# load_center() smoke check
cc = load_center()
assert cc.name == "center" and cc.K.shape == (3,3) and cc.pose is None
print("OK test_cameras")
