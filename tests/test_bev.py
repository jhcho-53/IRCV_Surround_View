import _env, numpy as np, cv2, config
from core.cameras import load_cameras
from core import geometry, bev
cams = load_cameras(); g = geometry.load_ground(); vf = geometry.load_vehicle()
r = bev.BevRenderer(cams, vf, g)
assert (r.H, r.W) == (960, 720), (r.H, r.W)          # extent 16x12 m @ 60 ppm
frac = (r.who < 0).mean()
assert 0.02 < frac < 0.08, frac                       # blind zone ~4.5%
imgs = [cv2.imread(f"{config.FRAMES_DIR}/camera_{n}/0261.jpg") for n in config.ORDER]
out = r.render(imgs, inpaint=True)
assert out.shape == (960,720,3) and out.sum() > 0
print("OK test_bev  blind=%.1f%%" % (100*frac))
