import _env, numpy as np, config, os, tempfile
from core.cameras import load_cameras
from core import geometry
cams = load_cameras()
g = geometry.fit_ground_plane(cams)
assert abs(np.linalg.norm(g["n"]) - 1) < 1e-6
vf = geometry.fit_vehicle_frame(cams, g)
assert vf["rms_cm"] < 12.0, vf["rms_cm"]            # golden ~7.1 cm
assert vf["residuals"]["rear"] < 4.0, vf["residuals"]["rear"]  # golden ~1.1 cm
# round-trip: save to a TEMP path (must NOT overwrite the golden artifacts) then load back
gp = tempfile.mktemp(suffix=".npz", dir=config.ARTIFACTS)
vp = tempfile.mktemp(suffix=".npz", dir=config.ARTIFACTS)
geometry.save_ground(g, gp); geometry.save_vehicle(vf, vp)
g2 = geometry.load_ground(gp); vf2 = geometry.load_vehicle(vp)
assert np.allclose(g2["n"], g["n"]) and np.allclose(g2["c"], g["c"])
assert np.allclose(vf2["R_ref_veh"], vf["R_ref_veh"]) and np.allclose(vf2["origin_ref"], vf["origin_ref"])
# transforms
cams0 = cams  # already loaded above in the test
T = geometry.T_vehicle_from_camera(cams0[6], vf)          # rear
assert T.shape == (4,4)
pv = geometry.to_vehicle(np.zeros((5,3)), vf)
assert pv.shape == (5,3)
os.remove(gp); os.remove(vp)
print("OK test_geometry  rms=%.1fcm  rear=%.1fcm" % (vf["rms_cm"], vf["residuals"]["rear"]))
