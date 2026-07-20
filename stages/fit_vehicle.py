import _boot
from core.cameras import load_cameras
from core import geometry
cams = load_cameras(); g = geometry.load_ground()
vf = geometry.fit_vehicle_frame(cams, g)
geometry.save_vehicle(vf)
print(f"vehicle-frame RMS {vf['rms_cm']:.1f} cm  saved -> {geometry.config.VEHICLE_NPZ}")
for n, e in vf["residuals"].items(): print(f"   {n:13s} {e:5.1f} cm")
