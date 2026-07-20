import _boot
from core.cameras import load_cameras
from core import geometry
g = geometry.fit_ground_plane(load_cameras())
geometry.save_ground(g)
print(f"ground normal {g['n'].round(4)}  saved -> {geometry.config.GROUND_NPZ}")
