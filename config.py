"""Single source of paths, constants, and measured inputs for the AVM BEV pipeline."""
import os

PIPELINE = os.path.dirname(os.path.abspath(__file__))   # this repo (cloned as IRCV_Surround_View/)

# ── Data root — supplied externally, NOT shipped in the repo ─────────────────
# The code is clone-only; the dataset (recordings, MC-Calib outputs, frames)
# lives under DATA_ROOT and is provided separately. Default: a `data/` dir next
# to this file — drop the external dataset there, or point elsewhere without
# editing this file:  `export AVM_DATA_ROOT=/your/path`.
DATA_ROOT = os.environ.get("AVM_DATA_ROOT", os.path.join(PIPELINE, "data"))
# ─────────────────────────────────────────────────────────────────────────────

# --- data / result paths (derived from DATA_ROOT) ---
CALIB_YAML      = f"{DATA_ROOT}/results/extrinsic_7cam_sync10_refi/calibrated_cameras_data.yml"  # MC-Calib output: K,D,pose (ref frame)
INTRINSICS_YAML = f"{DATA_ROOT}/results/intrinsics_all_cameras.yml"                              # camera_0..7 (has center)
FRAMES_DIR      = f"{DATA_ROOT}/frames_151806"       # camera_<name>/<setidx>.jpg  (sync full frames)
FLOOR_BOARD_DIR = f"{DATA_ROOT}/extracted/calib_extrinsic_example_left_right_floor20260715_202434"  # Cam_001=side_left_1, Cam_002=side_left_2
RESULTS_DIR     = f"{DATA_ROOT}/results"
EXPORT_DIR      = f"{DATA_ROOT}/calib"               # per-camera <name>.yml output
ARTIFACTS       = f"{PIPELINE}/artifacts"            # repo-relative (kept in git)
GROUND_NPZ      = f"{ARTIFACTS}/ground_plane.npz"
VEHICLE_NPZ     = f"{ARTIFACTS}/vehicle_frame.npz"

# Bundled golden copies of the MC-Calib outputs, byte-identical to the run that
# produced this repo. load_cameras()/load_center() fall back to these when the
# external CALIB_YAML/INTRINSICS_YAML above are absent, so a fresh clone renders
# the BEV from committed calibration once you supply only the image frames.
CALIB_YAML_GOLDEN      = f"{ARTIFACTS}/golden/calib/_mccalib_raw_ref_frame.yml"
INTRINSICS_YAML_GOLDEN = f"{ARTIFACTS}/golden/calib/_intrinsics_all_cameras.yml"

# --- camera model ---
ORDER = ["front_left","front_right","side_left_1","side_left_2","side_right_1","side_right_2","rear"]

# --- Charuco board ---
ARUCO_DICT   = "DICT_5X5_1000"
SQUARES_X    = 8
SQUARES_Y    = 7
SQUARE_LEN_M = 0.12
MARKER_LEN_M = 0.09

# --- BEV defaults (vehicle frame: X fwd, Y left, Z up; origin = rear-axle centre on ground) ---
BEV_EXTENT = dict(Xmin=-6.0, Xmax=10.0, Ymin=-6.0, Ymax=6.0)   # metres
BEV_PPM    = 60                                                # pixels per metre
# Ioniq 5 footprint (m): rear overhang, front reach from rear axle, half width
CAR_XREAR, CAR_XFRONT, CAR_HALF_W = -0.790, 3.845, 0.945

# --- measured HORIZONTAL camera positions for the vehicle-frame fit (m, X fwd, Y left, from rear-axle centre) ---
# NOTE: heights come from the CALIBRATION (front on bonnet ~1.05 m, side/rear on roof ~1.63 m).
# The tape heights mixed references (front=ground, side/rear=axle) so they are NOT used in the fit.
MEASURED_XY = {
    "rear":        (-0.20,  0.00),
    "side_left_2": ( 0.43,  0.55),  "side_left_1": ( 1.58,  0.55),
    "front_left":  ( 3.48,  0.65),  "front_right": ( 3.48, -0.65),
    "side_right_2":( 0.43, -0.55),  "side_right_1":( 1.58, -0.55),
}
