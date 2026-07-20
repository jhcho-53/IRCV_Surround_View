# Ioniq 5 Around-View (AVM/BEV) Calibration — Hands-on Guide

End-to-end walkthrough of calibrating a fisheye around-view rig on a Hyundai Ioniq 5 and producing a **metric bird's-eye-view (BEV) from 7 cameras** — from a raw ROS 2 recording to per-camera intrinsics + extrinsics in the vehicle frame, plus a stitched top-down video. (The rig has an 8th camera, `center`, which never observed the board — it gets intrinsics only and is **not** part of the BEV.)

This documents the actual steps taken, the numbers achieved, and the pitfalls hit — so you can reproduce or adapt it.

---

## 0. What we built

- **Rig:** **7 BEV cameras** on an Ioniq 5 — `front_left, front_right, side_left_1, side_left_2, side_right_1, side_right_2, rear` (roof/bonnet mounted). An 8th camera `center` is recorded and gets intrinsics, but is excluded from the BEV (never saw the board → no extrinsic).
- **Target:** Charuco board — `DICT_5X5_1000`, **8×7 squares**, square **0.12 m**, marker **0.09 m**.
- **Engine:** [MC-Calib](https://github.com/rameau-fr/MC-Calib) (C++) for intrinsics/extrinsics; a custom Python pipeline (this repo) for frame extraction, ground/vehicle-frame fitting, BEV rendering, and export.
- **Output:** the 7 board-visible cameras calibrated into one group, tied to a metric vehicle frame (rear-axle origin), and rendered as a top-down BEV video. (`center` never saw the board → intrinsics only.)

```
ROS2 bag ──▶ extract frames ──▶ MC-Calib (intrinsics, extrinsics)
                                      │
                        ground plane (floor board)  +  vehicle frame (tape measure)
                                      │
                      ┌───────────────┼────────────────┐
                  render_bev      render_video      export_calib
                  (metric still)   (top-down mp4)   (<camera>.yml)
```

---

## 1. Environment

The pipeline needs only **Python + OpenCV (with fisheye + Charuco/aruco) + NumPy** — no system OpenCV or ROS. Create a conda env and install from `requirements.txt`:

```bash
# 1) create & activate an environment
conda create -n avm python=3.11 -y
conda activate avm

# 2) install the two runtime dependencies (numpy, opencv-contrib)
cd <path-to-this-repo>
pip install -r requirements.txt

# 3) sanity check
python -c "import cv2, numpy; print(cv2.__version__, numpy.__version__); \
           print('aruco', hasattr(cv2.aruco,'CharucoDetector'), '| fisheye', hasattr(cv2.fisheye,'projectPoints'))"
```
Tested on **Python 3.13 · OpenCV 4.11.0 · NumPy 2.5.1**. `cv2.aruco.CharucoDetector` requires OpenCV ≥ 4.7 (satisfied by the pinned wheel). Python ≥ 3.9 works; 3.11–3.13 recommended for wheel availability.

With the env active, run any stage or tool directly. `export_calib` works from a fresh clone (it reads the bundled calibration + fitted artifacts, no external data needed):
```bash
python stages/export_calib.py     # writes <camera>.yml into $AVM_DATA_ROOT/calib
```

> **MC-Calib** — the C++ intrinsic/extrinsic engine used in §3.2–3.3 — is a **separate build**, not a Python dependency. This repo ships everything to reproduce our build in [`mccalib/`](mccalib/): the source patch ([`mccalib-avm.patch`](mccalib/mccalib-avm.patch), pinned to a specific upstream commit), the config templates, and a build recipe ([`mccalib/README.md`](mccalib/README.md)). This Python pipeline only **consumes** MC-Calib's output YAML — and byte-identical copies of that output are bundled (see the data note in §2), so you can render the BEV without building MC-Calib at all.

---

## 2. The data

> **Code vs. data.** The code is **clone-only**; the dataset (recordings, MC-Calib outputs, frames) is **supplied separately** and never committed. Every location derives from one root, **`DATA_ROOT`** (in **`config.py`**) — it defaults to a `data/` dir next to the repo, or point elsewhere with `export AVM_DATA_ROOT=/your/path`. The repo clones as `IRCV_Surround_View/`; run every command from inside it. Subdirectory names below are the `config.py` defaults — edit them there if your data differs.
>
> **Expected layout under `$AVM_DATA_ROOT`** (create only the subset a given step needs):
> ```
> $AVM_DATA_ROOT/
> ├── extracted/<per-camera board dir>/Cam_00N/*.jpg    # §3.2 intrinsics (MC-Calib input)
> ├── extracted/extrinsic_7cam_sync10/Cam_00N/*.jpg     # §3.3 extrinsics (built by build_input.py)
> ├── extracted/<...>_floor/{Cam_001,Cam_002}/*.jpg     # §3.4 ground plane (board flat on floor)
> ├── frames_151806/camera_<name>/<idx:04d>.jpg         # §3.6 BEV render (extract_frames.py output)
> ├── results/intrinsics_all_cameras.yml                # §3.2 output (camera_0..7)
> └── results/extrinsic_7cam_sync10_refi/calibrated_cameras_data.yml   # §3.3 output
> ```
> The two `results/*.yml` are the only MC-Calib products the Python pipeline reads — **byte-identical copies are committed** under `artifacts/golden/calib/`, and `core/cameras.py` falls back to them when the external files are absent. So once you supply the **image frames**, the BEV renders straight from the clone with no MC-Calib run. The fitted `artifacts/{ground_plane,vehicle_frame}.npz` are committed too, so §3.4/§3.5 can be skipped.

**Original ROS 2 bag** — full frames, all 8 cameras, ~30 fps, `sensor_msgs/CompressedImage` (JPEG **1920×1200**). A rosbag2 recording:
```
<recording>/                       # e.g.  <DATA_ROOT>/bags/calib_XXXXXXXX
├── <recording>_0.db3              # rosbag2 SQLite database
│      topics(id, name, type)                    ← one row per topic
│      messages(id, topic_id, timestamp, data)   ← data = CDR-serialized message blob
└── metadata.yaml                  # topic list, per-topic message_count, duration

8 image topics, type sensor_msgs/msg/CompressedImage:
   /camera_front_left/image_rgb/compressed     /camera_side_right_1/image_rgb/compressed
   /camera_front_right/image_rgb/compressed    /camera_side_right_2/image_rgb/compressed
   /camera_side_left_1/image_rgb/compressed    /camera_rear/image_rgb/compressed
   /camera_side_left_2/image_rgb/compressed    /camera_center/image_rgb/compressed

each message blob = CDR( std_msgs/Header , format:string , data:uint8[] )
                    └─ data[] holds the JPEG bytes (SOI FF D8 … FF D9 EOI)
```
`core/frames.py` reads `topics` for `*/image_rgb/compressed`, walks `messages` per topic by timestamp, carves the JPEG (SOI…EOI) out of each blob, and matches frames across cameras by nearest timestamp.

**Per-camera board recordings** (for intrinsics/extrinsics): single-camera + 2-camera overlap (`..._floor`) sequences, one folder per camera.

⚠️ The bag's `camera_info` topic is a **placeholder** (`plumb_bob`, D=0) — ignore it; the lenses are **fisheye**.

---

## 3. Step-by-step

### 3.1 Extract synchronized frames from the bag
```bash
python stages/extract_frames.py  path/to/<recording>_0.db3  path/to/frames_out
#   point config.py's FRAMES_DIR at this output dir so the later stages find it
```
Writes `<frames_dir>/camera_<name>/<setidx:04d>.jpg` (nearest-timestamp sync) + `timestamps.csv`. All 8 recorded cameras are extracted, but the BEV consumes only the **7** in `config.ORDER` (`center` is dropped). Each set groups the nearest-in-time frame from every camera; `timestamps.csv` records the per-set inter-camera `max_dt` so later steps can filter to well-synchronized sets.

> Cameras are **free-running (~30 fps), not hardware-synced** (~100 ms start offset). Frames are matched by nearest timestamp; hold the board still at each pose to avoid motion error.

### 3.2 Intrinsic calibration (MC-Calib, per camera)
Each camera's Charuco recording is calibrated separately (fisheye/Kannala) with the **MC-Calib `calibrate` binary** — build it from the bundled patch first (see [`mccalib/`](mccalib/)). The config templates are shipped; materialize them for your data root, then run one per camera:
```bash
python mccalib/materialize_configs.py                 # __DATA_ROOT__ -> $AVM_DATA_ROOT, writes $AVM_DATA_ROOT/configs/
<mc-calib-build>/apps/calibrate/calibrate  $AVM_DATA_ROOT/configs/front_left_intrinsic.yml
#   … repeat for the other 7 cameras …
```
Key config values (already set): `number_x_square: 8`, `number_y_square: 7`, `square_size: 0.12`, `distortion_model: 1` (fisheye). Consolidate the 8 per-camera intrinsics into `results/intrinsics_all_cameras.yml` (`camera_0..7`) — the `cam_params_path` the extrinsic step reads.

> **Pitfall — dictionary:** MC-Calib hardcodes `DICT_6X6_1000`; this rig's board is `DICT_5X5_1000`. The bundled [`mccalib-avm.patch`](mccalib/mccalib-avm.patch) fixes both occurrences (apply it before building). Also the board is **(squaresX=8, squaresY=7)** in OpenCV's convention (the flir launch args' x/y are transposed) — verified by detection.

### 3.3 Extrinsic calibration (MC-Calib, multi-camera)
The 7 cameras that saw the handheld board were calibrated together. Because the recording came as per-camera board-visible frames, we built a position-aligned input (same sync index across cameras, blank-filled gaps), then ran the extrinsic config:
```bash
python stages/build_input.py <src_percam_dir> $AVM_DATA_ROOT/extracted/extrinsic_7cam_sync10
<mc-calib-build>/apps/calibrate/calibrate  $AVM_DATA_ROOT/configs/extrinsic_7cam_sync10_refi.yml
```
That config sets `number_camera: 7`, `cam_params_path: …/intrinsics_all_cameras.yml`, and — critically — `fix_intrinsic: 0` (jointly refining the intrinsics is what most improves the multi-camera fit; filtering to well-synchronized frames helps too).

All 7 cameras should end in one `camera_group: 0`. Output: `results/extrinsic_7cam_sync10_refi/calibrated_cameras_data.yml` (poses relative to `front_left`).

### 3.4 Ground plane (from a board lying on the floor)
```bash
python stages/fit_ground.py       # -> artifacts/ground_plane.npz
```
Fits a plane through the on-floor board corners seen by `side_left_1`/`side_left_2` (the `..._floor` recording). **Plane RMS 17 mm.** Cross-check: the independently-estimated "board-up" normal agreed to **3°**.

### 3.5 Vehicle frame (tape-measured camera positions)
```bash
python stages/fit_vehicle.py      # -> artifacts/vehicle_frame.npz
```
Defines **X=forward, Y=left, Z=up, origin = rear-axle centre projected on the ground (Z=0)**. Fits a 2D rigid transform from the measured **horizontal** camera positions (`config.MEASURED_XY`) to the calibrated centres on the ground plane. **RMS 7.1 cm** (rear 1.1 cm).

> **Pitfall — height references:** the tape heights mixed conventions (front cameras measured from the **ground** ≈1.10 m on the bonnet; side/rear from the **axle centre** ≈1.33 m → +0.35 m axle height = ~1.68 m on the roof). We use **only the horizontal measurements** for the fit; heights come from the calibration and are cross-validated by the physical mounting (bonnet ≈1.05 m, roof ≈1.63 m) and the rear-axle-to-ground height **0.353 m ≈ tyre radius**.

### 3.6 Render the BEV
```bash
# metric still with overlays (car footprint, 1 m grid, camera dots, forward arrow)
python stages/render_bev.py 261    # writes config.RESULTS_DIR/bev_vehicle_261.jpg (add --out PATH to override)
#   optional zoom/coverage:  render_bev.py 261 --extent -3 7 -4 4 --ppm 100

# clean top-down video over the whole sequence (no overlays)
python stages/render_video.py     # -> results/bev_vehicle_sequence.mp4 (523 frames, 30 fps)
```
Each ground pixel is sampled from the best-incidence camera; the **ego blind zone (~4.5 %)** — ground the car occludes — is TELEA-inpainted from neighbours.

> **What the BEV shows:** only things **on the ground** (floor lines) stitch correctly across seams. Anything **above** the ground (the ego car body, walls, other vehicles) stretches/ghosts — this is inherent flat-ground-BEV parallax, not a calibration error. Production AVM masks the ego region and draws a car icon. The blind-zone inpaint is **synthetic** (hides obstacles) — fine for visualisation, not for safety.

### 3.7 Export per-camera calibration
```bash
python stages/export_calib.py     # -> $EXPORT_DIR/<camera>.yml   (config.EXPORT_DIR, default $DATA_ROOT/calib)
```
Each `<camera>.yml` (OpenCV `FileStorage`): `camera_matrix`, `distortion_coefficients` (fisheye 4 coeffs), `T_vehicle_from_camera` (4×4, `X_vehicle = T · X_camera`), `position_xyz_m`, `rotation_rodrigues`. `center.yml` is intrinsics-only.

### 3.8 Diagnostics
```bash
python tools/coverage_map.py                              # who-sees-what map (blind=black)
python tools/seam_blend.py front_right side_right_1 261   # 2-cam overlap: floor aligned? / parallax?
python tools/board_check.py side_left_1 <folder>          # detection + horizontality vs ground
```

---

## 4. Data structures (what each step expects / produces)

**Input — ROS 2 bag** (fed to `extract_frames`):
```
<recording>/
  <name>_0.db3        # rosbag2 sqlite; topics /camera_<name>/image_rgb/compressed
  metadata.yaml       #   type sensor_msgs/CompressedImage (JPEG, 1920x1200)
```

**Synchronized frames** (`extract_frames` output → `render_bev`/`render_video` input):
```
FRAMES_DIR/                         # config.FRAMES_DIR
  camera_front_left/0000.jpg 0001.jpg ...   # <setidx:04d>.jpg — same index = same instant across cameras
  camera_front_right/  ...          # one folder per recorded camera (8, incl. camera_center)
  ...
  timestamps.csv                    # cols: set, <name>_ns (per camera), max_dt_ms
```

**MC-Calib input — single-camera (intrinsics) and multi-camera (extrinsics).** MC-Calib globs `root_path/<cam_prefix><NNN>/*.jpg`, matching frames across cameras by sorted position:
```
<root_path>/
  Cam_001/  00000.jpg 00001.jpg ...   # camera 0 (camera_0 in the output YAML)
  Cam_002/  ...                       # camera 1 ...   (all folders MUST have equal count, index-aligned)
```
`build_input` produces exactly this for the 7 BEV cameras (blank-fills gaps so counts match):
```
<outdir>/
  Cam_001 .. Cam_007/  <idx>.jpg      # symlink → real frame, or → _blank.jpg where that cam didn't see the board
  _blank.jpg
  camera_mapping.txt                  # "Cam_001 = camera_front_left", ...  (order = config.ORDER minus center)
```

**Floor-board recording** (ground-plane input, `config.FLOOR_BOARD_DIR`): a 2-camera MC-Calib-style dir where the board lies flat on the ground:
```
FLOOR_BOARD_DIR/
  Cam_001/*.jpg    # side_left_1
  Cam_002/*.jpg    # side_left_2
```

**MC-Calib output consumed by the pipeline** (`config.CALIB_YAML`, OpenCV FileStorage):
```
calibrated_cameras_data.yml
  camera_0 .. camera_6:               # order = config.ORDER
    camera_matrix       3x3           #  fx,0,cx, 0,fy,cy, 0,0,1
    distortion_vector   1x4           #  fisheye/Kannala k1..k4
    camera_pose_matrix  4x4           #  camera -> reference(front_left)
    img_width, img_height, distortion_type(=1), camera_group(=0)
intrinsics_all_cameras.yml            # config.INTRINSICS_YAML — camera_0..7 (adds camera_7 = center)
```

**Intermediates** (`artifacts/`, produced by the fit stages):
```
ground_plane.npz     keys: n (3,) unit normal in ref frame,  c (3,) point on plane
vehicle_frame.npz    keys: R_ref_veh (3x3) vehicle->ref rotation,  t_ref_veh (3,) rear-axle origin in ref
```

**Export** (`export_calib` output, `config.EXPORT_DIR/<camera>.yml`):
```
camera_name, image_width, image_height, distortion_model,
camera_matrix (3x3), distortion_coefficients (1x4 fisheye),
T_vehicle_from_camera (4x4: X_vehicle = T · X_camera),
position_xyz_m (1x3), rotation_rodrigues (1x3)      # center.yml: intrinsics + extrinsic_status only
```

**Measured input** (`config.MEASURED_XY`) — tape-measured horizontal camera positions used to fit the vehicle frame:
```python
MEASURED_XY = { "<camera>": (X_forward_m, Y_left_m), ... }   # from rear-axle centre; heights NOT used (see §3.5)
```

## 5. Repository layout

```
config.py            single source of paths, constants, MEASURED_XY (edit here, not in scripts)
core/                import-only library
  cameras.py         load_cameras()/load_center() -> Camera(K,D,pose,...); project() (fisheye); ORDER
  board.py           Charuco (DICT_5X5_1000, 8x7); detect(); pnp_fisheye()
  frames.py          rosbag2 read, JPEG extract, nearest-timestamp sync
  geometry.py        fit_ground_plane(); fit_vehicle_frame(); save/load; to_vehicle(); T_vehicle_from_camera()
  bev.py             BevRenderer: precompute ground<->camera map once; render() + blind-zone inpaint
stages/              thin CLIs: extract_frames, build_input, fit_ground, fit_vehicle,
                     render_bev, render_video, export_calib   (each starts with `import _boot`)
tools/               coverage_map, seam_blend, board_check
mccalib/             MC-Calib source patch, config templates, materialize_configs.py, build recipe
artifacts/           ground_plane.npz, vehicle_frame.npz  +  golden/ (bundled MC-Calib outputs + refs)
tests/               plain-assert tests (run with python; no pytest)
$AVM_DATA_ROOT/      external data + outputs — supplied separately, not in the repo (see §2)
```

Verify the pipeline — each test prints `OK ...`, or `SKIP ...` when the external data it needs is absent:
```bash
for t in tests/test_*.py; do python "$t"; done
```
From a fresh clone (no data), the calibration-load, geometry, and export tests pass against the bundled artifacts; the frame/floor/bag tests skip. With data present, set `AVM_DATA_ROOT` (and `AVM_TEST_BAG` for the rosbag tests) to run everything. Golden checks: `render_bev 261` pixel MAD ≈ 0; `export_calib` YAML identical to golden; geometry reproduces RMS 7.1 cm / 17 mm.
