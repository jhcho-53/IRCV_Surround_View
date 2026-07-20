# MC-Calib for this rig (build recipe + configs)

The intrinsic (§3.2) and extrinsic (§3.3) calibration steps run the
[MC-Calib](https://github.com/rameau-fr/MC-Calib) C++ `calibrate` binary. That
engine is **not** vendored here — this directory ships everything needed to
reproduce *our* MC-Calib build and runs from a clone:

- [`mccalib-avm.patch`](mccalib-avm.patch) — the source patch we applied.
- [`configs/`](configs/) — the per-camera intrinsic + multi-camera extrinsic
  config templates (paths use the `__DATA_ROOT__` placeholder).
- [`materialize_configs.py`](materialize_configs.py) — fills in your data root.

## 1. Build MC-Calib from the pinned commit + patch

MC-Calib depends on OpenCV built with `opencv_contrib` (for the `aruco` module),
Ceres, and Boost. The upstream-supported path is the provided Docker image; a
native conda toolchain also works (that is what our patch's Boost tweak is for).

```bash
git clone https://github.com/rameau-fr/MC-Calib.git
cd MC-Calib
git checkout 4a8f9db1c821fb924893f21b15bf85838bd0aa74   # commit this patch was made against
git apply /path/to/this-repo/mccalib/mccalib-avm.patch

mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
# native/conda toolchains that only ship shared Boost:  add -DBoost_USE_STATIC_LIBS=OFF
make -j"$(nproc)"
# -> build/apps/calibrate/calibrate
```

### What the patch changes (4 files)

- **`McCalib/include/McCalib.hpp`** — the Charuco dictionary is hardcoded; we
  change `DICT_6X6_1000` → `DICT_5X5_1000` (both the pre-4.7 and ≥4.7 aruco
  branches) to match this rig's board. **Without this, no board is detected.**
- **`CMakeLists.txt`, `McCalib/CMakeLists.txt`, `apps/calibrate/CMakeLists.txt`**
  — make `Boost_USE_STATIC_LIBS` overridable and drop the standalone
  `Boost::system` component (header-only since Boost 1.69; no CMake component in
  Boost ≥ 1.90 / conda-forge). Pure build-toolchain tweaks; the Docker build is
  unaffected (it still defaults to static Boost).

## 2. Materialize the configs for your data root

The templates reference your recordings/outputs via `__DATA_ROOT__`. Fill it in:

```bash
export AVM_DATA_ROOT=/path/to/your/data        # same root the Python pipeline uses
python mccalib/materialize_configs.py          # writes $AVM_DATA_ROOT/configs/*.yml
```

Each config's `root_path` must point at the extracted board images for that
camera (MC-Calib layout: `Cam_001/…`, `Cam_002/…`), and `save_path` at where the
result YAML should land. Board geometry is already set for this rig
(`number_x_square: 8`, `number_y_square: 7`, `square_size: 0.12`,
`distortion_model: 1` = fisheye).

## 3. Run

```bash
# intrinsics — one run per camera (fisheye/Kannala)
<mc-calib-build>/apps/calibrate/calibrate  $AVM_DATA_ROOT/configs/front_left_intrinsic.yml
# … repeat for the other 7 cameras …

# extrinsics — all 7 board-visible cameras together (fix_intrinsic: 0)
<mc-calib-build>/apps/calibrate/calibrate  $AVM_DATA_ROOT/configs/extrinsic_7cam_sync10_refi.yml
```

The extrinsic config reads `cam_params_path: __DATA_ROOT__/results/intrinsics_all_cameras.yml`
— a single file combining the 8 per-camera intrinsic results as `camera_0..7`
(order: front_left, front_right, side_left_1, side_left_2, side_right_1,
side_right_2, rear, center). Assemble it from the per-camera intrinsic outputs.

The Python pipeline then consumes the two products
(`.../extrinsic_7cam_sync10_refi/calibrated_cameras_data.yml` and
`.../intrinsics_all_cameras.yml`). Byte-identical copies of both from our run are
committed under [`../artifacts/golden/calib/`](../artifacts/golden/calib/), and
`core/cameras.py` falls back to them automatically when the external files are
absent — so you can render the BEV without re-running MC-Calib at all.
