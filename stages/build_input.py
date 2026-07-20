#!/usr/bin/env python3
"""Build MC-Calib position-aligned Cam_001..CamNNN input from pre-extracted
per-camera image folders whose filenames are the shared synchronization index.

MC-Calib matches frames across cameras by SORTED POSITION, so every Cam folder
must contain the SAME filename set. We take the union of all indices (frames
where the board was seen by >=1 camera) and, per camera, symlink the real image
where it exists or a single shared black frame where it doesn't (MC-Calib finds
no board there -> identical result to having the real board-less frame).
usage: build_input.py <src_percam_dir> <outdir> [--max_dt_ms F]
"""
import _boot
import argparse
import csv
import glob
import os
import numpy as np
import cv2
import config

# fixed order -> Cam_001.. ; MUST match intrinsics_all_cameras.yml camera_0.. (center is camera_7, dropped)
ORDER = config.ORDER


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")  # .../calib_20260716_150504
    ap.add_argument("out")  # output root (gets Cam_001..)
    ap.add_argument("--max_dt_ms", type=float, default=None,
                     help="keep only frames whose timestamps.csv max_dt_ms <= this (better sync)")
    a = ap.parse_args()
    SRC, OUT = a.src, a.out

    keep_dt = None
    if a.max_dt_ms is not None:
        keep_dt = set()
        with open(os.path.join(SRC, "timestamps.csv")) as f:
            for row in csv.DictReader(f):
                if float(row["max_dt_ms"]) <= a.max_dt_ms:
                    keep_dt.add(f"{int(row['index']):04d}")

    # per-camera set of indices (filename stems)
    idx_of = {}
    for cam in ORDER:
        d = os.path.join(SRC, f"camera_{cam}")
        s = set(os.path.splitext(os.path.basename(p))[0] for p in glob.glob(os.path.join(d, "*.jpg")))
        if keep_dt is not None:
            s = {x for x in s if x in keep_dt}
        idx_of[cam] = s
    union = sorted(set().union(*idx_of.values()), key=lambda s: int(s))
    print(f"cameras: {len(ORDER)} | union frames (board seen by >=1 cam): {len(union)}"
          + (f"  [sync filter max_dt<={a.max_dt_ms}ms]" if keep_dt is not None else ""))

    os.makedirs(OUT, exist_ok=True)
    blank = os.path.join(OUT, "_blank.jpg")
    cv2.imwrite(blank, np.zeros((1200, 1920, 3), np.uint8))

    for i, cam in enumerate(ORDER):
        dst = os.path.join(OUT, f"Cam_{i + 1:03d}")
        os.makedirs(dst, exist_ok=True)
        real = 0
        for idx in union:
            src_img = os.path.join(SRC, f"camera_{cam}", f"{idx}.jpg")
            link = os.path.join(dst, f"{idx}.jpg")
            if os.path.lexists(link):
                os.remove(link)
            if os.path.exists(src_img):
                os.symlink(os.path.abspath(src_img), link)
                real += 1
            else:
                os.symlink(os.path.abspath(blank), link)
        print(f"  Cam_{i + 1:03d} {cam:14s}: {real} real board frames / {len(union)} (rest blank)")

    with open(os.path.join(OUT, "camera_mapping.txt"), "w") as f:
        for i, cam in enumerate(ORDER):
            f.write(f"Cam_{i + 1:03d} = camera_{cam}  (== intrinsics camera_{i})\n")
    print(f"\ninput ready at {OUT}  (each Cam folder has {len(union)} aligned frames)")


if __name__ == "__main__":
    main()
