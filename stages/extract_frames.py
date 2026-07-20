#!/usr/bin/env python3
"""Extract FULL synchronized frames (no board filtering) from a rosbag2 .db3.
Every sync-set index has one frame per camera (nearest timestamp), filename =
set index -> aligned across cameras. Writes camera_<name>/<idx>.jpg (original
JPEG bytes, lossless) + timestamps.csv.
usage: extract_frames.py <bag.db3> <outdir> [--step N]
"""
import _boot
import argparse
import csv
import os
import config
from core import frames

# canonical 8-camera order (7 calibrated + center), matches intrinsics_all_cameras.yml camera_0..7
CAMS = [f"camera_{name}" for name in config.ORDER] + ["camera_center"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("db3")
    ap.add_argument("out")
    ap.add_argument("--step", type=int, default=1, help="subsample reference frames")
    args = ap.parse_args()

    tid = frames.read_image_topics(args.db3)
    cams = [c for c in CAMS if c in tid]
    print("cameras:", [c.replace("camera_", "") for c in cams])

    sets = frames.extract_synced(args.db3, want=cams, tol_ms=25.0, step=args.step)

    for c in cams:
        os.makedirs(os.path.join(args.out, c), exist_ok=True)

    rows = []
    for s in sets:
        for c in cams:
            with open(os.path.join(args.out, c, f"{s['setidx']:04d}.jpg"), "wb") as f:
                f.write(s["images"][c])
        tss = s["tss"]
        rows.append([s["setidx"]] + [tss.get(c, "") for c in CAMS] + [round(s["max_dt_ms"], 1)])

    with open(os.path.join(args.out, "timestamps.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["set"] + [c + "_ns" for c in CAMS] + ["max_dt_ms"])
        w.writerows(rows)

    print(f"wrote {len(sets)} synchronized full-frame sets to {args.out}")


if __name__ == "__main__":
    main()
