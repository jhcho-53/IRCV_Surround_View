#!/usr/bin/env python3
"""Clean vehicle-frame BEV video (no overlays) over a range of synchronized frames.
The BEV<->camera mapping is fixed by the geometry (core.bev.BevRenderer), so it is
built once and each frame is just a gather + blind-zone inpaint.
usage: render_video.py [--out PATH] [--fps 30] [--start 0] [--end N]
"""
import _boot
import argparse
import os
import cv2
import config
from core.cameras import load_cameras
from core import geometry, bev


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=f"{config.RESULTS_DIR}/bev_vehicle_sequence.mp4")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--end", type=int, default=None)
    args = ap.parse_args()

    n_files = len([f for f in os.listdir(f"{config.FRAMES_DIR}/camera_rear") if f.endswith(".jpg")])
    end = args.end if args.end is not None else n_files - 1

    cams = load_cameras()
    vf = geometry.load_vehicle()
    ground = geometry.load_ground()
    r = bev.BevRenderer(cams, vf, ground)
    print(f"BEV mapping ready: {r.W}x{r.H}  blind zone {100 * r.nocov.mean():.1f}%")

    vw = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (r.W, r.H))
    if not vw.isOpened():
        print("ERROR: VideoWriter failed to open"); raise SystemExit(1)

    for k in range(args.start, end + 1):
        images = [cv2.imread(f"{config.FRAMES_DIR}/camera_{name}/{k:04d}.jpg") for name in config.ORDER]
        out = r.render(images, inpaint=True)
        vw.write(out)
        if k % 50 == 0: print(f"  frame {k}/{end}")
    vw.release()
    print("wrote", args.out, f"{r.W}x{r.H} @ {args.fps}fps  frames {args.start}..{end}")


if __name__ == "__main__":
    main()
