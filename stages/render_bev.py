#!/usr/bin/env python3
"""BEV still render CLI: composite (core.bev.BevRenderer) + overlays -- car
footprint white fill (colour sampled from bright ego pixels), Ioniq5 outline,
1 m grid, camera dots+labels, origin dot + forward arrow, caption.
Reproduces bev_vehicle_frame.py pixel-for-(near-)pixel; overlay order/params
match that legacy script exactly:
  composite -> grid -> sample car colour -> inpaint blind zone -> footprint
  fill -> outline -> origin/arrow -> camera dots -> caption.
usage: render_bev.py [setidx=261] [--extent Xmin Xmax Ymin Ymax] [--ppm N] [--out PATH]
"""
import _boot
import argparse
import cv2
import numpy as np
import config
from core.cameras import load_cameras
from core import geometry, bev

COL = [(80,160,255),(255,160,80),(80,255,160),(160,80,255),(80,220,220),(220,80,220),(200,200,80)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("setidx", nargs="?", type=int, default=261)
    ap.add_argument("--extent", nargs=4, type=float, metavar=("XMIN","XMAX","YMIN","YMAX"), default=None)
    ap.add_argument("--ppm", type=float, default=config.BEV_PPM)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    extent = config.BEV_EXTENT
    if args.extent is not None:
        Xmin, Xmax, Ymin, Ymax = args.extent
        extent = dict(Xmin=Xmin, Xmax=Xmax, Ymin=Ymin, Ymax=Ymax)
    out_path = args.out or f"{config.RESULTS_DIR}/bev_vehicle_{args.setidx}.jpg"

    cams = load_cameras()
    vf = geometry.load_vehicle()
    ground = geometry.load_ground()
    r = bev.BevRenderer(cams, vf, ground, extent=extent, ppm=args.ppm)

    images = [cv2.imread(f"{config.FRAMES_DIR}/camera_{name}/{args.setidx:04d}.jpg") for name in config.ORDER]

    # raw composite (no inpaint yet -- legacy draws the grid before inpainting)
    out = r.render(images, inpaint=False)

    Xmin, Xmax, Ymin, Ymax = r.Xmin, r.Xmax, r.Ymin, r.Ymax

    # 1 m grid (over the full extent)
    for gx in range(int(np.ceil(Xmin)), int(np.floor(Xmax)) + 1):
        _, ry = r.px_of(gx, 0); cv2.line(out, (0, ry), (r.W - 1, ry), (70, 70, 70), 1)
    for gy in range(int(np.ceil(Ymin)), int(np.floor(Ymax)) + 1):
        cxp, _ = r.px_of(0, gy); cv2.line(out, (cxp, 0), (cxp, r.H - 1), (70, 70, 70), 1)

    # --- fill the car footprint with the real body colour (sampled from the ego car in the BEV) ---
    r0 = int((Xmax - config.CAR_XFRONT) * r.ppm); r1 = int((Xmax - config.CAR_XREAR) * r.ppm)
    c0 = int((Ymax - config.CAR_HALF_W) * r.ppm); c1 = int((Ymax + config.CAR_HALF_W) * r.ppm)
    r0c, r1c = max(r0, 0), min(r1, r.H); c0c, c1c = max(c0, 0), min(c1, r.W)
    roi = out[r0c:r1c, c0c:c1c].reshape(-1, 3)
    bright = roi[roi.mean(1) > 150]
    car_col = np.median(bright, axis=0) if len(bright) > 200 else np.array([238, 238, 238])
    car_col = tuple(int(v) for v in car_col)
    print("car body colour (BGR) used for mask:", car_col)

    # blind zone = ground no camera can see (ego vehicle occludes it) -> inpainted from neighbours (synthetic)
    print(f"blind zone (no camera coverage): {100 * r.nocov.mean():.1f}% -> inpainted from neighbours (synthetic)")
    out = cv2.inpaint(out, r.nocov, 7, cv2.INPAINT_TELEA)
    cv2.rectangle(out, (c0, r0), (c1, r1), car_col, -1)   # real car footprint stays an explicit mask

    # Ioniq 5 outline
    p1 = r.px_of(config.CAR_XREAR, config.CAR_HALF_W); p2 = r.px_of(config.CAR_XFRONT, -config.CAR_HALF_W)
    cv2.rectangle(out, p1, p2, (0, 215, 255), 2)

    # rear-axle center (origin) + forward arrow
    oc = r.px_of(0, 0); cv2.circle(out, oc, 5, (0, 0, 255), -1)
    cv2.putText(out, "rear-axle O", (oc[0] + 6, oc[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    fa = r.px_of(1.2, 0); cv2.arrowedLine(out, oc, fa, (0, 0, 255), 2, tipLength=0.3)
    cv2.putText(out, "X fwd", (fa[0] + 4, fa[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

    # camera positions (vehicle frame) + labels
    centers = np.array([cm.pose[:3, 3] for cm in cams])
    cv_pos = geometry.to_vehicle(centers, vf)
    for i, cam in enumerate(cams):
        col, row = r.px_of(cv_pos[i, 0], cv_pos[i, 1])
        cv2.circle(out, (col, row), 4, COL[i % len(COL)], -1)
        cv2.putText(out, cam.name[:9], (col + 5, row), cv2.FONT_HERSHEY_SIMPLEX, 0.4, COL[i % len(COL)], 1)

    cv2.putText(out, f"Ioniq5 vehicle-frame BEV  set {args.setidx}  grid=1m  (X up=fwd, Y left)",
                (8, r.H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    cv2.imwrite(out_path, out)
    print("wrote", out_path, f"{r.W}x{r.H}")


if __name__ == "__main__":
    main()
