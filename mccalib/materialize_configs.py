#!/usr/bin/env python3
"""Materialize the MC-Calib config templates by substituting the data root.

The templates in ``mccalib/configs/`` use the placeholder ``__DATA_ROOT__`` for
the location of your (externally supplied) recordings and outputs. MC-Calib reads
plain YAML paths and does not expand environment variables, so this script writes
concrete configs with the placeholder replaced by an absolute path.

    python mccalib/materialize_configs.py                        # -> $AVM_DATA_ROOT/configs
    python mccalib/materialize_configs.py --data-root /d/DM      # -> /d/DM/configs
    python mccalib/materialize_configs.py --data-root /d/DM --out /tmp/cfg

Then run MC-Calib on the concrete configs, e.g.:
    <mc-calib-build>/apps/calibrate/calibrate  <out>/front_left_intrinsic.yml
"""
import argparse
import glob
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data-root", default=os.environ.get("AVM_DATA_ROOT"),
                    help="absolute path to your data root (default: $AVM_DATA_ROOT)")
    ap.add_argument("--out", default=None,
                    help="output dir for the concrete configs (default: <data-root>/configs)")
    a = ap.parse_args()
    if not a.data_root:
        ap.error("no data root: set AVM_DATA_ROOT or pass --data-root")
    data_root = os.path.abspath(os.path.expanduser(a.data_root))
    out = a.out or os.path.join(data_root, "configs")
    os.makedirs(out, exist_ok=True)

    n = 0
    for tpl in sorted(glob.glob(os.path.join(HERE, "configs", "*.yml"))):
        with open(tpl) as f:
            text = f.read()
        text = text.replace("__DATA_ROOT__", data_root)
        dst = os.path.join(out, os.path.basename(tpl))
        with open(dst, "w") as f:
            f.write(text)
        n += 1
        print("wrote", dst)
    print(f"materialized {n} configs into {out}  (data_root={data_root})")


if __name__ == "__main__":
    main()
