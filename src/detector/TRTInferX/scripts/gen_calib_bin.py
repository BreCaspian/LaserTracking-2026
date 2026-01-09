#!/usr/bin/env python3
import argparse
import glob
import os
import sys

import cv2
import numpy as np


def must_exist(path: str, kind: str) -> None:
    if not os.path.exists(path):
        print(f"[ERROR] {kind} not found: {path}", file=sys.stderr)
        sys.exit(1)


def letterbox(im, new_shape=640, color=(114, 114, 114)):
    h, w = im.shape[:2]
    r = min(new_shape / h, new_shape / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    im = cv2.resize(im, (nw, nh), interpolation=cv2.INTER_LINEAR)
    pad_w, pad_h = new_shape - nw, new_shape - nh
    top, bottom = pad_h // 2, pad_h - pad_h // 2
    left, right = pad_w // 2, pad_w - pad_w // 2
    im = cv2.copyMakeBorder(
        im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color
    )
    return im


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate INT8 calibration bin.")
    parser.add_argument("--images", required=True, help="Folder with images")
    parser.add_argument("--out", required=True, help="Output calib.bin path")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=16)
    args = parser.parse_args()

    must_exist(args.images, "Images directory")
    files = sorted(glob.glob(os.path.join(args.images, "*")))
    if len(files) < args.batch:
        print(
            f"[ERROR] Need at least {args.batch} images, found {len(files)}",
            file=sys.stderr,
        )
        sys.exit(1)

    buf = []
    for f in files[: args.batch]:
        im = cv2.imread(f)
        if im is None:
            print(f"[WARN] Skip unreadable image: {f}", file=sys.stderr)
            continue
        im = letterbox(im, args.imgsz)
        im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
        im = im.astype(np.float32) / 255.0
        im = np.transpose(im, (2, 0, 1))
        buf.append(im)

    if len(buf) < args.batch:
        print(
            f"[ERROR] Valid images {len(buf)} < batch {args.batch}",
            file=sys.stderr,
        )
        sys.exit(1)

    out_dir = os.path.dirname(os.path.abspath(args.out))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    arr = np.stack(buf, axis=0)
    arr.tofile(args.out)
    print(f"[OK] calib.bin saved: {args.out} shape={arr.shape}")


if __name__ == "__main__":
    main()
