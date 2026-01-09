#!/usr/bin/env python3
import argparse
import os
import shutil
import sys

from ultralytics import YOLO


def must_exist(path: str, kind: str) -> None:
    if not os.path.exists(path):
        print(f"[ERROR] {kind} not found: {path}", file=sys.stderr)
        sys.exit(1)


def main() -> None:
    parser = argparse.ArgumentParser(description="Export YOLO .pt to ONNX.")
    parser.add_argument("--pt", required=True, help="Path to .pt model")
    parser.add_argument("--out", required=True, help="Target ONNX path")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=1)
    parser.add_argument("--dynamic", action="store_true", help="Enable dynamic shapes")
    parser.add_argument("--nms", action="store_true", help="Export with NMS")
    parser.add_argument("--data", default=None, help="Dataset yaml (optional)")
    args = parser.parse_args()

    must_exist(args.pt, "PT model")
    out_dir = os.path.dirname(os.path.abspath(args.out))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    model = YOLO(args.pt)
    export_kwargs = {
        "format": "onnx",
        "imgsz": args.imgsz,
        "batch": args.batch,
        "dynamic": args.dynamic,
        "nms": args.nms,
    }
    if args.data:
        export_kwargs["data"] = args.data

    onnx_path = model.export(**export_kwargs)
    if isinstance(onnx_path, (list, tuple)):
        onnx_path = onnx_path[0]

    must_exist(onnx_path, "ONNX export")
    shutil.copyfile(onnx_path, args.out)
    print(f"[OK] ONNX saved: {args.out}")


if __name__ == "__main__":
    main()
