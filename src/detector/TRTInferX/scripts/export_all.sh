#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

PT=""
IMAGES_DIR="${ROOT_DIR}/test/images"
OUT_DIR="${ROOT_DIR}/models/exports"
IMGSZ=640
FP16_BATCH=1
INT8_BATCH=8

usage() {
  cat <<EOF
Usage: $(basename "$0") --pt /path/to/model.pt [--images DIR] [--out-dir DIR]
       [--imgsz 640] [--fp16-batch 1] [--int8-batch 8]
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --pt) PT="$2"; shift 2 ;;
    --images) IMAGES_DIR="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --imgsz) IMGSZ="$2"; shift 2 ;;
    --fp16-batch) FP16_BATCH="$2"; shift 2 ;;
    --int8-batch) INT8_BATCH="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "[ERROR] Unknown arg: $1" >&2; usage; exit 1 ;;
  esac
done

if [[ -z "${PT}" ]]; then
  echo "[ERROR] --pt is required" >&2
  usage
  exit 1
fi

"${SCRIPT_DIR}/export_fp16_engine.sh" \
  --pt "${PT}" \
  --out-dir "${OUT_DIR}" \
  --imgsz "${IMGSZ}" \
  --batch "${FP16_BATCH}" \
  --dynamic 1 \
  --nms 1

"${SCRIPT_DIR}/export_int8_engine.sh" \
  --pt "${PT}" \
  --images "${IMAGES_DIR}" \
  --out-dir "${OUT_DIR}" \
  --imgsz "${IMGSZ}" \
  --batch "${INT8_BATCH}" \
  --dynamic 1

echo "[OK] Export complete: ${OUT_DIR}"
