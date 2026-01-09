#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

PT=""
OUT_DIR="${ROOT_DIR}/models/exports"
IMGSZ=640
BATCH=1
DYNAMIC=1
NMS=1

PYTHON_BIN="${PYTHON_BIN:-python3}"
TRTEXEC="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"

usage() {
  cat <<EOF
Usage: $(basename "$0") --pt /path/to/model.pt [--out-dir DIR] [--imgsz 640] [--batch 1] [--dynamic 0|1] [--nms 0|1]

Env:
  PYTHON_BIN  Python executable (default: python3)
  TRTEXEC     trtexec path (default: /usr/src/tensorrt/bin/trtexec)
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --pt) PT="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --imgsz) IMGSZ="$2"; shift 2 ;;
    --batch) BATCH="$2"; shift 2 ;;
    --dynamic) DYNAMIC="$2"; shift 2 ;;
    --nms) NMS="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "[ERROR] Unknown arg: $1" >&2; usage; exit 1 ;;
  esac
done

if [[ -z "${PT}" ]]; then
  echo "[ERROR] --pt is required" >&2
  usage
  exit 1
fi

mkdir -p "${OUT_DIR}"

ONNX_PATH="${OUT_DIR}/best.onnx"
ENGINE_PATH="${OUT_DIR}/best_fp16.engine"

echo "[INFO] Export ONNX (nms=${NMS}, dynamic=${DYNAMIC})"
"${PYTHON_BIN}" "${SCRIPT_DIR}/export_onnx.py" \
  --pt "${PT}" \
  --out "${ONNX_PATH}" \
  --imgsz "${IMGSZ}" \
  --batch "${BATCH}" \
  $( [[ "${DYNAMIC}" == "1" ]] && echo "--dynamic" ) \
  $( [[ "${NMS}" == "1" ]] && echo "--nms" )

echo "[INFO] Build FP16 engine"
"${TRTEXEC}" \
  --onnx="${ONNX_PATH}" \
  --saveEngine="${ENGINE_PATH}" \
  --fp16 \
  --minShapes=images:${BATCH}x3x${IMGSZ}x${IMGSZ} \
  --optShapes=images:${BATCH}x3x${IMGSZ}x${IMGSZ} \
  --maxShapes=images:${BATCH}x3x${IMGSZ}x${IMGSZ}

echo "[OK] FP16 engine: ${ENGINE_PATH}"
