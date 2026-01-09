#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

PT=""
IMAGES_DIR="${ROOT_DIR}/test/images"
OUT_DIR="${ROOT_DIR}/models/exports"
IMGSZ=640
BATCH=8
DYNAMIC=1

PYTHON_BIN="${PYTHON_BIN:-python3}"
TRTEXEC="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"

usage() {
  cat <<EOF
Usage: $(basename "$0") --pt /path/to/model.pt [--images DIR] [--out-dir DIR]
       [--imgsz 640] [--batch 8] [--dynamic 0|1]

Env:
  PYTHON_BIN  Python executable (default: python3)
  TRTEXEC     trtexec path (default: /usr/src/tensorrt/bin/trtexec)
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --pt) PT="$2"; shift 2 ;;
    --images) IMAGES_DIR="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --imgsz) IMGSZ="$2"; shift 2 ;;
    --batch) BATCH="$2"; shift 2 ;;
    --dynamic) DYNAMIC="$2"; shift 2 ;;
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

ONNX_PATH="${OUT_DIR}/best_raw.onnx"
ENGINE_PATH="${OUT_DIR}/best_int8.engine"
CALIB_BIN="${OUT_DIR}/calib.bin"
CALIB_CACHE="${OUT_DIR}/trtexec.cache"

echo "[INFO] Export ONNX (nms=0, dynamic=${DYNAMIC})"
"${PYTHON_BIN}" "${SCRIPT_DIR}/export_onnx.py" \
  --pt "${PT}" \
  --out "${ONNX_PATH}" \
  --imgsz "${IMGSZ}" \
  --batch "${BATCH}" \
  $( [[ "${DYNAMIC}" == "1" ]] && echo "--dynamic" )

if [[ ! -f "${CALIB_BIN}" ]]; then
  echo "[INFO] Generate calib.bin"
  "${PYTHON_BIN}" "${SCRIPT_DIR}/gen_calib_bin.py" \
    --images "${IMAGES_DIR}" \
    --out "${CALIB_BIN}" \
    --imgsz "${IMGSZ}" \
    --batch "${BATCH}"
fi

echo "[INFO] Build INT8 engine"
"${TRTEXEC}" \
  --onnx="${ONNX_PATH}" \
  --saveEngine="${ENGINE_PATH}" \
  --int8 --fp16 \
  --loadInputs=images:${CALIB_BIN} \
  --calib=${CALIB_CACHE} \
  --minShapes=images:${BATCH}x3x${IMGSZ}x${IMGSZ} \
  --optShapes=images:${BATCH}x3x${IMGSZ}x${IMGSZ} \
  --maxShapes=images:${BATCH}x3x${IMGSZ}x${IMGSZ}

echo "[OK] INT8 engine: ${ENGINE_PATH}"
