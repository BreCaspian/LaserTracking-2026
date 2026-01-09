# TRTInferX For YOLOv11 🚀

专为 **华北理工大学 HORIZON 战队 ROBOMASTER 2026 雷达组雷达定位系统与反无人机激光追踪系统**设计的 YOLOv11 INT8 PTQ 高性能 TensorRT 推理引擎。

**https://github.com/BreCaspian/TRTInferX**

---

## 特性

- 面向 YOLOv11 **目标检测**的高性能推理引擎，支持 FP16 和 INT8 与静态和动态 batch，兼容 `nms=True`（引擎内 EfficientNMS）与 `nms=False`（raw 输出 + 内部 NMS）两种导出路径。预处理、解码与坐标还原在 CUDA 侧完成，输入规范与 Ultralytics 默认流程一致，兼顾性能与可迁移性。

- 实测（KITTI 视频，RTX 3060 Laptop GPU）最高稳定约 **301.9 FPS**（INT8 动态 batch=16）；通过测算得出满负载端到端最高可达约 **1522.66 FPS**（INT8 batch=32，infStreams=2，含传输）/**746.28 FPS**（FP16 batch=64，infStreams=1，含传输）；理论算力上限（`trtexec --noDataTransfers`）可达约 **1858 FPS**（INT8 batch=128），用于衡量纯推理上限，端到端会受 H2D/D2H 影响。

- 注：测试环境在实测中未最大发挥推理引擎上限，**真实性能应接近测算结果**（为了有效获得相关指标运算平台本身有大量不相关负载开销）；另外，测试模型来自Ultralytics官方所公布的  [yolov11n](https://docs.ultralytics.com/zh/models/yolo11/) ，未对模型结构进行调整，若优化模型结构可进一步提升性能上限。
- 所有 测试模型（.pt/.onnx/.engine）、测试数据（视频、图片）、测试结果（视频、图片） 都可以在 Release 中下载

---

## 依赖

- CUDA Toolkit >= 11.8（推荐 12.x；需匹配显卡驱动）
- TensorRT >= 10.0（运行时与构建时版本必须一致）
- nvinfer_plugin 与 TensorRT 主版本一致（例如 10.x 对 10.x）
- nvonnxparser（仅构建 ONNX→engine 时需要）
- OpenCV >= 4.5（示例程序使用）
- CMake >= 3.18
- C++17 编译器（GCC 9+/Clang 10+）

---

## 测试环境

- Computer: Lenovo Legion Y9000P IAH7H
- CPU: 12th Gen Intel Core i9-12900H
- GPU: NVIDIA GA106M (GeForce RTX 3060 Mobile / Max-Q)
- OS: Ubuntu 22.04.5 LTS
- CUDA: 13.0 (nvcc 13.0.48, Driver 580.95.05, CUDA runtime 13.0)
- TensorRT: 10.14.1 (system packages, libnvinfer/libnvinfer_plugin)
- OpenCV: 4.5.4 (system), 4.12.0 (conda/python)

---

## 目录结构 (务必严格按照工程结构构建工程！！！)

```
yolov11/
├── TRTInferX/
│   ├── include/                     # 公共头文件与 API 定义
│   │   ├── api.h                    # 对外统一 API（ImageInput/Det/Api）
│   │   ├── Inference.h              # TRT 推理封装与运行时上下文
│   │   ├── preprocess.h             # 预处理接口声明
│   │   ├── postprocess.h            # 后处理/NMS 接口声明
│   │   ├── logging.h                # 日志与调试
│   │   └── macros.h                 # 通用宏与错误检查
│   ├── src/                         # 推理主流程实现
│   │   ├── Api.cpp                  # API 实现（load/infer/inferWithInfo）
│   │   └── Inference.cpp            # TRT 推理主流程（IO/调度/后处理）
│   ├── kernel/                      # CUDA 预处理与后处理核函数
│   │   ├── preprocess.cu            # letterbox + normalize
│   │   └── postprocess.cu           # raw decode + NMS/坐标还原
│   ├── examples/
│   │   └── yolo11/
│   │       ├── main.cpp             # 示例入口
│   │       └── include/main.h
│   ├── scripts/                     # 导出/校准脚本
│   ├── CMakeLists.txt               # 主工程构建脚本
│   └── build/                       # 编译输出
├── models/
│   ├── initial/                     # 原始 .pt 权重
│   └── exports/                     # 引擎与校准文件输出目录
│       ├── best_fp16.engine
│       ├── best_int8.engine
│       ├── best.onnx / best_raw.onnx
│       ├── calib.bin / trtexec.cache
└── test/
    ├── images/coco128/images/       # 测试输入图片
    ├── videos/                      # 测试视频
    └── output/                      # 测试输出图片/视频
```


注：最顶层文件夹名称可以自定义，其余请按照上述工程结构构建，否则可能会影响 engine 模型导出及校准的自动化脚本运行！

---

## 编译

```bash
cd TRTInferX
mkdir -p build
cd build
cmake .. \
  -DTRT_INCLUDE_DIR=/path/to/TensorRT/include \
  -DTRT_LIB_DIR=/path/to/TensorRT/lib \
  -DCUDA_TOOLKIT_ROOT_DIR=/path/to/cuda \
  -DCMAKE_CUDA_ARCHITECTURES=86
cmake --build . -j
```

> 若未设置 `CMAKE_CUDA_ARCHITECTURES`，会自动通过 `nvidia-smi` 检测并设置；否则使用默认 `86`。

---

工程会优先使用自动检测的 GPU 架构进行编译优化，你也可以手动覆盖：

```bash
cmake .. -DCMAKE_CUDA_ARCHITECTURES=86
```

支持的架构示例（`compute_XX / SM_XX`）：

| 架构 | 说明 |
| --- | --- |
| **SM60 / compute_60** | Pascal：Quadro GP100, Tesla P100, DGX-1 |
| **SM61 / compute_61** | Pascal：GTX 10 系列, Titan Xp, Tesla P4/P40 |
| **SM62 / compute_62** | Jetson TX2 |
| **SM70 / compute_70** | Volta：Tesla V100 |
| **SM72 / compute_72** | Xavier / Xavier NX |
| **SM75 / compute_75** | Turing：RTX 20 系列, Tesla T4 |
| **SM80 / compute_80** | Ampere：A100 |
| **SM86 / compute_86** | Ampere：RTX 3060/3070/3080/3090 等 |
| **SM87 / compute_87** | Jetson Orin |
| **SM89 / compute_89** | Lovelace：RTX 4090/4080 |
| **SM90 / compute_90** | Hopper：H100 |

---

## 从 .pt 导出 FP16/INT8 .engine 用于推理引擎


关于模型导出策略选择：
- FP16：`nms=True`（引擎内 NMS）+ 静态 `batch=1` 最稳、延迟最低。
- INT8：`nms=False` + `trtexec` 校准 + TRTInferX 内部 NMS 通常最容易成功；追求极致性能可尝试 `nms=True` 的 INT8，但成功率受模型与校准数据影响。

当前默认导出策略（脚本）：
- FP16：`nms=True`，输出 packed `[B,300,6]` 引擎内 NMS。
- INT8：`nms=False` raw 输出，`trtexec` 校准后由 TRTInferX 在 GPU 内部做 NMS。

> 说明：FP16 并非必须 `nms=True`，但推荐 engine 内 NMS（packed 输出更稳、后处理更简单）；`nms=False` 也可走 TRTInferX 内部 NMS，但就多了一段 decode+NMS，流程更复杂，收益不明显。

> 提醒：若修改 `--imgsz` 或 `--int8-batch`，请重新生成 `calib.bin`；静态 batch engine 必须用相同 batch 构建运行。

**激活Conda虚拟环境，工程根目录执行**（例如 `/home/yao/TEST/yolov11`）（~~可以去喝杯咖啡~~，导出时间较长由系统性能而定）：

```bash
TRTInferX/scripts/export_all.sh \
  --pt models/initial/yolo11n.pt \
  --images test/images/coco128/images \
  --out-dir models/exports \
  --imgsz 640 \
  --fp16-batch 1 \
  --int8-batch 1
```

前置条件：
- Python 环境已安装 `ultralytics`
- `test/images/coco128/images` 图片数量 >= `--int8-batch`（默认 8），用于 INT8 PTQ 校准输入

单独导出：

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate torch

PYTHON_BIN=$(which python) TRTInferX/scripts/export_fp16_engine.sh \
  --pt models/initial/yolo11n.pt \
  --out-dir models/exports \
  --imgsz 640 \
  --batch 1 \
  --dynamic 1 \
  --nms 1
```

```bash
PYTHON_BIN=$(which python) TRTInferX/scripts/export_int8_engine.sh \
  --pt models/initial/yolo11n.pt \
  --images test/images/coco128/images \
  --out-dir models/exports \
  --imgsz 640 \
  --batch 1 \
  --dynamic 1
```

> 需要时可通过环境变量指定 Python/TRT：
> `PYTHON_BIN=/path/to/python TRTEXEC=/path/to/trtexec TRTInferX/scripts/export_all.sh ...`

---

## Example 运行

下述命令默认在 `TRTInferX/build` 目录执行。

```bash
cd TRTInferX/build
./trt_yolo_example \
  --engine ../../models/exports/best_fp16.engine \
  --image ../../test/images/coco128/images/000000000036.jpg \
  --classes 1 \
  --conf 0.25 \
  --nms-score 0.25 \
  --nms-iou 0.45 \
  --raw-sigmoid \
  --raw-xyxy \
  --batch 4 \
  --streams 2 \
  --auto-streams \
  --min-streams 1
```

也可以在 `examples/yolo11/` 目录下单独编译 example。

> 若 engine 为 **静态 batch**（例如使用 `min=opt=max=16` 构建），运行时需要传入相同 batch；示例程序会用同一张图填充 batch。

**摄像头模式（用于实时推理和模型正确性测试，主机自带摄像头对推理引擎性能不构成评估标准）：**

```bash
./trt_yolo_example --engine ../../models/exports/best_fp16.engine --camera --camera-id 0
```

> 默认窗口显示并叠加 FPS/Infer/GPU/检测数；按 `q` 退出。`--no-display` 时仅推理不显示。

**视频模式（视频测试并保存输出）：**

```bash
./trt_yolo_example \
  --engine ../../models/exports/best_fp16.engine \
  --video ../../test/videos/input.mp4 \
  --video-out ../../test/output/output.mp4 \
  --no-display
```

> 视频模式默认 batch=1；可用 `--video-batch 1/4/8/16` 聚合帧做离线吞吐测试（帧尾不足会丢弃）。会在视频帧上绘制 FPS/Infer/GPU/检测数。

---

## 性能测算（trtexec）

在 `TRTInferX/build` 目录执行，`--noDataTransfers` 用于排除 H2D/D2H，测纯GPU推理吞吐：

```bash
/usr/src/tensorrt/bin/trtexec \
  --loadEngine=../../models/exports/best_int8_b128.engine \
  --shapes=images:128x3x640x640 \
  --warmUp=200 \
  --duration=10 \
  --noDataTransfers \
  --infStreams=1
```

包含数据传输的端到端测算：

```bash
/usr/src/tensorrt/bin/trtexec \
  --loadEngine=../../models/exports/best_int8_b128.engine \
  --shapes=images:128x3x640x640 \
  --warmUp=200 \
  --duration=10 \
  --infStreams=1
```

实际性能测算相关结论：
- 纯推理吞吐峰值：INT8 batch=128（infStreams=1, NoDataTransfers）约 1858 FPS。
- 端到端吞吐峰值：INT8 batch=32（infStreams=2, 含传输）约 1523 FPS。
- 最低端到端延迟：FP16 batch=1（infStreams=1, 含传输）更稳更低。
- 大 batch 下传输开销成为主要瓶颈，INT8 raw 输出 D2H 成本显著高于 FP16 packed 输出。
- infStreams=2 对小 batch 有明显收益，对大 batch 可能无收益或负收益且显存线性增加。

---

## 说明

- 对于 **不同显卡架构**，建议在目标设备上重新构建 `.engine`，以获得最佳性能。
- INT8 PTQ 引擎可由 Ultralytics 导出脚本生成，然后直接加载。
- 若需要 **GPU NMS**，推荐使用 `nms=True` 导出 packed 输出（`output0`，形如 `[B, max_det, 6]`）。
- 可通过 `--streams` 设置 2-4 个 CUDA stream 轮转推理，提升吞吐（默认为 1 ）。
- 可通过 `--auto-streams` 开启自适应动态多流，基于近期吞吐动态增减活跃 streams。
- **注意**：**Python 导出的 engine 与 C++ 运行时必须使用同一套 TensorRT**。如果 Python 在 Conda 中使用了 TensorRT，而 C++ 链接的是系统 `/lib/x86_64-linux-gnu` 的 TensorRT，就会出现反序列化失败（magicTag 不匹配）。解决办法：让 C++ 链接 Conda 的 TRT（`-DTRT_INCLUDE_DIR=... -DTRT_LIB_DIR=...`），或保证 Python 使用系统 TRT 导出。
- 参考解决命令（使用 Conda TRT 编译 C++）：
  ```bash
  cmake .. \
    -DTRT_INCLUDE_DIR=$CONDA_PREFIX/include \
    -DTRT_LIB_DIR=$CONDA_PREFIX/lib
  cmake --build . -j
  ```
- FP16/INT8 选择：优先使用 FP16 验证精度与流程；INT8 用于极致性能，需校准数据，且阈值需要按业务重新调整。
- 无法创建 NMS engine（workspace 不足）：高 batch 下 NMS 插件需要更大的 scratch/workspace，当前限制过小导致所有 tactic 被跳过；在 TRTInferX/src/Inference.cpp 中增大 setMemoryPoolLimit(kWORKSPACE, …) 后重新编译即可。

---

## 常见坑（已通过测试）

### 1) Engine 反序列化失败（magicTag mismatch）

现象：`trtexec --loadEngine` 或 TRTInferX 报错无法加载。  
原因：engine 与运行时 TensorRT 不一致或 engine 已损坏。  
解决：用**系统 TRT**从 ONNX 重新构建（示例假设 `models/exports/best.onnx` 已就位）。

FP16 验证流程（示例假设在 `TRTInferX/build` 下执行）：

```bash
/usr/src/tensorrt/bin/trtexec \
  --onnx=../../models/exports/best.onnx \
  --saveEngine=../../models/exports/best_fp16.engine \
  --fp16 \
  --minShapes=images:1x3x640x640 \
  --optShapes=images:1x3x640x640 \
  --maxShapes=images:1x3x640x640

./trt_yolo_example \
  --engine ../../models/exports/best_fp16.engine \
  --image ../../test/images/coco128/images/000000000036.jpg \
  --classes 1 --conf 0.25 --batch 1 --streams 1 --no-display \
  --output ../../test/output/output.jpg
```

### 2) INT8 校准失败（engine 变 0 MiB）

原因：带 NMS 的 ONNX 在 `trtexec` 校准阶段容易触发 shape 相关错误。  
推荐：用 `nms=False` 的 ONNX + 由 TRTInferX 内部做 GPU NMS。

**生成校准输入（calib.bin）**：

```bash
cd ../../
$CONDA_PREFIX/bin/python - <<'PY'
import os, glob, cv2, numpy as np
src = "./test/images/coco128/images"
out = "./models/exports/calib.bin"
imgsz = 640
batch = 16
def letterbox(im, new_shape=640, color=(114,114,114)):
    h, w = im.shape[:2]
    r = min(new_shape / h, new_shape / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    im = cv2.resize(im, (nw, nh), interpolation=cv2.INTER_LINEAR)
    pad_w, pad_h = new_shape - nw, new_shape - nh
    top, bottom = pad_h // 2, pad_h - pad_h // 2
    left, right = pad_w // 2, pad_w - pad_w // 2
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return im
files = sorted(glob.glob(os.path.join(src, "*")))[:batch]
assert len(files) == batch, "校准图像数量不足"
buf = []
for f in files:
    im = cv2.imread(f)
    im = letterbox(im, imgsz)
    im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
    im = im.astype(np.float32) / 255.0
    im = np.transpose(im, (2,0,1))
    buf.append(im)
arr = np.stack(buf, axis=0)
arr.tofile(out)
print("saved", out, arr.shape)
PY
cd TRTInferX/build
```
下面的指令功能相同更加方便

```
source ~/miniconda3/etc/profile.d/conda.sh
conda activate torch

python TRTInferX/scripts/gen_calib_bin.py \
  --images test/images/coco128/images \
  --out models/exports/calib.bin \
  --imgsz 640 \
  --batch 4
```

> 注意：`calib.bin` 的**数据量必须与 trtexec 的输入形状完全一致**。  
> 例如 `--min/opt/maxShapes=images:4x3x640x640` 就要求 `calib.bin` 是 **4×3×640×640** 的 float32 数据；  
> 如果你之前用 batch=1 生成了 `calib.bin`，再用 batch=4 构建新的 INT8 engine 就会报 “Unexpected file size”。  
> **INT8 静态 btach 模型只要你输入的 batch 或 imgsz 改了，就必须重新生成 `calib.bin`** ！！！

---

**nms=False + INT8（推荐的稳定 PTQ INT8方案）**

当使用 `trtexec` 做 INT8 校准时，**带 NMS 的 ONNX 容易校准失败**。推荐流程：

1) 导出 **nms=False** 的 ONNX  
2) `trtexec` 生成 INT8 engine  
3) 由 TRTInferX 在 GPU 内完成 **decode + EfficientNMS**

示例：

```bash
PYTHON_BIN=$(which python) TRTInferX/scripts/export_onnx.py \
  --pt models/initial/yolo11n.pt \
  --out models/exports/best_raw.onnx \
  --imgsz 640 \
  --batch 16 \
  --dynamic
```

```bash
/usr/src/tensorrt/bin/trtexec \
  --onnx=models/exports/best_raw.onnx \
  --saveEngine=models/exports/best_int8.engine \
  --int8 --fp16 \
  --loadInputs=images:models/exports/calib.bin \
  --calib=models/exports/trtexec.cache \
  --minShapes=images:16x3x640x640 \
  --optShapes=images:16x3x640x640 \
  --maxShapes=images:16x3x640x640
```

> TRTInferX 会自动识别 `output0` 为 raw 输出，并在 GPU 内完成 NMS。


**TRTInferX 验证**：

```bash
./trt_yolo_example \
  --engine ../../models/exports/best_int8.engine \
  --image ../../test/images/coco128/images/000000000036.jpg \
  --classes 1 --conf 0.25 --batch 16 --streams 1 --no-display \
  --output ../../test/output/output.jpg
```

### 3) 动态 batch engine

当 `min=opt=max=16` 时，engine 是**静态 batch=16**，必须传 `--batch 16`。  
若需要 1~16 动态 batch，请重新构建：

```bash
/usr/src/tensorrt/bin/trtexec \
  --onnx=../../models/exports/best.onnx \
  --saveEngine=../../models/exports/best_int8.engine \
  --int8 --fp16 \
  --loadInputs=images:../../models/exports/calib.bin \
  --calib=../../models/exports/trtexec.cache \
  --minShapes=images:1x3x640x640 \
  --optShapes=images:8x3x640x640 \
  --maxShapes=images:16x3x640x640
```

> 说明：`min/opt/max` 定义的是一个范围，只要 4 在范围内（1–16），运行时就能用 `batch=4`。  
> `optShapes` 只是“优化优先的 batch”，不代表只支持它；如果你主要用 batch=4，把 `optShapes` 改成 4 会更接近最优性能。

> 性能提示：静态 batch 通常更快（tactic 固定、缓存命中高）；动态 batch 虽然提供灵活性，但 **形状切换会带来很多额外开销和明显抖动**。追求极致性能时优先固定 batch。

### 4) Raw 输出格式（目标支持）

> [!TIP]
> `nms=False` raw 输出时，目标支持 `4+cls` 与 `4+obj+cls` 两种头部；若存在 obj，会自动按 `score = obj * cls` 融合，默认 **cxcywh** 格式。  
> 多类模型务必传入正确的 `--classes`（例如 COCO 为 80），否则会触发回退日志并导致分数/筛选不可靠。

常见 raw 形状参考（以 `[B, C, N]` 为例）：

- `C=5`：单类融合分数（`cx,cy,w,h,score`）
- `C=6`：单类 `obj+cls`（`cx,cy,w,h,obj,cls`）
- `C=84`：COCO80（`cx,cy,w,h,cls[80]`，无 obj）
- `C=85`：COCO80（`cx,cy,w,h,obj,cls[80]`）

> 也支持 `[B, N, C]` 布局，程序会自动识别并统一处理。

```bash
./trt_yolo_example \
  --engine ../../models/exports/best_int8.engine \
  --image ../../test/images/coco128/images/000000000036.jpg \
  --classes 1 --conf 0.25 \
  --nms-score 0.25 --nms-iou 0.45 \
  --raw-sigmoid \
  --batch 16 --streams 1 --no-display \
  --output ../../test/output/output.jpg
```

### 5) 关于阈值和分数（解决满屏乱框）

- Ultralytics ONNX 输出通常已做 sigmoid。默认 `raw_score_sigmoid=false` 避免把 0 logit 压成 0.5，从而 8400 候选里 300 个都过阈值。
- 如需查看概率，可加 `--raw-sigmoid`，但请同步把 `--conf/--nms-score` 提高到 0.4~0.5，否则容易出现满屏乱框。
- 只用原始 logit 时，0.08 对应 sigmoid 后约 0.52；INT8 量化会让 logit 有轻微偏移，做一次 sigmoid + 合理阈值即可。
- 如果你看到一堆 0.5 分数的杂框，原因是把接近 0 的 logit 做了 sigmoid。解决两条路线：
  1) 不再 sigmoid（不加 `--raw-sigmoid`），保持低阈值即可正常留真框。
  2) 必须 sigmoid 时，同时抬高阈值，如 `--conf 0.5 --nms-score 0.5`（或更高），让 0.5 一档的假框被过滤。
- 精度是否稳定取决于分数定义与阈值匹配：sigmoid 开关与阈值需配套调整；阈值不匹配会导致满屏杂框或漏检。INT8 会让分数有轻微偏移，建议在验证集上扫一遍阈值（如 sigmoid 模式下试 0.25/0.35/0.5），选定后固定。
- 若 raw 输出为 **score-only**（`raw channels=1 has_obj=0`，即 `cx,cy,w,h,score`），不要再 `--raw-sigmoid`，否则分数被压到 ~0.5 导致满屏框。此时使用原始分数阈值（如 0.08~0.15）；若全部被过滤，说明阈值偏高或 INT8 校准分布不匹配，可适当降低阈值（如 0.03）或**改用 `nms=True` 的 INT8 引擎（推荐）**。

> 计时说明：`infer(ms)` 为端到端计时，可能包含同步或视频写出等开销；`gpu(ms)` 为 CUDA event 计时（只包住 TRT enqueue）。要评估纯推理请以 `gpu(ms)` 为准。

---

## 统一接口API

上游输入：
- CPU/GPU 双路径，必须显式声明 stride 为**字节步长**，避免 HWC/GPU 混淆。
- 建议字段：`mem{CPU/GPU}, data, width/height, stride_bytes, color{BGR/RGB/GRAY}, layout{HWC/CHW}, dtype{UINT8/FP16/FP32}, prep{LETTERBOX/RESIZE}, target_w/h`，GPU 补 `device_id`、`cuda_stream`，可选 `timestamp_ms`、`roi`。

下游输出：
- `Det { x1,y1,x2,y2(原图坐标), score, cls, batch, mask/pose 可选 }`。
- `PreprocInfo { scale, scale_x, scale_y, padw, padh, src_w, src_h }`，用于对齐下游坐标。

引擎配置/选项：
- `EngineConfig { engine_path, device, max_batch, streams, auto_streams, prep, target_w/h, out_mode(auto/packed/raw), nms_score, nms_iou }`
- `InferOptions { conf, iou, apply_sigmoid=false, max_det, stream_override, box_fmt(cxcywh/xyxy) }`
- 内部自适应 nms=True/False、packed/raw；静态批强制对齐，动态批 setInputShape。
- raw 输出路径的 NMS 阈值在加载时固定（使用 `EngineConfig.nms_score/nms_iou`），`InferOptions` 中的阈值不会影响 raw NMS。
- C++ 对外接口（已提供 `include/api.h`/`src/Api.cpp`）：
  - `Api::load(cfg)` 加载引擎，`infer(batch, opt)` 返回统一 `Det`。
  - 当前实现支持 CPU/GPU 输入（BGR/HWC/uint8），GPU 路径直接从 device ptr 做 CUDA 预处理（零拷贝）。
  - `LETTERBOX` 保持比例并补边；`RESIZE` 直接拉伸到输入尺寸，坐标语义不同。
  - **GPU 输入注意事项（零拷贝 GPU 输入关键 ！！！）（可实现理论最快极限推理速度）**：
    - 只支持 `BGR/HWC/uint8`。
    - `stride_bytes` 必须是**字节**步长，否则 GPU 行访问会错。
    - 上游若用其他 CUDA stream 写入，需传 `cuda_stream`，否则请自行同步。
  - 示例：
```cpp
    EngineConfig cfg{"../../models/exports/best_fp16.engine", 0, 16, 2, false, PreprocessMode::LETTERBOX, 640, 640, OutputMode::AUTO};
    Api engine; engine.load(cfg);
    InferOptions opt; opt.conf=0.25f; opt.iou=0.45f; opt.apply_sigmoid=false;
    ImageInput in{MemoryType::CPU, mat.data, mat.cols, mat.rows, (int)mat.step,
                  ColorSpace::BGR, Layout::HWC, DType::UINT8,
                  PreprocessMode::LETTERBOX, 640, 640};
    auto res = engine.infer({in}, opt);           // 只要 Det
    auto res2 = engine.inferWithInfo({in}, opt);  // Det + 预处理尺度信息
    ```

---

## GPU NMS 导出示例

```bash
yolo export model=path/to/best.pt format=engine int8=True nms=True batch=16 dynamic=True
```

> `nms=True` + `dynamic=True` 需要设置 `batch>1` 作为最大 batch。


---

<div align="center">

Copyright © 2026 ROBOMASTER · 华北理工大学 HORIZON 战队 · 雷达组 - YAOYUZHUO  
未经许可，禁止擅自转载、修改或用于任何商业用途。  
若需引用或使用本文档内容，请注明来源。  
2026年01月06日

</div>