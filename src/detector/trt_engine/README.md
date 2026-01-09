# TRT 引擎后端

该后端封装 TRTInferX，保持 detector 接口不变。

## 目录文件
- `include/detector_trt/trt_detector.h`：TRT 检测器接口
- `src/trt_detector.cpp`：TRT 检测器实现
- `config/trt.yaml`：TRT 后端配置

## 配置
使用 `config/trt.yaml`，并在 `detector/config/detector.yaml` 中选择 `backend: "trt_engine"`。

关键字段：
- `engine_path`：TRT engine 路径
- `device_id`：GPU 设备索引
- `target_w/target_h`：输入尺寸（必须与 engine 一致）
- `preprocess`：`letterbox` 或 `resize`（需与训练/导出一致）
- `output_mode`：`auto` / `raw_only` / `raw_with_nms` / `packed_nms`
- `box_fmt`：`cxcywh` 或 `xyxy`（仅 raw 模式）
- `apply_sigmoid`：raw 输出是否需要 sigmoid（按导出方式配置）
- `conf/iou/max_det/top_k`：后处理阈值与 top-k
- `use_gpu_input`：优先使用 GPU 输入（相机提供 GPU buffer 时生效）

## GPU 输入
当相机管线提供 GPU BGR/HWC/uint8 缓冲时，检测器可直接使用 GPU 输入；
否则回退到 CPU `cv::Mat`。

## 输出
输出会转换为 `detector::Detection`（label=`ball`），再由
`toMeasurement` 转为 `TargetMeasurement`，保持下游接口不变。
