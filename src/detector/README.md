# detector

检测功能包（传统视觉 + TRT 引擎可选），统一接口便于替换为其他检测方法。（实际比赛请更换检测模型）

## 功能

- HSV 分割 + 形态学处理 + 轮廓圆度筛选
- 可选高斯模糊 + 填充率筛选（圆内面积/圆面积）
- 输出：中心点 / 半径 / bbox / 置信度
- 单目标检测（选择最高分目标，classical 为 `confidence * radius`）
- 简单跟踪：指数平滑 + 速度预测 + 失帧补偿
- 高性能 TRT 推理（TRTInferX 引擎）

## 角色与上下游

- 上游：`hik_camera::Frame::bgr`（CPU）或 GPU 直通（TRT 选用）
- 输出：`common::TargetMeasurement`
- 下游：control

## 构建

```bash
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

## 运行

```bash
./detector_demo --config ../config/detector.yaml --cam 0
```

或读取视频：

```bash
./detector_demo --config ../config/detector.yaml --video /path/to/video.mp4
```

可选关闭显示：

```bash
./detector_demo --config ../config/detector.yaml --cam 0 --no-show
```

## 接入 hik_camera

```bash
./detector_hik_demo --camera-config ../../hik_camera/config/config.yaml --detector-config ../config/detector.yaml
```

可选关闭显示：

```bash
./detector_hik_demo --camera-config ../../hik_camera/config/config.yaml --detector-config ../config/detector.yaml --no-show
```

## 接口说明

- 输入：`cv::Mat bgr`
- 输出：`std::vector<Detection>`（见 `detector_common/detector.h`）
- 统一输出：`detector::toMeasurement` → `common::TargetMeasurement`
- 时间戳：`TargetMeasurement.timestamp` 以毫秒（ms）传递
- 后续替换：实现 `Detector` 接口即可
- 配置入口：`config/detector.yaml` 选择 `backend`

## 追踪参数（classical.yaml）

- `track_enable`：启用简单跟踪
- `max_missed`：最大允许连续丢检帧数
- `smooth_alpha`：中心/半径平滑系数（0~1，越大越跟随）
- `max_jump_px`：与预测点的最大跳变像素
- `max_pred_ms`：最大预测时间（毫秒）
- `downscale`：下采样比例（0.25~1.0）
- `use_roi`：启用跟踪 ROI
- `roi_expand_px`：ROI 扩展像素（原图）
- `roi_min_size`：ROI 最小边（原图）
- `roi_fallback_full`：ROI 未检出时退回全图

## TRT 配置（trt.yaml）

- `engine_path`：TRT 引擎（`.engine`）
- `device_id`：GPU 设备索引
- `max_batch/streams/auto_streams/num_classes`：TRTInferX 引擎参数
- `target_w/target_h`：固定输入尺寸（与引擎一致）
- `preprocess`：`letterbox` 或 `resize`
- `output_mode`：`auto` / `raw_only` / `raw_with_nms` / `packed_nms`
- `box_fmt`：`cxcywh` 或 `xyxy`（raw 解码使用）
- `apply_sigmoid`：raw 输出是否需要 sigmoid
- `conf/iou/max_det/top_k`：后处理阈值与 top-k
- `use_gpu_input`：优先使用 GPU 输入（需相机提供 GPU buffer）

## control 对接建议

- 使用 `common::TargetMeasurement` 作为统一输出
- `uv` 直接进入像素误差计算模块

## 注意事项

- **感知–控制耦合自激振荡**：当检测框抖动驱动云台抖动、云台抖动又反过来影响检测时，会出现越控越抖的闭环振荡（曲线呈现锯齿状）。
- **推荐处理**：在 detector 输出层对 `TargetMeasurement.uv` 做稳态滤波。
