# hik_camera

- 海康 MVS 工业相机驱动（USB3/GigE）。
- 提供类库接口与示例程序。
- 目标相机：[MV-CS200-10UMUC](src/hik_camera/doc/MV-CS200-10UMUC_20241104_9799.pdf)（USB3.0，5472x3648，Bayer GB8）。

## 上下游接口

- 输入：相机设备 + 配置文件
- 输出：`hik_camera::Frame`（`bgr`/`raw`/时间戳）
- 下游：detector/control（只消费图像与时间戳）

## 构建

```bash
mkdir -p build
cd build
cmake .. -DMVS_ROOT=/opt/MVS
make -j$(nproc)
```

启用 CUDA/NPP：

```bash
cmake .. -DMVS_ROOT=/opt/MVS -DHIK_CAMERA_ENABLE_CUDA=ON -DHIK_CAMERA_ENABLE_NPP=ON
```

仅 CPU（关闭 CUDA/NPP）：

```bash
cmake .. -DMVS_ROOT=/opt/MVS -DHIK_CAMERA_ENABLE_CUDA=OFF -DHIK_CAMERA_ENABLE_NPP=OFF
```

CUDA 架构优化（可选）：

```bash
cmake .. -DMVS_ROOT=/opt/MVS -DCMAKE_CUDA_ARCHITECTURES=86
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

说明：如遇 NVCC 报 `compute_` 架构相关错误，可显式指定 `CMAKE_CUDA_ARCHITECTURES`。

## 运行示例

```bash
./hik_camera_demo --config ../config/config.yaml --show
```

## 接口要点

- 阻塞读取：`HikCamera::read(Frame*, timeout_ms)`
- 回调模式：`HikCamera::setFrameCallback(cb)`（不要与 `read()` 混用）
- `Frame::bgr`：BGR8 `cv::Mat`
- `Frame::raw_data`：原始 payload（需开启 `output_raw`）
- `Frame::bgr_ptr`：BGR 缓冲指针（启用零拷贝时使用）
- `Frame::bgr_size`：BGR 缓冲大小（字节）

## 配置说明（config/config.yaml）

- 设备选择：`serial_number`/`use_first_device`/`device_index`
- 触发与帧率：`trigger_mode`/`trigger_source`，`frame_rate_enable`/`frame_rate`
- 畸变校正：`undistort_enable=1` + `calib_path`
- `calib_path` 支持 ROS YAML（`camera_matrix`/`distortion_coefficients` 或 `K`/`D`）
- 自动曝光限幅：`auto_exposure_time_lower_us`/`auto_exposure_time_upper_us`（会自动夹紧到设备范围）
- Gamma/亮度：`gamma_enable`/`gamma_selector`/`gamma`，`brightness_enable`/`brightness`
- 重连：`reconnect_enable` + `reconnect_*`
- 参数持久化：`feature_load_*`、`feature_save_*`
- ROI：`roi_enable`/`roi_offset_x`/`roi_offset_y`/`roi_width`/`roi_height`
- 输出：`output_bgr`/`output_raw`/`copy_raw`（`copy_raw=0` 仅在回调模式下有效）
- 零拷贝帧池：`zero_copy_enable` + `buffer_pool_size`（仅影响 BGR 输出）
- GPU pipeline 预留：`gpu_pipeline_enable`/`gpu_device_id`/`gpu_stream_enable`
- GPU 去马赛克预留：`gpu_demosaic_enable` + `gpu_demosaic_backend` + `bayer_pattern`
  - `gpu_demosaic_backend=auto`：优先 OpenCV CUDA，其次 NPP
  - `gpu_demosaic_backend=opencv`：只用 OpenCV CUDA
  - `gpu_demosaic_backend=npp`：只用 NPP（不依赖 OpenCV CUDA）
- `bayer_pattern` 默认建议 `GB`（MV-CS200-10UMUC 常见 Bayer GB）

## 高性能推理支持

该功能面向高性能推理引擎（如 ORT CUDA / TensorRT）设计，用于减少 CPU 拷贝并降低 H2D 阻塞：

- 零拷贝帧池：采集直接写入预分配缓冲池，避免重复分配
- pinned 内存：`cudaHostRegister` 注册帧池，提高 H2D 带宽
- GPU 去马赛克：OpenCV CUDA 或 NPP 直接在 GPU 生成 BGR

### 路径解析规则

- 相对路径默认以配置文件所在目录为基准。
- 以 `config/` 开头的路径会自动映射到 `src/hik_camera/config/`。

### GPU 功能生效判断（日志输出）

- `Pinned pool enabled`：说明 pinned 内存注册生效。
- `GPU demosaic active (opencv/npp)`：说明 GPU 去马赛克已启用。

## CUDA/NPP 开关说明：避免环境不支持导致编译失败

- 只走 CPU：重新 `cmake` 时加 `-DHIK_CAMERA_ENABLE_CUDA=OFF -DHIK_CAMERA_ENABLE_NPP=OFF`。
- 已启用 CUDA/NPP 但想临时不用：在 `config.yaml` 把 `gpu_pipeline_enable`/`gpu_demosaic_enable` 设为 `0`。

## 环境要求与测试设备

环境要求（建议）：

- OS：Ubuntu 22.04 LTS
- CUDA：12+（含运行时与驱动）
- NPP：随 CUDA Toolkit 提供
- TensorRT：可选（推理引擎使用）
- OpenCV：4.x（含基础模块即可；OpenCV CUDA 可选）

测试设备（本项目验证环境）：

- Computer: Lenovo Legion Y9000P IAH7H
- CPU: 12th Gen Intel Core i9-12900H
- GPU: NVIDIA GA106M (GeForce RTX 3060 Mobile / Max-Q)
- OS: Ubuntu 22.04.5 LTS
- CUDA: 13.0 (nvcc 13.0.48, Driver 580.95.05, CUDA runtime 13.0)
- TensorRT: 10.14.1 (system packages, libnvinfer/libnvinfer_plugin)
- OpenCV: 4.5.4 (system), 4.12.0 (conda/python)

## 备注

- 为最大 FPS，建议关闭 `--show`，并避免在采集线程做重计算。
- `read()` 模式始终拷贝 raw；`copy_raw=0` 仅在回调模式下 raw 指针有效。
- `zero_copy_enable=1` 时，`bgr_ptr` 仅在下一帧覆盖前有效，不保证跨帧持久。
- `pixel_format` 使用 MVS 数值枚举（例如 BayerRG8=17301513，BGR8=35127317）。
- GPU 去马赛克依赖 CUDA；OpenCV CUDA 或 NPP 至少其一可用。

## 已知问题

运行 `hik_camera_demo --show` 时有可能出现以下异常：

- `OpenCV resize: (-215) inv_scale_x > 0`，或
- `OutOfMemoryError` 申请 TB 级内存，或
- 日志出现 `SetEnumValue failed: PixelFormat`，同时无法输出 FPS。

综合排查显示，主要原因通常为 **MVS SDK 版本和运行时库不一致** 或 **相机像素格式设置失败**：

- 若编译时使用的 MVS 头文件与运行时加载的 `libMvCameraControl.so` 版本不一致，`MV_FRAME_OUT_INFO_EX` 结构体会被错误解析，导致帧宽高异常（进而触发 OOM 或 resize ）。
- 像素格式设置失败时，帧转换路径可能异常，导致 `frame.bgr` 无效。

建议排查：

- 确认运行环境的 MVS SDK 版本与编译时一致（可查看 `/opt/MVS/ReleaseNote_EN.txt`）。
- 确认运行时 `LD_LIBRARY_PATH` 指向正确的 MVS 动态库目录。
- 若日志提示 `PixelFormat` 设置失败，请检查相机配置是否支持当前 `pixel_format`。
