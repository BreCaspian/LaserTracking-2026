# LaserTracking

<p align="center">
  <img src="docs/Horizon.png" width="250" alt="Horizon Team">
</p>

- **[ROBOMASTER 2026 华北理工大学 HORIZON战队 雷达组 反无人机激光追踪系统视觉部分 先行开源](https://bbs.robomaster.com/article/1883301?source=1)**
- **[电控部分二轴云台控制开源](https://github.com/NCST-Horizon-RM/Horizon_frame_f1)**

---
## 概览


<p align="center">
  <img src="docs/LaserTracking-Pipeline.png"
       alt="LaserTracking"
       height="">
</p>
<p align="center"><em>LaserTracking Pipeline Overview</em></p>


* ***common***：统一数据结构与配置读取接口（***TargetMeasurement*** / ***GimbalState*** / ***GimbalCommand*** / ***CameraModel*** / ***Boresight***）。
* ***hik_camera***：海康 MVS 相机驱动，支持 CPU/GPU 去马赛克与零拷贝。专为高性能推理引擎设计
* ***detector***：目标检测（传统视觉或 TRT），输出 ***TargetMeasurement***。（*后续根据比赛需要替换检测模型,现为测试 Demo*）
* ***control***：**控制核心** 像素误差 → 云台角度指令，包含丢失目标搜索策略。
* ***gimbal_serial***：串口收发云台状态与指令。
* ***tool/boresight_calibrator***：同轴校准工具（生成 ***u_L/v_L***）。
* ***tool/parallax_estimator***：视差距离估算工具。

---
## Demo实际效果

<p align="center">
  <img src="docs/demo.gif" width="" alt="Demo">
  <br>
  <em>Demo : Search & Tracking</em>
</p>

**注：实际测试认为使用高帧率工业相机最佳，CS200虽分辨率高但帧率过低导致新数据刷新速度不够快，具体表现为追踪有明显延迟**

---
## 2026年超级对抗赛北部赛区实际效果

<p align="center">
  <img src="docs/plane.jpg" width="" alt="plane">
</p>

**注：实际上场方案是该方案改良版，学妹进行了有效优化并更换了新相机**

---

## 工程结构

```
LaserTracking/
├── README.md
├── docs
└── src/
    ├── CMakeLists.txt            # 顶层构建入口
    ├── common/                   # 共享类型与配置读取
    ├── control/                  # 控制器与系统串联 demo
    ├── detector/                 # 传统视觉/ TRT 检测
    ├── gimbal_serial/            # 云台串口收发
    ├── hik_camera/               # 海康相机驱动
    ├── tool/
    │   ├── parallax_estimator/   # 视差估算工具
    │   └── boresight_calibrator/ # 同轴校准工具
    └── log/                      # 运行日志
```

## 依赖与环境

- OS：Ubuntu 22.04（推荐）
- OpenCV：4.x
- CUDA / TensorRT：可选（TRT 检测与 GPU Pipeline）
- 海康 MVS SDK：用于 `hik_camera`（`-DMVS_ROOT=/opt/MVS`）

### 测试设备

- Computer: Lenovo Legion Y9000P IAH7H
- CPU: 12th Gen Intel Core i9-12900H
- GPU: NVIDIA GA106M (GeForce RTX 3060 Mobile / Max-Q)
- OS: Ubuntu 22.04.5 LTS
- CUDA: 13.0 (nvcc 13.0.48, Driver 580.95.05, CUDA runtime 13.0)
- TensorRT: 10.14.1 (system packages, libnvinfer/libnvinfer_plugin)
- OpenCV: 4.5.4 (system), 4.12.0 (conda/python)


## 构建

```bash
cd /home/XXX/LaserTracking/src
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

构建后可执行文件位于各子目录：

- `src/build/control/control_system_demo`
- `src/build/detector/detector_demo`
- `src/build/hik_camera/hik_camera_demo`
- `src/build/gimbal_serial/gimbal_serial_demo`
- `src/build/tool/boresight_calibrator/boresight_calibrator`

## 运行系统（控制串联）

```bash
cd /home/XXX/LaserTracking/src/build/control
./control_system_demo \
  --camera-config ../../hik_camera/config/config.yaml \
  --detector-config ../../detector/config/detector.yaml \
  --control-config ../../control/config/control.yaml \
  --port /dev/ttyACM0 \
  --baud 115200 \
  --send-hz 400
```

## 日志

`control_system_demo` 会自动将终端输出追加写入：

```
src/log/log.txt
```

每次启动会在日志中追加空行与时间戳。

## 同轴校准（boresight）

### 使用工具

```bash
cd /home/XXX/LaserTracking/src/build/tool/boresight_calibrator
./boresight_calibrator \
  --camera-config ../../hik_camera/config/config.yaml \
  --detector-config ../../detector/config/detector.yaml \
  --boresight-config ../../control/config/control.yaml \
  --boresight-out ./boresight.yaml
```

### 校准流程

1) 固定目标，让检测框稳定。  
2) 调整红色十字到激光点位置。  
3) 保存 `u_L/v_L` 并写回 `control.yaml`。

注意：`u_L/v_L` 对距离敏感，建议在常用工作距离进行校准。

## 关键配置

- `src/control/config/control.yaml`：控制参数与 `u_L/v_L`。
- `src/detector/config/detector.yaml`：检测后端选择与参数。
- `src/hik_camera/config/config.yaml`：相机参数与去马赛克设置。

## 常见问题

- **可执行文件找不到**：顶层构建后位于 `src/build/<module>/` 子目录。
- **串口打不开**：确认设备名与权限（如 `/dev/ttyACM0`）。
- **没有检测目标**：检查 detector 配置、光照、目标颜色与相机曝光。


---

## 硬件清单

以下为激光追踪系统参考实现所使用的硬件配置。

<div align="center" style="display:flex; justify-content:center; align-items:center; gap:0;">
  <img src="docs/LaserTracking.jpg" height="300" alt="laser_tracking" style="display:block; margin:0; padding:0;">
  <img src="docs/overview.png" height="300" alt="Hardware Overview" style="display:block; margin:0; padding:0;">
</div>
<div align="center">
  <em>Hardware overview</em>
</div>

---

### 工业相机与激光发射器

- **工业相机：** 海康威视 **MV-CS200-10UMUC**（USB3.0）
>[!TIP]
> **实际测试认为使用高帧率工业相机最佳**
>
>**CS200虽分辨率高但帧率过低导致新数据刷新速度不够快，具体表现为追踪有明显延迟**
>
>**请使用*RM*组委会官方推荐激光发射器，文档中所使用激光只用于*Demo***

- **镜头：** **MVL-KF3528M-12MP**（35 mm，1200 万像素）

* ~~**激光模块：** 工业级可见光激光器~~

  * ~~**波长范围：** 635 / 637 / 650 / 660 / 685 nm（可调）~~
  * ~~**光型：** 圆点 / 一字线 / 十字线（按型号选择）~~


    

<p align="center">
  <img src="docs/CS200.png" width="500" alt="Hikvision MV-CS200-10UMUC">
  <br>
  <em>Hikvision MV-CS200-10UMUC (USB3.0)</em>
</p>

<p align="center">
  <img src="docs/MVL-KF3528M-12MP.png" width="500" alt="MVL-KF3528M-12MP Lens">
  <br>
  <em>MVL-KF3528M-12MP (35mm, 12MP)</em>
</p>

<p align="center">
  <img src="docs/Laser.png" width="500" alt="Laser Module">
  <br>
  <em>Visible laser module (reference model)</em>
</p>

---

### 控制与执行

- **云台：** 两轴云台（Pan-Tilt）
- **电机：** **GM6020**（示例型号）
- **微控制器（MCU）：** **STM32F103C8T6**（用于底层控制与串口通信）

>[!TIP]
>推荐使用更高精度编码器

<table align="center">
  <tr>
    <td align="center" valign="middle">
      <img src="docs/GM6020.png" height="325" alt="GM6020 Motor"><br>
      <em>GM6020 motor (example)</em>
    </td>
    <td align="center" valign="middle">
      <img src="docs/STM32F103C8T6.jpg" height="325" alt="STM32F103C8T6"><br>
      <em>STM32F103C8T6 MCU</em>
    </td>
  </tr>
</table>




---

### 运算平台

- **主机平台：** x86_64 架构 Linux 主机
- **GPU：** NVIDIA RTX 系列显卡（可选，用于 CUDA / TensorRT 加速）
- **接口：** USB 3.0（相机），UART（云台控制）

---

### 通信接口

- **相机接口：** USB 3.0
- **云台通信：** 串口（UART）

---

### 供电

- **相机供电：** USB 供电
- **云台与激光供电：** TB48S 电池

---

### 说明

- 实际可达到的帧率受 USB 3.0 链路带宽与相机分辨率限制。
- GPU 加速并非必需，但在高帧率检测与闭环控制中强烈推荐使用。
- 激光模块为**参考型号**，可根据功率、波长及光型需求替换为等效产品。推荐使用RM组委会推荐激光发射器


---
## 线程模型参考

<p align="center">
  <img src="docs/Thread-Model.png"
       alt="Thread Model"
       height="">
</p>
<p align="center"><em>Thread Model Overview</em></p>

注：提供系统线程模型供分析参考

---
## 后记

**今天是2026年6月2日，学弟学妹刚刚结束北部赛区超级对抗赛，学弟学妹取得了很大进步，做得很好，ta们是最棒的！**

在该方案的基础上学妹对部分功能进行了优化与完善，在比赛中获得了不错的效果，方案将在完善后开源出来。

该方案在同轴校准上存在一定问题，通过理论确实可以推算出在一定距离上限的情况下可以忽略同轴校准，但是赛场距离有限，想要取得一个优秀的效果很难忽略高精度同轴校准；

方案目前置入了相关推算工具可以获得在固定距离下所需偏差大小，但是在赛场上我们需要动态过程，在距离变化的情况下获得偏差(*Can LiDAR assist in accurately measuring distance in real time?*)

学妹通过手眼标定优化了这个问题。

---

<p align="center">
  <img src="docs/boresight-calibration.png" width="" alt="boresight calibration">
</p>


---

## 许可证

除非文件头、子目录说明或第三方组件声明另有说明，本仓库中由作者原创的**源代码、配置文件与文档内容**，均按照 **GNU Affero General Public License v3.0 only（AGPL-3.0-only）** 授权发布。

**SPDX-License-Identifier: AGPL-3.0-only**

你可以在 **AGPL-3.0-only** 的条款下使用、复制、修改和再分发本项目。若你**分发本项目的修改版本**，或将修改版本作为**网络服务**向用户提供交互访问，则应按照 AGPL-3.0-only 的要求，提供相应的**完整对应源代码**，并保留原有的**版权声明、许可证声明及修改说明**。

本仓库中引用的**第三方库、SDK、工具链、图片、商标、硬件产品名称及相关资料**，不因本仓库的 AGPL-3.0-only 声明而改变其原有权利归属或许可条件。相关内容应遵循其各自的**许可证、EULA 或权利人声明**。

本项目按**现状**提供，不提供任何明示或默示担保，包括但不限于**适销性、特定用途适用性和非侵权担保**。详细条款请参见仓库根目录下的 `LICENSE` 文件。



---

<p align="center">
  <img src="docs/we.jpg" width="" alt="we">
</p>


<p align="center">
  <img src="docs/best.jpg" width="" alt="jpg">
</p>


<p align="center">
  <img src="docs/radar.jpg" width="" alt="radar">
</p>


---


<p align="center">
  <img src="docs/Horizon.png" width="200" alt="Horizon Team">
</p>



<div align="center">
Copyright © 2026 ROBOMASTER · 华北理工大学 HORIZON 战队 · 雷达组 - YAOYUZHUO<br/>
Licensed under the GNU Affero General Public License v3.0 (AGPL-3.0).<br/>
Use, modification, and redistribution are permitted under the terms of AGPL-3.0.<br/>
The complete corresponding source must be made available.<br/>
2026 年 01 月 09 日

</div>
