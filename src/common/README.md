# common

跨包共享的类型与配置工具，避免结构体分散定义。

## 输入/输出

- 输入：配置文件（YAML）
- 输出：统一数据结构与配置读取接口

## 共享结构

- `common::TargetMeasurement`：检测输出（uv/置信度/时间戳）
- `common::GimbalState`：云台回传（pitch/yaw/前馈/时间戳，单位 deg/deg/s）
- `common::GimbalCommand`：云台指令（pitch/yaw/前馈/时间戳，单位 deg/deg/s）
- `common::CameraModel`：相机内参（fx/fy/cx/cy）
- `common::Boresight`：激光/相机对准点（u_L/v_L）

## 配置接口

- `common::loadCameraModel(path, &model)`
- `common::loadBoresight(path, &boresight)`
- `common::nowMs()`：主机毫秒时间戳
## 坐标与单位约定

- yaw：左为正右为负
- pitch：下为正上为负
- 角度单位：deg
- 角速度单位：deg/s
