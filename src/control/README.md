# control

**核心控制部分 像素误差 → 云台控制模块**

---

## 上下游接口

- 上游：`common::TargetMeasurement` + `common::CameraModel` + `common::Boresight`
- 云台反馈：`common::GimbalState`（deg/deg/s）
- 输出：`common::GimbalCommand`（pitch/yaw + pitch_rate/yaw_rate）
- 下游：gimbal_serial（串口收发）
- 时间戳：`TargetMeasurement.timestamp` 单位为毫秒（ms）
- 控制周期：使用 `ctrl_dt_*` 估计 update 周期用于限速
- 角度单位：**deg**，角速度单位：**deg/s**（与电控一致）
- 坐标约定：
  - 图像坐标：u 向右为正，v 向下为正
  - 云台角度：yaw 左为正右为负；pitch 下为正上为负
  - 控制符号：`yaw_sign` 作用于 u 轴误差；`pitch_sign` 作用于 v 轴误差
  - 当前实测：`pitch_sign = -1` 时收敛，以实测为准

## 构建

```bash
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

## 运行（系统串联）

```bash
./control_system_demo \
  --camera-config ../../hik_camera/config/config.yaml \
  --detector-config ../../detector/config/detector.yaml \
  --control-config ../config/control.yaml \
  --port /dev/ttyACM0 \
  --baud 115200 \
  --send-hz 400
```

## 云台测试

```bash
./control_demo --control-config ../config/control.yaml --u 320 --v 240 --pitch 0 --yaw 0
```

## 配置

`config/control.yaml`：控制参数、相机内参、boresight 像素点。

关键方向参数：
- `yaw_sign`: yaw 方向符号（+1=右正左负，-1=左正右负）
- `pitch_sign`: pitch 方向符号（+1=下正上负，-1=上正下负）

## 核心流程

- 像素误差（px）：`du = u - u_L`，`dv = v - v_L`
- 角度误差（deg）：`dyaw = yaw_sign * atan(du / fx) * 180/pi`，`dpitch = pitch_sign * atan(dv / fy) * 180/pi`
- 死区与滞回（px）：`deadband_px`，并使用 1px 滞回避免边界抖动
- 位置式控制（deg）：`yaw_cmd = state.yaw + kp * dyaw`，`pitch_cmd = state.pitch + kp * dpitch`
- 输出平滑：`lowpass_alpha` 一阶 IIR（历史指令低通）
- 限速保护：使用 `steady_clock` 估计 `dt`，单次步进受 `max_angle_rate * dt` 约束

## 控制原理

控制器以视觉误差为输入，使用相机内参将像素误差转换为视线角误差，并以比例控制生成云台绝对角度指令。输出侧加入死区和滞回抑制微抖，通过一阶 IIR 低通滤波抑制高频噪声，速度限幅保证指令满足云台动态能力。目标丢失时切换到搜索轨迹，目标恢复后回到跟踪。角速度前馈与阻尼反馈用于提升动态跟随与抑制过冲。

## 角速度前馈

- 开关：`use_velocity_ff`
- 估计：由像素速度换算角速度（单位 deg/s）
- 保护：低通 `ff_alpha` + 限幅 `ff_rate_max` + dt 上限 `ff_dt_max_ms`
- 说明：control 不做前馈注入，仅发送前馈值，由电控决定是否使用

## 速度反馈

- 开关：`use_damping`
- 来源：`damping_source = meas | gimbal`
- 作用：对角速度做负反馈，提升跟随速度并抑制过冲
- 参数：`damping_kd`（秒，乘以 deg/s 形成角度补偿）

## 发送帧字段

- Byte1-4：pitch 角度（float，deg）
- Byte5-8：yaw 角度（float，deg）
- Byte9-12：pitch 前馈角速度（float，deg/s）
- Byte13-16：yaw 前馈角速度（float，deg/s）
- Byte17-20：时间戳（ms）

## 丢失目标搜索策略

目标丢失时支持两种策略，使用 `scan_pattern` 切换：

- `circle`：平滑画圆搜索（简单、稳定，但是容易出现视野死区）
- `spiral`：恒路径速度扩张式阿基米德螺线（覆盖效率更高）
  - 起始方向对准目标丢失前的方向（基于最后一次有效测量）

常用参数（见 `config/control.yaml`）：

- 画圆：`scan_radius_deg`，`scan_rate_hz`
- 螺线：`scan_spacing_deg`，`scan_speed_deg_s`，`scan_r_max_deg`，`scan_k_yaw`，`scan_k_pitch`
- 螺线往返：`scan_spiral_return`（到达最大半径后沿原路径回到中心并往复）
- 搜索抖动抑制：`scan_enter_delay_ms`，`scan_reacq_confirm_frames`
- 启动策略：`startup_check_frames`（仅当 `startup_prep_ms <= 0` 时生效），`startup_home_pitch`，`startup_home_yaw`
- 启动准备阶段：`startup_prep_ms`（总时长），`startup_hold_ms`，`startup_home_ms`，`startup_validate_ms`
- 启动顺序：`startup_validate_first`（先校验再回正）
- 早退：`startup_allow_early_exit`（校验阶段检测到目标则跳过回正）
- 回正门控：`startup_require_home` + `startup_home_tol_deg` + `startup_home_max_extra_ms`

建议：

- 想快速找回，先提高 `scan_speed_deg_s`，再根据检测稳定性调整。
- `scan_k_yaw > scan_k_pitch` 可实现左右宽、上下窄的椭圆搜索。

### 恒速扩张螺线

- 非闭合外扩路径，比画圆更快覆盖新增区域。
- 相邻圈距恒定，覆盖均匀，恒路径速度避免中心慢、外圈快。
- `scan_k_yaw/scan_k_pitch` 用于各向异性视场缩放。

### 参数建议

- 圈距：`scan_spacing_deg ≈ 0.8 * min(FOV_eff)`（示例≈9.05°）
- 速度：检测稳定时 15–18°/s
- 半径：结合云台限位设置

推荐一组偏快参数（检测稳定时）：

```yaml
scan_pattern: "spiral"
scan_spacing_deg: 9.05
scan_speed_deg_s: 18.0
scan_r_max_deg: 25.0
scan_k_yaw: 1.5
scan_k_pitch: 1.0
```

丢失抖动明显时：

- `scan_speed_deg_s` 降到 12–15
- `scan_spacing_deg` 降到 7–8
