# boresight_calibrator

激光-相机同轴（boresight）校准工具。

## 流程

1) 固定目标，检测框中心作为目标中心。  
2) 手动移动十字准星，让其与激光点重合。  
3) 最终准星坐标即 `u_L/v_L`（写入控制配置）。

注意：该偏置对距离敏感，建议在常用工作距离处校准。

- **固定差值=固定偏置**只在固定距离或近距离范围成立。
- 激光与相机存在平移基线，不同距离会产生不同像素偏置。
- 若需多距离准确：采集多距离的 `u_L/v_L` 曲线，或建立视差模型（参考 `parallax_estimator`）。

## 构建

```bash
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

## 运行

```bash
./boresight_calibrator \
  --camera-config ../../hik_camera/config/config.yaml \
  --detector-config ../../detector/config/detector.yaml \
  --boresight-config ../../control/config/control.yaml \
  --boresight-out ./boresight.yaml
```

## 使用教程

### 1. 现场准备

- 固定相机与激光发射器，确保刚性连接不松动。
- 选择一个清晰、静止的目标（建议白色小球或亮色纸面）接收平面于与相机镜面平行最佳。
- 让激光点落在目标附近，目标保持静止。

### 2. 启动工具

```bash
./boresight_calibrator \
  --camera-config ../../hik_camera/config/config.yaml \
  --detector-config ../../detector/config/detector.yaml \
  --boresight-config ../../control/config/control.yaml \
  --boresight-out ./boresight.yaml
```

说明：
- 工具会显示两组十字：绿色为检测目标中心，红色为当前 boresight。
- 左上角会显示 `u_L/v_L` 与偏移量 `du/dv`。

### 3. 校准步骤

1) 确认画面中有检测框和绿色十字（目标中心）。  
2) 使用方向键或鼠标，把红色十字移动到激光点上。  
3) 观察 `du/dv` 稳定后，按 `s` 保存。  
4) 将保存的 `boresight.yaml` 中 `u_L/v_L` 写回控制配置。

### 4. 常见问题

- **没有检测框**：检查 detector 配置、光照与目标颜色。
- **激光点不清晰**：降低曝光、增大激光功率或换高反光目标。
- **偏移不稳定**：目标不静止或云台在微动，先锁定目标再校准。

## 操作说明

- 方向键：移动十字（像素）
- `W/A/S/D`：移动十字（备用）
- `[` / `]` 或 `-` / `+`：减小/增大步长
- 鼠标左键：移动十字到点击位置
- 鼠标右键：设置当前十字为初始值
- 鼠标滚轮：调节步长
- `c`：将十字重置到当前目标中心，同时设置为初始值
- `r`：重置为初始值
- `i`：将当前十字作为新的“初始值”
- `s`：保存 boresight 到 `--boresight-out`
- `q` / `ESC`：退出
