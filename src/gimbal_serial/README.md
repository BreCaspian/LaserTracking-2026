# gimbal_serial

云台串口收发包：按 22B 协议解析状态并发送指令。电控通信协议详细参考 [LaserTracking-2-Axis-Gimbal-ControlPart](https://github.com/BreCaspian/LaserTracking-2-Axis-Gimbal-ControlPart)

## 上下游接口

- 上游：`common::GimbalCommand`
- 输出：`common::GimbalState`
- 下游：control（用于闭环）

## 构建

```bash
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

## 运行

```bash
./gimbal_serial_demo --port /dev/ttyUSB0 --baud 115200 --pitch 1.0 --yaw -1.0 --send-hz 300
```

高频模式示例（统计发送/接收频率与调度抖动）：

```bash
./gimbal_serial_demo --port /dev/ttyUSB0 --baud 115200 --pitch 1.0 --yaw -1.0 --send-hz 500 --stats
```
*注意云台会有明显抖动*

## 说明

- 接收帧：22B（0xCD ... 0xDC）
- 发送帧：22B（0xCD ... 0xDC）
- 解析/打包位于 `include/gimbal_serial/protocol.h`
- 支持粘包/错位处理（寻找 0xCD/0xDC 对齐）
- 坐标约定：yaw 左为正右为负；pitch 下为正上为负
- 单位：角度 **deg**，角速度 **deg/s**
- demo 持续发送固定 `pitch/yaw` 指令，速率由 `--send-hz` 控制

## 帧字段（新协议）

- 0：0xCD
- 1-4：pitch (float, deg)
- 5-8：yaw (float, deg)
- 9-12：pitch_rate (float, deg/s)
- 13-16：yaw_rate (float, deg/s)
- 17-20：timestamp_ms (uint32)
- 21：0xDC


## 参数说明

- `--port` 串口设备名，例如 `/dev/ttyACM0`（默认 `/dev/ttyUSB0`）
- `--baud` 波特率，例如 `115200`（默认 115200）
- `--pitch` 发送的 pitch 角度（float, deg，默认 0）
- `--yaw` 发送的 yaw 角度（float, deg，默认 0）
- `--send-hz` 发送频率（Hz，默认 50；<=0 时每循环都发送，约 1ms 一次）
- `--stats` 打印 rx/tx 频率与最大调度延迟
- `--dump-hex` 打印原始回传字节（带时间戳与长度）
- `--reconnect` 断线重连开关（0/1，默认 1，需携带值）
- `--reconnect-delay-ms` 重连间隔（ms，默认 500）
- `--read-timeout-ms` 串口读超时（ms，默认 2；内部最小 1ms）

## 统计字段含义

- `rx_hz/tx_hz`：当前 1s 窗口内的收/发频率
- `max_late_us`：发送调度最大延迟（us）
- `read_err/write_err`：串口读/写错误累计数
- `reconnects`：触发重连的次数
- `bad_frames`：解析时发现的无效帧数量（帧头存在但帧尾不匹配）
- `drop_rate`：按**字节**统计的丢弃比例 = `discarded_bytes / total_bytes`
- 说明：串口是字节流，回传可能粘包/拆包，`drop_rate` 用字节口径更接近真实误码情况
