# parallax_estimator

激光-相机视差距离评估工具

## 功能

给定相机内参与激光-相机基线，输出像素偏置随距离变化的估计，并计算“可忽略偏置”的距离阈值。

## 输入参数说明（config/parallax.yaml）

- `fx, fy`：相机内参（像素）通过相机标定获得
- `bx, by`：相机光心到激光出光点的基线（米）手动测量估计
  - `bx`：水平向右为正
  - `by`：垂直向下为正
- `z_min, z_max`：评估距离范围（米）
- `z_ref`：参考距离（米）
  - `0` 表示无穷远对齐
  - 例如在 30m 处做过 boresight，则填 `30`
- `step`：输出采样步长（米）
- `eps_list`：像素阈值列表（px），例如 `[2, 3, 5]`

## 数学模型（工具内部口径）

绝对偏置（口径 A）：

```
du(Z) = fx * bx / Z
dv(Z) = fy * by / Z
```

相对偏置（口径 B，若设置 z_ref）：

```
du_res(Z) = du(Z) - du(z_ref)
dv_res(Z) = dv(Z) - dv(z_ref)
```

“偏置可忽略”的距离下限：

```
z_min = max( fx|bx|/eps , fy|by|/eps )
```

## 输出内容

- 绝对偏置阈值：每个 `eps` 对应的 `z >= ...` 距离下限
- 相对偏置区间（若设置了 `z_ref`）
- 采样表（可导出 CSV）：`z, du, dv`

## 构建

```bash
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

## 运行示例

绝对偏置：

```bash
./parallax_demo --config ../config/parallax.yaml
```

输出 CSV：

```bash
./parallax_demo --config ../config/parallax.yaml --csv ../outputs/parallax.csv
```

相对偏置（参考距离 z_ref 生效）：

```bash
./parallax_demo --config ../config/parallax.yaml --relative
```

## 示例输出解释

示例命令：

```bash
./parallax_demo --config ../config/parallax.yaml --csv ../outputs/parallax.csv
```

示例输出：

```
fx=18346.2 fy=18215.6 bx=0.02 by=0
z_range=[1, 28] step=1
eps_list: 2, 3, 5
abs eps=2px -> z >= 183.462 m
abs eps=3px -> z >= 122.308 m
abs eps=5px -> z >= 73.3848 m
Saved CSV: ../outputs/parallax.csv
Samples: 28
z=1 du=366.924 dv=0
z=28 du=13.1044 dv=0
```

说明：

- `abs eps=2px -> z >= 183.462 m`：若要求偏置≤2px，至少需要 183m。
- `Samples: 28`：在 1~28m 范围内按 1m 步长采样。
- `z=1 du=366.924`：1m 距离时水平偏置约 367px（基线 2cm + 长焦放大）。
- `z=28 du=13.1044`：28m 距离仍有约 13px 偏置。
- `Saved CSV`：距离-偏置表已导出。
