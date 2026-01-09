# detector_classical

传统视觉检测后端（HSV/轮廓/圆度/ROI），作为稳定 fallback。**注意：仅用于测试使用**

## 处理流程

1) BGR → HSV，按阈值分割得到颜色掩膜  
2) 形态学开闭运算去噪、连通  
3) 轮廓提取，按面积/圆度/填充率筛选  
4) 输出单目标（`confidence * radius` 评分最高）  
5) 可选：基于上一帧目标的 ROI 加速  
6) 可选：简单跟踪（平滑 + 速度预测 + 丢检补偿）

## 输入/输出

- 输入：`cv::Mat bgr`
- 输出：`std::vector<detector::Detection>`（label=`pingpong`）
- 统一输出：`detector::toMeasurement` → `detector::TargetMeasurement`

## 接口

- `detector_classical/pingpong_detector.h`

## 配置

配置文件：`config/classical.yaml`

核心参数说明：
- `h_min/h_max`：HSV H 阈值
- `s_min/s_max`：HSV S 阈值
- `v_min/v_max`：HSV V 阈值
- `blur_kernel`：高斯模糊核（0=关闭）
- `morph_kernel`：形态学核大小
- `morph_iter`：形态学迭代次数
- `min_area/max_area`：轮廓面积阈值
- `min_circularity`：最小圆度
- `min_fill_ratio`：填充率阈值
- `min_radius/max_radius`：半径阈值
- `downscale`：下采样比例
- `use_roi`：ROI 开关
- `roi_expand_px`：ROI 扩展像素
- `roi_min_size`：ROI 最小边
- `roi_fallback_full`：ROI 失败退回全图
- `track_enable`：简单跟踪开关
- `max_missed`：最大丢检帧数
- `smooth_alpha`：平滑系数（0~1）
- `max_jump_px`：预测跳变阈值
- `max_pred_ms`：最大预测时间（ms）

## 使用方式

通过 `detector.yaml` 选择后端：

```yaml
backend: "classical"
classical_config: "../classical/config/classical.yaml"
```
