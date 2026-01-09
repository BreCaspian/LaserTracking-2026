## Detector Models (ONNX + TensorRT Engines)

> **Note:** Model files are **not stored in this directory**.
> Please download them from the official release page.

### Download https://github.com/BreCaspian/LaserTracking-2026/releases/tag/2026.01.09

---

### Overview

This release provides the detector demo model assets used by **LaserTracking**.
The models are distributed via GitHub Releases .

### Contents
- **ONNX models:** `best.onnx`, `best_raw.onnx`, `ball.onnx`
- **TensorRT engines:** `best_fp16.engine`, `best_int8.engine`
- **Calibration / misc:** `calib.bin`
- **YOLOv11 model (training artifact):** `ball.pt`

### Notes
- TensorRT engine files are **hardware- and TensorRT-version-dependent**.  
  If an engine cannot be loaded on your system, please rebuild it locally from the provided ONNX models.
- Recommended usage:
  - Use **ONNX** models for portability and cross-platform compatibility.
  - Use **TensorRT engines** for maximum runtime performance.
- For model conversion, please use the automated scripts in **TRTInferX**, or visit:  
  https://github.com/BreCaspian/TRTInferX

---

<p align="center">
  <img src="../../../docs/Horizon.png" width="200" alt="Horizon Team">
</p>



<div align="center">
Copyright © 2026 ROBOMASTER · 华北理工大学 HORIZON 战队 · 雷达组 - YAOYUZHUO<br/>
Licensed under the GNU Affero General Public License v3.0 (AGPL-3.0).<br/>
Use, modification, and redistribution are permitted under the terms of AGPL-3.0.<br/>
The complete corresponding source must be made available.<br/>
2026 年 01 月 09 日

</div>
