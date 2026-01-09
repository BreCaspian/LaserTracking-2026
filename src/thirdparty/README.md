# Third-Party Dependencies

This directory contains third-party prebuilt binaries required by the project.
These dependencies are **optional** and only required for specific features .

## ONNX Runtime (GPU)
```

thirdparty/
├── onnxruntime -> onnxruntime-linux-x64-gpu-1.23.2
└── onnxruntime-linux-x64-gpu-1.23.2/

```
- `onnxruntime-linux-x64-gpu-1.23.2`: Official prebuilt ONNX Runtime GPU package.
- `onnxruntime`: Symbolic link to the active ONNX Runtime version.

## Installation

```bash
cd src/thirdparty
tar -xzf onnxruntime-linux-x64-gpu-1.23.2.tgz
ln -s onnxruntime-linux-x64-gpu-1.23.2 onnxruntime
```

## Notes

- This directory is excluded from version control due to its size.
- ONNX Runtime GPU requires compatible CUDA and cuDNN versions.
