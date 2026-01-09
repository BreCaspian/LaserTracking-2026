#pragma once

#include "Inference.h"
#include <cstdint>
#include <memory>
#include <vector>

namespace TRTInferX
{
    enum class MemoryType
    {
        CPU,
        GPU
    };

    enum class ColorSpace
    {
        BGR,
        RGB,
        GRAY
    };

    enum class Layout
    {
        HWC,
        CHW
    };

    enum class DType
    {
        UINT8,
        FP16,
        FP32
    };

    enum class PreprocessMode
    {
        LETTERBOX,
        RESIZE
    };

    enum class OutputMode
    {
        AUTO,
        PACKED_NMS,
        RAW_WITH_NMS,
        RAW_ONLY
    };

    struct ImageInput
    {
        MemoryType mem{MemoryType::CPU};
        void *data{nullptr}; // CPU: Mat.data, GPU: device ptr
        int width{0};
        int height{0};
        int stride_bytes{0}; // bytes stride
        ColorSpace color{ColorSpace::BGR};
        Layout layout{Layout::HWC};
        DType dtype{DType::UINT8};
        PreprocessMode prep{PreprocessMode::LETTERBOX};
        int target_w{0};
        int target_h{0};
        int device_id{0};      // GPU only
        void *cuda_stream{nullptr}; // GPU only
        int64_t timestamp_ms{0};
        int roi_x{0}, roi_y{0}, roi_w{0}, roi_h{0}; // optional ROI
    };

    struct Det
    {
        float x1{0}, y1{0}, x2{0}, y2{0}; // original image coords
        float score{0};
        int cls{0};
        int batch{0};
        void *mask{nullptr};
        void *pose{nullptr};
    };

    struct PreprocInfo
    {
        float scale{1.0f};
        float scale_x{1.0f};
        float scale_y{1.0f};
        float padw{0.0f};
        float padh{0.0f};
        int src_w{0};
        int src_h{0};
    };

    struct Result
    {
        std::vector<Det> dets;
        PreprocInfo preproc;
    };

    struct EngineConfig
    {
        std::string engine_path;
        int device{0};
        int max_batch{16};
        int streams{1};
        bool auto_streams{false};
        PreprocessMode prep{PreprocessMode::LETTERBOX};
        int target_w{640};
        int target_h{640};
        OutputMode out_mode{OutputMode::AUTO};
        int num_classes{1};
        float nms_score{0.25f}; // used when building internal NMS engine
        float nms_iou{0.45f};   // used when building internal NMS engine
    };

    struct InferOptions
    {
        float conf{0.25f};
        float iou{0.45f};
        bool apply_sigmoid{false};
        int max_det{300};
        int stream_override{-1};
        int box_fmt{0}; // 0=cxcywh, 1=xyxy
    };

    class Api
    {
    public:
        Api() = default;
        ~Api() = default;
        bool load(const EngineConfig &cfg);
        // 返回统一 Det
        std::vector<std::vector<Det>> infer(const std::vector<ImageInput> &batch, const InferOptions &opt);
        // 返回 Det + 预处理尺度信息
        std::vector<Result> inferWithInfo(const std::vector<ImageInput> &batch, const InferOptions &opt);
        void warmup(int batch, int iters);

    private:
        bool validateConfig() const;
        EngineConfig cfg_;
        InferOptions default_opt_;
        std::unique_ptr<TRTInferV1::TRTInfer> core_;
    };
}
