#include "detector_trt/trt_detector.h"

#include <algorithm>
#include <filesystem>
#include <iostream>

#include <opencv2/core.hpp>

#include "api.h"

namespace detector_trt {
namespace {
constexpr int kDefaultTopK = 1;
constexpr float kDefaultConf = 0.25f;
constexpr float kDefaultIou = 0.45f;

std::string toLower(std::string v) {
    std::transform(v.begin(), v.end(), v.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return v;
}

TRTInferX::PreprocessMode parsePreprocess(const std::string& v) {
    std::string s = toLower(v);
    if (s == "resize") {
        return TRTInferX::PreprocessMode::RESIZE;
    }
    return TRTInferX::PreprocessMode::LETTERBOX;
}

TRTInferX::OutputMode parseOutputMode(const std::string& v) {
    std::string s = toLower(v);
    if (s == "packed_nms") {
        return TRTInferX::OutputMode::PACKED_NMS;
    }
    if (s == "raw_with_nms") {
        return TRTInferX::OutputMode::RAW_WITH_NMS;
    }
    if (s == "raw_only") {
        return TRTInferX::OutputMode::RAW_ONLY;
    }
    return TRTInferX::OutputMode::AUTO;
}
}  // namespace

struct TrtDetector::Impl {
    TRTInferX::Api api;
    TRTInferX::EngineConfig engine_cfg;
    TRTInferX::InferOptions infer_opt;
    bool loaded = false;
    bool use_gpu_input = true;
    int top_k = kDefaultTopK;
    GpuInput gpu_input{};
    bool have_gpu_input = false;
    bool gpu_input_logged = false;
};

TrtDetector::TrtDetector() : impl_(std::make_unique<Impl>()) {}
TrtDetector::~TrtDetector() = default;

bool TrtDetector::loadConfig(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open TRT config: " << path << "\n";
        return false;
    }

    auto& cfg = impl_->engine_cfg;
    auto& opt = impl_->infer_opt;

    fs["engine_path"] >> cfg.engine_path;
    fs["device_id"] >> cfg.device;
    fs["target_w"] >> cfg.target_w;
    fs["target_h"] >> cfg.target_h;
    fs["max_batch"] >> cfg.max_batch;
    fs["streams"] >> cfg.streams;
    fs["auto_streams"] >> cfg.auto_streams;
    fs["num_classes"] >> cfg.num_classes;
    std::string preprocess = "letterbox";
    fs["preprocess"] >> preprocess;
    cfg.prep = parsePreprocess(preprocess);
    std::string out_mode = "auto";
    fs["output_mode"] >> out_mode;
    cfg.out_mode = parseOutputMode(out_mode);

    fs["conf"] >> opt.conf;
    fs["iou"] >> opt.iou;
    fs["max_det"] >> opt.max_det;
    fs["apply_sigmoid"] >> opt.apply_sigmoid;
    std::string box_fmt = "cxcywh";
    fs["box_fmt"] >> box_fmt;
    opt.box_fmt = (toLower(box_fmt) == "xyxy") ? 1 : 0;

    fs["top_k"] >> impl_->top_k;
    fs["use_gpu_input"] >> impl_->use_gpu_input;

    if (cfg.engine_path.empty()) {
        std::cerr << "TRT config missing engine_path\n";
        return false;
    }

    const std::filesystem::path base = std::filesystem::path(path).parent_path();
    std::filesystem::path engine_path(cfg.engine_path);
    if (engine_path.is_relative()) {
        cfg.engine_path = (base / engine_path).string();
    }

    if (cfg.max_batch <= 0) {
        cfg.max_batch = 1;
    }
    if (cfg.num_classes <= 0) {
        cfg.num_classes = 1;
    }
    if (impl_->top_k <= 0) {
        impl_->top_k = kDefaultTopK;
    }
    if (opt.conf <= 0.0f) {
        opt.conf = kDefaultConf;
    }
    if (opt.iou <= 0.0f) {
        opt.iou = kDefaultIou;
    }

    if (!impl_->api.load(cfg)) {
        std::cerr << "Failed to load TRT engine: " << cfg.engine_path << "\n";
        return false;
    }

    const bool raw_output = (cfg.out_mode == TRTInferX::OutputMode::RAW_ONLY ||
                             cfg.out_mode == TRTInferX::OutputMode::RAW_WITH_NMS);
    std::cout << "TRT engine loaded: raw_output=" << (raw_output ? "yes" : "no")
              << " input=" << cfg.target_w << "x" << cfg.target_h << "\n";

    impl_->loaded = true;
    return true;
}

void TrtDetector::setGpuInput(const GpuInput& input) {
    impl_->gpu_input = input;
    impl_->have_gpu_input = (input.bgr != nullptr && input.width > 0 && input.height > 0);
}

void TrtDetector::clearGpuInput() {
    impl_->gpu_input = GpuInput{};
    impl_->have_gpu_input = false;
}

std::vector<detector::Detection> TrtDetector::detect(const cv::Mat& bgr, int64_t timestamp) {
    std::vector<detector::Detection> out;
    if (!impl_->loaded) {
        return out;
    }

    TRTInferX::ImageInput input{};
    if (impl_->use_gpu_input && impl_->have_gpu_input) {
        input.mem = TRTInferX::MemoryType::GPU;
        input.data = impl_->gpu_input.bgr;
        input.width = impl_->gpu_input.width;
        input.height = impl_->gpu_input.height;
        input.stride_bytes = impl_->gpu_input.stride_bytes;
        input.cuda_stream = impl_->gpu_input.cuda_stream;
        input.timestamp_ms = impl_->gpu_input.timestamp_ms;
        if (!impl_->gpu_input_logged) {
            std::cout << "TRT input is GPU buffer (use_gpu_input=1)\n";
            impl_->gpu_input_logged = true;
        }
    } else {
        if (bgr.empty()) {
            return out;
        }
        input.mem = TRTInferX::MemoryType::CPU;
        input.data = bgr.data;
        input.width = bgr.cols;
        input.height = bgr.rows;
        input.stride_bytes = static_cast<int>(bgr.step);
        input.timestamp_ms = timestamp;
    }

    input.color = TRTInferX::ColorSpace::BGR;
    input.layout = TRTInferX::Layout::HWC;
    input.dtype = TRTInferX::DType::UINT8;
    input.prep = impl_->engine_cfg.prep;
    input.target_w = impl_->engine_cfg.target_w;
    input.target_h = impl_->engine_cfg.target_h;
    input.device_id = impl_->engine_cfg.device;

    std::vector<TRTInferX::ImageInput> batch{input};
    auto dets = impl_->api.infer(batch, impl_->infer_opt);
    if (dets.empty() || dets.front().empty()) {
        return out;
    }

    auto& d0 = dets.front();
    std::sort(d0.begin(), d0.end(),
              [](const TRTInferX::Det& a, const TRTInferX::Det& b) { return a.score > b.score; });

    int keep = std::min(static_cast<int>(d0.size()), impl_->top_k);
    out.reserve(static_cast<size_t>(keep));
    for (int i = 0; i < keep; ++i) {
        const auto& det = d0[static_cast<size_t>(i)];
        detector::Detection d;
        d.valid = true;
        d.label = "ball";
        d.bbox = cv::Rect(cv::Point2f(det.x1, det.y1),
                          cv::Point2f(det.x2, det.y2));
        d.center = cv::Point2f((det.x1 + det.x2) * 0.5f, (det.y1 + det.y2) * 0.5f);
        d.radius = std::max(d.bbox.width, d.bbox.height) * 0.5f;
        d.confidence = det.score;
        out.push_back(d);
    }

    return out;
}

}  // namespace detector_trt
