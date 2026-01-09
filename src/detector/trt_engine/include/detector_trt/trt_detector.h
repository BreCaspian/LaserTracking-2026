#ifndef DETECTOR_TRT_DETECTOR_H
#define DETECTOR_TRT_DETECTOR_H

#include <cstdint>
#include <string>
#include <vector>

#include "detector_common/detector.h"

namespace detector_trt {

struct GpuInput {
    uint8_t* bgr = nullptr;
    int width = 0;
    int height = 0;
    int stride_bytes = 0;
    void* cuda_stream = nullptr;
    int64_t timestamp_ms = 0;
};

class TrtDetector : public detector::Detector {
public:
    TrtDetector();
    ~TrtDetector() override;

    bool loadConfig(const std::string& path);

    // Optional GPU path; when set and enabled, detect() will prefer this buffer.
    void setGpuInput(const GpuInput& input);
    void clearGpuInput();

    std::vector<detector::Detection> detect(const cv::Mat& bgr, int64_t timestamp) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace detector_trt

#endif  // DETECTOR_TRT_DETECTOR_H
