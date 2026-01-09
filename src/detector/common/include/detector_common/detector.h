#ifndef DETECTOR_DETECTOR_H
#define DETECTOR_DETECTOR_H

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace detector {

struct Detection {
    bool valid = false;
    std::string label;
    cv::Point2f center{};
    float radius = 0.0f;
    cv::Rect bbox{};
    float confidence = 0.0f;
};

class Detector {
public:
    virtual ~Detector() = default;
    virtual std::vector<Detection> detect(const cv::Mat& bgr, int64_t timestamp) = 0;
};

}  // namespace detector

#endif  // DETECTOR_DETECTOR_H
