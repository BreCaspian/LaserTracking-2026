#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <cstdint>

#include <opencv2/core.hpp>

namespace common {

struct TargetMeasurement {
    bool valid = false;
    int64_t timestamp = 0;
    cv::Point2f uv{};
    float confidence = 0.0f;
};

struct GimbalState {
    float pitch = 0.0f;
    float yaw = 0.0f;
    float pitch_rate = 0.0f;
    float yaw_rate = 0.0f;
    uint32_t timestamp = 0;
};

struct GimbalCommand {
    float pitch = 0.0f;
    float yaw = 0.0f;
    float pitch_rate = 0.0f;
    float yaw_rate = 0.0f;
    uint32_t timestamp = 0;
};

struct CameraModel {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
};

struct Boresight {
    double u_L = 0.0;
    double v_L = 0.0;
};

}  // namespace common

#endif  // COMMON_TYPES_H
