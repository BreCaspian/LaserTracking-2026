#ifndef DETECTOR_PINGPONG_DETECTOR_H
#define DETECTOR_PINGPONG_DETECTOR_H

#include "detector_common/detector.h"

#include <opencv2/core.hpp>

namespace detector {

struct PingPongConfig {
    int h_min = 8;
    int h_max = 22;
    int s_min = 120;
    int s_max = 255;
    int v_min = 80;
    int v_max = 255;
    int blur_kernel = 0;
    int morph_kernel = 5;
    int morph_iter = 2;
    double min_area = 50.0;
    double max_area = 1e9;
    double min_circularity = 0.7;
    double min_fill_ratio = 0.6;
    double min_radius = 3.0;
    double max_radius = 2000.0;
    double downscale = 1.0;
    bool use_roi = true;
    int roi_expand_px = 120;
    int roi_min_size = 64;
    bool roi_fallback_full = true;
    bool track_enable = true;
    int max_missed = 5;
    double smooth_alpha = 0.6;
    double max_jump_px = 80.0;
    int max_pred_ms = 200;
};

class PingPongDetector : public Detector {
public:
    explicit PingPongDetector(PingPongConfig cfg = {});

    bool loadConfig(const std::string& path);
    void setConfig(const PingPongConfig& cfg);
    const PingPongConfig& config() const;

    std::vector<Detection> detect(const cv::Mat& bgr, int64_t timestamp) override;

private:
    static double calcCircularity(double area, double perimeter);
    static void hsvThreshold(const cv::Mat& hsv, const PingPongConfig& cfg, cv::Mat* mask);

    PingPongConfig cfg_;
    bool has_track_ = false;
    int missed_ = 0;
    int64_t last_ts_ = 0;
    cv::Point2f track_center_{};
    float track_radius_ = 0.0f;
    float track_confidence_ = 0.0f;
    cv::Point2f velocity_{};
};

}  // namespace detector

#endif  // DETECTOR_PINGPONG_DETECTOR_H
