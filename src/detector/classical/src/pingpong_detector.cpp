#include "detector_classical/pingpong_detector.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include <opencv2/imgproc.hpp>

namespace detector {

PingPongDetector::PingPongDetector(PingPongConfig cfg) : cfg_(cfg) {}

bool PingPongDetector::loadConfig(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open config: " << path << "\n";
        return false;
    }
    fs["h_min"] >> cfg_.h_min;
    fs["h_max"] >> cfg_.h_max;
    fs["s_min"] >> cfg_.s_min;
    fs["s_max"] >> cfg_.s_max;
    fs["v_min"] >> cfg_.v_min;
    fs["v_max"] >> cfg_.v_max;
    fs["blur_kernel"] >> cfg_.blur_kernel;
    fs["morph_kernel"] >> cfg_.morph_kernel;
    fs["morph_iter"] >> cfg_.morph_iter;
    fs["min_area"] >> cfg_.min_area;
    fs["max_area"] >> cfg_.max_area;
    fs["min_circularity"] >> cfg_.min_circularity;
    fs["min_fill_ratio"] >> cfg_.min_fill_ratio;
    fs["min_radius"] >> cfg_.min_radius;
    fs["max_radius"] >> cfg_.max_radius;
    if (!fs["downscale"].empty()) {
        fs["downscale"] >> cfg_.downscale;
    }
    if (!fs["use_roi"].empty()) {
        fs["use_roi"] >> cfg_.use_roi;
    }
    if (!fs["roi_expand_px"].empty()) {
        fs["roi_expand_px"] >> cfg_.roi_expand_px;
    }
    if (!fs["roi_min_size"].empty()) {
        fs["roi_min_size"] >> cfg_.roi_min_size;
    }
    if (!fs["roi_fallback_full"].empty()) {
        fs["roi_fallback_full"] >> cfg_.roi_fallback_full;
    }
    if (!fs["track_enable"].empty()) {
        fs["track_enable"] >> cfg_.track_enable;
    }
    if (!fs["max_missed"].empty()) {
        fs["max_missed"] >> cfg_.max_missed;
    }
    if (!fs["smooth_alpha"].empty()) {
        fs["smooth_alpha"] >> cfg_.smooth_alpha;
    }
    if (!fs["max_jump_px"].empty()) {
        fs["max_jump_px"] >> cfg_.max_jump_px;
    }
    if (!fs["max_pred_ms"].empty()) {
        fs["max_pred_ms"] >> cfg_.max_pred_ms;
    }
    return true;
}

void PingPongDetector::setConfig(const PingPongConfig& cfg) {
    cfg_ = cfg;
}

const PingPongConfig& PingPongDetector::config() const {
    return cfg_;
}

double PingPongDetector::calcCircularity(double area, double perimeter) {
    if (perimeter <= 1e-6) {
        return 0.0;
    }
    return 4.0 * M_PI * area / (perimeter * perimeter);
}

void PingPongDetector::hsvThreshold(const cv::Mat& hsv, const PingPongConfig& cfg, cv::Mat* mask) {
    if (!mask) {
        return;
    }
    cv::Scalar lower1(cfg.h_min, cfg.s_min, cfg.v_min);
    cv::Scalar upper1(cfg.h_max, cfg.s_max, cfg.v_max);
    if (cfg.h_min <= cfg.h_max) {
        cv::inRange(hsv, lower1, upper1, *mask);
        return;
    }
    cv::Mat mask1, mask2;
    cv::Scalar lower2(0, cfg.s_min, cfg.v_min);
    cv::Scalar upper2(cfg.h_max, cfg.s_max, cfg.v_max);
    cv::Scalar lower3(cfg.h_min, cfg.s_min, cfg.v_min);
    cv::Scalar upper3(179, cfg.s_max, cfg.v_max);
    cv::inRange(hsv, lower2, upper2, mask1);
    cv::inRange(hsv, lower3, upper3, mask2);
    cv::bitwise_or(mask1, mask2, *mask);
}

std::vector<Detection> PingPongDetector::detect(const cv::Mat& bgr, int64_t timestamp) {
    std::vector<Detection> results;
    if (bgr.empty()) {
        return results;
    }

    auto toSeconds = [](int64_t now, int64_t last) -> double {
        int64_t delta = now - last;
        if (delta <= 0) {
            return 0.0;
        }
        int64_t scale_ref = std::max(std::abs(now), std::abs(last));
        if (scale_ref > 1000000000000LL) {
            return static_cast<double>(delta) / 1e9;
        }
        if (scale_ref > 1000000000LL) {
            return static_cast<double>(delta) / 1e6;
        }
        return static_cast<double>(delta) / 1e3;
    };

    double dt_s = 0.0;
    if (has_track_ && timestamp > last_ts_) {
        dt_s = toSeconds(timestamp, last_ts_);
    }
    const bool has_dt = dt_s > 1e-6 && dt_s < 1.0;

    cv::Point2f predicted = track_center_;
    if (has_track_ && has_dt) {
        predicted = track_center_ + velocity_ * static_cast<float>(dt_s);
    }

    double scale = cfg_.downscale;
    if (scale <= 0.0 || scale > 1.0) {
        scale = 1.0;
    }

    cv::Mat work = bgr;
    if (scale < 1.0) {
        cv::resize(bgr, work, cv::Size(), scale, scale, cv::INTER_AREA);
    }

    cv::Rect roi_rect(0, 0, work.cols, work.rows);
    if (cfg_.use_roi && has_track_) {
        cv::Point2f pred_s = predicted * static_cast<float>(scale);
        int pad = std::max(1, static_cast<int>(std::round(cfg_.roi_expand_px * scale)));
        int min_size = std::max(8, static_cast<int>(std::round(cfg_.roi_min_size * scale)));
        int size = std::max(min_size, pad * 2);
        int x0 = static_cast<int>(std::round(pred_s.x - size / 2.0f));
        int y0 = static_cast<int>(std::round(pred_s.y - size / 2.0f));
        roi_rect = cv::Rect(x0, y0, size, size) &
            cv::Rect(0, 0, work.cols, work.rows);
        if (roi_rect.width < min_size || roi_rect.height < min_size) {
            roi_rect = cv::Rect(0, 0, work.cols, work.rows);
        }
    }

    auto detectInMat = [&](const cv::Mat& src, const cv::Point& offset) -> std::vector<Detection> {
        cv::Mat hsv;
        if (cfg_.blur_kernel > 1) {
            int k = cfg_.blur_kernel | 1;
            cv::Mat blurred;
            cv::GaussianBlur(src, blurred, cv::Size(k, k), 0);
            cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
        } else {
            cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
        }

        cv::Mat mask;
        hsvThreshold(hsv, cfg_, &mask);

        int k = std::max(1, cfg_.morph_kernel | 1);
        if (k > 1) {
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), cfg_.morph_iter);
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), cfg_.morph_iter);
        }

        double area_scale = scale * scale;
        double radius_scale = scale;
        double min_area = cfg_.min_area * area_scale;
        double max_area = cfg_.max_area * area_scale;
        double min_radius = cfg_.min_radius * radius_scale;
        double max_radius = cfg_.max_radius * radius_scale;

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<Detection> candidates;
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < min_area || area > max_area) {
                continue;
            }
            double perimeter = cv::arcLength(contour, true);
            double circularity = calcCircularity(area, perimeter);
            if (circularity < cfg_.min_circularity) {
                continue;
            }
            cv::Point2f center;
            float radius = 0.0f;
            cv::minEnclosingCircle(contour, center, radius);
            if (radius < min_radius || radius > max_radius) {
                continue;
            }
            double circle_area = M_PI * radius * radius;
            double fill = circle_area > 1e-6 ? area / circle_area : 0.0;
            if (fill < cfg_.min_fill_ratio) {
                continue;
            }

            cv::Point2f center_s = center + cv::Point2f(static_cast<float>(offset.x),
                                                        static_cast<float>(offset.y));
            cv::Point2f center_o = center_s * static_cast<float>(1.0 / scale);
            float radius_o = radius * static_cast<float>(1.0 / scale);

            cv::Rect bbox = cv::boundingRect(contour);
            cv::Rect bbox_s(bbox.x + offset.x, bbox.y + offset.y, bbox.width, bbox.height);
            cv::Rect bbox_o(
                static_cast<int>(std::round(bbox_s.x / scale)),
                static_cast<int>(std::round(bbox_s.y / scale)),
                static_cast<int>(std::round(bbox_s.width / scale)),
                static_cast<int>(std::round(bbox_s.height / scale)));

            Detection det;
            det.valid = true;
            det.label = "pingpong";
            det.center = center_o;
            det.radius = radius_o;
            det.bbox = bbox_o;
            det.confidence = static_cast<float>(std::min(1.0, circularity));
            det.confidence = static_cast<float>(std::min(1.0, det.confidence * fill));
            candidates.push_back(det);
        }
        return candidates;
    };

    std::vector<Detection> candidates = detectInMat(work(roi_rect), roi_rect.tl());
    if (candidates.empty() && cfg_.use_roi && cfg_.roi_fallback_full &&
        (roi_rect.width != work.cols || roi_rect.height != work.rows)) {
        candidates = detectInMat(work, cv::Point(0, 0));
    }

    const Detection* best = nullptr;
    double best_score = 0.0;
    for (const auto& det : candidates) {
        double score = det.confidence * det.radius;
        if (score > best_score) {
            best_score = score;
            best = &det;
        }
    }

    if (!cfg_.track_enable) {
        if (best) {
            results.push_back(*best);
        }
        return results;
    }

    bool has_meas = best != nullptr;

    if (has_meas && best->valid) {
        Detection det = *best;
        bool accept = true;
        if (has_track_ && has_dt) {
            cv::Point2f predicted = track_center_ + velocity_ * static_cast<float>(dt_s);
            double jump = cv::norm(det.center - predicted);
            if (jump > cfg_.max_jump_px) {
                accept = false;
            }
        }

        if (accept) {
            cv::Point2f prev_center = track_center_;
            if (!has_track_) {
                track_center_ = det.center;
                track_radius_ = det.radius;
                track_confidence_ = det.confidence;
                velocity_ = cv::Point2f(0.0f, 0.0f);
            } else {
                float alpha = static_cast<float>(cfg_.smooth_alpha);
                track_center_ = det.center * alpha + track_center_ * (1.0f - alpha);
                track_radius_ = det.radius * alpha + track_radius_ * (1.0f - alpha);
                track_confidence_ = det.confidence * alpha + track_confidence_ * (1.0f - alpha);
                if (has_dt) {
                    velocity_ = (track_center_ - prev_center) * static_cast<float>(1.0 / dt_s);
                }
            }

            has_track_ = true;
            missed_ = 0;
            last_ts_ = timestamp;

            Detection out = det;
            out.center = track_center_;
            out.radius = track_radius_;
            out.confidence = std::min(1.0f, std::max(0.0f, track_confidence_));
            out.bbox = cv::Rect(
                static_cast<int>(std::round(out.center.x - out.radius)),
                static_cast<int>(std::round(out.center.y - out.radius)),
                static_cast<int>(std::round(out.radius * 2.0f)),
                static_cast<int>(std::round(out.radius * 2.0f)));
            results.push_back(out);
            return results;
        }
    }

    if (has_track_) {
        missed_++;
        if (has_dt && missed_ <= cfg_.max_missed && (dt_s * 1000.0) <= cfg_.max_pred_ms) {
            track_center_ = track_center_ + velocity_ * static_cast<float>(dt_s);
            track_confidence_ *= 0.9f;

            Detection out;
            out.valid = true;
            out.label = "pingpong";
            out.center = track_center_;
            out.radius = track_radius_;
            out.confidence = std::min(1.0f, std::max(0.0f, track_confidence_ * 0.8f));
            out.bbox = cv::Rect(
                static_cast<int>(std::round(out.center.x - out.radius)),
                static_cast<int>(std::round(out.center.y - out.radius)),
                static_cast<int>(std::round(out.radius * 2.0f)),
                static_cast<int>(std::round(out.radius * 2.0f)));
            results.push_back(out);
            last_ts_ = timestamp;
            return results;
        }
    }

    has_track_ = false;
    missed_ = 0;
    velocity_ = cv::Point2f(0.0f, 0.0f);
    return results;
}

}  // namespace detector
