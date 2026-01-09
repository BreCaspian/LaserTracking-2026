#include "detector_registry/backend.h"

#include <filesystem>
#include <iostream>

#include <opencv2/core.hpp>

#include "detector_classical/pingpong_detector.h"

#ifdef DETECTOR_WITH_TRT
#include "detector_trt/trt_detector.h"
#endif

namespace detector {

bool loadDetectorConfig(const std::string& path, DetectorConfig* out) {
    if (!out) {
        return false;
    }
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        return false;
    }
    if (fs["backend"].empty() && fs["classical_config"].empty() && fs["trt_config"].empty()) {
        return false;
    }
    DetectorConfig cfg;
    if (!fs["backend"].empty()) {
        fs["backend"] >> cfg.backend;
    }
    if (!fs["classical_config"].empty()) {
        fs["classical_config"] >> cfg.classical_config;
    }
    if (!fs["trt_config"].empty()) {
        fs["trt_config"] >> cfg.trt_config;
    }
    const std::filesystem::path base = std::filesystem::path(path).parent_path();
    if (!cfg.classical_config.empty()) {
        std::filesystem::path p(cfg.classical_config);
        if (p.is_relative()) {
            cfg.classical_config = (base / p).string();
        }
    }
    if (!cfg.trt_config.empty()) {
        std::filesystem::path p(cfg.trt_config);
        if (p.is_relative()) {
            cfg.trt_config = (base / p).string();
        }
    }
    *out = cfg;
    return true;
}

std::unique_ptr<Detector> createDetectorFromConfig(const DetectorConfig& cfg, std::string* err) {
    if (cfg.backend == "classical") {
        auto det = std::make_unique<PingPongDetector>();
        if (!cfg.classical_config.empty()) {
            if (!det->loadConfig(cfg.classical_config)) {
                if (err) {
                    *err = "Failed to load classical config: " + cfg.classical_config;
                }
                return nullptr;
            }
        }
        return det;
    }

    if (cfg.backend == "trt_engine") {
#ifdef DETECTOR_WITH_TRT
        auto det = std::make_unique<detector_trt::TrtDetector>();
        if (!cfg.trt_config.empty()) {
            if (!det->loadConfig(cfg.trt_config)) {
                if (err) {
                    *err = "Failed to load TRT config: " + cfg.trt_config;
                }
                return nullptr;
            }
        }
        return det;
#else
        if (err) {
            *err = "TRT backend not enabled at build time";
        }
        return nullptr;
#endif
    }

    if (err) {
        *err = "Unknown detector backend: " + cfg.backend;
    }
    return nullptr;
}

std::unique_ptr<Detector> createDetectorFromPath(const std::string& path, std::string* err) {
    DetectorConfig cfg;
    if (loadDetectorConfig(path, &cfg)) {
        return createDetectorFromConfig(cfg, err);
    }
    auto det = std::make_unique<PingPongDetector>();
    if (!det->loadConfig(path)) {
        if (err) {
            *err = "Failed to load detector config: " + path;
        }
        return nullptr;
    }
    return det;
}

}  // namespace detector
