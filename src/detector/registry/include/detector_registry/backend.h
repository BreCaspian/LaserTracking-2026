#ifndef DETECTOR_BACKEND_H
#define DETECTOR_BACKEND_H

#include <memory>
#include <string>

#include "detector_common/detector.h"

namespace detector {

struct DetectorConfig {
    std::string backend = "classical";  // classical | trt_engine
    std::string classical_config;
    std::string trt_config;
};

bool loadDetectorConfig(const std::string& path, DetectorConfig* out);
std::unique_ptr<Detector> createDetectorFromConfig(const DetectorConfig& cfg, std::string* err);
std::unique_ptr<Detector> createDetectorFromPath(const std::string& path, std::string* err);

}  // namespace detector

#endif  // DETECTOR_BACKEND_H
