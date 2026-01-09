#ifndef TOOL_PARALLAX_ESTIMATOR_H
#define TOOL_PARALLAX_ESTIMATOR_H

#include <cstdint>
#include <string>
#include <vector>

namespace tool {

struct ParallaxConfig {
    double fx = 0.0;
    double fy = 0.0;
    double bx = 0.0;
    double by = 0.0;
    double z_min = 1.0;
    double z_max = 200.0;
    double z_ref = 0.0;  // 0=inf
    double step = 1.0;
    std::vector<double> eps_list;
};

struct ParallaxSample {
    double z = 0.0;
    double du = 0.0;
    double dv = 0.0;
};

bool loadParallaxConfig(const std::string& path, ParallaxConfig* cfg);

std::vector<ParallaxSample> evaluateAbsolute(const ParallaxConfig& cfg);
std::vector<ParallaxSample> evaluateRelative(const ParallaxConfig& cfg);

double minDistanceForEps(const ParallaxConfig& cfg, double eps_px);
std::pair<double, double> relativeRangeForEps(const ParallaxConfig& cfg, double eps_px);

bool saveCsv(const std::string& path, const std::vector<ParallaxSample>& samples);

}  // namespace tool

#endif  // TOOL_PARALLAX_ESTIMATOR_H
