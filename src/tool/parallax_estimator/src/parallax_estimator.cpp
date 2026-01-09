#include "tool/parallax_estimator.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>

#include <opencv2/core.hpp>

namespace tool {

namespace {

void computeOffset(double fx, double fy, double bx, double by, double z, double* du, double* dv) {
    if (!du || !dv || z <= 1e-9) {
        return;
    }
    *du = fx * bx / z;
    *dv = fy * by / z;
}
}  // namespace

bool loadParallaxConfig(const std::string& path, ParallaxConfig* cfg) {
    if (!cfg) {
        return false;
    }
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open config: " << path << "\n";
        return false;
    }
    fs["fx"] >> cfg->fx;
    fs["fy"] >> cfg->fy;
    fs["bx"] >> cfg->bx;
    fs["by"] >> cfg->by;
    fs["z_min"] >> cfg->z_min;
    fs["z_max"] >> cfg->z_max;
    if (!fs["z_ref"].empty()) {
        fs["z_ref"] >> cfg->z_ref;
    }
    if (!fs["step"].empty()) {
        fs["step"] >> cfg->step;
    }
    if (!fs["eps_list"].empty()) {
        cv::FileNode eps = fs["eps_list"];
        cfg->eps_list.clear();
        for (const auto& v : eps) {
            cfg->eps_list.push_back(static_cast<double>(v));
        }
    }
    return true;
}

std::vector<ParallaxSample> evaluateAbsolute(const ParallaxConfig& cfg) {
    std::vector<ParallaxSample> out;
    if (cfg.fx <= 0.0 || cfg.fy <= 0.0 || cfg.z_min <= 0.0 || cfg.z_max <= cfg.z_min) {
        return out;
    }
    double step = cfg.step > 0.0 ? cfg.step : 1.0;
    for (double z = cfg.z_min; z <= cfg.z_max + 1e-9; z += step) {
        ParallaxSample s;
        s.z = z;
        computeOffset(cfg.fx, cfg.fy, cfg.bx, cfg.by, z, &s.du, &s.dv);
        out.push_back(s);
    }
    return out;
}

std::vector<ParallaxSample> evaluateRelative(const ParallaxConfig& cfg) {
    std::vector<ParallaxSample> out;
    if (cfg.fx <= 0.0 || cfg.fy <= 0.0 || cfg.z_min <= 0.0 || cfg.z_max <= cfg.z_min) {
        return out;
    }
    double z_ref = cfg.z_ref;
    if (z_ref <= 0.0) {
        z_ref = 1e9;
    }
    double du_ref = 0.0;
    double dv_ref = 0.0;
    computeOffset(cfg.fx, cfg.fy, cfg.bx, cfg.by, z_ref, &du_ref, &dv_ref);
    double step = cfg.step > 0.0 ? cfg.step : 1.0;
    for (double z = cfg.z_min; z <= cfg.z_max + 1e-9; z += step) {
        ParallaxSample s;
        s.z = z;
        double du = 0.0;
        double dv = 0.0;
        computeOffset(cfg.fx, cfg.fy, cfg.bx, cfg.by, z, &du, &dv);
        s.du = du - du_ref;
        s.dv = dv - dv_ref;
        out.push_back(s);
    }
    return out;
}

double minDistanceForEps(const ParallaxConfig& cfg, double eps_px) {
    if (eps_px <= 0.0 || cfg.fx <= 0.0 || cfg.fy <= 0.0) {
        return 0.0;
    }
    double z_u = (std::abs(cfg.bx) < 1e-9) ? 0.0 : (cfg.fx * std::abs(cfg.bx) / eps_px);
    double z_v = (std::abs(cfg.by) < 1e-9) ? 0.0 : (cfg.fy * std::abs(cfg.by) / eps_px);
    return std::max(z_u, z_v);
}

std::pair<double, double> relativeRangeForEps(const ParallaxConfig& cfg, double eps_px) {
    if (eps_px <= 0.0 || cfg.fx <= 0.0 || cfg.fy <= 0.0) {
        return {0.0, 0.0};
    }
    double z_ref = cfg.z_ref;
    bool ref_inf = z_ref <= 0.0;
    double inv_ref = ref_inf ? 0.0 : (1.0 / z_ref);
    const double inf = std::numeric_limits<double>::infinity();
    double delta_u = (std::abs(cfg.bx) < 1e-9) ? inf : (eps_px / (cfg.fx * std::abs(cfg.bx)));
    double delta_v = (std::abs(cfg.by) < 1e-9) ? inf : (eps_px / (cfg.fy * std::abs(cfg.by)));
    double delta = std::min(delta_u, delta_v);
    double inv_min = std::max(0.0, inv_ref - delta);
    double inv_max = inv_ref + delta;
    double z_min = inv_max > 1e-12 ? 1.0 / inv_max : 0.0;
    double z_max = inv_min > 1e-12 ? 1.0 / inv_min : inf;
    return {z_min, z_max};
}

bool saveCsv(const std::string& path, const std::vector<ParallaxSample>& samples) {
    std::ofstream out(path);
    if (!out.is_open()) {
        return false;
    }
    out << "z,du,dv\n";
    for (const auto& s : samples) {
        out << s.z << "," << s.du << "," << s.dv << "\n";
    }
    return true;
}

}  // namespace tool
