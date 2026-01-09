#include "tool/parallax_estimator.h"

#include <iostream>
#include <string>

namespace {
void printUsage() {
    std::cout << "Usage: parallax_demo --config <path> [--csv <path>] [--relative]\n";
}
}  // namespace

int main(int argc, char** argv) {
    std::string config_path = "../config/parallax.yaml";
    std::string csv_path;
    bool use_relative = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (arg == "--csv" && i + 1 < argc) {
            csv_path = argv[++i];
        } else if (arg == "--relative") {
            use_relative = true;
        } else if (arg == "--help") {
            printUsage();
            return 0;
        }
    }

    tool::ParallaxConfig cfg;
    if (!tool::loadParallaxConfig(config_path, &cfg)) {
        return 1;
    }

    std::cout << "fx=" << cfg.fx << " fy=" << cfg.fy << " bx=" << cfg.bx << " by=" << cfg.by << "\n";
    std::cout << "z_range=[" << cfg.z_min << ", " << cfg.z_max << "] step=" << cfg.step << "\n";
    if (!cfg.eps_list.empty()) {
        std::cout << "eps_list: ";
        for (size_t i = 0; i < cfg.eps_list.size(); ++i) {
            std::cout << cfg.eps_list[i] << (i + 1 == cfg.eps_list.size() ? "\n" : ", ");
        }
    }

    if (!cfg.eps_list.empty()) {
        for (double eps : cfg.eps_list) {
            double z_min = tool::minDistanceForEps(cfg, eps);
            std::cout << "abs eps=" << eps << "px -> z >= " << z_min << " m\n";
            if (cfg.z_ref > 0.0) {
                auto range = tool::relativeRangeForEps(cfg, eps);
                std::cout << "rel eps=" << eps << "px -> z in [" << range.first << ", " << range.second << "] m\n";
            }
        }
    }

    std::vector<tool::ParallaxSample> samples =
        use_relative ? tool::evaluateRelative(cfg) : tool::evaluateAbsolute(cfg);

    if (!csv_path.empty()) {
        if (tool::saveCsv(csv_path, samples)) {
            std::cout << "Saved CSV: " << csv_path << "\n";
        } else {
            std::cerr << "Failed to save CSV: " << csv_path << "\n";
        }
    }

    std::cout << "Samples: " << samples.size() << "\n";
    if (!samples.empty()) {
        const auto& s0 = samples.front();
        const auto& s1 = samples.back();
        std::cout << "z=" << s0.z << " du=" << s0.du << " dv=" << s0.dv << "\n";
        std::cout << "z=" << s1.z << " du=" << s1.du << " dv=" << s1.dv << "\n";
    }

    return 0;
}
