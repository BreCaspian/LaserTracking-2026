#include "common/time_utils.h"
#include "control/config.h"
#include "control/controller.h"

#include <iostream>
#include <string>

int main(int argc, char** argv) {
    std::string control_config = "../config/control.yaml";
    float test_u = 100.0f;
    float test_v = 100.0f;
    float state_pitch = 0.0f;
    float state_yaw = 0.0f;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--control-config" && i + 1 < argc) {
            control_config = argv[++i];
        } else if (arg == "--u" && i + 1 < argc) {
            test_u = std::stof(argv[++i]);
        } else if (arg == "--v" && i + 1 < argc) {
            test_v = std::stof(argv[++i]);
        } else if (arg == "--pitch" && i + 1 < argc) {
            state_pitch = std::stof(argv[++i]);
        } else if (arg == "--yaw" && i + 1 < argc) {
            state_yaw = std::stof(argv[++i]);
        }
    }

    control::ControlConfig ctrl_cfg;
    common::CameraModel cam_model;
    common::Boresight boresight;
    if (!control::loadControlConfig(control_config, &ctrl_cfg, &cam_model, &boresight)) {
        std::cerr << "Failed to load control config\n";
        return 1;
    }

    control::Controller controller(ctrl_cfg);
    common::TargetMeasurement meas;
    meas.valid = true;
    meas.timestamp = common::nowMs();
    meas.uv = cv::Point2f(test_u, test_v);
    meas.confidence = 1.0f;

    common::GimbalState state;
    state.pitch = state_pitch;
    state.yaw = state_yaw;
    state.timestamp = static_cast<uint32_t>(meas.timestamp);

    common::GimbalCommand cmd = controller.update(meas, cam_model, boresight, state);
    std::cout << "CMD pitch=" << cmd.pitch << " yaw=" << cmd.yaw
              << " ts=" << cmd.timestamp << "\n";
    return 0;
}
