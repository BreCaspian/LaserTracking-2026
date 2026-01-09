#include "control/config.h"

#include <iostream>

#include <opencv2/core.hpp>

namespace control {

bool loadControlConfig(const std::string& path, ControlConfig* cfg,
                       common::CameraModel* cam, common::Boresight* bs) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open control config: " << path << "\n";
        return false;
    }
    if (cfg) {
        fs["kp"] >> cfg->kp;
        fs["deadband_px"] >> cfg->deadband_px;
        fs["max_angle_rate"] >> cfg->max_angle_rate;
        fs["lowpass_alpha"] >> cfg->lowpass_alpha;
        if (!fs["yaw_sign"].empty()) {
            fs["yaw_sign"] >> cfg->yaw_sign;
        }
        if (!fs["pitch_sign"].empty()) {
            fs["pitch_sign"] >> cfg->pitch_sign;
        }
        if (!fs["ctrl_dt_nominal_ms"].empty()) {
            fs["ctrl_dt_nominal_ms"] >> cfg->ctrl_dt_nominal_ms;
        }
        if (!fs["ctrl_dt_min_ms"].empty()) {
            fs["ctrl_dt_min_ms"] >> cfg->ctrl_dt_min_ms;
        }
        if (!fs["ctrl_dt_max_ms"].empty()) {
            fs["ctrl_dt_max_ms"] >> cfg->ctrl_dt_max_ms;
        }
        if (!fs["use_velocity_ff"].empty()) {
            fs["use_velocity_ff"] >> cfg->use_velocity_ff;
        }
        if (!fs["ff_alpha"].empty()) {
            fs["ff_alpha"] >> cfg->ff_alpha;
        }
        if (!fs["ff_rate_max"].empty()) {
            fs["ff_rate_max"] >> cfg->ff_rate_max;
        }
        if (!fs["ff_dt_max_ms"].empty()) {
            fs["ff_dt_max_ms"] >> cfg->ff_dt_max_ms;
        }
        if (!fs["use_damping"].empty()) {
            fs["use_damping"] >> cfg->use_damping;
        }
        if (!fs["damping_source"].empty()) {
            fs["damping_source"] >> cfg->damping_source;
        }
        if (!fs["damping_kd"].empty()) {
            fs["damping_kd"] >> cfg->damping_kd;
        }
        if (!fs["damping_dt_max_ms"].empty()) {
            fs["damping_dt_max_ms"] >> cfg->damping_dt_max_ms;
        }
        if (!fs["scan_enable"].empty()) {
            fs["scan_enable"] >> cfg->scan_enable;
        }
        if (!fs["scan_radius_deg"].empty()) {
            fs["scan_radius_deg"] >> cfg->scan_radius_deg;
        }
        if (!fs["scan_rate_hz"].empty()) {
            fs["scan_rate_hz"] >> cfg->scan_rate_hz;
        }
        if (!fs["scan_pattern"].empty()) {
            fs["scan_pattern"] >> cfg->scan_pattern;
        }
        if (!fs["scan_spacing_deg"].empty()) {
            fs["scan_spacing_deg"] >> cfg->scan_spacing_deg;
        }
        if (!fs["scan_speed_deg_s"].empty()) {
            fs["scan_speed_deg_s"] >> cfg->scan_speed_deg_s;
        }
        if (!fs["scan_r_max_deg"].empty()) {
            fs["scan_r_max_deg"] >> cfg->scan_r_max_deg;
        }
        if (!fs["scan_spiral_return"].empty()) {
            fs["scan_spiral_return"] >> cfg->scan_spiral_return;
        }
        if (!fs["scan_k_yaw"].empty()) {
            fs["scan_k_yaw"] >> cfg->scan_k_yaw;
        }
        if (!fs["scan_k_pitch"].empty()) {
            fs["scan_k_pitch"] >> cfg->scan_k_pitch;
        }
        if (!fs["scan_enter_delay_ms"].empty()) {
            fs["scan_enter_delay_ms"] >> cfg->scan_enter_delay_ms;
        }
        if (!fs["scan_reacq_confirm_frames"].empty()) {
            fs["scan_reacq_confirm_frames"] >> cfg->scan_reacq_confirm_frames;
        }
        if (!fs["startup_check_frames"].empty()) {
            fs["startup_check_frames"] >> cfg->startup_check_frames;
        }
        if (!fs["startup_home_pitch"].empty()) {
            fs["startup_home_pitch"] >> cfg->startup_home_pitch;
        }
        if (!fs["startup_home_yaw"].empty()) {
            fs["startup_home_yaw"] >> cfg->startup_home_yaw;
        }
        if (!fs["startup_prep_ms"].empty()) {
            fs["startup_prep_ms"] >> cfg->startup_prep_ms;
        }
        if (!fs["startup_hold_ms"].empty()) {
            fs["startup_hold_ms"] >> cfg->startup_hold_ms;
        }
        if (!fs["startup_home_ms"].empty()) {
            fs["startup_home_ms"] >> cfg->startup_home_ms;
        }
        if (!fs["startup_validate_ms"].empty()) {
            fs["startup_validate_ms"] >> cfg->startup_validate_ms;
        }
        if (!fs["startup_min_state_frames"].empty()) {
            fs["startup_min_state_frames"] >> cfg->startup_min_state_frames;
        }
        if (!fs["startup_require_home"].empty()) {
            fs["startup_require_home"] >> cfg->startup_require_home;
        }
        if (!fs["startup_home_tol_deg"].empty()) {
            fs["startup_home_tol_deg"] >> cfg->startup_home_tol_deg;
        }
        if (!fs["startup_home_max_extra_ms"].empty()) {
            fs["startup_home_max_extra_ms"] >> cfg->startup_home_max_extra_ms;
        }
        if (!fs["startup_validate_first"].empty()) {
            fs["startup_validate_first"] >> cfg->startup_validate_first;
        }
        if (!fs["startup_allow_early_exit"].empty()) {
            fs["startup_allow_early_exit"] >> cfg->startup_allow_early_exit;
        }
    }
    if (cam) {
        fs["fx"] >> cam->fx;
        fs["fy"] >> cam->fy;
        fs["cx"] >> cam->cx;
        fs["cy"] >> cam->cy;
    }
    if (bs) {
        fs["u_L"] >> bs->u_L;
        fs["v_L"] >> bs->v_L;
    }
    return true;
}

}  // namespace control
