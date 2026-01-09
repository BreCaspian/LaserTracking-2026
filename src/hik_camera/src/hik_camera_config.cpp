#include "hik_camera/hik_camera.h"

#include <filesystem>
#include <fstream>
#include <sstream>

namespace hik_camera {
namespace {
template <typename T>
bool read_node(const cv::FileStorage& fs, const char* key, T* out) {
    cv::FileNode node = fs[key];
    if (node.empty()) {
        return false;
    }
    node >> *out;
    return true;
}
}  // namespace

bool loadCameraConfig(const std::string& path, CameraConfig* config) {
    if (!config) {
        return false;
    }

    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        return false;
    }

    read_node(fs, "serial_number", &config->serial_number);
    read_node(fs, "device_index", &config->device_index);
    read_node(fs, "use_first_device", &config->use_first_device);
    read_node(fs, "trigger_mode", &config->trigger_mode);
    read_node(fs, "trigger_source", &config->trigger_source);
    read_node(fs, "frame_rate_enable", &config->frame_rate_enable);
    read_node(fs, "frame_rate", &config->frame_rate);
    read_node(fs, "auto_exposure", &config->auto_exposure);
    read_node(fs, "exposure_time_us", &config->exposure_time_us);
    read_node(fs, "auto_exposure_time_lower_us", &config->auto_exposure_time_lower_us);
    read_node(fs, "auto_exposure_time_upper_us", &config->auto_exposure_time_upper_us);
    read_node(fs, "auto_gain", &config->auto_gain);
    read_node(fs, "gain", &config->gain);
    read_node(fs, "balance_white_auto", &config->balance_white_auto);
    read_node(fs, "gamma_enable", &config->gamma_enable);
    read_node(fs, "gamma_selector", &config->gamma_selector);
    read_node(fs, "gamma", &config->gamma);
    read_node(fs, "brightness_enable", &config->brightness_enable);
    read_node(fs, "brightness", &config->brightness);
    read_node(fs, "digital_shift_enable", &config->digital_shift_enable);
    read_node(fs, "digital_shift", &config->digital_shift);
    read_node(fs, "pixel_format", &config->pixel_format);
    read_node(fs, "output_bgr", &config->output_bgr);
    read_node(fs, "output_raw", &config->output_raw);
    read_node(fs, "copy_raw", &config->copy_raw);
    read_node(fs, "rotate_180", &config->rotate_180);
    read_node(fs, "grab_timeout_ms", &config->grab_timeout_ms);
    read_node(fs, "image_node_num", &config->image_node_num);
    read_node(fs, "grab_strategy", &config->grab_strategy);
    read_node(fs, "undistort_enable", &config->undistort_enable);
    read_node(fs, "calib_path", &config->calib_path);
    read_node(fs, "reconnect_enable", &config->reconnect_enable);
    read_node(fs, "reconnect_max_failures", &config->reconnect_max_failures);
    read_node(fs, "reconnect_retry_delay_ms", &config->reconnect_retry_delay_ms);
    read_node(fs, "reconnect_max_retries", &config->reconnect_max_retries);
    read_node(fs, "feature_load_enable", &config->feature_load_enable);
    read_node(fs, "feature_load_path", &config->feature_load_path);
    read_node(fs, "feature_save_enable", &config->feature_save_enable);
    read_node(fs, "feature_save_path", &config->feature_save_path);
    read_node(fs, "feature_save_on_close", &config->feature_save_on_close);
    read_node(fs, "zero_copy_enable", &config->zero_copy_enable);
    read_node(fs, "buffer_pool_size", &config->buffer_pool_size);
    read_node(fs, "gpu_pipeline_enable", &config->gpu_pipeline_enable);
    read_node(fs, "gpu_device_id", &config->gpu_device_id);
    read_node(fs, "gpu_stream_enable", &config->gpu_stream_enable);
    read_node(fs, "gpu_demosaic_enable", &config->gpu_demosaic_enable);
    read_node(fs, "gpu_demosaic_backend", &config->gpu_demosaic_backend);
    read_node(fs, "bayer_pattern", &config->bayer_pattern);

    bool roi_enable = false;
    read_node(fs, "roi_enable", &roi_enable);
    config->roi.enable = roi_enable;
    read_node(fs, "roi_offset_x", &config->roi.offset_x);
    read_node(fs, "roi_offset_y", &config->roi.offset_y);
    read_node(fs, "roi_width", &config->roi.width);
    read_node(fs, "roi_height", &config->roi.height);

    std::filesystem::path base_dir = std::filesystem::absolute(std::filesystem::path(path)).parent_path();
    std::filesystem::path project_dir = base_dir.parent_path();
    auto resolve_path = [&](const std::string& in) -> std::string {
        if (in.empty()) {
            return in;
        }
        std::filesystem::path p(in);
        if (!p.is_relative()) {
            return p.string();
        }
        std::string raw = p.generic_string();
        if (raw.rfind("config/", 0) == 0) {
            return (project_dir / p).string();
        }
        return (base_dir / p).string();
    };
    config->calib_path = resolve_path(config->calib_path);
    config->feature_load_path = resolve_path(config->feature_load_path);
    config->feature_save_path = resolve_path(config->feature_save_path);

    return true;
}

}  // namespace hik_camera
