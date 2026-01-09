#include "hik_camera/hik_camera.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>

namespace hik_camera {
namespace {
std::atomic<bool> g_zero_copy_pool_logged{false};
std::atomic<bool> g_zero_copy_frame_logged{false};

uint32_t get_frame_width(const MV_FRAME_OUT_INFO_EX& info) {
    return info.nExtendWidth > 0 ? info.nExtendWidth : info.nWidth;
}

uint32_t get_frame_height(const MV_FRAME_OUT_INFO_EX& info) {
    return info.nExtendHeight > 0 ? info.nExtendHeight : info.nHeight;
}

uint64_t combine_timestamp(uint32_t high, uint32_t low) {
    return (static_cast<uint64_t>(high) << 32) | static_cast<uint64_t>(low);
}

bool readRosMatrix(const cv::FileNode& node, cv::Mat* out) {
    if (node.empty() || !out) {
        return false;
    }
    int rows = 0;
    int cols = 0;
    node["rows"] >> rows;
    node["cols"] >> cols;
    std::vector<double> data;
    node["data"] >> data;
    if (rows <= 0 || cols <= 0 || static_cast<size_t>(rows * cols) != data.size()) {
        return false;
    }
    cv::Mat mat(rows, cols, CV_64F);
    std::memcpy(mat.data, data.data(), data.size() * sizeof(double));
    *out = mat;
    return true;
}
}  // namespace

bool HikCamera::prepareBuffers() {
    MVCC_INTVALUE payload{};
    if (MV_CC_GetIntValue(handle_, "PayloadSize", &payload) != MV_OK) {
        std::cerr << "Get PayloadSize failed" << "\n";
        return false;
    }
    payload_size_ = payload.nCurValue;

    MVCC_INTVALUE width{};
    MVCC_INTVALUE height{};
    if (MV_CC_GetIntValue(handle_, "Width", &width) != MV_OK ||
        MV_CC_GetIntValue(handle_, "Height", &height) != MV_OK) {
        std::cerr << "Get Width/Height failed" << "\n";
        return false;
    }

    frame_cache_.width = width.nCurValue;
    frame_cache_.height = height.nCurValue;

    if (config_.output_bgr) {
        frame_cache_.bgr_data.resize(frame_cache_.width * frame_cache_.height * 3);
        frame_cache_.bgr = cv::Mat(frame_cache_.height, frame_cache_.width, CV_8UC3, frame_cache_.bgr_data.data());
    }
    if (config_.output_raw) {
        frame_cache_.raw_data.resize(payload_size_);
    }

    configureGpuPipeline();

    if (config_.output_bgr && config_.zero_copy_enable) {
        size_t bgr_size = static_cast<size_t>(frame_cache_.width) * frame_cache_.height * 3;
        int pool_size = std::max(1, config_.buffer_pool_size);
        if (bgr_pool_bytes_ != bgr_size || static_cast<int>(bgr_pool_.size()) != pool_size) {
            unregisterPinnedPool();
            bgr_pool_bytes_ = bgr_size;
            bgr_pool_.assign(static_cast<size_t>(pool_size), std::vector<uint8_t>(bgr_size));
            bgr_pool_index_ = 0;
            if (!g_zero_copy_pool_logged.exchange(true)) {
                std::cout << "Zero-copy pool enabled: pool_size=" << pool_size
                          << " bgr_bytes=" << bgr_size << "\n";
            }
        }
        configurePinnedPool();
    } else {
        unregisterPinnedPool();
        bgr_pool_.clear();
        bgr_pool_bytes_ = 0;
        bgr_pool_index_ = 0;
    }

    convert_param_.nWidth = frame_cache_.width;
    convert_param_.nHeight = frame_cache_.height;
    convert_param_.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

    if (config_.undistort_enable && config_.output_bgr && config_.calib_path.empty()) {
        std::cerr << "Undistort enabled but calib_path is empty\n";
        return false;
    }

    if (config_.undistort_enable && config_.output_bgr && !config_.calib_path.empty()) {
        cv::FileStorage fs;
        try {
            fs.open(config_.calib_path, cv::FileStorage::READ);
        } catch (const cv::Exception&) {
            fs.release();
        }
        if (!fs.isOpened()) {
            std::ifstream in(config_.calib_path);
            if (!in.is_open()) {
                std::cerr << "Failed to open calib file: " << config_.calib_path << "\n";
                return false;
            }
            std::ostringstream oss;
            oss << in.rdbuf();
            std::string content = oss.str();
            if (content.rfind("%YAML:", 0) != 0) {
                content = "%YAML:1.0\n" + content;
            }
            try {
                fs.open(content, cv::FileStorage::READ | cv::FileStorage::MEMORY);
            } catch (const cv::Exception& e) {
                std::cerr << "Failed to parse calib file: " << config_.calib_path
                          << " err=" << e.what() << "\n";
                return false;
            }
        }
        int calib_width = 0;
        int calib_height = 0;
        fs["image_width"] >> calib_width;
        fs["image_height"] >> calib_height;
        cv::FileNode cam_node = fs["camera_matrix"];
        if (!readRosMatrix(cam_node, &camera_matrix_)) {
            cam_node = fs["K"];
            readRosMatrix(cam_node, &camera_matrix_);
        }
        cv::FileNode dist_node = fs["distortion_coefficients"];
        if (!readRosMatrix(dist_node, &dist_coeffs_)) {
            dist_node = fs["D"];
            readRosMatrix(dist_node, &dist_coeffs_);
        }
        if (camera_matrix_.empty()) {
            fs["K"] >> camera_matrix_;
        }
        if (dist_coeffs_.empty()) {
            fs["D"] >> dist_coeffs_;
        }
        if (camera_matrix_.empty() || dist_coeffs_.empty()) {
            std::cerr << "Invalid calibration data in: " << config_.calib_path << "\n";
            return false;
        }
        if (calib_width > 0 && calib_height > 0) {
            if (calib_width != static_cast<int>(frame_cache_.width) ||
                calib_height != static_cast<int>(frame_cache_.height)) {
                std::cerr << "Calib size mismatch: file " << calib_width << "x" << calib_height
                          << " vs frame " << frame_cache_.width << "x" << frame_cache_.height << "\n";
            }
        }
        cv::initUndistortRectifyMap(camera_matrix_, dist_coeffs_, cv::Mat(),
            camera_matrix_, cv::Size(frame_cache_.width, frame_cache_.height),
            CV_16SC2, undistort_map1_, undistort_map2_);
        undistort_buffer_.resize(frame_cache_.width * frame_cache_.height * 3);
        undistort_input_ = cv::Mat(frame_cache_.height, frame_cache_.width, CV_8UC3);
    }

    return true;
}

bool HikCamera::grabOnce(Frame* out, int timeout_ms) {
    MV_FRAME_OUT out_frame{};
    int ret = MV_CC_GetImageBuffer(handle_, &out_frame, timeout_ms);
    if (ret != MV_OK) {
        return false;
    }
    bool ok = buildFrameFromBuffer(out, out_frame, false);
    MV_CC_FreeImageBuffer(handle_, &out_frame);
    return ok;
}

void HikCamera::grabLoop() {
    int failure_count = 0;
    int reconnect_attempts = 0;
    while (!exit_thread_ && grabbing_) {
        if (!handle_) {
            if (config_.reconnect_enable && reopenDevice()) {
                if (MV_CC_StartGrabbing(handle_) == MV_OK) {
                    failure_count = 0;
                    reconnect_attempts = 0;
                    continue;
                }
            }
            std::this_thread::sleep_for(
                std::chrono::milliseconds(config_.reconnect_retry_delay_ms));
            continue;
        }
        MV_FRAME_OUT out_frame{};
        int ret = MV_CC_GetImageBuffer(handle_, &out_frame, config_.grab_timeout_ms);
        if (ret != MV_OK) {
            failure_count++;
            if (config_.reconnect_enable &&
                failure_count >= std::max(1, config_.reconnect_max_failures)) {
                if (config_.reconnect_max_retries > 0 &&
                    reconnect_attempts >= config_.reconnect_max_retries) {
                    std::cerr << "Reconnect retry limit reached\n";
                    failure_count = 0;
                } else {
                    reconnect_attempts++;
                    std::cerr << "Camera disconnected, attempting reconnect...\n";
                    if (handle_) {
                        MV_CC_StopGrabbing(handle_);
                    }
                    if (reopenDevice()) {
                        if (MV_CC_StartGrabbing(handle_) == MV_OK) {
                            std::cerr << "Reconnect succeeded\n";
                            failure_count = 0;
                            continue;
                        }
                    }
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(config_.reconnect_retry_delay_ms));
                }
            }
            continue;
        }
        failure_count = 0;
        reconnect_attempts = 0;
        Frame frame;
        if (buildFrameFromBuffer(&frame, out_frame, !config_.copy_raw)) {
            if (callback_) {
                callback_(frame);
            }
        }
        MV_CC_FreeImageBuffer(handle_, &out_frame);
    }
}

bool HikCamera::buildFrameFromBuffer(Frame* out, const MV_FRAME_OUT& out_frame, bool allow_zero_copy) {
    if (!out) {
        return false;
    }

    Frame& frame = *out;
    const auto& info = out_frame.stFrameInfo;
    frame.width = get_frame_width(info);
    frame.height = get_frame_height(info);
    frame.frame_id = info.nFrameNum;
    frame.device_timestamp = combine_timestamp(info.nDevTimeStampHigh, info.nDevTimeStampLow);
    frame.host_timestamp = info.nHostTimeStamp;
    frame.pixel_type = info.enPixelType;

    frame.raw_ptr = nullptr;
    frame.raw_size = 0;
    frame.raw_is_copy = true;
    frame.bgr_ptr = nullptr;
    frame.bgr_size = 0;
    frame.bgr_is_copy = true;
    frame.gpu_bgr_ptr = nullptr;
    frame.gpu_stride_bytes = 0;
    frame.gpu_valid = false;
    frame.gpu_stream = nullptr;

    if (config_.output_raw) {
        size_t raw_size = info.nFrameLenEx > 0 ? static_cast<size_t>(info.nFrameLenEx)
                                               : static_cast<size_t>(info.nFrameLen);
        if (!config_.copy_raw && allow_zero_copy) {
            frame.raw_ptr = out_frame.pBufAddr;
            frame.raw_size = raw_size;
            frame.raw_is_copy = false;
        } else {
            frame.raw_data.resize(raw_size);
            std::memcpy(frame.raw_data.data(), out_frame.pBufAddr, raw_size);
            frame.raw_ptr = frame.raw_data.data();
            frame.raw_size = raw_size;
            frame.raw_is_copy = true;
        }
    }

    if (config_.output_bgr) {
        size_t bgr_size = static_cast<size_t>(frame.width) * frame.height * 3;
        uint8_t* bgr_dst = nullptr;
        if (config_.zero_copy_enable && !bgr_pool_.empty() && bgr_pool_bytes_ == bgr_size) {
            bgr_dst = bgr_pool_[bgr_pool_index_].data();
            bgr_pool_index_ = (bgr_pool_index_ + 1) % bgr_pool_.size();
            frame.bgr_ptr = bgr_dst;
            frame.bgr_size = bgr_size;
            frame.bgr_is_copy = false;
            frame.bgr_data.clear();
            if (!g_zero_copy_frame_logged.exchange(true)) {
                std::cout << "Zero-copy BGR frame active (bgr_is_copy=0)\n";
            }
        } else {
            frame.bgr_data.resize(bgr_size);
            bgr_dst = frame.bgr_data.data();
            frame.bgr_ptr = bgr_dst;
            frame.bgr_size = bgr_size;
            frame.bgr_is_copy = true;
        }
        frame.bgr = cv::Mat(frame.height, frame.width, CV_8UC3, bgr_dst);

        bool gpu_demosaic_done = false;
        if (config_.gpu_demosaic_enable) {
            gpu_demosaic_done = tryGpuDemosaic(out_frame, info, frame, bgr_dst);
        }

        if (info.enPixelType == PixelType_Gvsp_BGR8_Packed) {
            std::memcpy(bgr_dst, out_frame.pBufAddr, bgr_size);
        } else if (!gpu_demosaic_done) {
            convert_param_.nWidth = frame.width;
            convert_param_.nHeight = frame.height;
            convert_param_.pSrcData = out_frame.pBufAddr;
            convert_param_.nSrcDataLen = info.nFrameLenEx > 0 ? info.nFrameLenEx : info.nFrameLen;
            convert_param_.enSrcPixelType = info.enPixelType;
            convert_param_.pDstBuffer = bgr_dst;
            convert_param_.nDstBufferSize = bgr_size;
            int convert_ret = MV_CC_ConvertPixelType(handle_, &convert_param_);
            if (convert_ret != MV_OK) {
                std::cerr << "ConvertPixelType failed: 0x" << std::hex << convert_ret << std::dec << "\n";
                return false;
            }
        }

        if (config_.rotate_180) {
            cv::flip(frame.bgr, frame.bgr, -1);
        }

        if (config_.undistort_enable && !undistort_map1_.empty() && !undistort_map2_.empty()) {
            std::memcpy(undistort_input_.data, bgr_dst, bgr_size);
            cv::Mat undistorted(frame.height, frame.width, CV_8UC3, undistort_buffer_.data());
            cv::remap(undistort_input_, undistorted, undistort_map1_, undistort_map2_, cv::INTER_LINEAR);
            std::memcpy(bgr_dst, undistort_buffer_.data(), bgr_size);
        }
    }

    return true;
}

}  // namespace hik_camera
