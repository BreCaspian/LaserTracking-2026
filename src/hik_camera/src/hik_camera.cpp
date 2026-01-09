#include "hik_camera/hik_camera.h"

#include <chrono>
#include <cstring>
#include <filesystem>
#include <iostream>

namespace hik_camera {

namespace {
std::atomic<int> g_sdk_ref{0};
std::mutex g_sdk_mutex;

bool clamp_int_value(void* handle, const char* name, int value, int* clamped_out) {
    MVCC_INTVALUE range{};
    if (MV_CC_GetIntValue(handle, name, &range) != MV_OK) {
        return false;
    }

    int clamped = std::max(static_cast<int>(range.nMin),
        std::min(value, static_cast<int>(range.nMax)));
    if (range.nInc > 1U) {
        clamped = static_cast<int>(range.nMin) +
            ((clamped - static_cast<int>(range.nMin)) / static_cast<int>(range.nInc)) *
                static_cast<int>(range.nInc);
    }
    if (clamped_out) {
        *clamped_out = clamped;
    }
    return MV_CC_SetIntValue(handle, name, clamped) == MV_OK;
}

bool set_enum_value(void* handle, const char* name, int64_t value) {
    int ret = MV_CC_SetEnumValue(handle, name, static_cast<unsigned int>(value));
    if (ret != MV_OK) {
        std::cerr << "SetEnumValue failed: " << name << " ret=0x" << std::hex << ret << std::dec << "\n";
        return false;
    }
    return true;
}

bool set_enum_value_optional(void* handle, const char* name, int64_t value) {
    MVCC_ENUMVALUE enum_value{};
    int ret = MV_CC_GetEnumValue(handle, name, &enum_value);
    if (ret != MV_OK) {
        std::cerr << "GetEnumValue failed (optional): " << name << " ret=0x" << std::hex << ret << std::dec << "\n";
        return true;
    }
    return set_enum_value(handle, name, value);
}

bool set_float_value(void* handle, const char* name, double value) {
    int ret = MV_CC_SetFloatValue(handle, name, static_cast<float>(value));
    if (ret != MV_OK) {
        std::cerr << "SetFloatValue failed: " << name << " ret=0x" << std::hex << ret << std::dec << "\n";
        return false;
    }
    return true;
}

bool set_float_value_clamped(void* handle, const char* name, double value) {
    MVCC_FLOATVALUE range{};
    int ret = MV_CC_GetFloatValue(handle, name, &range);
    if (ret != MV_OK) {
        std::cerr << "GetFloatValue failed: " << name << " ret=0x" << std::hex << ret << std::dec << "\n";
        return false;
    }
    double clamped = std::max(static_cast<double>(range.fMin),
        std::min(value, static_cast<double>(range.fMax)));
    return set_float_value(handle, name, clamped);
}

bool set_float_value_clamped_optional(void* handle, const char* name, double value) {
    MVCC_FLOATVALUE range{};
    int ret = MV_CC_GetFloatValue(handle, name, &range);
    if (ret != MV_OK) {
        std::cerr << "GetFloatValue failed (optional): " << name << " ret=0x" << std::hex << ret << std::dec << "\n";
        return true;
    }
    double clamped = std::max(static_cast<double>(range.fMin),
        std::min(value, static_cast<double>(range.fMax)));
    return set_float_value(handle, name, clamped);
}

bool set_brightness_value_optional(void* handle, double value) {
    MVCC_FLOATVALUE float_range{};
    int ret = MV_CC_GetFloatValue(handle, "Brightness", &float_range);
    if (ret == MV_OK) {
        double clamped = std::max(static_cast<double>(float_range.fMin),
            std::min(value, static_cast<double>(float_range.fMax)));
        return set_float_value(handle, "Brightness", clamped);
    }

    MVCC_INTVALUE int_range{};
    ret = MV_CC_GetIntValue(handle, "Brightness", &int_range);
    if (ret == MV_OK) {
        int clamped = std::max(static_cast<int>(int_range.nMin),
            std::min(static_cast<int>(value), static_cast<int>(int_range.nMax)));
        return clamp_int_value(handle, "Brightness", clamped, nullptr);
    }

    std::cerr << "Brightness not supported on this device\n";
    return true;
}

bool set_bool_value(void* handle, const char* name, bool value) {
    int ret = MV_CC_SetBoolValue(handle, name, value);
    if (ret != MV_OK) {
        std::cerr << "SetBoolValue failed: " << name << " ret=0x" << std::hex << ret << std::dec << "\n";
        return false;
    }
    return true;
}

bool set_bool_value_optional(void* handle, const char* name, bool value) {
    bool cur = false;
    int ret = MV_CC_GetBoolValue(handle, name, &cur);
    if (ret != MV_OK) {
        std::cerr << "GetBoolValue failed (optional): " << name << " ret=0x" << std::hex << ret << std::dec << "\n";
        return true;
    }
    return set_bool_value(handle, name, value);
}

}  // namespace

HikCamera::HikCamera() = default;

HikCamera::~HikCamera() {
    close();
}

bool HikCamera::initSdkOnce() {
    std::lock_guard<std::mutex> lock(g_sdk_mutex);
    if (g_sdk_ref.fetch_add(1) == 0) {
        int ret = MV_CC_Initialize();
        if (ret != MV_OK) {
            g_sdk_ref.fetch_sub(1);
            std::cerr << "MV_CC_Initialize failed: 0x" << std::hex << ret << std::dec << "\n";
            return false;
        }
    }
    return true;
}

void HikCamera::releaseSdkOnce() {
    std::lock_guard<std::mutex> lock(g_sdk_mutex);
    if (g_sdk_ref.fetch_sub(1) == 1) {
        MV_CC_Finalize();
    }
}

bool HikCamera::open(const CameraConfig& config) {
    if (handle_) {
        return true;
    }

    if (!initSdkOnce()) {
        return false;
    }

    config_ = config;
    if (!reopenDevice()) {
        releaseSdkOnce();
        return false;
    }

    return true;
}

void HikCamera::close() {
    if (handle_ && config_.feature_save_enable && config_.feature_save_on_close &&
        !config_.feature_save_path.empty()) {
        applyFeatureSave(config_.feature_save_path);
    }
    stopGrabbing();
    closeDevice();
    if (handle_) {
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
    }
    releaseGpuResources();
    exception_registered_ = false;
    releaseSdkOnce();
}

bool HikCamera::openDevice() {
    int ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK) {
        std::cerr << "MV_CC_OpenDevice failed: 0x" << std::hex << ret << std::dec << "\n";
        return false;
    }
    return true;
}

void HikCamera::closeDevice() {
    if (handle_) {
        MV_CC_CloseDevice(handle_);
    }
}

bool HikCamera::applyConfig() {
    bool ok = true;
    ok &= set_enum_value(handle_, "AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
    ok &= set_enum_value(handle_, "TriggerMode", config_.trigger_mode ? MV_TRIGGER_MODE_ON : MV_TRIGGER_MODE_OFF);
    if (config_.trigger_mode) {
        ok &= set_enum_value(handle_, "TriggerSource", config_.trigger_source);
    }

    ok &= set_enum_value(handle_, "ExposureMode", MV_EXPOSURE_MODE_TIMED);
    ok &= set_enum_value(handle_, "ExposureAuto",
        config_.auto_exposure ? MV_EXPOSURE_AUTO_MODE_CONTINUOUS : MV_EXPOSURE_AUTO_MODE_OFF);
    if (!config_.auto_exposure) {
        ok &= set_float_value_clamped(handle_, "ExposureTime", config_.exposure_time_us);
    } else {
        double lower = config_.auto_exposure_time_lower_us;
        double upper = config_.auto_exposure_time_upper_us;
        if (lower > 0.0 && upper > 0.0 && lower > upper) {
            std::swap(lower, upper);
        }
        if (lower > 0.0) {
            ok &= set_float_value_clamped_optional(handle_, "AutoExposureTimeLowerLimit", lower);
        }
        if (upper > 0.0) {
            ok &= set_float_value_clamped_optional(handle_, "AutoExposureTimeUpperLimit", upper);
        }
    }

    ok &= set_enum_value(handle_, "GainAuto",
        config_.auto_gain ? MV_GAIN_MODE_CONTINUOUS : MV_GAIN_MODE_OFF);
    if (!config_.auto_gain) {
        ok &= set_float_value_clamped(handle_, "Gain", config_.gain);
    }

    ok &= set_enum_value(handle_, "BalanceWhiteAuto",
        config_.balance_white_auto ? MV_BALANCEWHITE_AUTO_CONTINUOUS : MV_BALANCEWHITE_AUTO_OFF);

    ok &= set_bool_value_optional(handle_, "GammaEnable", config_.gamma_enable);
    if (config_.gamma_enable) {
        ok &= set_enum_value_optional(handle_, "GammaSelector", config_.gamma_selector);
        ok &= set_float_value_clamped_optional(handle_, "Gamma", config_.gamma);
    }

    if (config_.brightness_enable) {
        ok &= set_brightness_value_optional(handle_, config_.brightness);
    }

    ok &= set_bool_value_optional(handle_, "DigitalShiftEnable", config_.digital_shift_enable);
    if (config_.digital_shift_enable) {
        ok &= set_float_value_clamped_optional(handle_, "DigitalShift", config_.digital_shift);
    }

    ok &= set_bool_value(handle_, "AcquisitionFrameRateEnable", config_.frame_rate_enable);
    if (config_.frame_rate_enable) {
        ok &= set_float_value_clamped(handle_, "AcquisitionFrameRate", config_.frame_rate);
    }

    ok &= set_enum_value(handle_, "PixelFormat", config_.pixel_format);

    ok &= setRoi(config_.roi);

    return ok;
}

bool HikCamera::applyFeatureLoad() {
    if (!handle_ || !config_.feature_load_enable || config_.feature_load_path.empty()) {
        return true;
    }
    int ret = MV_CC_FeatureLoad(handle_, const_cast<char*>(config_.feature_load_path.c_str()));
    if (ret != MV_OK) {
        std::cerr << "FeatureLoad failed: " << config_.feature_load_path
                  << " ret=0x" << std::hex << ret << std::dec << "\n";
        return false;
    }
    return true;
}

bool HikCamera::applyFeatureSave(const std::string& path) {
    if (!handle_ || path.empty()) {
        return false;
    }
    std::filesystem::path save_path(path);
    if (save_path.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(save_path.parent_path(), ec);
        if (ec) {
            std::cerr << "Create directory failed: " << save_path.parent_path().string()
                      << " err=" << ec.message() << "\n";
        }
    }
    int ret = MV_CC_FeatureSave(handle_, const_cast<char*>(path.c_str()));
    if (ret != MV_OK) {
        std::cerr << "FeatureSave failed: " << path
                  << " ret=0x" << std::hex << ret << std::dec << "\n";
        return false;
    }
    std::cout << "FeatureSave ok: " << path << "\n";
    return true;
}

void HikCamera::registerExceptionCallback() {
    if (!handle_ || exception_registered_) {
        return;
    }
    int ret = MV_CC_RegisterExceptionCallBack(handle_, ExceptionCallback, this);
    if (ret != MV_OK) {
        std::cerr << "RegisterExceptionCallBack failed ret=0x" << std::hex << ret << std::dec << "\n";
        return;
    }
    exception_registered_ = true;
}

void HikCamera::handleDisconnect() {
    if (!config_.reconnect_enable) {
        return;
    }
    std::lock_guard<std::mutex> lock(reconnect_mutex_);
    if (reconnecting_) {
        return;
    }
    reconnecting_ = true;
    if (handle_) {
        MV_CC_StopGrabbing(handle_);
    }
    if (reopenDevice()) {
        if (MV_CC_StartGrabbing(handle_) == MV_OK) {
            reconnecting_ = false;
            return;
        }
    }
    reconnecting_ = false;
}

void __stdcall HikCamera::ExceptionCallback(unsigned int msg_type, void* user) {
    if (msg_type != MV_EXCEPTION_DEV_DISCONNECT) {
        return;
    }
    auto* self = static_cast<HikCamera*>(user);
    if (!self) {
        return;
    }
    self->handleDisconnect();
}

bool HikCamera::reopenDevice() {
    if (handle_) {
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
    }

    auto fail_close = [this]() {
        if (handle_) {
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
        }
    };

    MV_CC_DEVICE_INFO_LIST device_list{};
    int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
    if (ret != MV_OK || device_list.nDeviceNum == 0) {
        std::cerr << "No camera found. ret=0x" << std::hex << ret << std::dec << "\n";
        return false;
    }

    MV_CC_DEVICE_INFO* selected = nullptr;
    if (!config_.serial_number.empty()) {
        for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
            auto* info = device_list.pDeviceInfo[i];
            if (!info) {
                continue;
            }
            if (getDeviceSerial(info) == config_.serial_number) {
                selected = info;
                break;
            }
        }
    } else if (config_.use_first_device) {
        selected = device_list.pDeviceInfo[0];
    } else if (config_.device_index >= 0 &&
               static_cast<unsigned int>(config_.device_index) < device_list.nDeviceNum) {
        selected = device_list.pDeviceInfo[config_.device_index];
    }

    if (!selected) {
        std::cerr << "Camera not found for the given selection." << "\n";
        fail_close();
        return false;
    }

    printDeviceInfo(selected);

    ret = MV_CC_CreateHandle(&handle_, selected);
    if (ret != MV_OK) {
        std::cerr << "MV_CC_CreateHandle failed: 0x" << std::hex << ret << std::dec << "\n";
        handle_ = nullptr;
        fail_close();
        return false;
    }

    if (!openDevice()) {
        fail_close();
        return false;
    }

    registerExceptionCallback();

    if (selected->nTLayerType == MV_GIGE_DEVICE) {
        int packet = MV_CC_GetOptimalPacketSize(handle_);
        if (packet > 0) {
            MV_CC_SetIntValueEx(handle_, "GevSCPSPacketSize", packet);
        }
    }

    if (config_.image_node_num > 0) {
        MV_CC_SetImageNodeNum(handle_, static_cast<unsigned int>(config_.image_node_num));
    }
    MV_CC_SetGrabStrategy(handle_, static_cast<MV_GRAB_STRATEGY>(config_.grab_strategy));

    if (!applyFeatureLoad()) {
        fail_close();
        return false;
    }

    if (!applyConfig()) {
        fail_close();
        return false;
    }

    if (!prepareBuffers()) {
        fail_close();
        return false;
    }

    return true;
}

bool HikCamera::startGrabbing() {
    if (!handle_) {
        return false;
    }
    if (grabbing_) {
        return true;
    }
    int ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK) {
        std::cerr << "MV_CC_StartGrabbing failed: 0x" << std::hex << ret << std::dec << "\n";
        return false;
    }
    grabbing_ = true;
    exit_thread_ = false;
    if (callback_) {
        grab_thread_ = std::thread(&HikCamera::grabLoop, this);
    }
    return true;
}

void HikCamera::stopGrabbing() {
    exit_thread_ = true;
    if (grab_thread_.joinable()) {
        grab_thread_.join();
    }
    if (handle_ && grabbing_) {
        MV_CC_StopGrabbing(handle_);
    }
    grabbing_ = false;
}

bool HikCamera::read(Frame* out, int timeout_ms) {
    if (!handle_ || !out) {
        return false;
    }
    if (callback_ && grab_thread_.joinable()) {
        return false;
    }
    if (!grabbing_) {
        if (!startGrabbing()) {
            return false;
        }
    }
    if (grabOnce(out, timeout_ms)) {
        return true;
    }
    if (!config_.reconnect_enable) {
        return false;
    }
    if (handle_) {
        MV_CC_StopGrabbing(handle_);
    }
    if (!reopenDevice()) {
        return false;
    }
    if (MV_CC_StartGrabbing(handle_) != MV_OK) {
        return false;
    }
    return grabOnce(out, timeout_ms);
}

void HikCamera::setFrameCallback(FrameCallback cb) {
    callback_ = std::move(cb);
    if (callback_ && grabbing_ && !grab_thread_.joinable()) {
        exit_thread_ = false;
        grab_thread_ = std::thread(&HikCamera::grabLoop, this);
    }
}

bool HikCamera::isOpen() const {
    return handle_ != nullptr;
}

bool HikCamera::isGrabbing() const {
    return grabbing_;
}

bool HikCamera::setExposure(double us) {
    if (!handle_) {
        return false;
    }
    config_.auto_exposure = false;
    return set_enum_value(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF) &&
           set_float_value_clamped(handle_, "ExposureTime", us);
}

bool HikCamera::setGain(double gain) {
    if (!handle_) {
        return false;
    }
    config_.auto_gain = false;
    return set_enum_value(handle_, "GainAuto", MV_GAIN_MODE_OFF) &&
           set_float_value_clamped(handle_, "Gain", gain);
}

bool HikCamera::setFrameRate(double fps) {
    if (!handle_) {
        return false;
    }
    config_.frame_rate_enable = true;
    return set_bool_value(handle_, "AcquisitionFrameRateEnable", true) &&
           set_float_value_clamped(handle_, "AcquisitionFrameRate", fps);
}

bool HikCamera::setTriggerMode(bool enable) {
    if (!handle_) {
        return false;
    }
    config_.trigger_mode = enable;
    return set_enum_value(handle_, "TriggerMode", enable ? MV_TRIGGER_MODE_ON : MV_TRIGGER_MODE_OFF);
}

bool HikCamera::setPixelFormat(int pixel_format) {
    if (!handle_) {
        return false;
    }
    if (grabbing_) {
        return false;
    }
    config_.pixel_format = pixel_format;
    if (!set_enum_value(handle_, "PixelFormat", pixel_format)) {
        return false;
    }
    return prepareBuffers();
}

bool HikCamera::setRoi(const RoiConfig& roi) {
    if (!handle_) {
        return false;
    }
    if (grabbing_) {
        return false;
    }
    if (!roi.enable) {
        bool ok = true;
        ok &= clamp_int_value(handle_, "OffsetX", 0, nullptr);
        ok &= clamp_int_value(handle_, "OffsetY", 0, nullptr);
        MVCC_INTVALUE width{}, height{};
        ok &= (MV_CC_GetIntValue(handle_, "Width", &width) == MV_OK);
        ok &= (MV_CC_GetIntValue(handle_, "Height", &height) == MV_OK);
        ok &= clamp_int_value(handle_, "Width", width.nMax, nullptr);
        ok &= clamp_int_value(handle_, "Height", height.nMax, nullptr);
        if (ok) {
            config_.roi = roi;
            ok &= prepareBuffers();
        }
        return ok;
    }

    MVCC_INTVALUE width_range{};
    MVCC_INTVALUE height_range{};
    MVCC_INTVALUE offset_x_range{};
    MVCC_INTVALUE offset_y_range{};
    if (MV_CC_GetIntValue(handle_, "Width", &width_range) != MV_OK ||
        MV_CC_GetIntValue(handle_, "Height", &height_range) != MV_OK ||
        MV_CC_GetIntValue(handle_, "OffsetX", &offset_x_range) != MV_OK ||
        MV_CC_GetIntValue(handle_, "OffsetY", &offset_y_range) != MV_OK) {
        return false;
    }

    int width = roi.width <= 0 ? static_cast<int>(width_range.nMax) : roi.width;
    int height = roi.height <= 0 ? static_cast<int>(height_range.nMax) : roi.height;

    width = std::max(static_cast<int>(width_range.nMin),
        std::min(width, static_cast<int>(width_range.nMax)));
    height = std::max(static_cast<int>(height_range.nMin),
        std::min(height, static_cast<int>(height_range.nMax)));

    int max_offset_x = std::max(0, static_cast<int>(width_range.nMax) - width);
    int max_offset_y = std::max(0, static_cast<int>(height_range.nMax) - height);

    int offset_x = std::max(static_cast<int>(offset_x_range.nMin),
        std::min(roi.offset_x, max_offset_x));
    int offset_y = std::max(static_cast<int>(offset_y_range.nMin),
        std::min(roi.offset_y, max_offset_y));

    bool ok = true;
    ok &= clamp_int_value(handle_, "Width", width, nullptr);
    ok &= clamp_int_value(handle_, "Height", height, nullptr);
    ok &= clamp_int_value(handle_, "OffsetX", offset_x, nullptr);
    ok &= clamp_int_value(handle_, "OffsetY", offset_y, nullptr);
    if (ok) {
        config_.roi = roi;
        ok &= prepareBuffers();
    }
    return ok;
}

const CameraConfig& HikCamera::config() const {
    return config_;
}

std::string HikCamera::getDeviceSerial(const MV_CC_DEVICE_INFO* info) {
    if (!info) {
        return {};
    }
    if (info->nTLayerType == MV_USB_DEVICE) {
        return std::string(reinterpret_cast<const char*>(info->SpecialInfo.stUsb3VInfo.chSerialNumber));
    }
    if (info->nTLayerType == MV_GIGE_DEVICE) {
        return std::string(reinterpret_cast<const char*>(info->SpecialInfo.stGigEInfo.chSerialNumber));
    }
    return {};
}

bool HikCamera::isUsbDevice(const MV_CC_DEVICE_INFO* info) {
    return info && info->nTLayerType == MV_USB_DEVICE;
}

void HikCamera::printDeviceInfo(const MV_CC_DEVICE_INFO* info) {
    if (!info) {
        return;
    }
    if (info->nTLayerType == MV_USB_DEVICE) {
        std::cout << "Device: USB3" << "\n";
        std::cout << "UserDefinedName: " << info->SpecialInfo.stUsb3VInfo.chUserDefinedName << "\n";
        std::cout << "SerialNumber: " << info->SpecialInfo.stUsb3VInfo.chSerialNumber << "\n";
    } else if (info->nTLayerType == MV_GIGE_DEVICE) {
        std::cout << "Device: GigE" << "\n";
        std::cout << "UserDefinedName: " << info->SpecialInfo.stGigEInfo.chUserDefinedName << "\n";
        std::cout << "SerialNumber: " << info->SpecialInfo.stGigEInfo.chSerialNumber << "\n";
    }
}

}  // namespace hik_camera
