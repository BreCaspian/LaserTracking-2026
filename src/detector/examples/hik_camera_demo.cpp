#include "detector_registry/backend.h"
#include "detector_common/measurement.h"
#include "hik_camera/hik_camera.h"
#if defined(DETECTOR_WITH_TRT)
#include "detector_trt/trt_detector.h"
#endif

#include <chrono>
#include <iostream>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace {
constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 720;
const char* kWindowName = "detector_hik";

int64_t nowMs() {
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
}
}  // namespace

int main(int argc, char** argv) {
    std::string camera_config_path = "../../hik_camera/config/config.yaml";
    std::string detector_config_path = "../config/detector.yaml";
    bool show = true;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--camera-config" && i + 1 < argc) {
            camera_config_path = argv[++i];
        } else if (arg == "--detector-config" && i + 1 < argc) {
            detector_config_path = argv[++i];
        } else if (arg == "--no-show") {
            show = false;
        }
    }

    hik_camera::CameraConfig cam_cfg;
    if (!hik_camera::loadCameraConfig(camera_config_path, &cam_cfg)) {
        std::cerr << "Failed to load camera config: " << camera_config_path << "\n";
        return 1;
    }

    std::string err;
    auto detector = detector::createDetectorFromPath(detector_config_path, &err);
    if (!detector) {
        std::cerr << "Failed to create detector: " << err << "\n";
        return 1;
    }
#if defined(DETECTOR_WITH_TRT)
    auto* trt_detector = dynamic_cast<detector_trt::TrtDetector*>(detector.get());
#endif

    hik_camera::HikCamera camera;
    if (!camera.open(cam_cfg)) {
        std::cerr << "Failed to open camera\n";
        return 1;
    }
    if (!camera.startGrabbing()) {
        std::cerr << "Failed to start grabbing\n";
        return 1;
    }

    bool window_ready = false;
    int invisible_count = 0;

    while (true) {
        hik_camera::Frame frame;
        if (!camera.read(&frame, cam_cfg.grab_timeout_ms)) {
            continue;
        }
        auto ts = nowMs();
#if defined(DETECTOR_WITH_TRT)
        if (trt_detector) {
            if (frame.gpu_valid && frame.gpu_bgr_ptr) {
                detector_trt::GpuInput gpu_input;
                gpu_input.bgr = frame.gpu_bgr_ptr;
                gpu_input.width = static_cast<int>(frame.width);
                gpu_input.height = static_cast<int>(frame.height);
                gpu_input.stride_bytes = frame.gpu_stride_bytes;
                gpu_input.cuda_stream = frame.gpu_stream;
                gpu_input.timestamp_ms = ts;
                trt_detector->setGpuInput(gpu_input);
            } else {
                trt_detector->clearGpuInput();
            }
        }
#endif
        bool need_bgr = true;
#if defined(DETECTOR_WITH_TRT)
        if (trt_detector && frame.gpu_valid && frame.gpu_bgr_ptr) {
            need_bgr = false;
        }
#endif
        if (need_bgr && frame.bgr.empty()) {
            continue;
        }
        auto detections = detector->detect(frame.bgr, ts);
        if (!detections.empty()) {
            auto meas = detector::toMeasurement(detections.front(), ts);
            if (meas.valid) {
                std::cout << "MEAS uv=(" << meas.uv.x << "," << meas.uv.y
                          << ") conf=" << meas.confidence
                          << " ts=" << meas.timestamp << "\n";
            }
        }

        if (show) {
            if (!window_ready) {
                try {
                    cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
                    cv::resizeWindow(kWindowName, kWindowWidth, kWindowHeight);
                    cv::moveWindow(kWindowName, 100, 100);
                    window_ready = true;
                } catch (const cv::Exception& e) {
                    std::cerr << "OpenCV GUI init failed: " << e.what() << "\n";
                    show = false;
                }
                if (!show) {
                    continue;
                }
            }

            cv::Mat vis;
            if (frame.bgr.empty()) {
                vis = cv::Mat(kWindowHeight, kWindowWidth, CV_8UC3, cv::Scalar(10, 10, 10));
            } else {
                vis = frame.bgr.clone();
            }
            double scale_x = static_cast<double>(vis.cols) / 1280.0;
            double scale_y = static_cast<double>(vis.rows) / 720.0;
            double scale = std::max(1.0, std::min(scale_x, scale_y));
            int line_thickness = std::max(2, static_cast<int>(std::round(2.0 * scale)));
            int text_thickness = std::max(2, static_cast<int>(std::round(2.0 * scale)));
            double font_scale = std::max(0.6, 0.6 * scale);
            int cross_half = std::max(12, static_cast<int>(std::round(16.0 * scale)));
            int cross_thickness = std::max(2, static_cast<int>(std::round(3.0 * scale)));
            for (const auto& det : detections) {
                cv::circle(vis, det.center, static_cast<int>(det.radius), cv::Scalar(0, 255, 0), line_thickness);
                cv::rectangle(vis, det.bbox, cv::Scalar(255, 0, 0), line_thickness);
                cv::putText(vis, det.label, det.center, cv::FONT_HERSHEY_SIMPLEX, font_scale,
                            cv::Scalar(0, 255, 0), text_thickness);
                cv::Point2f c = det.center;
                cv::line(vis, cv::Point2f(c.x - cross_half, c.y), cv::Point2f(c.x + cross_half, c.y),
                         cv::Scalar(0, 0, 255), cross_thickness);
                cv::line(vis, cv::Point2f(c.x, c.y - cross_half), cv::Point2f(c.x, c.y + cross_half),
                         cv::Scalar(0, 0, 255), cross_thickness);
            }
            cv::Mat display;
            if (vis.cols != kWindowWidth || vis.rows != kWindowHeight) {
                cv::resize(vis, display, cv::Size(kWindowWidth, kWindowHeight));
            } else {
                display = vis;
            }
            cv::imshow(kWindowName, display);
            int key = cv::waitKey(1);
            if (key == 27 || key == 'q' || key == 'Q') {
                break;
            }
            double visible = -1.0;
            try {
                visible = cv::getWindowProperty(kWindowName, cv::WND_PROP_VISIBLE);
            } catch (const cv::Exception&) {
                visible = -1.0;
            }
            if (visible >= 0.0 && visible < 1.0) {
                invisible_count++;
            } else {
                invisible_count = 0;
            }
            if (invisible_count >= 3) {
                break;
            }
            cv::resizeWindow(kWindowName, kWindowWidth, kWindowHeight);
        }
    }

    camera.stopGrabbing();
    camera.close();
    if (show && window_ready) {
        cv::destroyWindow(kWindowName);
    }
    return 0;
}
