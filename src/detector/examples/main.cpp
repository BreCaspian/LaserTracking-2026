#include "detector_registry/backend.h"

#include <chrono>
#include <iostream>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace {
int64_t nowMs() {
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
}
}  // namespace

int main(int argc, char** argv) {
    std::string config_path = "../config/detector.yaml";
    std::string video_path;
    int cam_id = 0;
    bool show = true;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (arg == "--video" && i + 1 < argc) {
            video_path = argv[++i];
        } else if (arg == "--cam" && i + 1 < argc) {
            cam_id = std::stoi(argv[++i]);
        } else if (arg == "--no-show") {
            show = false;
        }
    }

    std::string err;
    auto detector = detector::createDetectorFromPath(config_path, &err);
    if (!detector) {
        std::cerr << "Failed to create detector: " << err << "\n";
        return 1;
    }

    cv::VideoCapture cap;
    if (!video_path.empty()) {
        cap.open(video_path);
    } else {
        cap.open(cam_id);
    }
    if (!cap.isOpened()) {
        std::cerr << "Failed to open video source\n";
        return 1;
    }

    while (true) {
        cv::Mat frame;
        if (!cap.read(frame)) {
            break;
        }
        auto detections = detector->detect(frame, nowMs());
        if (show) {
            for (const auto& det : detections) {
                cv::circle(frame, det.center, static_cast<int>(det.radius), cv::Scalar(0, 255, 0), 2);
                cv::rectangle(frame, det.bbox, cv::Scalar(255, 0, 0), 1);
                cv::putText(frame, det.label, det.center, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 255, 0), 2);
            }
            cv::imshow("detector", frame);
            int key = cv::waitKey(1);
            if (key == 27 || key == 'q' || key == 'Q') {
                break;
            }
        }
    }

    return 0;
}
