#include "hik_camera/hik_camera.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#ifdef _WIN32
#include <windows.h>
#endif

namespace {
constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 720;
const std::string kWindowName = "hik_camera";

std::atomic<bool> g_should_exit{false};

#ifdef _WIN32
BOOL WINAPI ConsoleCtrlHandler(DWORD ctrl_type) {
    switch (ctrl_type) {
        case CTRL_C_EVENT:
        case CTRL_BREAK_EVENT:
        case CTRL_CLOSE_EVENT:
        case CTRL_SHUTDOWN_EVENT:
            g_should_exit.store(true);
            return TRUE;
        default:
            return FALSE;
    }
}
#else
void SignalHandler(int /*signum*/) {
    g_should_exit.store(true);
}
#endif

void RenderPlaceholder(const std::string& msg) {
    cv::Mat canvas(kWindowHeight, kWindowWidth, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::putText(canvas, msg, cv::Point(30, 80),
                cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 255), 2);
    cv::imshow(kWindowName, canvas);
}

}  // namespace

int main(int argc, char** argv) {
    hik_camera::CameraConfig config;
    bool show = false;
    std::string config_path;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (arg == "--show") {
            show = true;
        }
    }

    if (!config_path.empty()) {
        if (!hik_camera::loadCameraConfig(config_path, &config)) {
            std::cerr << "Failed to load config: " << config_path << "\n";
        }
    }

#ifdef _WIN32
    SetConsoleCtrlHandler(ConsoleCtrlHandler, TRUE);
#else
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);
#endif

    hik_camera::HikCamera camera;
    if (!camera.open(config)) {
        std::cerr << "Failed to open camera\n";
        return 1;
    }

    if (!camera.startGrabbing()) {
        std::cerr << "Failed to start grabbing\n";
        return 1;
    }

    auto last = std::chrono::steady_clock::now();
    int frame_count = 0;

    bool window_created = false;
    auto last_frame_time = std::chrono::steady_clock::now();

    if (show) {
        std::cerr << "Creating OpenCV window...\n";
        try {
            cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
            cv::resizeWindow(kWindowName, kWindowWidth, kWindowHeight);
            cv::moveWindow(kWindowName, 100, 100);
            window_created = true;
            RenderPlaceholder("Waiting for frames... (Q/q to quit, X to close)");
            cv::waitKey(1);
            std::cerr << "OpenCV window created.\n";
        } catch (const cv::Exception& e) {
            std::cerr << "OpenCV GUI not available (namedWindow/imshow failed):\n"
                      << e.what() << "\n";
            std::cerr << "Tip: check $DISPLAY or run with local desktop/VNC/X11 forwarding.\n";
            show = false;
        }
    }

    while (!g_should_exit.load()) {
        hik_camera::Frame frame;
        int timeout_ms = std::min(config.grab_timeout_ms, 50);
        const bool ok = camera.read(&frame, timeout_ms);

        if (show && window_created) {
            if (ok && config.output_bgr && !frame.bgr.empty()) {
                cv::Mat display;
                if (frame.bgr.cols != kWindowWidth || frame.bgr.rows != kWindowHeight) {
                    cv::resize(frame.bgr, display, cv::Size(kWindowWidth, kWindowHeight));
                } else {
                    display = frame.bgr;
                }
                cv::imshow(kWindowName, display);
                last_frame_time = std::chrono::steady_clock::now();
            } else {
                auto now = std::chrono::steady_clock::now();
                auto no_frame_ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame_time).count();
                std::string msg = "No frames yet.";
                if (!config.output_bgr) {
                    msg += " (config.output_bgr=false)";
                }
                if (no_frame_ms > 2000) {
                    msg += " Check trigger mode / exposure / stream.";
                }
                msg += " (Q/q quit, X to close)";
                RenderPlaceholder(msg);
            }

            int key = cv::waitKey(1);
            if (key == 'q' || key == 'Q') {
                g_should_exit.store(true);
            }

            double visible = -1.0;
            try {
                visible = cv::getWindowProperty(kWindowName, cv::WND_PROP_VISIBLE);
            } catch (...) {
                visible = -1.0;
            }
            if (visible == 0.0) {
                g_should_exit.store(true);
            }

            if (g_should_exit.load()) {
                cv::destroyWindow(kWindowName);
                break;
            }

            cv::resizeWindow(kWindowName, kWindowWidth, kWindowHeight);
        }

        if (ok) {
            frame_count++;
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last).count();
            if (elapsed >= 1000) {
                double fps = frame_count * 1000.0 / static_cast<double>(elapsed);
                std::cout << "FPS: " << fps << "\n";
                frame_count = 0;
                last = now;
            }
        }
    }

    camera.stopGrabbing();
    camera.close();

    if (show) {
        cv::destroyAllWindows();
    }
    return 0;
}
