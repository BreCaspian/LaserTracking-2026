#include "common/config_io.h"
#include "common/time_utils.h"
#include "detector_common/measurement.h"
#include "detector_registry/backend.h"
#include "hik_camera/hik_camera.h"

#include <cmath>
#include <iostream>
#include <mutex>
#include <string>

#include <opencv2/opencv.hpp>

namespace {
constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 720;
const std::string kWindowName = "boresight_calibrator";

struct Settings {
    std::string camera_config = "../../hik_camera/config/config.yaml";
    std::string detector_config = "../../detector/config/detector.yaml";
    std::string boresight_config;
    std::string boresight_out;
    int step_px = 2;
    bool show = true;
};

struct MouseState {
    std::mutex mu;
    bool pending = false;
    int event = 0;
    int x = 0;
    int y = 0;
    int flags = 0;
    int view_w = 0;
    int view_h = 0;
};

struct MouseEvent {
    int event = 0;
    int x = 0;
    int y = 0;
    int flags = 0;
    int view_w = 0;
    int view_h = 0;
};

void drawCrosshair(cv::Mat& img, const cv::Point& p, const cv::Scalar& color) {
    const int base = std::max(1, std::min(img.cols, img.rows));
    const int size = std::max(16, base / 60);
    const int thickness = std::max(1, base / 700);
    const int halo = thickness + 2;
    cv::line(img, cv::Point(p.x - size, p.y), cv::Point(p.x + size, p.y), cv::Scalar(0, 0, 0),
             halo, cv::LINE_AA);
    cv::line(img, cv::Point(p.x, p.y - size), cv::Point(p.x, p.y + size), cv::Scalar(0, 0, 0),
             halo, cv::LINE_AA);
    cv::line(img, cv::Point(p.x - size, p.y), cv::Point(p.x + size, p.y), color,
             thickness, cv::LINE_AA);
    cv::line(img, cv::Point(p.x, p.y - size), cv::Point(p.x, p.y + size), color,
             thickness, cv::LINE_AA);
}

void drawOverlayText(cv::Mat& img, const std::vector<std::string>& lines) {
    const int x = 30;
    int y = 60;
    const int base = std::max(1, std::min(img.cols, img.rows));
    double scale = base / 900.0;
    scale = std::max(0.9, std::min(scale, 2.2));
    const int thickness = std::max(2, static_cast<int>(std::round(scale * 2.0)));
    const int line_step = static_cast<int>(std::round(32.0 * scale));
    for (const auto& line : lines) {
        cv::putText(img, line, cv::Point(x, y),
                    cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 0),
                    thickness + 3, cv::LINE_AA);
        cv::putText(img, line, cv::Point(x, y),
                    cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(255, 0, 0),
                    thickness + 1, cv::LINE_AA);
        y += line_step;
    }
}

void MouseCallback(int event, int x, int y, int flags, void* userdata) {
    if (!userdata) {
        return;
    }
    auto* state = static_cast<MouseState*>(userdata);
    std::lock_guard<std::mutex> lock(state->mu);
    state->event = event;
    state->x = x;
    state->y = y;
    state->flags = flags;
    state->pending = true;
}

bool saveBoresight(const std::string& path, double u, double v) {
    if (path.empty()) {
        return false;
    }
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        return false;
    }
    fs << "u_L" << u;
    fs << "v_L" << v;
    return true;
}

bool parseArgs(int argc, char** argv, Settings* out) {
    if (!out) {
        return false;
    }
    Settings s;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--camera-config" && i + 1 < argc) {
            s.camera_config = argv[++i];
        } else if (arg == "--detector-config" && i + 1 < argc) {
            s.detector_config = argv[++i];
        } else if (arg == "--boresight-config" && i + 1 < argc) {
            s.boresight_config = argv[++i];
        } else if (arg == "--boresight-out" && i + 1 < argc) {
            s.boresight_out = argv[++i];
        } else if (arg == "--step" && i + 1 < argc) {
            s.step_px = std::max(1, std::stoi(argv[++i]));
        } else if (arg == "--no-show") {
            s.show = false;
        }
    }
    *out = s;
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    Settings settings;
    if (!parseArgs(argc, argv, &settings)) {
        std::cerr << "Failed to parse args\n";
        return 1;
    }

    hik_camera::CameraConfig cam_cfg;
    if (!hik_camera::loadCameraConfig(settings.camera_config, &cam_cfg)) {
        std::cerr << "Failed to load camera config\n";
        return 1;
    }

    std::string det_err;
    auto detector = detector::createDetectorFromPath(settings.detector_config, &det_err);
    if (!detector) {
        std::cerr << "Failed to create detector: " << det_err << "\n";
        return 1;
    }

    hik_camera::HikCamera camera;
    if (!camera.open(cam_cfg) || !camera.startGrabbing()) {
        std::cerr << "Failed to open camera\n";
        return 1;
    }

    common::Boresight boresight{};
    bool has_boresight = false;
    if (!settings.boresight_config.empty()) {
        if (common::loadBoresight(settings.boresight_config, &boresight)) {
            has_boresight = true;
        }
    }

    cv::Point2f target_center{};
    bool has_target = false;
    double init_u = 0.0;
    double init_v = 0.0;
    bool init_set = false;
    int last_key = -1;
    std::string last_action = "none";
    MouseState mouse_state;

    if (settings.show) {
        cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
        cv::resizeWindow(kWindowName, kWindowWidth, kWindowHeight);
        cv::moveWindow(kWindowName, 100, 100);
        cv::setMouseCallback(kWindowName, MouseCallback, &mouse_state);
    }

    while (true) {
        hik_camera::Frame frame;
        if (!camera.read(&frame, cam_cfg.grab_timeout_ms) || frame.bgr.empty()) {
            continue;
        }
        int64_t ts = common::nowMs();
        std::vector<detector::Detection> dets = detector->detect(frame.bgr, ts);
        if (!dets.empty() && dets.front().valid) {
            target_center = dets.front().center;
            has_target = true;
        }

        if (!has_boresight) {
            boresight.u_L = frame.bgr.cols * 0.5;
            boresight.v_L = frame.bgr.rows * 0.5;
            has_boresight = true;
        }
        if (!init_set) {
            init_u = boresight.u_L;
            init_v = boresight.v_L;
            init_set = true;
        }

        if (settings.show) {
            cv::Mat view = frame.bgr.clone();
            if (!dets.empty() && dets.front().valid) {
                cv::rectangle(view, dets.front().bbox, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
            }
            if (has_target) {
                drawCrosshair(view, cv::Point(static_cast<int>(target_center.x),
                                              static_cast<int>(target_center.y)),
                              cv::Scalar(0, 255, 0));
            }
            drawCrosshair(view,
                          cv::Point(static_cast<int>(boresight.u_L),
                                    static_cast<int>(boresight.v_L)),
                          cv::Scalar(0, 0, 255));
            double du = boresight.u_L - init_u;
            double dv = boresight.v_L - init_v;
            std::vector<std::string> lines;
            lines.push_back("STATUS: " + std::string(has_target ? "TARGET LOCKED" : "NO TARGET"));
            lines.push_back("Boresight u_L=" + std::to_string(boresight.u_L) +
                            " v_L=" + std::to_string(boresight.v_L));
            lines.push_back("Offset du=" + std::to_string(du) +
                            " dv=" + std::to_string(dv) +
                            " step=" + std::to_string(settings.step_px));
            if (has_target) {
                lines.push_back("Target center=(" + std::to_string(target_center.x) +
                                "," + std::to_string(target_center.y) + ")");
            }
            lines.push_back("Last key=" + std::to_string(last_key) + " action=" + last_action);
            lines.push_back("Keys: arrows move  [/] or +/- step  c=center  i=set_init  r=reset  s=save  q=quit");
            lines.push_back("Fallback move: w/a/s/d  Mouse: L=move R=set_init Wheel=step");
            drawOverlayText(view, lines);
            cv::Mat display;
            if (view.cols != kWindowWidth || view.rows != kWindowHeight) {
                cv::resize(view, display, cv::Size(kWindowWidth, kWindowHeight));
            } else {
                display = view;
            }
            cv::imshow(kWindowName, display);
            {
                std::lock_guard<std::mutex> lock(mouse_state.mu);
                mouse_state.view_w = view.cols;
                mouse_state.view_h = view.rows;
            }
        }

        int key_raw = settings.show ? cv::waitKeyEx(1) : -1;
        int key = key_raw >= 0 ? (key_raw & 0xFF) : -1;
        last_key = key_raw;
        const bool key_left = (key_raw == 2424832 || key_raw == 65361);
        const bool key_right = (key_raw == 2555904 || key_raw == 65363);
        const bool key_up = (key_raw == 2490368 || key_raw == 65362);
        const bool key_down = (key_raw == 2621440 || key_raw == 65364);
        if (key_left) {
            boresight.u_L -= settings.step_px;
            last_action = "move_left";
            continue;
        }
        if (key_right) {
            boresight.u_L += settings.step_px;
            last_action = "move_right";
            continue;
        }
        if (key_up) {
            boresight.v_L -= settings.step_px;
            last_action = "move_up";
            continue;
        }
        if (key_down) {
            boresight.v_L += settings.step_px;
            last_action = "move_down";
            continue;
        }
        if (key == 27 || key == 'q' || key == 'Q') {
            last_action = "quit";
            break;
        }
        if (key == '[' || key == '-') {
            settings.step_px = std::max(1, settings.step_px - 1);
            last_action = "step_down";
        } else if (key == ']' || key == '+' || key == '=') {
            settings.step_px = std::min(50, settings.step_px + 1);
            last_action = "step_up";
        } else if (key == 'r' || key == 'R') {
            boresight.u_L = init_u;
            boresight.v_L = init_v;
            last_action = "reset";
        } else if ((key == 'c' || key == 'C') && has_target) {
            boresight.u_L = target_center.x;
            boresight.v_L = target_center.y;
            init_u = boresight.u_L;
            init_v = boresight.v_L;
            init_set = true;
            last_action = "center_on_target";
        } else if (key == 'i' || key == 'I') {
            init_u = boresight.u_L;
            init_v = boresight.v_L;
            init_set = true;
            last_action = "set_init";
        } else if (key == 's' || key == 'S') {
            if (saveBoresight(settings.boresight_out, boresight.u_L, boresight.v_L)) {
                std::cout << "Saved boresight to " << settings.boresight_out << "\n";
                last_action = "save_ok";
            } else {
                std::cout << "Save failed (set --boresight-out)\n";
                last_action = "save_fail";
            }
        } else if (key == 'a' || key == 'A') {
            boresight.u_L -= settings.step_px;
            last_action = "move_left(wasd)";
        } else if (key == 'd' || key == 'D') {
            boresight.u_L += settings.step_px;
            last_action = "move_right(wasd)";
        } else if (key == 'w' || key == 'W') {
            boresight.v_L -= settings.step_px;
            last_action = "move_up(wasd)";
        } else if (key == 'x' || key == 'X') {
            boresight.v_L += settings.step_px;
            last_action = "move_down(wasd)";
        }

        if (settings.show) {
            MouseEvent evt;
            bool has_evt = false;
            {
                std::lock_guard<std::mutex> lock(mouse_state.mu);
                if (mouse_state.pending) {
                    evt.event = mouse_state.event;
                    evt.x = mouse_state.x;
                    evt.y = mouse_state.y;
                    evt.flags = mouse_state.flags;
                    evt.view_w = mouse_state.view_w;
                    evt.view_h = mouse_state.view_h;
                    mouse_state.pending = false;
                    has_evt = true;
                }
            }
            if (has_evt && evt.view_w > 0 && evt.view_h > 0) {
                const double sx = static_cast<double>(evt.view_w) / kWindowWidth;
                const double sy = static_cast<double>(evt.view_h) / kWindowHeight;
                const double u = evt.x * sx;
                const double v = evt.y * sy;
                if (evt.event == cv::EVENT_LBUTTONDOWN) {
                    boresight.u_L = std::min(std::max(0.0, u), static_cast<double>(evt.view_w - 1));
                    boresight.v_L = std::min(std::max(0.0, v), static_cast<double>(evt.view_h - 1));
                    last_action = "mouse_move";
                } else if (evt.event == cv::EVENT_RBUTTONDOWN) {
                    init_u = boresight.u_L;
                    init_v = boresight.v_L;
                    init_set = true;
                    last_action = "mouse_set_init";
                } else if (evt.event == cv::EVENT_MOUSEWHEEL) {
                    int delta = cv::getMouseWheelDelta(evt.flags);
                    if (delta > 0) {
                        settings.step_px = std::min(200, settings.step_px + 1);
                        last_action = "mouse_step_up";
                    } else if (delta < 0) {
                        settings.step_px = std::max(1, settings.step_px - 1);
                        last_action = "mouse_step_down";
                    }
                }
            }
        }
    }

    return 0;
}
