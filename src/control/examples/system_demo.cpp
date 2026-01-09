#include "common/time_utils.h"
#include "control/config.h"
#include "control/controller.h"
#include "detector_common/measurement.h"
#include "detector_registry/backend.h"
#if defined(DETECTOR_WITH_TRT)
#include "detector_trt/trt_detector.h"
#endif
#include "gimbal_serial/protocol.h"
#include "gimbal_serial/serial_port.h"
#include "hik_camera/hik_camera.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

namespace {
constexpr int kDisplayWidth = 1280;
constexpr int kDisplayHeight = 720;
constexpr int kPlotWidth = 640;
constexpr int kPlotHeight = 240;
constexpr int kPanelPadding = 20;
constexpr int kPanelWidth = kPlotWidth * 2 + kPanelPadding;
constexpr int kPanelHeight = kDisplayHeight + kPlotHeight * 2 + kPanelPadding * 2;
constexpr size_t kHistorySize = 200;
constexpr double kJumpThreshPx = 250.0;
constexpr double kStateJumpDeg = 5.0;
constexpr int64_t kDebugLogIntervalMs = 500;
constexpr int64_t kEpochMsThreshold = 1000000000000LL;

struct SharedMeasurement {
    std::mutex mu;
    common::TargetMeasurement meas;
    bool has_meas = false;
};

struct SharedControlState {
    std::mutex mu;
    common::GimbalCommand cmd;
    common::GimbalState state;
    bool has_cmd = false;
    bool has_state = false;
};

class TeeBuf : public std::streambuf {
public:
    TeeBuf(std::streambuf* a, std::streambuf* b) : a_(a), b_(b) {}
protected:
    int overflow(int c) override {
        if (c == EOF) {
            return !EOF;
        }
        const int r1 = a_->sputc(static_cast<char>(c));
        const int r2 = b_->sputc(static_cast<char>(c));
        return (r1 == EOF || r2 == EOF) ? EOF : c;
    }
    int sync() override {
        const int r1 = a_->pubsync();
        const int r2 = b_->pubsync();
        return (r1 == 0 && r2 == 0) ? 0 : -1;
    }
private:
    std::streambuf* a_;
    std::streambuf* b_;
};

std::string formatWallTimestamp() {
    const auto now = std::chrono::system_clock::now();
    const auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &now_time_t);
#else
    localtime_r(&now_time_t, &tm);
#endif
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count() % 1000;
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S")
        << "." << std::setw(3) << std::setfill('0') << ms;
    return oss.str();
}

void drawPlot(const std::deque<double>& series_a,
              const std::deque<double>& series_b,
              const std::string& label_a,
              const std::string& label_b,
              cv::Mat* plot) {
    plot->setTo(cv::Scalar(20, 20, 20));
    if (series_a.empty() && series_b.empty()) {
        return;
    }
    double max_val = 1.0;
    for (double v : series_a) {
        max_val = std::max(max_val, std::abs(v));
    }
    for (double v : series_b) {
        max_val = std::max(max_val, std::abs(v));
    }
    int n = static_cast<int>(std::max(series_a.size(), series_b.size()));
    if (n < 2) {
        return;
    }
    auto draw_series = [&](const std::deque<double>& series, const cv::Scalar& color) {
        if (series.size() < 2) {
            return;
        }
        int width = plot->cols;
        int height = plot->rows;
        for (size_t i = 1; i < series.size(); ++i) {
            int x0 = static_cast<int>((i - 1) * (width - 1) / static_cast<double>(n - 1));
            int x1 = static_cast<int>(i * (width - 1) / static_cast<double>(n - 1));
            double v0 = series[i - 1];
            double v1 = series[i];
            int y0 = static_cast<int>(height / 2.0 - (v0 / max_val) * (height / 2.0 - 10));
            int y1 = static_cast<int>(height / 2.0 - (v1 / max_val) * (height / 2.0 - 10));
            cv::line(*plot, cv::Point(x0, y0), cv::Point(x1, y1), color, 2, cv::LINE_AA);
        }
    };
    draw_series(series_a, cv::Scalar(0, 255, 255));
    draw_series(series_b, cv::Scalar(0, 128, 255));
    cv::line(*plot, cv::Point(0, plot->rows / 2), cv::Point(plot->cols, plot->rows / 2),
             cv::Scalar(80, 80, 80), 1, cv::LINE_AA);
    cv::putText(*plot, label_a, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::putText(*plot, label_b, cv::Point(10, 45), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 128, 255), 2, cv::LINE_AA);
}

void drawSinglePlot(const std::deque<double>& series,
                    const std::string& label,
                    cv::Mat* plot) {
    plot->setTo(cv::Scalar(20, 20, 20));
    if (series.size() < 2) {
        return;
    }
    double max_val = 1.0;
    for (double v : series) {
        max_val = std::max(max_val, std::abs(v));
    }
    int n = static_cast<int>(series.size());
    for (size_t i = 1; i < series.size(); ++i) {
        int x0 = static_cast<int>((i - 1) * (plot->cols - 1) / static_cast<double>(n - 1));
        int x1 = static_cast<int>(i * (plot->cols - 1) / static_cast<double>(n - 1));
        double v0 = series[i - 1];
        double v1 = series[i];
        int y0 = static_cast<int>(plot->rows / 2.0 - (v0 / max_val) * (plot->rows / 2.0 - 10));
        int y1 = static_cast<int>(plot->rows / 2.0 - (v1 / max_val) * (plot->rows / 2.0 - 10));
        cv::line(*plot, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }
    cv::line(*plot, cv::Point(0, plot->rows / 2), cv::Point(plot->cols, plot->rows / 2),
             cv::Scalar(80, 80, 80), 1, cv::LINE_AA);
    cv::putText(*plot, label, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
}

void pushHistory(std::deque<double>* series, double value) {
    series->push_back(value);
    if (series->size() > kHistorySize) {
        series->pop_front();
    }
}

std::atomic<bool> g_should_exit{false};

void SignalHandler(int /*signum*/) {
    g_should_exit.store(true);
}
}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    std::string camera_config = "../../hik_camera/config/config.yaml";
    std::string detector_config = "../../detector/config/detector.yaml";
    std::string control_config = "../config/control.yaml";
    std::string log_path = "../../log/log.txt";
    std::string port = "/dev/ttyACM0";
    int baud = 115200;
    int send_hz = 100;
    bool show = true;
    bool window_created = false;
    bool window_ready = false;
    int window_guard = 0;
    bool exit_on_close = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--camera-config" && i + 1 < argc) {
            camera_config = argv[++i];
        } else if (arg == "--detector-config" && i + 1 < argc) {
            detector_config = argv[++i];
        } else if (arg == "--control-config" && i + 1 < argc) {
            control_config = argv[++i];
        } else if (arg == "--log-file" && i + 1 < argc) {
            log_path = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            port = argv[++i];
        } else if (arg == "--baud" && i + 1 < argc) {
            baud = std::stoi(argv[++i]);
        } else if (arg == "--send-hz" && i + 1 < argc) {
            send_hz = std::stoi(argv[++i]);
        } else if (arg == "--no-show") {
            show = false;
        } else if (arg == "--exit-on-close") {
            exit_on_close = true;
        }
    }

    std::ofstream log_file(log_path, std::ios::app);
    std::streambuf* cout_buf = std::cout.rdbuf();
    std::streambuf* cerr_buf = std::cerr.rdbuf();
    TeeBuf tee_out(cout_buf, log_file.is_open() ? log_file.rdbuf() : cout_buf);
    TeeBuf tee_err(cerr_buf, log_file.is_open() ? log_file.rdbuf() : cerr_buf);
    std::cout.rdbuf(&tee_out);
    std::cerr.rdbuf(&tee_err);
    if (log_file.is_open()) {
        log_file << "\n\n=== " << formatWallTimestamp() << " ===\n";
        log_file.flush();
        std::cout << "Log file (append): " << log_path << "\n";
    } else {
        std::cerr << "Failed to open log file: " << log_path << "\n";
    }

    std::cout << "camera_config=" << camera_config << "\n";
    std::cout << "detector_config=" << detector_config << "\n";
    std::cout << "control_config=" << control_config << "\n";
    std::cout << std::flush;

    hik_camera::CameraConfig cam_cfg;
    if (!hik_camera::loadCameraConfig(camera_config, &cam_cfg)) {
        std::cerr << "Failed to load camera config\n";
        return 1;
    }
    std::cout << "cam_cfg.feature_load_enable=" << cam_cfg.feature_load_enable
              << " feature_save_enable=" << cam_cfg.feature_save_enable
              << " feature_save_on_close=" << cam_cfg.feature_save_on_close << "\n";
    std::cout << std::flush;

    std::string det_err;
    auto detector = detector::createDetectorFromPath(detector_config, &det_err);
    if (!detector) {
        std::cerr << "Failed to create detector: " << det_err << "\n";
        return 1;
    }
#if defined(DETECTOR_WITH_TRT)
    auto* trt_detector = dynamic_cast<detector_trt::TrtDetector*>(detector.get());
#endif

    control::ControlConfig ctrl_cfg;
    common::CameraModel cam_model;
    common::Boresight boresight;
    if (!loadControlConfig(control_config, &ctrl_cfg, &cam_model, &boresight)) {
        std::cerr << "Failed to load control config\n";
        return 1;
    }
    control::Controller controller(ctrl_cfg);

    hik_camera::HikCamera camera;
    if (!camera.open(cam_cfg) || !camera.startGrabbing()) {
        std::cerr << "Failed to open camera\n";
        return 1;
    }
    std::cout << "Camera opened\n" << std::flush;

    gimbal_serial::SerialPort serial;
    if (!serial.open(port, baud)) {
        std::cerr << "Failed to open serial\n";
        return 1;
    }
    std::cout << "Serial opened: " << port << " baud=" << baud << "\n";
    std::cout << std::flush;

    if (show) {
        try {
            cv::namedWindow("control_panel", cv::WINDOW_NORMAL);
            cv::resizeWindow("control_panel", kPanelWidth, kPanelHeight);
            cv::moveWindow("control_panel", 50, 50);
            cv::Mat init_view(kDisplayHeight, kDisplayWidth, CV_8UC3, cv::Scalar(10, 10, 10));
            cv::putText(init_view, "Starting...", cv::Point(40, 60),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            cv::Mat panel(kPanelHeight, kPanelWidth, CV_8UC3, cv::Scalar(10, 10, 10));
            init_view.copyTo(panel(cv::Rect(0, 0, kDisplayWidth, kDisplayHeight)));
            cv::imshow("control_panel", panel);
            cv::waitKey(1);
            window_created = true;
            std::cout << "GUI windows created\n";
            std::cout << std::flush;
        } catch (const cv::Exception& e) {
            std::cerr << "OpenCV GUI not available: " << e.what() << "\n";
            show = false;
        }
    }

    if (ctrl_cfg.startup_check_frames <= 0) {
        common::GimbalCommand zero_cmd;
        zero_cmd.pitch = 0.0f;
        zero_cmd.yaw = 0.0f;
        zero_cmd.pitch_rate = 0.0f;
        zero_cmd.yaw_rate = 0.0f;
        uint8_t packet[gimbal_serial::kTxFrameSize]{};
        for (int i = 0; i < 5; ++i) {
            zero_cmd.timestamp = static_cast<uint32_t>(common::nowMs());
            gimbal_serial::packGimbalCommand(zero_cmd, packet);
            serial.write(packet, static_cast<int>(sizeof(packet)));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    SharedMeasurement shared_meas;
    SharedControlState shared_ctrl;

    std::thread control_thread([&]() {
        gimbal_serial::FrameParser parser;
        std::vector<uint8_t> rx_buf(512);
        common::GimbalState last_state;
        common::GimbalCommand last_cmd;
        common::GimbalState prev_state;
        bool has_prev_state = false;
        bool has_last_cmd = false;
        int64_t last_debug_ts = 0;
        int64_t last_meas_ts_used = -1;

        const auto send_period =
            std::chrono::microseconds(send_hz > 0 ? static_cast<int>(1000000 / send_hz) : 20000);
        auto next_send = std::chrono::steady_clock::now();
        auto last_loop_ts = next_send;
        auto last_log = next_send;
        double loop_dt_ms_sum = 0.0;
        int loop_dt_count = 0;
        double meas_age_ms_sum = 0.0;
        int meas_age_count = 0;
        double serial_age_ms_sum = 0.0;
        int serial_age_count = 0;
        int send_count = 0;
        double cmd_step_pitch_sum = 0.0;
        double cmd_step_yaw_sum = 0.0;
        double cmd_step_pitch_max = 0.0;
        double cmd_step_yaw_max = 0.0;
        int cmd_step_count = 0;
        int rate_limit_hit_pitch = 0;
        int rate_limit_hit_yaw = 0;
        auto last_send_ts = next_send;
        bool has_last_send_ts = false;

        while (!g_should_exit.load()) {
            auto loop_now = std::chrono::steady_clock::now();
            loop_dt_ms_sum += std::chrono::duration<double, std::milli>(loop_now - last_loop_ts).count();
            loop_dt_count++;
            last_loop_ts = loop_now;

            int n = serial.read(rx_buf.data(), static_cast<int>(rx_buf.size()), 2);
            if (n > 0) {
                gimbal_serial::GimbalState state;
                if (parser.push(rx_buf.data(), static_cast<size_t>(n), &state)) {
                    auto invalid_state = [](float v) {
                        return !std::isfinite(v) || std::fpclassify(v) == FP_SUBNORMAL;
                    };
                    if (invalid_state(state.pitch) || invalid_state(state.yaw)) {
                        std::cout << "DBG state_invalid pitch=" << state.pitch
                                  << " yaw=" << state.yaw
                                  << " ts=" << state.timestamp << "\n";
                        continue;
                    }
                    int64_t now_ms = common::nowSystemMs();
                    if (state.timestamp >= kEpochMsThreshold && now_ms >= kEpochMsThreshold) {
                        int64_t age_ms = now_ms - static_cast<int64_t>(state.timestamp);
                        serial_age_ms_sum += static_cast<double>(age_ms);
                        serial_age_count++;
                    }
                    if (has_prev_state) {
                        double dp = std::abs(state.pitch - prev_state.pitch);
                        double dy = std::abs(state.yaw - prev_state.yaw);
                        if (dp > kStateJumpDeg || dy > kStateJumpDeg) {
                            std::cout << "DBG state_jump dp=" << dp << " dy=" << dy
                                      << " pitch=" << state.pitch
                                      << " yaw=" << state.yaw
                                      << " ts=" << state.timestamp << "\n";
                        }
                    }
                    last_state = state;
                    prev_state = state;
                    has_prev_state = true;
                    {
                        std::lock_guard<std::mutex> lock(shared_ctrl.mu);
                        shared_ctrl.state = state;
                        shared_ctrl.has_state = true;
                    }
                }
            }

            auto now = std::chrono::steady_clock::now();
            while (now >= next_send) {
                auto send_now = std::chrono::steady_clock::now();
                double dt_s = 0.0;
                if (has_last_send_ts) {
                    dt_s = std::chrono::duration<double>(send_now - last_send_ts).count();
                }
                last_send_ts = send_now;
                has_last_send_ts = true;

                int64_t now_ms = common::nowMs();
                common::TargetMeasurement meas_snapshot;
                bool has_meas = false;
                {
                    std::lock_guard<std::mutex> lock(shared_meas.mu);
                    if (shared_meas.has_meas) {
                        meas_snapshot = shared_meas.meas;
                        has_meas = true;
                    }
                }
                if (!has_meas) {
                    meas_snapshot.valid = false;
                    meas_snapshot.timestamp = now_ms;
                    meas_snapshot.uv = cv::Point2f(0.0f, 0.0f);
                    meas_snapshot.confidence = 0.0f;
                }

                common::TargetMeasurement use_meas = meas_snapshot;
                if (!has_meas || !meas_snapshot.valid || meas_snapshot.timestamp <= 0) {
                    use_meas.timestamp = now_ms;
                }

                if (meas_snapshot.timestamp > 0) {
                    meas_age_ms_sum += static_cast<double>(now_ms - meas_snapshot.timestamp);
                    meas_age_count++;
                }

                common::GimbalCommand cmd;
                const bool has_new_meas = (use_meas.timestamp != last_meas_ts_used);
                if (has_new_meas || !has_last_cmd) {
                    cmd = controller.update(use_meas, cam_model, boresight, last_state);
                    last_meas_ts_used = use_meas.timestamp;  // 只在新测量帧到来时更新控制指令，其它时间保持上一条指令；解决相机帧率低造成的 闭环自激振荡
                    if (!use_meas.valid) {
                        int64_t dbg_ts = common::nowMs();
                        if (dbg_ts - last_debug_ts >= kDebugLogIntervalMs) {
                            std::cout << "DBG meas_invalid hold_state pitch=" << last_state.pitch
                                      << " yaw=" << last_state.yaw
                                      << " ts=" << dbg_ts << "\n";
                            last_debug_ts = dbg_ts;
                        }
                    }
                } else {
                    cmd = last_cmd;
                    cmd.timestamp = static_cast<uint32_t>(common::nowMs());
                }
                uint8_t packet[gimbal_serial::kTxFrameSize]{};
                gimbal_serial::packGimbalCommand(cmd, packet);
                serial.write(packet, static_cast<int>(sizeof(packet)));
                if (has_last_cmd && dt_s > 0.0) {
                    double max_step = ctrl_cfg.max_angle_rate * dt_s;
                    double dp = std::abs(cmd.pitch - last_cmd.pitch);
                    double dy = std::abs(cmd.yaw - last_cmd.yaw);
                    cmd_step_pitch_sum += dp;
                    cmd_step_yaw_sum += dy;
                    cmd_step_pitch_max = std::max(cmd_step_pitch_max, dp);
                    cmd_step_yaw_max = std::max(cmd_step_yaw_max, dy);
                    cmd_step_count++;
                    if (max_step > 0.0) {
                        if (dp >= 0.95 * max_step) {
                            rate_limit_hit_pitch++;
                        }
                        if (dy >= 0.95 * max_step) {
                            rate_limit_hit_yaw++;
                        }
                    }
                }
                last_cmd = cmd;
                has_last_cmd = true;
                {
                    std::lock_guard<std::mutex> lock(shared_ctrl.mu);
                    shared_ctrl.cmd = cmd;
                    shared_ctrl.has_cmd = true;
                }

                send_count++;
                next_send += send_period;
                now = std::chrono::steady_clock::now();
            }

            auto log_now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(log_now - last_log).count() >= 1000) {
                double elapsed_ms = static_cast<double>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(log_now - last_log).count());
                double send_rate = elapsed_ms > 0.0 ? send_count * 1000.0 / elapsed_ms : 0.0;
                double avg_loop_dt_ms = loop_dt_count > 0 ? loop_dt_ms_sum / loop_dt_count : 0.0;
                double avg_meas_age_ms = meas_age_count > 0 ? meas_age_ms_sum / meas_age_count : 0.0;
                double avg_serial_age_ms = serial_age_count > 0 ? serial_age_ms_sum / serial_age_count : 0.0;
                double avg_cmd_step_pitch = cmd_step_count > 0 ? cmd_step_pitch_sum / cmd_step_count : 0.0;
                double avg_cmd_step_yaw = cmd_step_count > 0 ? cmd_step_yaw_sum / cmd_step_count : 0.0;
                double pitch_limit_ratio = cmd_step_count > 0
                    ? static_cast<double>(rate_limit_hit_pitch) / cmd_step_count
                    : 0.0;
                double yaw_limit_ratio = cmd_step_count > 0
                    ? static_cast<double>(rate_limit_hit_yaw) / cmd_step_count
                    : 0.0;
                std::cout << "STAT_CTRL send_hz=" << send_rate
                          << " loop_dt_ms=" << avg_loop_dt_ms
                          << " meas_age_ms=" << avg_meas_age_ms
                          << " cmd_step_deg(p,y)=(" << avg_cmd_step_pitch << "," << avg_cmd_step_yaw << ")"
                          << " cmd_step_max(p,y)=(" << cmd_step_pitch_max << "," << cmd_step_yaw_max << ")"
                          << " rate_hit(p,y)=(" << pitch_limit_ratio << "," << yaw_limit_ratio << ")"
                          << " serial_age_ms=";
                if (serial_age_count > 0) {
                    std::cout << avg_serial_age_ms;
                } else {
                    std::cout << "na";
                }
                std::cout << "\n";

                loop_dt_ms_sum = 0.0;
                loop_dt_count = 0;
                meas_age_ms_sum = 0.0;
                meas_age_count = 0;
                serial_age_ms_sum = 0.0;
                serial_age_count = 0;
                send_count = 0;
                cmd_step_pitch_sum = 0.0;
                cmd_step_yaw_sum = 0.0;
                cmd_step_pitch_max = 0.0;
                cmd_step_yaw_max = 0.0;
                cmd_step_count = 0;
                rate_limit_hit_pitch = 0;
                rate_limit_hit_yaw = 0;
                last_log = log_now;
            }
        }
    });

    std::deque<double> err_history;
    std::deque<double> pitch_history;
    std::deque<double> yaw_history;
    std::deque<double> pitch_rate_history;
    std::deque<double> yaw_rate_history;
    std::deque<double> pitch_fb_history;
    std::deque<double> yaw_fb_history;
    uint64_t frame_count = 0;
    uint64_t meas_count = 0;
    double infer_ms_sum = 0.0;
    int infer_count = 0;
    double frame_age_ms_sum = 0.0;
    int frame_age_count = 0;
    double frame_to_infer_ms_sum = 0.0;
    int frame_to_infer_count = 0;
    auto last_log = std::chrono::steady_clock::now();
    bool last_meas_valid = false;
    cv::Point2f last_meas_uv(0.0f, 0.0f);
    double last_meas_conf = 0.0;
    int lost_streak = 0;
    int64_t last_det_log_ts = 0;

    std::cout << "Enter detect/control loops\n";
    std::cout << std::flush;
    while (true) {
        if (g_should_exit.load()) {
            break;
        }
        try {
            hik_camera::Frame frame;
            bool got_frame = camera.read(&frame, cam_cfg.grab_timeout_ms);
            bool has_gpu_input = false;
            if (got_frame) {
#if defined(DETECTOR_WITH_TRT)
                has_gpu_input = (trt_detector && frame.gpu_valid && frame.gpu_bgr_ptr);
#endif
                if (!has_gpu_input && frame.bgr.empty()) {
                    got_frame = false;
                }
            }
            if (!got_frame) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            auto frame_ready_ts = std::chrono::steady_clock::now();
            int64_t ts = common::nowMs();
            int64_t host_now_ms = common::nowSystemMs();
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
            std::vector<detector::Detection> dets;
            auto infer_start = std::chrono::steady_clock::now();
            if (got_frame) {
                if (frame.host_timestamp > 0) {
                    int64_t host_ts_ms = frame.host_timestamp;
                    if (host_ts_ms > kEpochMsThreshold * 1000LL) {
                        host_ts_ms /= 1000;
                    }
                    if (host_now_ms >= kEpochMsThreshold && host_ts_ms >= kEpochMsThreshold) {
                        double age_ms = static_cast<double>(host_now_ms - host_ts_ms);
                        frame_age_ms_sum += age_ms;
                        frame_age_count++;
                    }
                }
                dets = detector->detect(frame.bgr, ts);
                auto infer_end = std::chrono::steady_clock::now();
                double infer_ms = std::chrono::duration<double, std::milli>(infer_end - infer_start).count();
                infer_ms_sum += infer_ms;
                infer_count++;
                frame_to_infer_ms_sum += std::chrono::duration<double, std::milli>(infer_end - frame_ready_ts).count();
                frame_to_infer_count++;
            } else {
                auto infer_end = infer_start;
                frame_to_infer_ms_sum += std::chrono::duration<double, std::milli>(infer_end - frame_ready_ts).count();
                frame_to_infer_count++;
            }
            common::TargetMeasurement meas;
            meas.timestamp = ts;
            if (!dets.empty()) {
                meas = detector::toMeasurement(dets.front(), ts);
            }
            if (got_frame) {
                frame_count++;
            }
            if (meas.valid) {
                meas_count++;
            }

            if (!got_frame) {
                std::cout << "DBG frame_miss ts=" << ts << "\n";
            }

            if (meas.valid) {
                lost_streak = 0;
                int64_t det_log_ts = ts;
                if (det_log_ts - last_det_log_ts >= 1000) {
                    if (!dets.empty()) {
                        const auto& det = dets.front();
                        double du = det.center.x - boresight.u_L;
                        double dv = det.center.y - boresight.v_L;
                        std::cout << "DBG det_status ts=" << ts
                                  << " uv=(" << det.center.x << "," << det.center.y << ")"
                                  << " du=" << du << " dv=" << dv
                                  << " bbox=(" << det.bbox.x << "," << det.bbox.y
                                  << "," << det.bbox.width << "," << det.bbox.height << ")"
                                  << " conf=" << det.confidence
                                  << " label=" << det.label
                                  << " det_count=" << dets.size()
                                  << " frame=(";
                        if (got_frame) {
                            std::cout << frame.width << "," << frame.height;
                        } else {
                            std::cout << "na,na";
                        }
                        std::cout << ")"
                                  << " boresight=(" << boresight.u_L << "," << boresight.v_L << ")"
                                  << "\n";
                    }
                    last_det_log_ts = det_log_ts;
                }
                if (!last_meas_valid) {
                    std::cout << "DBG meas_acquire ts=" << ts
                              << " uv=(" << meas.uv.x << "," << meas.uv.y << ")"
                              << " conf=" << meas.confidence << "\n";
                } else {
                    double du = meas.uv.x - last_meas_uv.x;
                    double dv = meas.uv.y - last_meas_uv.y;
                    double dist = std::sqrt(du * du + dv * dv);
                    if (dist > kJumpThreshPx) {
                        std::cout << "DBG meas_jump ts=" << ts
                                  << " dist_px=" << dist
                                  << " uv=(" << meas.uv.x << "," << meas.uv.y << ")"
                                  << " last=(" << last_meas_uv.x << "," << last_meas_uv.y << ")\n";
                        if (!dets.empty()) {
                            const auto& det = dets.front();
                            std::cout << "DBG det0 center=(" << det.center.x << "," << det.center.y << ")"
                                      << " bbox=(" << det.bbox.x << "," << det.bbox.y
                                      << "," << det.bbox.width << "," << det.bbox.height << ")"
                                      << " conf=" << det.confidence
                                      << " label=" << det.label
                                      << " count=" << dets.size()
                                      << "\n";
                        }
                    }
                }
            } else {
                lost_streak++;
                if (last_meas_valid) {
                    std::cout << "DBG meas_lost ts=" << ts
                              << " lost_streak=" << lost_streak << "\n";
                }
            }

            last_meas_valid = meas.valid;
            if (meas.valid) {
                last_meas_uv = meas.uv;
                last_meas_conf = meas.confidence;
            }

            if (got_frame) {
                std::lock_guard<std::mutex> lock(shared_meas.mu);
                shared_meas.meas = meas;
                shared_meas.has_meas = true;
            }

            common::GimbalCommand cmd_snapshot;
            common::GimbalState state_snapshot;
            bool has_cmd = false;
            bool has_state = false;
            {
                std::lock_guard<std::mutex> lock(shared_ctrl.mu);
                if (shared_ctrl.has_cmd) {
                    cmd_snapshot = shared_ctrl.cmd;
                    has_cmd = true;
                }
                if (shared_ctrl.has_state) {
                    state_snapshot = shared_ctrl.state;
                    has_state = true;
                }
            }

            if (has_cmd) {
                double err_px = 0.0;
                if (meas.valid) {
                    double du = meas.uv.x - boresight.u_L;
                    double dv = meas.uv.y - boresight.v_L;
                    err_px = std::sqrt(du * du + dv * dv);
                }
                pushHistory(&err_history, err_px);
                pushHistory(&pitch_history, cmd_snapshot.pitch);
                pushHistory(&yaw_history, cmd_snapshot.yaw);
                pushHistory(&pitch_rate_history, cmd_snapshot.pitch_rate);
                pushHistory(&yaw_rate_history, cmd_snapshot.yaw_rate);
            }
            if (has_state) {
                pushHistory(&pitch_fb_history, state_snapshot.pitch);
                pushHistory(&yaw_fb_history, state_snapshot.yaw);
            }

            auto log_now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(log_now - last_log).count() >= 1000) {
                double fps = frame_count * 1000.0 /
                    static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(log_now - last_log).count());
                double avg_infer_ms = infer_count > 0 ? infer_ms_sum / infer_count : 0.0;
                double avg_frame_age_ms = frame_age_count > 0 ? frame_age_ms_sum / frame_age_count : 0.0;
                double avg_frame_to_infer_ms =
                    frame_to_infer_count > 0 ? frame_to_infer_ms_sum / frame_to_infer_count : 0.0;
                std::cout << "STAT fps=" << fps
                          << " infer_ms=" << avg_infer_ms
                          << " frame_to_infer_ms=" << avg_frame_to_infer_ms
                          << " frame_age_ms=";
                if (frame_age_count > 0) {
                    std::cout << avg_frame_age_ms;
                } else {
                    std::cout << "na";
                }
                std::cout
                          << " meas=" << meas_count;
                if (has_cmd) {
                    std::cout << " cmd(p,y)=(" << cmd_snapshot.pitch << "," << cmd_snapshot.yaw << ")";
                }
                if (has_state) {
                    std::cout << " fb(p,y)=(" << state_snapshot.pitch << "," << state_snapshot.yaw << ")";
                }
                std::cout
                          << " uv=(" << meas.uv.x << "," << meas.uv.y << ")"
                          << " valid=" << (meas.valid ? 1 : 0)
                          << "\n";
                frame_count = 0;
                meas_count = 0;
                infer_ms_sum = 0.0;
                infer_count = 0;
                frame_age_ms_sum = 0.0;
                frame_age_count = 0;
                frame_to_infer_ms_sum = 0.0;
                frame_to_infer_count = 0;
                last_log = log_now;
            }

            if (show && window_created) {
                cv::Mat view;
                if (got_frame) {
                    cv::resize(frame.bgr, view, cv::Size(kDisplayWidth, kDisplayHeight));
                } else {
                    view = cv::Mat(kDisplayHeight, kDisplayWidth, CV_8UC3, cv::Scalar(10, 10, 10));
                    cv::putText(view, "Waiting for frames...", cv::Point(40, 60),
                                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
                }

                double sx = 1.0;
                double sy = 1.0;
                cv::Point boresight_pt(kDisplayWidth / 2, kDisplayHeight / 2);
                if (got_frame) {
                    sx = static_cast<double>(kDisplayWidth) / frame.bgr.cols;
                    sy = static_cast<double>(kDisplayHeight) / frame.bgr.rows;
                    boresight_pt = cv::Point(static_cast<int>(boresight.u_L * sx),
                                             static_cast<int>(boresight.v_L * sy));
                }
                cv::drawMarker(view, boresight_pt, cv::Scalar(0, 0, 255),
                               cv::MARKER_CROSS, 30, 3, cv::LINE_AA);

                if (meas.valid && got_frame) {
                    cv::Point target_pt(static_cast<int>(meas.uv.x * sx),
                                        static_cast<int>(meas.uv.y * sy));
                    cv::circle(view, target_pt, 12, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
                }

                const int font = cv::FONT_HERSHEY_SIMPLEX;
                if (has_cmd) {
                    cv::putText(view, "Pitch cmd: " + std::to_string(cmd_snapshot.pitch),
                                cv::Point(20, 40), font, 0.8, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
                    cv::putText(view, "Yaw cmd: " + std::to_string(cmd_snapshot.yaw),
                                cv::Point(20, 70), font, 0.8, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
                }
                if (has_state) {
                    cv::putText(view, "Pitch fb: " + std::to_string(state_snapshot.pitch),
                                cv::Point(20, 100), font, 0.8, cv::Scalar(180, 255, 255), 2, cv::LINE_AA);
                    cv::putText(view, "Yaw fb: " + std::to_string(state_snapshot.yaw),
                                cv::Point(20, 130), font, 0.8, cv::Scalar(180, 255, 255), 2, cv::LINE_AA);
                }
                cv::putText(view, "Conf: " + std::to_string(last_meas_conf),
                            cv::Point(20, 160), font, 0.8, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

                try {
                    cv::Mat plot(kPlotHeight, kPlotWidth, CV_8UC3);
                    drawPlot(pitch_history, yaw_history, "pitch_cmd (deg)", "yaw_cmd (deg)", &plot);

                    cv::Mat plot_err(kPlotHeight, kPlotWidth, CV_8UC3);
                    drawSinglePlot(err_history, "err_px", &plot_err);

                    cv::Mat plot_rate(kPlotHeight, kPlotWidth, CV_8UC3);
                    drawPlot(pitch_rate_history, yaw_rate_history,
                             "pitch_rate_cmd (deg/s)", "yaw_rate_cmd (deg/s)", &plot_rate);

                    cv::Mat plot_fb(kPlotHeight, kPlotWidth, CV_8UC3);
                    drawPlot(pitch_fb_history, yaw_fb_history, "pitch_fb (deg)", "yaw_fb (deg)", &plot_fb);

                    cv::Mat panel(kPanelHeight, kPanelWidth, CV_8UC3, cv::Scalar(10, 10, 10));
                    view.copyTo(panel(cv::Rect(0, 0, kDisplayWidth, kDisplayHeight)));
                    plot.copyTo(panel(cv::Rect(0, kDisplayHeight + kPanelPadding, kPlotWidth, kPlotHeight)));
                    plot_err.copyTo(panel(cv::Rect(kPlotWidth + kPanelPadding,
                                                   kDisplayHeight + kPanelPadding,
                                                   kPlotWidth, kPlotHeight)));
                    plot_rate.copyTo(panel(cv::Rect(0,
                                                    kDisplayHeight + kPanelPadding * 2 + kPlotHeight,
                                                    kPlotWidth, kPlotHeight)));
                    plot_fb.copyTo(panel(cv::Rect(kPlotWidth + kPanelPadding,
                                                  kDisplayHeight + kPanelPadding * 2 + kPlotHeight,
                                                  kPlotWidth, kPlotHeight)));
                    cv::imshow("control_panel", panel);
                } catch (const cv::Exception& e) {
                    std::cerr << "OpenCV GUI error: " << e.what() << "\n";
                    show = false;
                }

                int key = cv::waitKey(1);
                window_ready = true;
                window_guard++;
                if (key == 'q' || key == 'Q') {
                    break;
                }
                if (exit_on_close && window_ready && window_guard > 5) {
                    double visible = cv::getWindowProperty("control_panel", cv::WND_PROP_VISIBLE);
                    if (visible >= 0.0 && visible < 1.0) {
                        break;
                    }
                }
            }
        } catch (const cv::Exception& e) {
            std::cerr << "OpenCV error: " << e.what() << "\n";
            show = false;
        } catch (const std::exception& e) {
            std::cerr << "Unhandled exception: " << e.what() << "\n";
            break;
        }
    }

    g_should_exit.store(true);
    if (control_thread.joinable()) {
        control_thread.join();
    }
    return 0;
}
