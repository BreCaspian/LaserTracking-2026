#include "gimbal_serial/protocol.h"
#include "gimbal_serial/serial_port.h"

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

namespace {
uint32_t nowMs() {
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}
}  // namespace

int main(int argc, char** argv) {
    std::string port = "/dev/ttyUSB0";
    int baud = 115200;
    float send_pitch = 0.0f;
    float send_yaw = 0.0f;
    int send_hz = 50;
    bool stats = false;
    bool dump_hex = false;
    bool reconnect_enable = true;
    int reconnect_delay_ms = 500;
    int read_timeout_ms = 2;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--port" && i + 1 < argc) {
            port = argv[++i];
        } else if (arg == "--baud" && i + 1 < argc) {
            baud = std::stoi(argv[++i]);
        } else if (arg == "--pitch" && i + 1 < argc) {
            send_pitch = std::stof(argv[++i]);
        } else if (arg == "--yaw" && i + 1 < argc) {
            send_yaw = std::stof(argv[++i]);
        } else if (arg == "--send-hz" && i + 1 < argc) {
            send_hz = std::stoi(argv[++i]);
        } else if (arg == "--stats") {
            stats = true;
        } else if (arg == "--dump-hex") {
            dump_hex = true;
        } else if (arg == "--reconnect" && i + 1 < argc) {
            reconnect_enable = std::stoi(argv[++i]) != 0;
        } else if (arg == "--reconnect-delay-ms" && i + 1 < argc) {
            reconnect_delay_ms = std::stoi(argv[++i]);
        } else if (arg == "--read-timeout-ms" && i + 1 < argc) {
            read_timeout_ms = std::stoi(argv[++i]);
        }
    }

    gimbal_serial::SerialPort serial;
    if (!serial.open(port, baud)) {
        return 1;
    }
    std::cout << "Serial opened: " << port << " baud=" << baud << "\n";

    gimbal_serial::FrameParser parser;
    std::vector<uint8_t> rx_buf(2048);

    auto next_send = std::chrono::steady_clock::now();
    const auto send_period = send_hz > 0
        ? std::chrono::microseconds(static_cast<int>(1000000 / send_hz))
        : std::chrono::milliseconds(100);

    uint64_t rx_count = 0;
    uint64_t tx_count = 0;
    uint64_t max_late_us = 0;
    uint64_t read_errors = 0;
    uint64_t write_errors = 0;
    uint64_t reconnects = 0;
    auto last_stat = std::chrono::steady_clock::now();

    while (true) {
        int timeout_ms = std::max(1, read_timeout_ms);
        int n = serial.read(rx_buf.data(), static_cast<int>(rx_buf.size()), timeout_ms);
        if (n > 0) {
            if (dump_hex) {
                uint32_t ts = nowMs();
                std::cout << "DUMP ts=" << ts << " len=" << n << " data=";
                for (int i = 0; i < n; ++i) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0')
                              << static_cast<int>(rx_buf[static_cast<size_t>(i)]) << " ";
                }
                std::cout << std::dec << "\n";
            }
            gimbal_serial::GimbalState state;
            if (parser.push(rx_buf.data(), static_cast<size_t>(n), &state)) {
                std::cout << "RX pitch=" << state.pitch
                          << " yaw=" << state.yaw
                          << " pitch_rate=" << state.pitch_rate
                          << " yaw_rate=" << state.yaw_rate
                          << " ts=" << state.timestamp << "\n";
                rx_count++;
            }
        } else if (n < 0) {
            read_errors++;
        }

        auto now = std::chrono::steady_clock::now();
        if (send_hz <= 0 || now >= next_send) {
            if (send_hz > 0 && now > next_send) {
                auto late_us = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(now - next_send).count());
                if (late_us > max_late_us) {
                    max_late_us = late_us;
                }
            }
            gimbal_serial::GimbalCommand cmd;
            cmd.pitch = send_pitch;
            cmd.yaw = send_yaw;
            cmd.pitch_rate = 0.0f;
            cmd.yaw_rate = 0.0f;
            cmd.timestamp = nowMs();
            uint8_t frame[gimbal_serial::kTxFrameSize]{};
            gimbal_serial::packGimbalCommand(cmd, frame);
            if (serial.write(frame, static_cast<int>(sizeof(frame))) < 0) {
                write_errors++;
            } else {
                tx_count++;
            }
            if (send_hz > 0) {
                next_send += send_period;
                if (next_send < now) {
                    next_send = now + send_period;
                }
            }
        }

        if (stats) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_stat).count();
            if (elapsed >= 1000) {
                double rx_hz = rx_count * 1000.0 / static_cast<double>(elapsed);
                double tx_hz = tx_count * 1000.0 / static_cast<double>(elapsed);
                const auto& pst = parser.stats();
                uint64_t total_bytes = pst.total_bytes;
                uint64_t dropped = pst.discarded_bytes;
                double drop_rate = total_bytes > 0
                    ? static_cast<double>(dropped) / static_cast<double>(total_bytes)
                    : 0.0;
                std::cout << "STAT rx_hz=" << rx_hz << " tx_hz=" << tx_hz
                          << " max_late_us=" << max_late_us
                          << " read_err=" << read_errors
                          << " write_err=" << write_errors
                          << " reconnects=" << reconnects
                          << " bad_frames=" << pst.bad_frames
                          << " drop_rate=" << drop_rate << "\n";
                rx_count = 0;
                tx_count = 0;
                max_late_us = 0;
                read_errors = 0;
                write_errors = 0;
                parser.resetStats();
                last_stat = now;
            }
        }

        if (reconnect_enable && (read_errors > 0 || write_errors > 0)) {
            serial.close();
            std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_delay_ms));
            if (serial.reopen()) {
                reconnects++;
                read_errors = 0;
                write_errors = 0;
            }
        }

        if (send_hz > 0) {
            auto sleep_target = next_send;
            if (sleep_target > now) {
                auto sleep_dur = sleep_target - now;
                if (sleep_dur > std::chrono::microseconds(300)) {
                    std::this_thread::sleep_for(sleep_dur - std::chrono::microseconds(200));
                }
            } else {
                std::this_thread::yield();
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    return 0;
}
