#include "gimbal_serial/protocol.h"

#include <algorithm>
#include <cstring>

namespace gimbal_serial {

bool parseGimbalState(const uint8_t* buf, size_t len, GimbalState* out) {
    if (!buf || !out) {
        return false;
    }
    if (len < kRxFrameSize) {
        return false;
    }
    if (buf[0] != kFrameHead || buf[21] != kFrameTail) {
        return false;
    }

    std::memcpy(&out->pitch, buf + 1, sizeof(float));
    std::memcpy(&out->yaw, buf + 5, sizeof(float));
    std::memcpy(&out->pitch_rate, buf + 9, sizeof(float));
    std::memcpy(&out->yaw_rate, buf + 13, sizeof(float));
    std::memcpy(&out->timestamp, buf + 17, sizeof(uint32_t));
    return true;
}

void packGimbalCommand(const GimbalCommand& cmd, uint8_t out[kTxFrameSize]) {
    out[0] = kFrameHead;
    std::memcpy(out + 1, &cmd.pitch, sizeof(float));
    std::memcpy(out + 5, &cmd.yaw, sizeof(float));
    std::memcpy(out + 9, &cmd.pitch_rate, sizeof(float));
    std::memcpy(out + 13, &cmd.yaw_rate, sizeof(float));
    std::memcpy(out + 17, &cmd.timestamp, sizeof(uint32_t));
    out[21] = kFrameTail;
}

bool FrameParser::push(const uint8_t* data, size_t len, GimbalState* out) {
    if (!data || len == 0) {
        return false;
    }
    stats_.total_bytes += len;
    buffer_.insert(buffer_.end(), data, data + len);
    bool got = false;
    while (buffer_.size() >= kRxFrameSize) {
        auto it = std::find(buffer_.begin(), buffer_.end(), kFrameHead);
        if (it == buffer_.end()) {
            if (buffer_.size() > kRxFrameSize - 1) {
                size_t keep = kRxFrameSize - 1;
                stats_.discarded_bytes += buffer_.size() - keep;
                buffer_.erase(buffer_.begin(), buffer_.end() - static_cast<long>(keep));
            }
            return got;
        }
        size_t start = static_cast<size_t>(it - buffer_.begin());
        if (buffer_.size() - start < kRxFrameSize) {
            if (start > 0) {
                stats_.discarded_bytes += start;
                buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<long>(start));
            }
            return got;
        }
        if (buffer_[start + 21] == kFrameTail) {
            bool ok = parseGimbalState(buffer_.data() + start, kRxFrameSize, out);
            buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<long>(start + kRxFrameSize));
            got = got || ok;
            continue;
        }
        stats_.bad_frames++;
        auto next = std::find(buffer_.begin() + static_cast<long>(start + 1), buffer_.end(), kFrameHead);
        if (next == buffer_.end()) {
            stats_.discarded_bytes += buffer_.size();
            buffer_.clear();
            return got;
        }
        stats_.discarded_bytes += static_cast<uint64_t>(next - buffer_.begin());
        buffer_.erase(buffer_.begin(), next);
    }
    return got;
}

const FrameParser::Stats& FrameParser::stats() const {
    return stats_;
}

void FrameParser::resetStats() {
    stats_ = {};
}

}  // namespace gimbal_serial
