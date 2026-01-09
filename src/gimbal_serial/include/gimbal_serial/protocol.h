#ifndef GIMBAL_SERIAL_PROTOCOL_H
#define GIMBAL_SERIAL_PROTOCOL_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "common/types.h"

namespace gimbal_serial {

constexpr uint8_t kFrameHead = 0xCD;
constexpr uint8_t kFrameTail = 0xDC;
constexpr size_t kRxFrameSize = 22;
constexpr size_t kTxFrameSize = 22;

using GimbalState = common::GimbalState;
using GimbalCommand = common::GimbalCommand;

bool parseGimbalState(const uint8_t* buf, size_t len, GimbalState* out);
void packGimbalCommand(const GimbalCommand& cmd, uint8_t out[kTxFrameSize]);

class FrameParser {
public:
    bool push(const uint8_t* data, size_t len, GimbalState* out);

    struct Stats {
        uint64_t bad_frames = 0;
        uint64_t discarded_bytes = 0;
        uint64_t total_bytes = 0;
    };

    const Stats& stats() const;
    void resetStats();

private:
    std::vector<uint8_t> buffer_;
    Stats stats_;
};

}  // namespace gimbal_serial

#endif  // GIMBAL_SERIAL_PROTOCOL_H
