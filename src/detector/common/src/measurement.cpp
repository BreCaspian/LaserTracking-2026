#include "detector_common/measurement.h"

namespace detector {

TargetMeasurement toMeasurement(const Detection& det, int64_t timestamp_ms) {
    TargetMeasurement out;
    out.valid = det.valid;
    out.timestamp = timestamp_ms;
    out.uv = det.center;
    out.confidence = det.confidence;
    return out;
}

}  // namespace detector
