#ifndef DETECTOR_MEASUREMENT_H
#define DETECTOR_MEASUREMENT_H

#include <cstdint>

#include <opencv2/core.hpp>

#include "common/types.h"
#include "detector_common/detector.h"

namespace detector {

using TargetMeasurement = common::TargetMeasurement;

TargetMeasurement toMeasurement(const Detection& det, int64_t timestamp_ms);

}  // namespace detector

#endif  // DETECTOR_MEASUREMENT_H
