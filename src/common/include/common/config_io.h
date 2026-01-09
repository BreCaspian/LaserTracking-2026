#ifndef COMMON_CONFIG_IO_H
#define COMMON_CONFIG_IO_H

#include <string>

#include "common/types.h"

namespace common {

bool loadCameraModel(const std::string& path, CameraModel* model);
bool loadBoresight(const std::string& path, Boresight* boresight);

}  // namespace common

#endif  // COMMON_CONFIG_IO_H
