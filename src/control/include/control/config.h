#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

#include <string>

#include "common/types.h"
#include "control/controller.h"

namespace control {

bool loadControlConfig(const std::string& path, ControlConfig* cfg,
                       common::CameraModel* cam, common::Boresight* bs);

}  // namespace control

#endif  // CONTROL_CONFIG_H
