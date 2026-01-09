#include "common/config_io.h"

#include <iostream>

#include <opencv2/core.hpp>

namespace common {

bool loadCameraModel(const std::string& path, CameraModel* model) {
    if (!model) {
        return false;
    }
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open camera model: " << path << "\n";
        return false;
    }
    fs["fx"] >> model->fx;
    fs["fy"] >> model->fy;
    fs["cx"] >> model->cx;
    fs["cy"] >> model->cy;
    return model->fx > 0.0 && model->fy > 0.0;
}

bool loadBoresight(const std::string& path, Boresight* boresight) {
    if (!boresight) {
        return false;
    }
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open boresight: " << path << "\n";
        return false;
    }
    fs["u_L"] >> boresight->u_L;
    fs["v_L"] >> boresight->v_L;
    return true;
}

}  // namespace common
