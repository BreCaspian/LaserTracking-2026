#ifndef __PUBLIC_H
#define __PUBLIC_H

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <list>
#include <sstream>
#include <string>
#include <atomic>
#include <thread>
#include <vector>

#include "NvInfer.h"
#include "NvInferPlugin.h"
#include "NvOnnxParser.h"
#include "cuda_runtime.h"
#include "cuda_runtime_api.h"
#include "logging.h"
#include "macros.h"
using namespace nvonnxparser;

#define CHECK(status)                                          \
    do                                                         \
    {                                                          \
        auto ret = (status);                                   \
        if (ret != 0)                                          \
        {                                                      \
            std::cerr << "Cuda failure: " << ret << std::endl; \
            abort();                                           \
        }                                                      \
    } while (0)

#define MAX_IMAGE_INPUT_SIZE_THRESH 10000 * 10000
#define MAX_OUTPUT_BBOX_COUNT 1000

#define PROCESSOR_1_MASK                    0x01 << 1
#define PROCESSOR_2_MASK                    0x01 << 2
#define PROCESSOR_3_MASK                    0x01 << 3

using namespace nvinfer1;

#endif  // __PUBLIC_H
