#include "hik_camera/gpu_demosaic_npp.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <limits>
#include <iostream>

#include <cuda_runtime.h>
#include <nppcore.h>
#include <nppi.h>
#include <nppi_color_conversion.h>

#include "hik_camera/PixelType.h"

// 注意 该算子是专为 TRTInferX 高性能推理引擎所设计，用于减少端到端推理延迟所，不具有通用性
// 当启用 HIK_CAMERA_WITH_NPP 时，用 NVIDIA 的 NPP 在 GPU 上把 8-bit Bayer 原始图像快速 demosaic 成 3 通道彩色图像，并在最后把 R/B 通道交换一次以匹配期望的颜色顺序
// 想用使用该部分请务必确保环境有效性！
// https://github.com/BreCaspian/TRTInferX

namespace hik_camera {

#if defined(HIK_CAMERA_WITH_NPP)
namespace {
std::atomic<bool> g_cuda_props_logged{false};
__global__ void swap_rb_kernel(uint8_t* data, int width, int height, int step) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int total = width * height;
    if (idx >= total) {
        return;
    }
    int x = idx % width;
    int y = idx / width;
    uint8_t* p = data + y * step + x * 3;
    uint8_t tmp = p[0];
    p[0] = p[2];
    p[2] = tmp;
}

std::string to_upper(std::string v) {
    std::transform(v.begin(), v.end(), v.begin(),
        [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return v;
}

const char* nppStatusString(NppStatus status) {
    switch (status) {
        case NPP_SUCCESS:
            return "NPP_SUCCESS";
        case NPP_NOT_SUPPORTED_MODE_ERROR:
            return "NPP_NOT_SUPPORTED_MODE_ERROR";
        case NPP_SIZE_ERROR:
            return "NPP_SIZE_ERROR";
        case NPP_BAD_ARGUMENT_ERROR:
            return "NPP_BAD_ARGUMENT_ERROR";
        case NPP_NULL_POINTER_ERROR:
            return "NPP_NULL_POINTER_ERROR";
        case NPP_MEMORY_ALLOCATION_ERR:
            return "NPP_MEMORY_ALLOCATION_ERR";
        default:
            return "NPP_ERROR";
    }
}
}  // namespace

int nppBayerFromPattern(const std::string& pattern) {
    std::string p = to_upper(pattern);
    if (p == "BG") {
        return NPPI_BAYER_BGGR;
    }
    if (p == "GB") {
        return NPPI_BAYER_GBRG;
    }
    if (p == "GR") {
        return NPPI_BAYER_GRBG;
    }
    if (p == "RG") {
        return NPPI_BAYER_RGGB;
    }
    return NPPI_BAYER_GBRG;
}

int nppBayerFromPixelType(uint32_t pixel_type) {
    switch (pixel_type) {
        case PixelType_Gvsp_BayerBG8:
        case PixelType_Gvsp_HB_BayerBG8:
            return NPPI_BAYER_BGGR;
        case PixelType_Gvsp_BayerGB8:
        case PixelType_Gvsp_HB_BayerGB8:
            return NPPI_BAYER_GBRG;
        case PixelType_Gvsp_BayerGR8:
        case PixelType_Gvsp_HB_BayerGR8:
            return NPPI_BAYER_GRBG;
        case PixelType_Gvsp_BayerRG8:
        case PixelType_Gvsp_HB_BayerRG8:
            return NPPI_BAYER_RGGB;
        default:
            return NPPI_BAYER_GBRG;
    }
}

bool nppDemosaicBayer8(const uint8_t* d_src,
                       int src_step,
                       uint8_t* d_dst,
                       int dst_step,
                       int width,
                       int height,
                       int npp_bayer,
                       cudaStream_t stream,
                       std::string* err) {
    if (!d_src || !d_dst || width <= 0 || height <= 0) {
        if (err) {
            *err = "invalid arguments";
        }
        return false;
    }

    NppiSize src_size{};
    src_size.width = width;
    src_size.height = height;
    NppiRect src_roi{};
    src_roi.x = 0;
    src_roi.y = 0;
    src_roi.width = width;
    src_roi.height = height;

    NppStreamContext ctx{};
    int device_id = 0;
    cudaError_t cuda_status = cudaGetDevice(&device_id);
    if (cuda_status != cudaSuccess) {
        if (err) {
            *err = "cudaGetDevice failed";
        }
        return false;
    }

    cudaDeviceProp props{};
    cuda_status = cudaGetDeviceProperties(&props, device_id);
    if (cuda_status != cudaSuccess) {
        if (err) {
            *err = "cudaGetDeviceProperties failed";
        }
        return false;
    }
    if (!g_cuda_props_logged.exchange(true)) {
        std::cout << "CUDA device " << device_id << ": " << props.name
                  << " cc=" << props.major << "." << props.minor
                  << " mp=" << props.multiProcessorCount << "\n";
    }

    unsigned int stream_flags = 0;
    cuda_status = cudaStreamGetFlags(stream, &stream_flags);
    if (cuda_status != cudaSuccess) {
        if (err) {
            *err = "cudaStreamGetFlags failed";
        }
        return false;
    }

    ctx.hStream = stream;
    ctx.nCudaDeviceId = device_id;
    ctx.nMultiProcessorCount = props.multiProcessorCount;
    ctx.nMaxThreadsPerMultiProcessor = props.maxThreadsPerMultiProcessor;
    ctx.nMaxThreadsPerBlock = props.maxThreadsPerBlock;
    ctx.nSharedMemPerBlock = props.sharedMemPerBlock;
    ctx.nCudaDevAttrComputeCapabilityMajor = props.major;
    ctx.nCudaDevAttrComputeCapabilityMinor = props.minor;
    ctx.nStreamFlags = stream_flags;

    auto grid = static_cast<NppiBayerGridPosition>(npp_bayer);
    NppStatus status = nppiCFAToRGB_8u_C1C3R_Ctx(
        d_src, src_step, src_size, src_roi, d_dst, dst_step, grid, NPPI_INTER_UNDEFINED, ctx);
    if (status != NPP_SUCCESS) {
        if (err) {
            *err = nppStatusString(status);
        }
        return false;
    }

    int threads = 256;
    int total = width * height;
    int blocks = (total + threads - 1) / threads;
    swap_rb_kernel<<<blocks, threads, 0, stream>>>(d_dst, width, height, dst_step);
    cudaError_t swap_err = cudaGetLastError();
    if (swap_err != cudaSuccess) {
        if (err) {
            *err = "swap_rb_kernel failed";
        }
        return false;
    }
    return true;
}
#endif

}  // namespace hik_camera
