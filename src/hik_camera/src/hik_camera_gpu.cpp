#include "hik_camera/hik_camera.h"

#include <algorithm>
#include <cctype>
#include <iostream>

#ifdef HIK_CAMERA_WITH_CUDA
#include <cuda_runtime.h>
#endif
#ifdef HIK_CAMERA_WITH_OPENCV_CUDA
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#endif
#ifdef HIK_CAMERA_WITH_NPP
#include "hik_camera/gpu_demosaic_npp.h"
#endif

namespace hik_camera {
namespace {
std::atomic<bool> g_pinned_pool_logged{false};
std::atomic<bool> g_gpu_pipeline_warned{false};
std::atomic<bool> g_gpu_demosaic_warned{false};
std::atomic<bool> g_gpu_demosaic_failed{false};
std::atomic<bool> g_gpu_demosaic_logged{false};
std::atomic<bool> g_gpu_demosaic_backend_warned{false};
std::atomic<bool> g_cuda_device_warned{false};
std::atomic<bool> g_cuda_memcpy_h2d_logged{false};
std::atomic<bool> g_cuda_memcpy_d2h_logged{false};

bool is_bayer8(uint32_t pixel_type) {
    return pixel_type == PixelType_Gvsp_BayerGR8 ||
        pixel_type == PixelType_Gvsp_BayerRG8 ||
        pixel_type == PixelType_Gvsp_BayerGB8 ||
        pixel_type == PixelType_Gvsp_BayerBG8 ||
        pixel_type == PixelType_Gvsp_HB_BayerGR8 ||
        pixel_type == PixelType_Gvsp_HB_BayerRG8 ||
        pixel_type == PixelType_Gvsp_HB_BayerGB8 ||
        pixel_type == PixelType_Gvsp_HB_BayerBG8;
}

#if defined(HIK_CAMERA_WITH_OPENCV_CUDA)
int bayer_code_from_pattern(const std::string& pattern) {
    std::string p = pattern;
    std::transform(p.begin(), p.end(), p.begin(),
        [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    if (p == "BG") {
        return cv::COLOR_BayerBG2BGR;
    }
    if (p == "GB") {
        return cv::COLOR_BayerGB2BGR;
    }
    if (p == "GR") {
        return cv::COLOR_BayerGR2BGR;
    }
    if (p == "RG") {
        return cv::COLOR_BayerRG2BGR;
    }
    return cv::COLOR_BayerGB2BGR;
}

int bayer_code_from_pixel_type(uint32_t pixel_type) {
    switch (pixel_type) {
        case PixelType_Gvsp_BayerBG8:
        case PixelType_Gvsp_HB_BayerBG8:
            return cv::COLOR_BayerBG2BGR;
        case PixelType_Gvsp_BayerGB8:
        case PixelType_Gvsp_HB_BayerGB8:
            return cv::COLOR_BayerGB2BGR;
        case PixelType_Gvsp_BayerGR8:
        case PixelType_Gvsp_HB_BayerGR8:
            return cv::COLOR_BayerGR2BGR;
        case PixelType_Gvsp_BayerRG8:
        case PixelType_Gvsp_HB_BayerRG8:
            return cv::COLOR_BayerRG2BGR;
        default:
            return cv::COLOR_BayerGB2BGR;
    }
}
#endif
}  // namespace

void HikCamera::releaseGpuResources() {
    unregisterPinnedPool();
#ifdef HIK_CAMERA_WITH_CUDA
    if (cuda_stream_) {
        cudaStreamDestroy(cuda_stream_);
        cuda_stream_ = nullptr;
    }
    if (npp_raw_dev_) {
        cudaFree(npp_raw_dev_);
        npp_raw_dev_ = nullptr;
        npp_raw_bytes_ = 0;
    }
    if (npp_bgr_dev_) {
        cudaFree(npp_bgr_dev_);
        npp_bgr_dev_ = nullptr;
        npp_bgr_bytes_ = 0;
    }
#endif
}

void HikCamera::configureGpuPipeline() {
#ifdef HIK_CAMERA_WITH_CUDA
    auto cleanup_cuda_stream = [&]() {
        if (cuda_stream_) {
            cudaStreamDestroy(cuda_stream_);
            cuda_stream_ = nullptr;
        }
    };
    auto cleanup_npp_buffers = [&]() {
        if (npp_raw_dev_) {
            cudaFree(npp_raw_dev_);
            npp_raw_dev_ = nullptr;
            npp_raw_bytes_ = 0;
        }
        if (npp_bgr_dev_) {
            cudaFree(npp_bgr_dev_);
            npp_bgr_dev_ = nullptr;
            npp_bgr_bytes_ = 0;
        }
    };

    if (config_.gpu_pipeline_enable || config_.gpu_demosaic_enable) {
        int device_count = 0;
        if (cudaGetDeviceCount(&device_count) == cudaSuccess && device_count > 0) {
            int dev = std::max(0, std::min(config_.gpu_device_id, device_count - 1));
            if (cudaSetDevice(dev) != cudaSuccess && !g_cuda_device_warned.exchange(true)) {
                std::cerr << "Failed to set CUDA device " << dev << "\n";
            }
            if (config_.gpu_stream_enable) {
                if (!cuda_stream_) {
                    cudaStreamCreateWithFlags(&cuda_stream_, cudaStreamNonBlocking);
                }
            } else {
                cleanup_cuda_stream();
            }
        } else if (!g_cuda_device_warned.exchange(true)) {
            std::cout << "CUDA device not available; GPU features disabled\n";
        }
    } else {
        cleanup_cuda_stream();
        cleanup_npp_buffers();
    }
#else
    if (config_.gpu_pipeline_enable && !g_gpu_pipeline_warned.exchange(true)) {
        std::cout << "GPU pipeline requested but CUDA not enabled; skip pinned pool\n";
    }
#endif
}

void HikCamera::unregisterPinnedPool() {
#ifdef HIK_CAMERA_WITH_CUDA
    if (bgr_pool_pinned_) {
        for (auto& buf : bgr_pool_) {
            if (!buf.empty()) {
                cudaHostUnregister(buf.data());
            }
        }
        bgr_pool_pinned_ = false;
    }
#endif
}

void HikCamera::configurePinnedPool() {
#ifdef HIK_CAMERA_WITH_CUDA
    if (config_.gpu_pipeline_enable && !bgr_pool_pinned_) {
        bool ok = true;
        for (auto& buf : bgr_pool_) {
            if (!buf.empty()) {
                auto ret = cudaHostRegister(buf.data(), buf.size(), cudaHostRegisterPortable);
                if (ret != cudaSuccess) {
                    ok = false;
                    break;
                }
            }
        }
        if (ok) {
            bgr_pool_pinned_ = true;
            if (!g_pinned_pool_logged.exchange(true)) {
                std::cout << "Pinned pool enabled (cudaHostRegister)\n";
            }
        } else {
            unregisterPinnedPool();
            std::cerr << "Pinned pool registration failed, fallback to pageable memory\n";
        }
    }
#else
    if (config_.gpu_pipeline_enable && !g_gpu_pipeline_warned.exchange(true)) {
        std::cout << "GPU pipeline requested but CUDA not enabled; skip pinned pool\n";
    }
#endif
}

bool HikCamera::tryGpuDemosaic(const MV_FRAME_OUT& out_frame,
    const MV_FRAME_OUT_INFO_EX& info,
    Frame& frame,
    uint8_t* bgr_dst) {
    (void)out_frame;
    (void)frame;
    (void)bgr_dst;
    if (!is_bayer8(info.enPixelType)) {
        if (!g_gpu_demosaic_warned.exchange(true)) {
            std::cout << "GPU demosaic requested but pixel type is not Bayer8; fallback to CPU\n";
        }
        return false;
    }

    std::string backend = config_.gpu_demosaic_backend;
    std::transform(backend.begin(), backend.end(), backend.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (backend.empty()) {
        backend = "auto";
    }
    if (backend != "auto" && backend != "opencv" && backend != "npp") {
        if (!g_gpu_demosaic_backend_warned.exchange(true)) {
            std::cout << "Unknown gpu_demosaic_backend='" << backend
                      << "', fallback to auto\n";
        }
        backend = "auto";
    }
    bool want_opencv = backend == "auto" || backend == "opencv";
    bool want_npp = backend == "auto" || backend == "npp";

#if defined(HIK_CAMERA_WITH_OPENCV_CUDA)
    if (want_opencv) {
        if (cv::cuda::getCudaEnabledDeviceCount() <= 0) {
            if (backend == "opencv" && !g_gpu_demosaic_warned.exchange(true)) {
                std::cout << "GPU demosaic requested but no CUDA device; fallback to CPU\n";
            }
        } else {
            try {
                int code = config_.bayer_pattern.empty()
                    ? bayer_code_from_pixel_type(info.enPixelType)
                    : bayer_code_from_pattern(config_.bayer_pattern);
                cv::Mat raw(frame.height, frame.width, CV_8UC1, out_frame.pBufAddr);
                cv::cuda::Stream stream = config_.gpu_stream_enable ? gpu_stream_ : cv::cuda::Stream();
                gpu_raw_.upload(raw, stream);
                cv::cuda::demosaicing(gpu_raw_, gpu_bgr_, code, -1, stream);
                gpu_bgr_.download(frame.bgr, stream);
                stream.waitForCompletion();
                if (!g_gpu_demosaic_logged.exchange(true)) {
                    std::cout << "GPU demosaic active (opencv)\n";
                }
                return true;
            } catch (const cv::Exception& e) {
                if (backend == "opencv" && !g_gpu_demosaic_failed.exchange(true)) {
                    std::cerr << "GPU demosaic failed; fallback to CPU: " << e.what() << "\n";
                }
            }
        }
    }
#else
    if (want_opencv && backend == "opencv" && !g_gpu_demosaic_warned.exchange(true)) {
        std::cout << "GPU demosaic requested but OpenCV CUDA not enabled; fallback to CPU\n";
    }
#endif

#if defined(HIK_CAMERA_WITH_NPP)
    if (want_npp) {
        int device_count = 0;
        if (cudaGetDeviceCount(&device_count) != cudaSuccess || device_count <= 0) {
            if (backend == "npp" && !g_gpu_demosaic_warned.exchange(true)) {
                std::cout << "GPU demosaic requested but no CUDA device; fallback to CPU\n";
            }
        } else {
            size_t raw_bytes = static_cast<size_t>(frame.width) * frame.height;
            size_t bgr_bytes = static_cast<size_t>(frame.width) * frame.height * 3;
            if (npp_raw_bytes_ != raw_bytes) {
                if (npp_raw_dev_) {
                    cudaFree(npp_raw_dev_);
                    npp_raw_dev_ = nullptr;
                }
                if (cudaMalloc(&npp_raw_dev_, raw_bytes) != cudaSuccess) {
                    npp_raw_dev_ = nullptr;
                }
                npp_raw_bytes_ = npp_raw_dev_ ? raw_bytes : 0;
            }
            if (npp_bgr_bytes_ != bgr_bytes) {
                if (npp_bgr_dev_) {
                    cudaFree(npp_bgr_dev_);
                    npp_bgr_dev_ = nullptr;
                }
                if (cudaMalloc(&npp_bgr_dev_, bgr_bytes) != cudaSuccess) {
                    npp_bgr_dev_ = nullptr;
                }
                npp_bgr_bytes_ = npp_bgr_dev_ ? bgr_bytes : 0;
            }

            if (!npp_raw_dev_ || !npp_bgr_dev_) {
                if (backend == "npp" && !g_gpu_demosaic_failed.exchange(true)) {
                    std::cerr << "GPU demosaic failed; CUDA allocation error\n";
                }
            } else {
                cudaStream_t stream = (config_.gpu_stream_enable && cuda_stream_) ? cuda_stream_ : nullptr;
                cudaError_t h2d_status = cudaMemcpyAsync(
                    npp_raw_dev_, out_frame.pBufAddr, raw_bytes, cudaMemcpyHostToDevice, stream);
                if (h2d_status == cudaSuccess && !g_cuda_memcpy_h2d_logged.exchange(true)) {
                    std::cout << "cudaMemcpyAsync H2D ok (npp_raw_dev_)\n";
                }
                int npp_code = config_.bayer_pattern.empty()
                    ? nppBayerFromPixelType(info.enPixelType)
                    : nppBayerFromPattern(config_.bayer_pattern);
                std::string err;
                if (nppDemosaicBayer8(npp_raw_dev_,
                        static_cast<int>(frame.width),
                        npp_bgr_dev_,
                        static_cast<int>(frame.width) * 3,
                        static_cast<int>(frame.width),
                        static_cast<int>(frame.height),
                        npp_code,
                        stream,
                        &err)) {
                    frame.gpu_bgr_ptr = npp_bgr_dev_;
                    frame.gpu_stride_bytes = static_cast<int>(frame.width) * 3;
                    frame.gpu_stream = stream;
                    frame.gpu_valid = true;
                    cudaError_t d2h_status = cudaMemcpyAsync(
                        bgr_dst, npp_bgr_dev_, bgr_bytes, cudaMemcpyDeviceToHost, stream);
                    if (d2h_status == cudaSuccess && !g_cuda_memcpy_d2h_logged.exchange(true)) {
                        std::cout << "cudaMemcpyAsync D2H ok (npp_bgr_dev_)\n";
                    }
                    cudaStreamSynchronize(stream);
                    if (!g_gpu_demosaic_logged.exchange(true)) {
                        std::cout << "GPU demosaic active (npp)\n";
                    }
                    return true;
                }
                if (backend == "npp" && !g_gpu_demosaic_failed.exchange(true)) {
                    std::cerr << "GPU demosaic failed; fallback to CPU: " << err << "\n";
                }
            }
        }
    }
#else
    if (want_npp && backend == "npp" && !g_gpu_demosaic_warned.exchange(true)) {
        std::cout << "GPU demosaic requested but NPP not enabled; fallback to CPU\n";
    }
#endif

    if (backend == "auto" && !g_gpu_demosaic_warned.exchange(true)) {
        std::cout << "GPU demosaic requested but no backend available; fallback to CPU\n";
    }
    return false;
}

}  // namespace hik_camera
