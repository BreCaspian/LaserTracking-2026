#include "preprocess.h"

// 我写的可能不算完美，可以尝试继续优化算子，算子目前已经很快了，单论算子来说优化空间极限，0.0X ms 的提升回报比较低
// 真正的瓶颈始终是H2D/D2H，只有做好这个才能让推理引擎真正吃饱吃好，不然推理引擎绝大多是时间都是在空转，都是在等饭吃

__device__ __forceinline__ int floor_int(float x)
{
    return __float2int_rd(x);
}

__global__ void warpaffine_kernel(
    const uint8_t *__restrict__ src, int src_line_size, int src_width,
    int src_height, float *__restrict__ dst, int dst_width,
    int dst_height, uint8_t const_value_st,
    AffineMatrix d2s, int edge)
{
    // Letterbox + BGR->RGB + /255 in a single pass
    const float m_x1 = d2s.value[0];
    const float m_y1 = d2s.value[1];
    const float m_z1 = d2s.value[2];
    const float m_x2 = d2s.value[3];
    const float m_y2 = d2s.value[4];
    const float m_z2 = d2s.value[5];
    const float inv255 = 1.0f / 255.0f;
    const int area = dst_width * dst_height;

    for (int position = blockDim.x * blockIdx.x + threadIdx.x; position < edge;
         position += blockDim.x * gridDim.x)
    {
        int dx = position - (position / dst_width) * dst_width;
        int dy = position / dst_width;
        float src_x = m_x1 * dx + m_y1 * dy + m_z1 + 0.5f;
        float src_y = m_x2 * dx + m_y2 * dy + m_z2 + 0.5f;
        float c0, c1, c2;

        if (src_x <= -1.0f || src_x >= src_width || src_y <= -1.0f || src_y >= src_height)
        {
            c0 = const_value_st;
            c1 = const_value_st;
            c2 = const_value_st;
        }
        else
        {
            int y_low = floor_int(src_y);
            int x_low = floor_int(src_x);
            int y_high = y_low + 1;
            int x_high = x_low + 1;

            uint8_t const_value[] = {const_value_st, const_value_st, const_value_st};
            float ly = src_y - y_low;
            float lx = src_x - x_low;
            float hy = 1.0f - ly;
            float hx = 1.0f - lx;
            float w1 = hy * hx, w2 = hy * lx, w3 = ly * hx, w4 = ly * lx;
            const uint8_t *v1 = const_value;
            const uint8_t *v2 = const_value;
            const uint8_t *v3 = const_value;
            const uint8_t *v4 = const_value;

            if (y_low >= 0)
            {
                if (x_low >= 0)
                    v1 = src + y_low * src_line_size + x_low * 3;
                if (x_high < src_width)
                    v2 = src + y_low * src_line_size + x_high * 3;
            }
            if (y_high < src_height)
            {
                if (x_low >= 0)
                    v3 = src + y_high * src_line_size + x_low * 3;
                if (x_high < src_width)
                    v4 = src + y_high * src_line_size + x_high * 3;
            }

            c0 = w1 * v1[0] + w2 * v2[0] + w3 * v3[0] + w4 * v4[0];
            c1 = w1 * v1[1] + w2 * v2[1] + w3 * v3[1] + w4 * v4[1];
            c2 = w1 * v1[2] + w2 * v2[2] + w3 * v3[2] + w4 * v4[2];
        }

        float t = c2;
        c2 = c0;
        c0 = t;

        c0 *= inv255;
        c1 *= inv255;
        c2 *= inv255;

        float *pdst_c0 = dst + dy * dst_width + dx;
        float *pdst_c1 = pdst_c0 + area;
        float *pdst_c2 = pdst_c1 + area;
        *pdst_c0 = c0;
        *pdst_c1 = c1;
        *pdst_c2 = c2;
    }
}

void preprocess_kernel_img(
    uint8_t *src, int src_width, int src_height, int src_line_size,
    float *dst, int dst_width, int dst_height, int resize_mode,
    cudaStream_t stream)
{
    AffineMatrix s2d, d2s;
    if (resize_mode)
    {
        float sx = dst_width / (float)src_width;
        float sy = dst_height / (float)src_height;
        s2d.value[0] = sx;
        s2d.value[1] = 0;
        s2d.value[2] = 0;
        s2d.value[3] = 0;
        s2d.value[4] = sy;
        s2d.value[5] = 0;
    }
    else
    {
        float scale = std::min(dst_height / (float)src_height, dst_width / (float)src_width);
        s2d.value[0] = scale;
        s2d.value[1] = 0;
        s2d.value[2] = -scale * src_width * 0.5 + dst_width * 0.5;
        s2d.value[3] = 0;
        s2d.value[4] = scale;
        s2d.value[5] = -scale * src_height * 0.5 + dst_height * 0.5;
    }

    cv::Mat m2x3_s2d(2, 3, CV_32F, s2d.value);
    cv::Mat m2x3_d2s(2, 3, CV_32F, d2s.value);
    cv::invertAffineTransform(m2x3_s2d, m2x3_d2s);

    memcpy(d2s.value, m2x3_d2s.ptr<float>(0), sizeof(d2s.value));

    int jobs = dst_height * dst_width;
    int threads = 256;
    int blocks = (jobs + threads - 1) / threads;
    int max_blocks = 4096;
    if (blocks > max_blocks)
        blocks = max_blocks;
    warpaffine_kernel<<<blocks, threads, 0, stream>>>(
        src, src_line_size, src_width,
        src_height, dst, dst_width,
        dst_height, 128, d2s, jobs);
}
