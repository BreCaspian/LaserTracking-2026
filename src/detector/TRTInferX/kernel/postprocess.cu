#include "postprocess.h"

// 我写的可能不算完美，可以尝试继续优化算子，算子目前已经很快了，单论算子来说优化空间极限，0.0X ms 的提升，回报比较低
// 真正的瓶颈始终是H2D/D2H，只有做好这个才能让推理引擎真正吃饱吃好，不然推理引擎绝大多是时间都是在空转，都是在等饭吃没活干

__global__ void restore_boxes_kernel(float *__restrict__ boxes, const int *__restrict__ num_dets, int max_det, int batch,
                                     const float *__restrict__ scale_x, const float *__restrict__ scale_y,
                                     const float *__restrict__ padw, const float *__restrict__ padh,
                                     const int *__restrict__ orig_w, const int *__restrict__ orig_h)
{
    int total = batch * max_det;
    for (int idx = blockIdx.x * blockDim.x + threadIdx.x; idx < total;
         idx += blockDim.x * gridDim.x)
    {
        int b = idx / max_det;
        int i = idx - b * max_det;
        if (i >= num_dets[b])
            continue;

        float sx = scale_x[b];
        float sy = scale_y[b];
        float dw = padw[b];
        float dh = padh[b];
        int ow = orig_w[b];
        int oh = orig_h[b];

        float *box = boxes + (b * max_det + i) * 4;
        float x1 = (box[0] - dw) / sx;
        float y1 = (box[1] - dh) / sy;
        float x2 = (box[2] - dw) / sx;
        float y2 = (box[3] - dh) / sy;

        x1 = fminf(fmaxf(x1, 0.0f), ow - 1.0f);
        y1 = fminf(fmaxf(y1, 0.0f), oh - 1.0f);
        x2 = fminf(fmaxf(x2, 0.0f), ow - 1.0f);
        y2 = fminf(fmaxf(y2, 0.0f), oh - 1.0f);

        box[0] = x1;
        box[1] = y1;
        box[2] = x2;
        box[3] = y2;
    }
}

void restore_boxes_gpu(float *boxes, const int *num_dets, int max_det, int batch,
                       const float *scale_x, const float *scale_y,
                       const float *padw, const float *padh,
                       const int *orig_w, const int *orig_h, cudaStream_t stream)
{
    int total = batch * max_det;
    int threads = 256;
    int blocks = (total + threads - 1) / threads;
    int max_blocks = 4096;
    if (blocks > max_blocks)
        blocks = max_blocks;
    restore_boxes_kernel<<<blocks, threads, 0, stream>>>(boxes, num_dets, max_det, batch,
                                                         scale_x, scale_y, padw, padh, orig_w, orig_h);
}

__global__ void restore_boxes_packed_kernel(float *__restrict__ packed, int max_det, int batch,
                                            const float *__restrict__ scale_x, const float *__restrict__ scale_y,
                                            const float *__restrict__ padw,
                                            const float *__restrict__ padh, const int *__restrict__ orig_w,
                                            const int *__restrict__ orig_h)
{
    int total = batch * max_det;
    for (int idx = blockIdx.x * blockDim.x + threadIdx.x; idx < total;
         idx += blockDim.x * gridDim.x)
    {
        int b = idx / max_det;
        int i = idx - b * max_det;
        float sx = scale_x[b];
        float sy = scale_y[b];
        float dw = padw[b];
        float dh = padh[b];
        int ow = orig_w[b];
        int oh = orig_h[b];

        float *box = packed + (b * max_det + i) * 6;
        float x1 = (box[0] - dw) / sx;
        float y1 = (box[1] - dh) / sy;
        float x2 = (box[2] - dw) / sx;
        float y2 = (box[3] - dh) / sy;

        x1 = fminf(fmaxf(x1, 0.0f), ow - 1.0f);
        y1 = fminf(fmaxf(y1, 0.0f), oh - 1.0f);
        x2 = fminf(fmaxf(x2, 0.0f), ow - 1.0f);
        y2 = fminf(fmaxf(y2, 0.0f), oh - 1.0f);

        box[0] = x1;
        box[1] = y1;
        box[2] = x2;
        box[3] = y2;
    }
}

void restore_boxes_packed_gpu(float *packed, int max_det, int batch,
                              const float *scale_x, const float *scale_y,
                              const float *padw, const float *padh,
                              const int *orig_w, const int *orig_h, cudaStream_t stream)
{
    int total = batch * max_det;
    int threads = 256;
    int blocks = (total + threads - 1) / threads;
    int max_blocks = 4096;
    if (blocks > max_blocks)
        blocks = max_blocks;
    restore_boxes_packed_kernel<<<blocks, threads, 0, stream>>>(
        packed, max_det, batch, scale_x, scale_y, padw, padh, orig_w, orig_h);
}

__device__ __forceinline__ float sigmoid(float x)
{
    return 1.0f / (1.0f + expf(-x));
}

__global__ void decode_boxes_scores_kernel(const float *__restrict__ raw, float *__restrict__ boxes,
                                           float *__restrict__ scores, int batch, int num_boxes,
                                           int num_classes, int channels_total,
                                           int apply_sigmoid, int box_format, int layout,
                                           int cls_offset, int has_obj)
{
    // Decode raw head to boxes/scores; supports [B,C,N] and [B,N,C]
    int total = batch * num_boxes;
    for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < total;
         i += blockDim.x * gridDim.x)
    {
        int b = i / num_boxes;
        int n = i - b * num_boxes;
        int channels = channels_total;
        const float *base;
        if (layout == 0)
        {
            base = raw + (b * channels) * num_boxes;
            float x1 = base[0 * num_boxes + n];
            float y1 = base[1 * num_boxes + n];
            float x2 = base[2 * num_boxes + n];
            float y2 = base[3 * num_boxes + n];
            if (box_format == 0)
            {
                float cx = x1;
                float cy = y1;
                float w = x2;
                float h = y2;
                x1 = cx - 0.5f * w;
                y1 = cy - 0.5f * h;
                x2 = cx + 0.5f * w;
                y2 = cy + 0.5f * h;
            }
            float *box = boxes + (b * num_boxes + n) * 4;
            box[0] = x1;
            box[1] = y1;
            box[2] = x2;
            box[3] = y2;

            float obj = 1.0f;
            if (has_obj)
            {
                float obj_raw = base[4 * num_boxes + n];
                obj = apply_sigmoid ? sigmoid(obj_raw) : obj_raw;
            }
            float *score = scores + (b * num_boxes + n) * num_classes;
            for (int c = 0; c < num_classes; ++c)
            {
                float v = base[(cls_offset + c) * num_boxes + n];
                float cls = apply_sigmoid ? sigmoid(v) : v;
                score[c] = has_obj ? (obj * cls) : cls;
            }
        }
        else
        {
            base = raw + (b * num_boxes + n) * channels;
            float x1 = base[0];
            float y1 = base[1];
            float x2 = base[2];
            float y2 = base[3];
            if (box_format == 0)
            {
                float cx = x1;
                float cy = y1;
                float w = x2;
                float h = y2;
                x1 = cx - 0.5f * w;
                y1 = cy - 0.5f * h;
                x2 = cx + 0.5f * w;
                y2 = cy + 0.5f * h;
            }
            float *box = boxes + (b * num_boxes + n) * 4;
            box[0] = x1;
            box[1] = y1;
            box[2] = x2;
            box[3] = y2;

            float obj = 1.0f;
            if (has_obj)
            {
                float obj_raw = base[4];
                obj = apply_sigmoid ? sigmoid(obj_raw) : obj_raw;
            }
            float *score = scores + (b * num_boxes + n) * num_classes;
            for (int c = 0; c < num_classes; ++c)
            {
                float v = base[cls_offset + c];
                float cls = apply_sigmoid ? sigmoid(v) : v;
                score[c] = has_obj ? (obj * cls) : cls;
            }
        }
    }
}

void decode_boxes_scores_gpu(const float *raw, float *boxes, float *scores, int batch,
                             int num_boxes, int num_classes, int channels_total,
                             int apply_sigmoid, int box_format, int layout,
                             int cls_offset, int has_obj, cudaStream_t stream)
{
    int total = batch * num_boxes;
    int threads = 256;
    int blocks = (total + threads - 1) / threads;
    int max_blocks = 4096;
    if (blocks > max_blocks)
        blocks = max_blocks;
    decode_boxes_scores_kernel<<<blocks, threads, 0, stream>>>(
        raw, boxes, scores, batch, num_boxes, num_classes, channels_total,
        apply_sigmoid, box_format, layout, cls_offset, has_obj);
}
