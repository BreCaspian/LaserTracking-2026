#ifndef TRTINFER_POSTPROCESS_H
#define TRTINFER_POSTPROCESS_H

#include "public.h"

void restore_boxes_gpu(float *boxes, const int *num_dets, int max_det, int batch,
                       const float *scale_x, const float *scale_y,
                       const float *padw, const float *padh,
                       const int *orig_w, const int *orig_h, cudaStream_t stream);
void restore_boxes_packed_gpu(float *packed, int max_det, int batch,
                              const float *scale_x, const float *scale_y,
                              const float *padw, const float *padh,
                              const int *orig_w, const int *orig_h, cudaStream_t stream);
// layout: 0 = [B, C, N] (channels first), 1 = [B, N, C] (boxes first)
void decode_boxes_scores_gpu(const float *raw, float *boxes, float *scores, int batch,
                             int num_boxes, int num_classes, int channels_total,
                             int apply_sigmoid, int box_format, int layout,
                             int cls_offset, int has_obj, cudaStream_t stream);

#endif  // TRTINFER_POSTPROCESS_H
