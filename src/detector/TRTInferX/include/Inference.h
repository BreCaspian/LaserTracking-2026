#ifndef __INFERENCE_H
#define __INFERENCE_H

#include "public.h"

namespace TRTInferV1
{
    enum class MemType
    {
        HOST = 0,
        DEVICE = 1
    };

    enum class PreprocessMode
    {
        LETTERBOX = 0,
        RESIZE = 1
    };

    struct RawImageInput
    {
        void *data{nullptr};
        int width{0};
        int height{0};
        int stride_bytes{0};
        MemType mem{MemType::HOST};
        void *cuda_stream{nullptr};
        PreprocessMode prep{PreprocessMode::LETTERBOX};
    };

    struct DetectObject
    {
        cv::Rect2f rect;
        int cls;
        float conf;
    };

    class TRTInfer
    {
    private:
        const char *INPUT_BLOB_NAME = "images";
        int num_classes = -1;
        int max_batch = -1;

    private:
        TRTLogger gLogger;
        IRuntime *runtime;
        ICudaEngine *engine;
        std::vector<IExecutionContext *> contexts;
        struct StreamContext
        {
            cudaStream_t stream_infer = nullptr;
            cudaStream_t stream_copy = nullptr;
            cudaEvent_t infer_start = nullptr;
            cudaEvent_t infer_end = nullptr;
            cudaEvent_t copy_done[2]{};
            int ping = 0;
            float last_gpu_ms = 0.0f;
            std::vector<void *> buffers;
            float *input_dev[2]{nullptr, nullptr};
            float *boxes_host = nullptr;
            float *scores_host = nullptr;
            int *classes_host = nullptr;
            int *num_dets_host = nullptr;
            float *packed_host = nullptr;
            float *raw_boxes_dev = nullptr;
            float *raw_scores_dev = nullptr;
            float *nms_boxes_dev = nullptr;
            float *nms_scores_dev = nullptr;
            int *nms_classes_dev = nullptr;
            int *nms_num_dets_dev = nullptr;
            uint8_t *img_host = nullptr;
            uint8_t *img_device = nullptr;
            float *scale_x_dev = nullptr;
            float *scale_y_dev = nullptr;
            float *padw_dev = nullptr;
            float *padh_dev = nullptr;
            int *origw_dev = nullptr;
            int *origh_dev = nullptr;
        };
        std::vector<StreamContext> stream_ctxs;
        std::atomic<size_t> rr_index{0};
        std::atomic<int> active_streams{1};
        int max_streams = 1;
        int min_streams = 1;
        bool auto_streams = false;
        int adjust_interval = 20;
        int adjust_warmup = 10;
        int adjust_count = 0;
        double ema_ms = 0.0;
        int inputIndex = -1;
        int output_num_dets = -1;
        int output_boxes = -1;
        int output_scores = -1;
        int output_classes = -1;
        int output_packed = -1;
        int raw_num_boxes = 0;
        int raw_num_classes = 0;
        int raw_channels_total = 0;
        int raw_cls_offset = 4;
        bool raw_has_obj = false;
        float nms_score_threshold = 0.08f;
        float nms_iou_threshold = 0.45f;
        int raw_box_format = 0;      // 0: cxcywh, 1: xyxy
        bool raw_score_sigmoid = false;  // Ultralytics ONNX 通常已 sigmoid
        bool debug_log = false;
        double last_gpu_ms = 0.0;
        Dims input_dims;
        Dims boxes_dims;
        Dims scores_dims;
        Dims classes_dims;
        Dims num_dets_dims;
        Dims packed_dims;
        Dims raw_dims;
        int max_det = 0;
        bool packed_output = false;
        bool raw_output = false;
        bool static_batch = false;
        int raw_layout = 0;  // 0: B,C,N ; 1: B,N,C
        ICudaEngine *nms_engine = nullptr;
        std::vector<IExecutionContext *> nms_contexts;

    private:
        int inter_frame_compensation = 0;
        bool _is_inited = false;

    private:
        bool initNmsEngine(int num_boxes, int num_classes, int max_output_boxes);

    public:
        TRTInfer(const int device);
        ~TRTInfer();

        bool initModule(const std::string engine_file_path, const int max_batch, const int num_classes, int stream_count = 1);
        void setNmsThresholds(float score_threshold, float iou_threshold);
        void setRawDecode(bool score_sigmoid, bool box_xyxy);
        void setDebug(bool enable);
        void enableAutoStreams(bool enable, int min_streams = 1);
        void unInitModule();
        void saveEngineFile(IHostMemory *data, const std::string engine_file_path);
        std::vector<std::vector<DetectObject>> doInference(std::vector<cv::Mat> &frames, float confidence_threshold);
        std::vector<std::vector<DetectObject>> doInference(const std::vector<RawImageInput> &frames, float confidence_threshold);
        void calculate_inter_frame_compensation(const int limited_fps);
        std::vector<std::vector<DetectObject>> doInferenceLimitFPS(std::vector<cv::Mat> &frames, float confidence_threshold, const int limited_fps);
        IHostMemory *createEngine(const std::string onnx_path, unsigned int maxBatchSize, int input_h, int input_w,
                                  int max_output_boxes = 300, float score_threshold = 0.25f, float iou_threshold = 0.45f);
        int getInputSizeH();
        int getInputSizeW();
        double getLastGpuInferMs() const;
    };
}

#endif // __INFERENCE_H
