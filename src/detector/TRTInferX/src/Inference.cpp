#include "Inference.h"
#include "preprocess.h"
#include "postprocess.h"

namespace TRTInferV1
{
    namespace
    {
        int64_t volume(const Dims &dims)
        {
            int64_t v = 1;
            for (int i = 0; i < dims.nbDims; ++i)
                v *= dims.d[i];
            return v;
        }

        ITensor *make_const(INetworkDefinition *network, const std::vector<int> &vals)
        {
            static std::list<std::vector<int>> storage;
            storage.emplace_back(vals);
            const auto &buf = storage.back();
            Weights w{DataType::kINT32, buf.data(), (int64_t)buf.size()};
            nvinfer1::Dims d{};
            d.nbDims = 1;
            d.d[0] = (int)buf.size();
            return network->addConstant(d, w)->getOutput(0);
        }

        ITensor *make_scalar(INetworkDefinition *network, int v)
        {
            return make_const(network, {v});
        }
    }

    TRTInfer::TRTInfer(const int device)
    {
        cudaSetDevice(device);
    }

    TRTInfer::~TRTInfer()
    {
    }

    void TRTInfer::setNmsThresholds(float score_threshold, float iou_threshold)
    {
        this->nms_score_threshold = score_threshold;
        this->nms_iou_threshold = iou_threshold;
    }

    void TRTInfer::setRawDecode(bool score_sigmoid, bool box_xyxy)
    {
        this->raw_score_sigmoid = score_sigmoid;
        this->raw_box_format = box_xyxy ? 1 : 0;
    }

    void TRTInfer::setDebug(bool enable)
    {
        this->debug_log = enable;
    }

    bool TRTInfer::initNmsEngine(int num_boxes, int num_classes, int max_output_boxes)
    {
        if (this->nms_engine)
            return true;
        if (num_boxes <= 0 || num_classes <= 0 || max_output_boxes <= 0)
        {
            this->gLogger.log(ILogger::Severity::kERROR, "Invalid NMS engine params");
            return false;
        }

        initLibNvInferPlugins(&this->gLogger, "");
        IBuilder *builder = createInferBuilder(this->gLogger);
        if (!builder)
        {
            this->gLogger.log(ILogger::Severity::kERROR, "Failed to create TRT builder");
            return false;
        }
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
        uint32_t flag = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
        INetworkDefinition *network = builder->createNetworkV2(flag);
        if (!network)
        {
            this->gLogger.log(ILogger::Severity::kERROR, "Failed to create TRT network");
            trt_destroy(builder);
            return false;
        }

        ITensor *boxes_in = network->addInput("boxes_in", DataType::kFLOAT, Dims3{-1, num_boxes, 4});
        ITensor *scores_in = network->addInput("scores_in", DataType::kFLOAT, Dims3{-1, num_boxes, num_classes});
        if (!boxes_in || !scores_in)
        {
            this->gLogger.log(ILogger::Severity::kERROR, "Failed to create NMS inputs");
            trt_destroy(network);
            trt_destroy(builder);
            return false;
        }

        int background = -1;
        int box_coding = 0;
        int score_activation = 0;
        int class_agnostic = (num_classes == 1) ? 1 : 0;

        PluginField fields[7];
        fields[0] = PluginField{"background_class", &background, PluginFieldType::kINT32, 1};
        fields[1] = PluginField{"box_coding", &box_coding, PluginFieldType::kINT32, 1};
        fields[2] = PluginField{"score_threshold", &this->nms_score_threshold, PluginFieldType::kFLOAT32, 1};
        fields[3] = PluginField{"iou_threshold", &this->nms_iou_threshold, PluginFieldType::kFLOAT32, 1};
        fields[4] = PluginField{"max_output_boxes", &max_output_boxes, PluginFieldType::kINT32, 1};
        fields[5] = PluginField{"score_activation", &score_activation, PluginFieldType::kINT32, 1};
        fields[6] = PluginField{"class_agnostic", &class_agnostic, PluginFieldType::kINT32, 1};
        PluginFieldCollection fc{7, fields};

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
        IPluginCreator *creator = getPluginRegistry()->getPluginCreator("EfficientNMS_TRT", "1");
        if (!creator)
        {
            this->gLogger.log(ILogger::Severity::kERROR, "EfficientNMS_TRT plugin not found");
            trt_destroy(network);
            trt_destroy(builder);
            return false;
        }
        IPluginV2 *plugin = creator->createPlugin("eff_nms", &fc);
        ITensor *nms_inputs[] = {boxes_in, scores_in};
        IPluginV2Layer *nms = network->addPluginV2(nms_inputs, 2, *plugin);
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

        if (!nms)
        {
            this->gLogger.log(ILogger::Severity::kERROR, "Failed to create NMS layer");
            trt_destroy(network);
            trt_destroy(builder);
            return false;
        }

        nms->getOutput(0)->setName("num_dets");
        nms->getOutput(1)->setName("boxes");
        nms->getOutput(2)->setName("scores");
        nms->getOutput(3)->setName("classes");
        network->markOutput(*nms->getOutput(0));
        network->markOutput(*nms->getOutput(1));
        network->markOutput(*nms->getOutput(2));
        network->markOutput(*nms->getOutput(3));

        IBuilderConfig *config = builder->createBuilderConfig();
        IOptimizationProfile *profile = builder->createOptimizationProfile();
        profile->setDimensions("boxes_in", OptProfileSelector::kMIN, Dims3{1, num_boxes, 4});
        profile->setDimensions("boxes_in", OptProfileSelector::kOPT, Dims3{this->max_batch, num_boxes, 4});
        profile->setDimensions("boxes_in", OptProfileSelector::kMAX, Dims3{this->max_batch, num_boxes, 4});
        profile->setDimensions("scores_in", OptProfileSelector::kMIN, Dims3{1, num_boxes, num_classes});
        profile->setDimensions("scores_in", OptProfileSelector::kOPT, Dims3{this->max_batch, num_boxes, num_classes});
        profile->setDimensions("scores_in", OptProfileSelector::kMAX, Dims3{this->max_batch, num_boxes, num_classes});
        config->addOptimizationProfile(profile);
        // Workspace for EfficientNMS (raise for large batch).
        config->setMemoryPoolLimit(MemoryPoolType::kWORKSPACE, 1ull << 30);
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
        config->setFlag(nvinfer1::BuilderFlag::kFP16);
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

        IHostMemory *serialized = builder->buildSerializedNetwork(*network, *config);
        if (!serialized)
        {
            this->gLogger.log(ILogger::Severity::kERROR, "Failed to build NMS engine");
            trt_destroy(config);
            trt_destroy(network);
            trt_destroy(builder);
            return false;
        }
        this->nms_engine = this->runtime->deserializeCudaEngine(serialized->data(), serialized->size());
        trt_destroy(serialized);
        trt_destroy(config);
        trt_destroy(network);
        trt_destroy(builder);
        if (!this->nms_engine)
        {
            this->gLogger.log(ILogger::Severity::kERROR, "Failed to deserialize NMS engine");
            return false;
        }

        this->nms_contexts.resize(this->max_streams, nullptr);
        for (int i = 0; i < this->max_streams; ++i)
        {
            this->nms_contexts[i] = this->nms_engine->createExecutionContext();
            if (!this->nms_contexts[i])
            {
                this->gLogger.log(ILogger::Severity::kERROR, "Failed to create NMS context");
                return false;
            }
        }
        return true;
    }

    bool TRTInfer::initModule(const std::string engine_file_path, const int max_batch, const int num_classes, int stream_count)
    {
        assert(max_batch > 0 && num_classes > 0);
        this->num_classes = num_classes;
        this->max_batch = max_batch;
        if (stream_count < 1)
            stream_count = 1;
        this->max_streams = stream_count;
        this->active_streams.store(stream_count);
        this->adjust_count = 0;
        this->ema_ms = 0.0;

        char *trtModelStream{nullptr};
        size_t size{0};
        std::ifstream file(engine_file_path, std::ios::binary);
        if (!file.good())
        {
            this->gLogger.log(ILogger::Severity::kERROR, "Engine bad file");
            return false;
        }
        file.seekg(0, file.end);
        size = file.tellg();
        file.seekg(0, file.beg);
        trtModelStream = new char[size];
        assert(trtModelStream);
        file.read(trtModelStream, size);
        file.close();

        initLibNvInferPlugins(&this->gLogger, "");
        this->runtime = createInferRuntime(this->gLogger);
        assert(runtime != nullptr);
        this->engine = this->runtime->deserializeCudaEngine(trtModelStream, size);
        assert(this->engine != nullptr);
        this->contexts.resize(stream_count, nullptr);
        for (int i = 0; i < stream_count; ++i)
        {
            this->contexts[i] = this->engine->createExecutionContext();
            assert(this->contexts[i] != nullptr);
        }
        delete[] trtModelStream;

        int io_tensors = engine->getNbIOTensors();
        std::cout << "[TRTInferX] IO tensors: " << io_tensors << std::endl;
        for (int i = 0; i < io_tensors; ++i)
        {
            const char *name = this->engine->getIOTensorName(i);
            auto dtype = this->engine->getTensorDataType(name);
            std::cout << "[TRTInferX] Tensor " << i << ": " << name << " dtype=" << static_cast<int>(dtype) << std::endl;
            if (strcmp(name, INPUT_BLOB_NAME) == 0)
                this->inputIndex = i;
            else if (strcmp(name, "num_dets") == 0)
                this->output_num_dets = i;
            else if (strcmp(name, "boxes") == 0)
                this->output_boxes = i;
            else if (strcmp(name, "scores") == 0)
                this->output_scores = i;
            else if (strcmp(name, "classes") == 0)
                this->output_classes = i;
            else if (strcmp(name, "output0") == 0)
                this->output_packed = i;
        }
        if (this->inputIndex == -1)
        {
            this->gLogger.log(ILogger::Severity::kERROR, "Missing input tensor 'images'.");
            return false;
        }
        bool has_nms_outputs = !(this->output_num_dets == -1 || this->output_boxes == -1 ||
                                 this->output_scores == -1 || this->output_classes == -1);
        if (!has_nms_outputs && this->output_packed == -1)
        {
            this->gLogger.log(ILogger::Severity::kERROR, "Missing outputs. Use nms=True engine or raw output engine.");
            return false;
        }

        this->input_dims = this->engine->getTensorShape(INPUT_BLOB_NAME);
        if (this->input_dims.nbDims != 4)
        {
            this->gLogger.log(ILogger::Severity::kERROR, "Input dims are not NCHW");
            return false;
        }

        if (this->input_dims.d[2] <= 0 || this->input_dims.d[3] <= 0)
        {
            Dims max_dims = this->engine->getProfileShape(INPUT_BLOB_NAME, 0, OptProfileSelector::kMAX);
            this->input_dims.d[2] = max_dims.d[2];
            this->input_dims.d[3] = max_dims.d[3];
        }

        if (this->input_dims.d[0] > 0)
        {
            this->static_batch = true;
            this->max_batch = this->input_dims.d[0];
        }
        else
        {
            Dims max_input_dims = this->engine->getProfileShape(INPUT_BLOB_NAME, 0, OptProfileSelector::kMAX);
            const int profile_max_batch = max_input_dims.d[0];
            if (this->max_batch > profile_max_batch)
                this->max_batch = profile_max_batch;
            this->input_dims.d[0] = this->max_batch;
            for (auto *ctx : this->contexts)
                ctx->setInputShape(INPUT_BLOB_NAME, this->input_dims);
        }

        if (!has_nms_outputs)
        {
            this->packed_dims = this->contexts[0]->getTensorShape("output0");
            if (this->packed_dims.d[0] <= 0 || this->packed_dims.d[1] <= 0 || this->packed_dims.d[2] <= 0)
            {
                Dims max_out = this->engine->getProfileShape("output0", 0, OptProfileSelector::kMAX);
                this->packed_dims = max_out;
            }
            if (this->packed_dims.nbDims != 3)
            {
                this->gLogger.log(ILogger::Severity::kERROR, "Output0 dims invalid");
                return false;
            }
            // 检测 raw layout: [B, C, N] 或 [B, N, C]，避免把 channels=5 误判为 boxes=5
            int cand0_channels = this->packed_dims.d[1]; // 视作 B,C,N
            int cand0_boxes = this->packed_dims.d[2];
            int cand1_channels = this->packed_dims.d[2]; // 视作 B,N,C
            int cand1_boxes = this->packed_dims.d[1];
            bool valid0 = cand0_channels > 4 && cand0_boxes > 0;
            bool valid1 = cand1_channels > 4 && cand1_boxes > 0;
            bool match0 = (this->num_classes > 0) &&
                          (cand0_channels == 4 + this->num_classes || cand0_channels == 5 + this->num_classes);
            bool match1 = (this->num_classes > 0) &&
                          (cand1_channels == 4 + this->num_classes || cand1_channels == 5 + this->num_classes);
            if (valid0 && valid1)
            {
                if (match0 && !match1)
                    this->raw_layout = 0;
                else if (!match0 && match1)
                    this->raw_layout = 1;
                else
                    this->raw_layout = (cand0_channels <= cand0_boxes) ? 0 : 1;
            }
            else
            {
                this->raw_layout = valid0 ? 0 : 1;
            }

            // packed: [B, max_det, 6]；raw: [B, C, N] or [B, N, C]
            if (this->packed_dims.d[2] == 6)
            {
                this->packed_output = true;
                this->max_det = this->packed_dims.d[1];
                std::cout << "[TRTInferX] Packed output dims: [" << this->packed_dims.d[0] << ", "
                          << this->packed_dims.d[1] << ", " << this->packed_dims.d[2] << "], layout=" << this->raw_layout << std::endl;
            }
            else
            {
                this->raw_output = true;
                this->raw_dims = this->packed_dims;
                int channels = (this->raw_layout == 0) ? this->raw_dims.d[1] : this->raw_dims.d[2];
                this->raw_num_boxes = (this->raw_layout == 0) ? this->raw_dims.d[2] : this->raw_dims.d[1];
                this->raw_channels_total = channels - 4;
                if (this->raw_channels_total <= 0 || this->raw_num_boxes <= 0)
                {
                    this->gLogger.log(ILogger::Severity::kERROR, "Raw output dims invalid");
                    return false;
                }
                int cls_count = 0;
                if (this->num_classes > 0)
                {
                    if (this->raw_channels_total == this->num_classes + 1)
                    {
                        this->raw_has_obj = true;
                        cls_count = this->num_classes;
                    }
                    else if (this->raw_channels_total == this->num_classes)
                    {
                        this->raw_has_obj = false;
                        cls_count = this->num_classes;
                    }
                    else if (this->raw_channels_total == 1)
                    {
                        this->raw_has_obj = false;
                        cls_count = 1;
                    }
                    else
                    {
                        this->raw_has_obj = false;
                        cls_count = this->raw_channels_total;
                        this->gLogger.log(ILogger::Severity::kINFO, "raw channels mismatch; inferred classes from output shape");
                    }
                }
                else
                {
                    this->raw_has_obj = false;
                    cls_count = (this->raw_channels_total > 0) ? this->raw_channels_total : 1;
                }
                this->raw_cls_offset = this->raw_has_obj ? 5 : 4;
                this->raw_num_classes = cls_count;
                if (this->raw_num_classes <= 0)
                {
                    this->gLogger.log(ILogger::Severity::kERROR, "Raw num_classes invalid");
                    return false;
                }
                this->max_det = 300;
                std::cout << "[TRTInferX] Raw output dims: [" << this->raw_dims.d[0] << ", "
                          << this->raw_dims.d[1] << ", " << this->raw_dims.d[2] << "], layout=" << this->raw_layout << std::endl;
                if (this->debug_log)
                {
                    std::cout << "[DEBUG] raw channels=" << this->raw_channels_total
                              << " has_obj=" << (this->raw_has_obj ? 1 : 0)
                              << " cls_offset=" << this->raw_cls_offset
                              << " classes=" << this->raw_num_classes << std::endl;
                }
                this->num_dets_dims.nbDims = 1;
                this->num_dets_dims.d[0] = this->max_batch;
                this->boxes_dims.nbDims = 3;
                this->boxes_dims.d[0] = this->max_batch;
                this->boxes_dims.d[1] = this->max_det;
                this->boxes_dims.d[2] = 4;
                this->scores_dims.nbDims = 2;
                this->scores_dims.d[0] = this->max_batch;
                this->scores_dims.d[1] = this->max_det;
                this->classes_dims.nbDims = 2;
                this->classes_dims.d[0] = this->max_batch;
                this->classes_dims.d[1] = this->max_det;
            }
        }
        else
        {
            this->num_dets_dims = this->contexts[0]->getTensorShape("num_dets");
            this->boxes_dims = this->contexts[0]->getTensorShape("boxes");
            this->scores_dims = this->contexts[0]->getTensorShape("scores");
            this->classes_dims = this->contexts[0]->getTensorShape("classes");

            if (this->boxes_dims.nbDims != 3)
            {
                this->gLogger.log(ILogger::Severity::kERROR, "NMS output dims invalid");
                return false;
            }
            this->max_det = this->boxes_dims.d[1];
        }

        if (this->raw_output)
        {
            if (this->num_classes != this->raw_num_classes)
            {
                this->gLogger.log(ILogger::Severity::kWARNING, "num_classes overridden by raw output shape");
                this->num_classes = this->raw_num_classes;
            }
            if (!this->initNmsEngine(this->raw_num_boxes, this->raw_num_classes, this->max_det))
                return false;
        }

        stream_ctxs.resize(stream_count);
        for (int i = 0; i < stream_count; ++i)
        {
            auto &sc = stream_ctxs[i];
            sc.buffers.resize(io_tensors, nullptr);
            // 输入双缓冲，便于拷贝与计算重叠
            size_t input_bytes = (size_t)this->max_batch * this->input_dims.d[1] * this->input_dims.d[2] *
                                 this->input_dims.d[3] * sizeof(float);
            CHECK(cudaMalloc(&sc.input_dev[0], input_bytes));
            CHECK(cudaMalloc(&sc.input_dev[1], input_bytes));
            if (this->packed_output)
            {
                CHECK(cudaMalloc(&sc.buffers[output_packed], volume(this->packed_dims) * sizeof(float)));
                sc.packed_host = (float *)malloc(volume(this->packed_dims) * sizeof(float));
            }
            else if (this->raw_output)
            {
                CHECK(cudaMalloc(&sc.buffers[output_packed], volume(this->raw_dims) * sizeof(float)));
                CHECK(cudaMalloc(&sc.raw_boxes_dev, this->max_batch * this->raw_num_boxes * 4 * sizeof(float)));
                CHECK(cudaMalloc(&sc.raw_scores_dev, this->max_batch * this->raw_num_boxes *
                                                      this->raw_num_classes * sizeof(float)));
                CHECK(cudaMalloc(&sc.nms_num_dets_dev, volume(this->num_dets_dims) * sizeof(int)));
                CHECK(cudaMalloc(&sc.nms_boxes_dev, volume(this->boxes_dims) * sizeof(float)));
                CHECK(cudaMalloc(&sc.nms_scores_dev, volume(this->scores_dims) * sizeof(float)));
                CHECK(cudaMalloc(&sc.nms_classes_dev, volume(this->classes_dims) * sizeof(int)));

                sc.num_dets_host = (int *)malloc(volume(this->num_dets_dims) * sizeof(int));
                sc.boxes_host = (float *)malloc(volume(this->boxes_dims) * sizeof(float));
                sc.scores_host = (float *)malloc(volume(this->scores_dims) * sizeof(float));
                sc.classes_host = (int *)malloc(volume(this->classes_dims) * sizeof(int));
            }
            else
            {
                CHECK(cudaMalloc(&sc.buffers[output_num_dets], volume(this->num_dets_dims) * sizeof(int)));
                CHECK(cudaMalloc(&sc.buffers[output_boxes], volume(this->boxes_dims) * sizeof(float)));
                CHECK(cudaMalloc(&sc.buffers[output_scores], volume(this->scores_dims) * sizeof(float)));
                CHECK(cudaMalloc(&sc.buffers[output_classes], volume(this->classes_dims) * sizeof(int)));

                sc.num_dets_host = (int *)malloc(volume(this->num_dets_dims) * sizeof(int));
                sc.boxes_host = (float *)malloc(volume(this->boxes_dims) * sizeof(float));
                sc.scores_host = (float *)malloc(volume(this->scores_dims) * sizeof(float));
                sc.classes_host = (int *)malloc(volume(this->classes_dims) * sizeof(int));
            }

            CHECK(cudaMalloc(&sc.scale_x_dev, this->max_batch * sizeof(float)));
            CHECK(cudaMalloc(&sc.scale_y_dev, this->max_batch * sizeof(float)));
            CHECK(cudaMalloc(&sc.padw_dev, this->max_batch * sizeof(float)));
            CHECK(cudaMalloc(&sc.padh_dev, this->max_batch * sizeof(float)));
            CHECK(cudaMalloc(&sc.origw_dev, this->max_batch * sizeof(int)));
            CHECK(cudaMalloc(&sc.origh_dev, this->max_batch * sizeof(int)));
            // 固定页内存，加速 H2D 拷贝
            CHECK(cudaMallocHost((void **)&sc.img_host, MAX_IMAGE_INPUT_SIZE_THRESH * 3 * sizeof(uint8_t)));
            CHECK(cudaMalloc((void **)&sc.img_device, MAX_IMAGE_INPUT_SIZE_THRESH * 3 * sizeof(uint8_t)));
            CHECK(cudaStreamCreate(&sc.stream_infer));
            CHECK(cudaEventCreateWithFlags(&sc.infer_start, cudaEventDefault));
            CHECK(cudaEventCreateWithFlags(&sc.infer_end, cudaEventDefault));
            sc.stream_copy = sc.stream_infer;  // 简化为单流，保证顺序正确
            sc.copy_done[0] = nullptr;
            sc.copy_done[1] = nullptr;
            sc.last_gpu_ms = 0.0f;
        }

        this->_is_inited = true;
        return true;
    }

    void TRTInfer::enableAutoStreams(bool enable, int min_streams)
    {
        this->auto_streams = enable;
        if (min_streams < 1)
            min_streams = 1;
        this->min_streams = std::min(min_streams, this->max_streams);
        if (this->active_streams.load() < this->min_streams)
            this->active_streams.store(this->min_streams);
    }

    void TRTInfer::unInitModule()
    {
        this->_is_inited = false;
        for (auto &sc : stream_ctxs)
        {
            for (auto &buf : sc.buffers)
                CHECK(cudaFree(buf));
            CHECK(cudaFree(sc.scale_x_dev));
            CHECK(cudaFree(sc.scale_y_dev));
            CHECK(cudaFree(sc.padw_dev));
            CHECK(cudaFree(sc.padh_dev));
            CHECK(cudaFree(sc.origw_dev));
            CHECK(cudaFree(sc.origh_dev));
            CHECK(cudaFree(sc.raw_boxes_dev));
            CHECK(cudaFree(sc.raw_scores_dev));
            CHECK(cudaFree(sc.nms_num_dets_dev));
            CHECK(cudaFree(sc.nms_boxes_dev));
            CHECK(cudaFree(sc.nms_scores_dev));
            CHECK(cudaFree(sc.nms_classes_dev));
            CHECK(cudaFree(sc.input_dev[0]));
            CHECK(cudaFree(sc.input_dev[1]));
            CHECK(cudaFree(sc.img_device));
            CHECK(cudaFreeHost(sc.img_host));
            if (sc.infer_start)
                CHECK(cudaEventDestroy(sc.infer_start));
            if (sc.infer_end)
                CHECK(cudaEventDestroy(sc.infer_end));
            if (sc.stream_infer)
                CHECK(cudaStreamDestroy(sc.stream_infer));
            free(sc.num_dets_host);
            free(sc.boxes_host);
            free(sc.scores_host);
            free(sc.classes_host);
            free(sc.packed_host);
        }
        stream_ctxs.clear();
        for (auto *ctx : nms_contexts)
        {
            if (ctx)
                trt_destroy(ctx);
        }
        nms_contexts.clear();
        if (this->nms_engine)
            trt_destroy(this->nms_engine);
        this->nms_engine = nullptr;
        for (auto *ctx : contexts)
        {
            if (ctx)
                trt_destroy(ctx);
        }
        contexts.clear();
        if (this->runtime)
            trt_destroy(this->runtime);
        if (this->engine)
            trt_destroy(this->engine);
        this->runtime = nullptr;
        this->engine = nullptr;
    }

    void TRTInfer::saveEngineFile(IHostMemory *data, const std::string engine_file_path)
    {
        std::string serialize_str;
        std::ofstream serialize_output_stream;
        serialize_str.resize(data->size());
        memcpy((void *)serialize_str.data(), data->data(), data->size());
        serialize_output_stream.open(engine_file_path);
        serialize_output_stream << serialize_str;
        serialize_output_stream.close();
    }

    std::vector<std::vector<DetectObject>> TRTInfer::doInference(std::vector<cv::Mat> &frames, float confidence_threshold)
    {
        std::vector<RawImageInput> inputs;
        inputs.reserve(frames.size());
        for (auto &m : frames)
        {
            RawImageInput ri;
            ri.data = m.data;
            ri.width = m.cols;
            ri.height = m.rows;
            ri.stride_bytes = (int)m.step;
            ri.mem = MemType::HOST;
            inputs.push_back(ri);
        }
        return this->doInference(inputs, confidence_threshold);
    }

    std::vector<std::vector<DetectObject>> TRTInfer::doInference(const std::vector<RawImageInput> &frames, float confidence_threshold)
    {
        if (!this->_is_inited)
        {
            this->gLogger.log(ILogger::Severity::kERROR, "Module not inited");
            return {};
        }
        if (frames.empty() || int(frames.size()) > this->max_batch)
        {
            this->gLogger.log(ILogger::Severity::kWARNING, "Invalid frames size");
            return {};
        }

        const int batch = (int)frames.size();
        int current_streams = this->active_streams.load();
        size_t stream_idx = rr_index.fetch_add(1) % current_streams;
        auto &sc = stream_ctxs[stream_idx];
        auto *ctx = contexts[stream_idx];
        cudaStream_t work_stream = sc.stream_infer;
        if (!frames.empty() && frames[0].cuda_stream) {
            work_stream = static_cast<cudaStream_t>(frames[0].cuda_stream);
            for (const auto &f : frames) {
                if (f.cuda_stream && f.cuda_stream != frames[0].cuda_stream) {
                    this->gLogger.log(ILogger::Severity::kERROR, "Mixed cuda_stream in batch is not supported");
                    return {};
                }
            }
        }
        if (this->debug_log && !frames.empty()) {
            const auto &f0 = frames[0];
            std::ostringstream oss;
            oss << "GPU input diag: mem=" << (f0.mem == MemType::DEVICE ? "DEVICE" : "HOST")
                << " w=" << f0.width << " h=" << f0.height
                << " stride_bytes=" << f0.stride_bytes
                << " stream=" << f0.cuda_stream;
            this->gLogger.log(ILogger::Severity::kINFO, oss.str().c_str());
        }
        std::vector<float> scale_x(batch);
        std::vector<float> scale_y(batch);
        std::vector<float> padw(batch);
        std::vector<float> padh(batch);
        std::vector<int> origw(batch);
        std::vector<int> origh(batch);

        std::vector<std::vector<DetectObject>> batch_res(batch);
        auto t0 = std::chrono::high_resolution_clock::now();
        sc.ping = (sc.ping + 1) & 1;
        float *buffer_idx = sc.input_dev[sc.ping];
        for (int b = 0; b < batch; ++b)
        {
            const auto &img = frames[b];
            if (!img.data || img.width <= 0 || img.height <= 0)
                continue;
            if (img.stride_bytes <= 0)
            {
                this->gLogger.log(ILogger::Severity::kERROR, "stride_bytes must be > 0");
                return {};
            }
            origw[b] = img.width;
            origh[b] = img.height;
            if ((size_t)img.width * img.height > (size_t)MAX_IMAGE_INPUT_SIZE_THRESH)
            {
                this->gLogger.log(ILogger::Severity::kERROR, "Input image exceeds MAX_IMAGE_INPUT_SIZE_THRESH");
                return {};
            }
            if (img.prep == PreprocessMode::RESIZE)
            {
                scale_x[b] = this->input_dims.d[3] / (float)img.width;
                scale_y[b] = this->input_dims.d[2] / (float)img.height;
                padw[b] = 0.0f;
                padh[b] = 0.0f;
            }
            else
            {
                float r = std::min(this->input_dims.d[3] / (float)img.width, this->input_dims.d[2] / (float)img.height);
                int new_w = int(img.width * r);
                int new_h = int(img.height * r);
                scale_x[b] = r;
                scale_y[b] = r;
                padw[b] = (this->input_dims.d[3] - new_w) * 0.5f;
                padh[b] = (this->input_dims.d[2] - new_h) * 0.5f;
            }

            size_t size_image_dst = (size_t)this->input_dims.d[3] * this->input_dims.d[2] * 3;
            size_t src_pitch = img.stride_bytes > 0 ? (size_t)img.stride_bytes : (size_t)img.width * 3;
            uint8_t *src_ptr = nullptr;
            int src_line_size = 0;
            if (img.mem == MemType::DEVICE)
            {
                if (img.cuda_stream)
                {
                    cudaEvent_t ready;
                    CHECK(cudaEventCreateWithFlags(&ready, cudaEventDisableTiming));
                    CHECK(cudaEventRecord(ready, static_cast<cudaStream_t>(img.cuda_stream)));
                    CHECK(cudaStreamWaitEvent(work_stream, ready, 0));
                    CHECK(cudaEventDestroy(ready));
                }
                src_ptr = static_cast<uint8_t *>(img.data);
                src_line_size = static_cast<int>(src_pitch);
            }
            else
            {
                size_t dst_pitch = (size_t)img.width * 3;
                CHECK(cudaMemcpy2DAsync(sc.img_device, dst_pitch, img.data, src_pitch, dst_pitch, img.height, cudaMemcpyHostToDevice, work_stream));
                src_ptr = sc.img_device;
                src_line_size = static_cast<int>(dst_pitch);
            }
            int resize_mode = (img.prep == PreprocessMode::RESIZE) ? 1 : 0;
            preprocess_kernel_img(src_ptr, img.width, img.height, src_line_size, buffer_idx,
                                  this->input_dims.d[3], this->input_dims.d[2], resize_mode, work_stream);
            buffer_idx += size_image_dst;
        }

        CHECK(cudaMemcpyAsync(sc.scale_x_dev, scale_x.data(), batch * sizeof(float), cudaMemcpyHostToDevice, work_stream));
        CHECK(cudaMemcpyAsync(sc.scale_y_dev, scale_y.data(), batch * sizeof(float), cudaMemcpyHostToDevice, work_stream));
        CHECK(cudaMemcpyAsync(sc.padw_dev, padw.data(), batch * sizeof(float), cudaMemcpyHostToDevice, work_stream));
        CHECK(cudaMemcpyAsync(sc.padh_dev, padh.data(), batch * sizeof(float), cudaMemcpyHostToDevice, work_stream));
        CHECK(cudaMemcpyAsync(sc.origw_dev, origw.data(), batch * sizeof(int), cudaMemcpyHostToDevice, work_stream));
        CHECK(cudaMemcpyAsync(sc.origh_dev, origh.data(), batch * sizeof(int), cudaMemcpyHostToDevice, work_stream));

        if (this->static_batch)
        {
            if (batch != this->input_dims.d[0])
            {
                this->gLogger.log(ILogger::Severity::kERROR, "Engine uses static batch; input batch mismatch.");
                return {};
            }
        }
        else
        {
            this->input_dims.d[0] = batch;
            ctx->setOptimizationProfileAsync(0, work_stream);
            ctx->setInputShape(INPUT_BLOB_NAME, this->input_dims);
        }
        for (int i = 0; i < (int)sc.buffers.size(); ++i)
        {
            const char *name = this->engine->getIOTensorName(i);
            if (i == this->inputIndex)
                ctx->setTensorAddress(name, sc.input_dev[sc.ping]);
            else
                ctx->setTensorAddress(name, sc.buffers[i]);
        }

        if (sc.infer_start && sc.infer_end)
            CHECK(cudaEventRecord(sc.infer_start, work_stream));
        if (!ctx->enqueueV3(work_stream))
        {
            this->gLogger.log(ILogger::Severity::kERROR, "DoInference failed");
            return {};
        }
        if (sc.infer_start && sc.infer_end)
            CHECK(cudaEventRecord(sc.infer_end, work_stream));
        if (this->packed_output)
        {
            restore_boxes_packed_gpu((float *)sc.buffers[output_packed], this->max_det, batch,
                                     sc.scale_x_dev, sc.scale_y_dev, sc.padw_dev, sc.padh_dev,
                                     sc.origw_dev, sc.origh_dev, work_stream);
            CHECK(cudaMemcpyAsync(sc.packed_host, sc.buffers[output_packed],
                                  batch * this->max_det * 6 * sizeof(float), cudaMemcpyDeviceToHost, work_stream));
            CHECK(cudaStreamSynchronize(work_stream));
            if (this->debug_log)
            {
                std::cout << "[DEBUG] packed output dets (show up to 5, raw values)" << std::endl;
                for (int i = 0; i < std::min(this->max_det, 5); ++i)
                {
                    const float *det = sc.packed_host + i * 6;
                    std::cout << "[DEBUG] det" << i << " score=" << det[4] << " cls=" << det[5]
                              << " box=(" << det[0] << "," << det[1] << "," << det[2] << "," << det[3] << ")\n";
                }
            }
            for (int b = 0; b < batch; ++b)
            {
                auto &res = batch_res[b];
                res.reserve(this->max_det);
                for (int i = 0; i < this->max_det; ++i)
                {
                    const float *det = sc.packed_host + (b * this->max_det + i) * 6;
                    float score = det[4];
                    if (score < confidence_threshold)
                        continue;
                    DetectObject obj;
                    obj.rect = cv::Rect2f(cv::Point2f(det[0], det[1]), cv::Point2f(det[2], det[3]));
                    obj.cls = (int)det[5];
                    obj.conf = score;
                    res.emplace_back(obj);
                }
            }
        }
        else if (this->raw_output)
        {
            decode_boxes_scores_gpu((float *)sc.buffers[output_packed], sc.raw_boxes_dev, sc.raw_scores_dev, batch,
                                    this->raw_num_boxes, this->raw_num_classes,
                                    this->raw_channels_total + 4,
                                    this->raw_score_sigmoid ? 1 : 0,
                                    this->raw_box_format, this->raw_layout, this->raw_cls_offset,
                                    this->raw_has_obj ? 1 : 0, work_stream);

            auto *nms_ctx = nms_contexts[stream_idx];
            nms_ctx->setOptimizationProfileAsync(0, work_stream);
            nms_ctx->setInputShape("boxes_in", Dims3{batch, this->raw_num_boxes, 4});
            nms_ctx->setInputShape("scores_in", Dims3{batch, this->raw_num_boxes, this->raw_num_classes});
            nms_ctx->setTensorAddress("boxes_in", sc.raw_boxes_dev);
            nms_ctx->setTensorAddress("scores_in", sc.raw_scores_dev);
            nms_ctx->setTensorAddress("num_dets", sc.nms_num_dets_dev);
            nms_ctx->setTensorAddress("boxes", sc.nms_boxes_dev);
            nms_ctx->setTensorAddress("scores", sc.nms_scores_dev);
            nms_ctx->setTensorAddress("classes", sc.nms_classes_dev);
            if (!nms_ctx->enqueueV3(work_stream))
            {
                this->gLogger.log(ILogger::Severity::kERROR, "NMS inference failed");
                return {};
            }

            restore_boxes_gpu(sc.nms_boxes_dev, sc.nms_num_dets_dev, this->max_det, batch,
                              sc.scale_x_dev, sc.scale_y_dev, sc.padw_dev, sc.padh_dev,
                              sc.origw_dev, sc.origh_dev, work_stream);

            CHECK(cudaMemcpyAsync(sc.num_dets_host, sc.nms_num_dets_dev, batch * sizeof(int),
                                  cudaMemcpyDeviceToHost, work_stream));
            CHECK(cudaMemcpyAsync(sc.boxes_host, sc.nms_boxes_dev,
                                  batch * this->max_det * 4 * sizeof(float), cudaMemcpyDeviceToHost, work_stream));
            CHECK(cudaMemcpyAsync(sc.scores_host, sc.nms_scores_dev,
                                  batch * this->max_det * sizeof(float), cudaMemcpyDeviceToHost, work_stream));
            CHECK(cudaMemcpyAsync(sc.classes_host, sc.nms_classes_dev,
                                  batch * this->max_det * sizeof(int), cudaMemcpyDeviceToHost, work_stream));
            CHECK(cudaStreamSynchronize(work_stream));

            if (this->debug_log)
            {
                int d0 = sc.num_dets_host[0];
                std::cout << "[DEBUG] raw layout=" << this->raw_layout
                          << " boxes=" << this->raw_num_boxes
                          << " classes=" << this->raw_num_classes
                          << " dets[0]=" << d0
                          << " sigmoid=" << this->raw_score_sigmoid
                          << " has_obj=" << (this->raw_has_obj ? 1 : 0)
                          << " box_format=" << (this->raw_box_format ? "xyxy" : "cxcywh")
                          << std::endl;
                for (int i = 0; i < std::min(d0, 5); ++i)
                {
                    float score = sc.scores_host[i];
                    int cls = sc.classes_host[i];
                    const float *box = sc.boxes_host + i * 4;
                    std::cout << "[DEBUG] det" << i << " score=" << score << " cls=" << cls
                              << " box=(" << box[0] << "," << box[1] << "," << box[2] << "," << box[3] << ")\n";
                }
            }

            for (int b = 0; b < batch; ++b)
            {
                int dets = std::min(sc.num_dets_host[b], this->max_det);
                auto &res = batch_res[b];
                res.reserve(dets);
                for (int i = 0; i < dets; ++i)
                {
                    float score = sc.scores_host[b * this->max_det + i];
                    if (score < confidence_threshold)
                        continue;
                    int cls = sc.classes_host[b * this->max_det + i];
                    const float *box = sc.boxes_host + (b * this->max_det + i) * 4;
                    DetectObject obj;
                    obj.rect = cv::Rect2f(cv::Point2f(box[0], box[1]), cv::Point2f(box[2], box[3]));
                    obj.cls = cls;
                    obj.conf = score;
                    res.emplace_back(obj);
                }
            }
        }
        else
        {
            restore_boxes_gpu((float *)sc.buffers[output_boxes], (int *)sc.buffers[output_num_dets], this->max_det, batch,
                              sc.scale_x_dev, sc.scale_y_dev, sc.padw_dev, sc.padh_dev,
                              sc.origw_dev, sc.origh_dev, work_stream);

            CHECK(cudaMemcpyAsync(sc.num_dets_host, sc.buffers[output_num_dets], batch * sizeof(int),
                                  cudaMemcpyDeviceToHost, work_stream));
            CHECK(cudaMemcpyAsync(sc.boxes_host, sc.buffers[output_boxes],
                                  batch * this->max_det * 4 * sizeof(float), cudaMemcpyDeviceToHost, work_stream));
            CHECK(cudaMemcpyAsync(sc.scores_host, sc.buffers[output_scores],
                                  batch * this->max_det * sizeof(float), cudaMemcpyDeviceToHost, work_stream));
            CHECK(cudaMemcpyAsync(sc.classes_host, sc.buffers[output_classes],
                                  batch * this->max_det * sizeof(int), cudaMemcpyDeviceToHost, work_stream));
            CHECK(cudaStreamSynchronize(work_stream));

            if (this->debug_log)
            {
                int d0 = sc.num_dets_host[0];
                std::cout << "[DEBUG] NMS dets[0]=" << d0 << " max_det=" << this->max_det << std::endl;
                for (int i = 0; i < std::min(d0, 5); ++i)
                {
                    float score = sc.scores_host[i];
                    int cls = sc.classes_host[i];
                    const float *box = sc.boxes_host + i * 4;
                    std::cout << "[DEBUG] det" << i << " score=" << score << " cls=" << cls
                              << " box=(" << box[0] << "," << box[1] << "," << box[2] << "," << box[3] << ")\n";
                }
            }

            for (int b = 0; b < batch; ++b)
            {
                int dets = std::min(sc.num_dets_host[b], this->max_det);
                auto &res = batch_res[b];
                res.reserve(dets);
                for (int i = 0; i < dets; ++i)
                {
                    float score = sc.scores_host[b * this->max_det + i];
                    if (score < confidence_threshold)
                        continue;
                    int cls = sc.classes_host[b * this->max_det + i];
                    const float *box = sc.boxes_host + (b * this->max_det + i) * 4;
                    DetectObject obj;
                    obj.rect = cv::Rect2f(cv::Point2f(box[0], box[1]), cv::Point2f(box[2], box[3]));
                    obj.cls = cls;
                    obj.conf = score;
                    res.emplace_back(obj);
                }
            }
        }
        if (sc.infer_start && sc.infer_end)
        {
            CHECK(cudaEventElapsedTime(&sc.last_gpu_ms, sc.infer_start, sc.infer_end));
            this->last_gpu_ms = sc.last_gpu_ms;
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        if (this->auto_streams && this->max_streams > 1)
        {
            double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            if (this->ema_ms == 0.0)
                this->ema_ms = ms;
            else
                this->ema_ms = 0.9 * this->ema_ms + 0.1 * ms;

            this->adjust_count++;
            if (this->adjust_count > this->adjust_warmup && (this->adjust_count % this->adjust_interval == 0))
            {
                int active = this->active_streams.load();
                if (ms < this->ema_ms * 0.9 && active < this->max_streams)
                    this->active_streams.store(active + 1);
                else if (ms > this->ema_ms * 1.1 && active > this->min_streams)
                    this->active_streams.store(active - 1);
            }
        }
        return batch_res;
    }

    void TRTInfer::calculate_inter_frame_compensation(const int limited_fps)
    {
        std::chrono::system_clock::time_point start_t = std::chrono::system_clock::now();
        double limit_work_time = 1000000L / limited_fps;
        std::this_thread::sleep_for(std::chrono::duration<double, std::micro>(limit_work_time));
        std::chrono::system_clock::time_point end_t = std::chrono::system_clock::now();
        this->inter_frame_compensation = std::chrono::duration<double, std::micro>(end_t - start_t).count() - limit_work_time;
    }

    std::vector<std::vector<DetectObject>> TRTInfer::doInferenceLimitFPS(std::vector<cv::Mat> &frames, float confidence_threshold, const int limited_fps)
    {
        double limit_work_time = 1000000L / limited_fps;
        std::chrono::system_clock::time_point start_t = std::chrono::system_clock::now();
        std::vector<std::vector<DetectObject>> result = this->doInference(frames, confidence_threshold);
        std::chrono::system_clock::time_point end_t = std::chrono::system_clock::now();
        std::chrono::duration<double, std::micro> work_time = end_t - start_t;
        if (work_time.count() < limit_work_time)
        {
            std::this_thread::sleep_for(std::chrono::duration<double, std::micro>(limit_work_time - work_time.count() - this->inter_frame_compensation));
        }
        return result;
    }

    IHostMemory *TRTInfer::createEngine(const std::string onnx_path, unsigned int maxBatchSize, int input_h, int input_w,
                                        int max_output_boxes, float score_threshold, float iou_threshold)
    {
        initLibNvInferPlugins(&this->gLogger, "");
        IBuilder *builder = createInferBuilder(this->gLogger);
        uint32_t flag = 0;
        INetworkDefinition *network = builder->createNetworkV2(flag);

        IParser *parser = createParser(*network, gLogger);
        if (!parser->parseFromFile(onnx_path.c_str(), static_cast<int32_t>(ILogger::Severity::kWARNING)))
        {
            this->gLogger.log(ILogger::Severity::kINTERNAL_ERROR, "failed parse the onnx model");
        }
        for (int32_t i = 0; i < parser->getNbErrors(); ++i)
        {
            std::cout << parser->getError(i)->desc() << std::endl;
        }

        ITensor *raw = network->getOutput(0);
        if (!raw)
        {
            this->gLogger.log(ILogger::Severity::kERROR, "ONNX output not found");
            return nullptr;
        }
        network->unmarkOutput(*raw);

        Dims raw_dims = raw->getDimensions();
        int channels = raw_dims.d[1];
        int num_classes = channels - 4;
        if (num_classes <= 0)
        {
            this->gLogger.log(ILogger::Severity::kERROR, "Invalid output channels");
            return nullptr;
        }

        IShuffleLayer *shuffle = network->addShuffle(*raw);
        Permutation perm{0, 2, 1};
        shuffle->setFirstTranspose(perm);
        ITensor *trans = shuffle->getOutput(0);

        ITensor *shape = network->addShape(*trans)->getOutput(0);
        ITensor *batch = network->addGather(*shape, *make_scalar(network, 0), 0)->getOutput(0);
        ITensor *num_boxes = network->addGather(*shape, *make_scalar(network, 1), 0)->getOutput(0);

        ITensor *start_boxes = make_const(network, {0, 0, 0});
        std::array<ITensor *, 3> size_boxes_inputs{batch, num_boxes, make_scalar(network, 4)};
        ITensor *size_boxes = network->addConcatenation(size_boxes_inputs.data(), 3)->getOutput(0);
        ITensor *stride_boxes = make_const(network, {1, 1, 1});
        ISliceLayer *boxes_slice = network->addSlice(*trans, Dims3{0, 0, 0}, Dims3{1, 1, 1}, Dims3{1, 1, 1});
        boxes_slice->setInput(1, *start_boxes);
        boxes_slice->setInput(2, *size_boxes);
        boxes_slice->setInput(3, *stride_boxes);
        ITensor *boxes = boxes_slice->getOutput(0);

        ITensor *start_scores = make_const(network, {0, 0, 4});
        std::array<ITensor *, 3> size_scores_inputs{batch, num_boxes, make_scalar(network, num_classes)};
        ITensor *size_scores = network->addConcatenation(size_scores_inputs.data(), 3)->getOutput(0);
        ITensor *stride_scores = make_const(network, {1, 1, 1});
        ISliceLayer *scores_slice = network->addSlice(*trans, Dims3{0, 0, 0}, Dims3{1, 1, 1}, Dims3{1, 1, 1});
        scores_slice->setInput(1, *start_scores);
        scores_slice->setInput(2, *size_scores);
        scores_slice->setInput(3, *stride_scores);
        ITensor *scores = scores_slice->getOutput(0);

        int background = -1;
        int box_coding = 0;
        int score_activation = 0;
        int class_agnostic = 0;

        PluginField fields[7];
        fields[0] = PluginField{"background_class", &background, PluginFieldType::kINT32, 1};
        fields[1] = PluginField{"box_coding", &box_coding, PluginFieldType::kINT32, 1};
        fields[2] = PluginField{"score_threshold", &score_threshold, PluginFieldType::kFLOAT32, 1};
        fields[3] = PluginField{"iou_threshold", &iou_threshold, PluginFieldType::kFLOAT32, 1};
        fields[4] = PluginField{"max_output_boxes", &max_output_boxes, PluginFieldType::kINT32, 1};
        fields[5] = PluginField{"score_activation", &score_activation, PluginFieldType::kINT32, 1};
        fields[6] = PluginField{"class_agnostic", &class_agnostic, PluginFieldType::kINT32, 1};
        PluginFieldCollection fc{7, fields};

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
        IPluginCreator *creator = getPluginRegistry()->getPluginCreator("EfficientNMS_TRT", "1");
        if (!creator)
        {
            this->gLogger.log(ILogger::Severity::kERROR, "EfficientNMS_TRT plugin not found");
            return nullptr;
        }
        IPluginV2 *plugin = creator->createPlugin("eff_nms", &fc);
        ITensor *nms_inputs[] = {boxes, scores};
        IPluginV2Layer *nms = network->addPluginV2(nms_inputs, 2, *plugin);
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

        nms->getOutput(0)->setName("num_dets");
        nms->getOutput(1)->setName("boxes");
        nms->getOutput(2)->setName("scores");
        nms->getOutput(3)->setName("classes");
        network->markOutput(*nms->getOutput(0));
        network->markOutput(*nms->getOutput(1));
        network->markOutput(*nms->getOutput(2));
        network->markOutput(*nms->getOutput(3));

        IBuilderConfig *config = builder->createBuilderConfig();
        IOptimizationProfile *profile = builder->createOptimizationProfile();
        profile->setDimensions(INPUT_BLOB_NAME, OptProfileSelector::kMIN, Dims4(1, 3, input_h, input_w));
        profile->setDimensions(INPUT_BLOB_NAME, OptProfileSelector::kOPT, Dims4(int(ceil(maxBatchSize / 2.0)), 3, input_h, input_w));
        profile->setDimensions(INPUT_BLOB_NAME, OptProfileSelector::kMAX, Dims4(maxBatchSize, 3, input_h, input_w));
        config->addOptimizationProfile(profile);
        // Workspace for TensorRT builder (model build phase).
        config->setMemoryPoolLimit(MemoryPoolType::kWORKSPACE, 1ull << 30);
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
        config->setFlag(nvinfer1::BuilderFlag::kFP16);
        config->setFlag(nvinfer1::BuilderFlag::kSPARSE_WEIGHTS);
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
        IHostMemory *serializedModel = builder->buildSerializedNetwork(*network, *config);

        trt_destroy(network);
        trt_destroy(parser);
        trt_destroy(config);
        trt_destroy(builder);

        return serializedModel;
    }

    int TRTInfer::getInputSizeH()
    {
        return this->input_dims.d[2];
    }

    int TRTInfer::getInputSizeW()
    {
        return this->input_dims.d[3];
    }

    double TRTInfer::getLastGpuInferMs() const
    {
        return this->last_gpu_ms;
    }
}
