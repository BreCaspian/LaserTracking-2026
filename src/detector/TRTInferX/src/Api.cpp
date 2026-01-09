#include "api.h"

namespace TRTInferX
{
    bool Api::load(const EngineConfig &cfg)
    {
        cfg_ = cfg;
        core_ = std::make_unique<TRTInferV1::TRTInfer>(cfg.device);
        core_->setNmsThresholds(cfg.nms_score, cfg.nms_iou);
        core_->setRawDecode(default_opt_.apply_sigmoid, default_opt_.box_fmt == 1);
        if (!core_->initModule(cfg.engine_path, cfg.max_batch, cfg.num_classes, cfg.streams))
            return false;
        if (!validateConfig())
            return false;
        if (cfg.auto_streams)
            core_->enableAutoStreams(true);
        return true;
    }

    std::vector<std::vector<Det>> Api::infer(const std::vector<ImageInput> &batch, const InferOptions &opt)
    {
        auto res = inferWithInfo(batch, opt);
        std::vector<std::vector<Det>> out;
        out.reserve(res.size());
        for (auto &r : res)
            out.emplace_back(std::move(r.dets));
        return out;
    }

    std::vector<Result> Api::inferWithInfo(const std::vector<ImageInput> &batch, const InferOptions &opt)
    {
        if (!core_ || batch.empty())
            return {};

        std::vector<TRTInferV1::RawImageInput> inputs;
        inputs.reserve(batch.size());
        std::vector<PreprocInfo> infos(batch.size());

        int engine_h = core_->getInputSizeH();
        int engine_w = core_->getInputSizeW();

        for (size_t idx = 0; idx < batch.size(); ++idx)
        {
            const auto &im = batch[idx];
            if (!im.data)
            {
                std::cerr << "[TRTInferX][API] Null data pointer at index " << idx << std::endl;
                return {};
            }
            if (im.width <= 0 || im.height <= 0)
            {
                std::cerr << "[TRTInferX][API] Invalid image size at index " << idx << std::endl;
                return {};
            }
            if (im.color != ColorSpace::BGR || im.layout != Layout::HWC || im.dtype != DType::UINT8)
            {
                std::cerr << "[TRTInferX][API] Unsupported input format at index " << idx
                          << " (expect BGR/HWC/uint8)" << std::endl;
                return {};
            }
            if (im.stride_bytes <= 0)
            {
                std::cerr << "[TRTInferX][API] Invalid stride_bytes at index " << idx
                          << " (must be > 0)" << std::endl;
                return {};
            }
            if (im.stride_bytes < im.width * 3)
            {
                std::cerr << "[TRTInferX][API] stride_bytes smaller than width*3 at index " << idx << std::endl;
                return {};
            }
            if (im.mem == MemoryType::GPU && !im.cuda_stream)
            {
                std::cerr << "[TRTInferX][API] GPU input without cuda_stream at index " << idx
                          << ": ensure external sync" << std::endl;
            }
            if ((im.target_w > 0 && im.target_w != engine_w) ||
                (im.target_h > 0 && im.target_h != engine_h))
            {
                std::cerr << "[TRTInferX][API] target_w/target_h mismatch at index " << idx
                          << " engine=" << engine_w << "x" << engine_h
                          << " input=" << im.target_w << "x" << im.target_h << std::endl;
            }
            TRTInferV1::RawImageInput ri;
            ri.data = im.data;
            ri.width = im.width;
            ri.height = im.height;
            ri.stride_bytes = im.stride_bytes;
            ri.mem = (im.mem == MemoryType::GPU) ? TRTInferV1::MemType::DEVICE : TRTInferV1::MemType::HOST;
            ri.cuda_stream = im.cuda_stream;
            ri.prep = (im.prep == PreprocessMode::RESIZE) ? TRTInferV1::PreprocessMode::RESIZE
                                                          : TRTInferV1::PreprocessMode::LETTERBOX;
            inputs.emplace_back(ri);

            // 记录预处理尺度信息，假定 letterbox/resize 与底层一致
            PreprocInfo pi;
            int tw = engine_w > 0 ? engine_w : (im.target_w > 0 ? im.target_w : cfg_.target_w);
            int th = engine_h > 0 ? engine_h : (im.target_h > 0 ? im.target_h : cfg_.target_h);
            pi.src_w = im.width;
            pi.src_h = im.height;
            if (im.prep == PreprocessMode::LETTERBOX)
            {
                float r = std::min(tw / float(im.width), th / float(im.height));
                pi.scale = r;
                pi.scale_x = r;
                pi.scale_y = r;
                pi.padw = (tw - im.width * r) * 0.5f;
                pi.padh = (th - im.height * r) * 0.5f;
            }
            else
            {
                pi.scale_x = tw > 0 ? float(tw) / im.width : 1.0f;
                pi.scale_y = th > 0 ? float(th) / im.height : 1.0f;
                pi.scale = std::min(pi.scale_x, pi.scale_y);
                pi.padw = 0.0f;
                pi.padh = 0.0f;
            }
            infos[idx] = pi;
        }

        core_->setRawDecode(opt.apply_sigmoid, opt.box_fmt == 1);

        auto dets = core_->doInference(inputs, opt.conf);

        std::vector<Result> out(dets.size());
        for (size_t b = 0; b < dets.size(); ++b)
        {
            out[b].dets.reserve(dets[b].size());
            out[b].preproc = infos[b];
            for (const auto &d : dets[b])
            {
                Det td;
                td.x1 = d.rect.x;
                td.y1 = d.rect.y;
                td.x2 = d.rect.x + d.rect.width;
                td.y2 = d.rect.y + d.rect.height;
                td.score = d.conf;
                td.cls = d.cls;
                td.batch = static_cast<int>(b);
                out[b].dets.push_back(td);
            }
        }
        return out;
    }

    void Api::warmup(int batch, int iters)
    {
        if (!core_)
            return;
        int b = std::max(1, batch);
        cv::Mat dummy(cfg_.target_h, cfg_.target_w, CV_8UC3, cv::Scalar::all(0));
        std::vector<cv::Mat> frames(b, dummy);
        for (int i = 0; i < iters; ++i)
            core_->doInference(frames, default_opt_.conf);
    }

    bool Api::validateConfig() const
    {
        if (!core_)
            return false;
        int engine_h = core_->getInputSizeH();
        int engine_w = core_->getInputSizeW();
        if (engine_h <= 0 || engine_w <= 0)
        {
            std::cerr << "[TRTInferX][API] Invalid engine input size." << std::endl;
            return false;
        }
        if ((cfg_.target_w > 0 && cfg_.target_w != engine_w) ||
            (cfg_.target_h > 0 && cfg_.target_h != engine_h))
        {
            std::cerr << "[TRTInferX][API] target_w/target_h mismatch with engine input size. "
                      << "engine=" << engine_w << "x" << engine_h
                      << " cfg=" << cfg_.target_w << "x" << cfg_.target_h << std::endl;
        }
        if (cfg_.prep != PreprocessMode::LETTERBOX && cfg_.prep != PreprocessMode::RESIZE)
        {
            std::cerr << "[TRTInferX][API] Unsupported preprocess mode in EngineConfig." << std::endl;
            return false;
        }
        return true;
    }
}
