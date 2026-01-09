#include "./include/main.h"
#include <iomanip>
#include <sstream>
#include <chrono>
#include <limits>

int main(int argc, char **argv)
{
    std::string engine_path = "best.engine";
    std::string image_path = "test/images/coco128/images/000000000036.jpg";
    std::string output_path = "output.jpg";
    std::string video_path;
    std::string video_out_path;
    bool no_display = false;
    bool use_camera = false;
    int camera_id = 0;
    int video_batch = 1;
    int num_classes = 1;
    int max_batch = 4;
    int streams = 2;
    bool auto_streams = false;
    int min_streams = 1;
    float conf_thres = 0.08f;
    float nms_score = 0.08f;
    float nms_iou = 0.45f;
    bool raw_sigmoid = false;  // 默认假定 ONNX 已 sigmoid
    bool raw_xyxy = false;
    bool debug_log = false;

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--engine" && i + 1 < argc)
            engine_path = argv[++i];
        else if (arg == "--image" && i + 1 < argc)
            image_path = argv[++i];
        else if (arg == "--camera")
            use_camera = true;
        else if (arg == "--camera-id" && i + 1 < argc)
            camera_id = std::stoi(argv[++i]);
        else if (arg == "--video" && i + 1 < argc)
            video_path = argv[++i];
        else if (arg == "--video-out" && i + 1 < argc)
            video_out_path = argv[++i];
        else if (arg == "--video-batch" && i + 1 < argc)
            video_batch = std::max(1, std::stoi(argv[++i]));
        else if (arg == "--classes" && i + 1 < argc)
            num_classes = std::stoi(argv[++i]);
        else if (arg == "--batch" && i + 1 < argc)
            max_batch = std::stoi(argv[++i]);
        else if (arg == "--streams" && i + 1 < argc)
            streams = std::stoi(argv[++i]);
        else if (arg == "--auto-streams")
            auto_streams = true;
        else if (arg == "--min-streams" && i + 1 < argc)
            min_streams = std::stoi(argv[++i]);
        else if (arg == "--conf" && i + 1 < argc)
            conf_thres = std::stof(argv[++i]);
        else if (arg == "--nms-score" && i + 1 < argc)
            nms_score = std::stof(argv[++i]);
        else if (arg == "--nms-iou" && i + 1 < argc)
            nms_iou = std::stof(argv[++i]);
        else if (arg == "--raw-sigmoid")
            raw_sigmoid = true;
        else if (arg == "--raw-xyxy")
            raw_xyxy = true;
        else if (arg == "--debug")
            debug_log = true;
        else if (arg == "--output" && i + 1 < argc)
            output_path = argv[++i];
        else if (arg == "--no-display")
            no_display = true;
    }

    if (!video_path.empty() && video_batch > max_batch)
    {
        std::cout << "[INFO] Adjusting --batch to match --video-batch (" << video_batch << ")." << std::endl;
        max_batch = video_batch;
    }

    TRTInferV1::TRTInfer myInfer(0);
    myInfer.setNmsThresholds(nms_score, nms_iou);
    myInfer.setRawDecode(raw_sigmoid, raw_xyxy);
    myInfer.setDebug(debug_log);
    if (!myInfer.initModule(engine_path, max_batch, num_classes, streams))
    {
        std::cerr << "Failed to initialize TRTInferX." << std::endl;
        return 1;
    }
    if (auto_streams)
        myInfer.enableAutoStreams(true, min_streams);

    if (!video_path.empty())
    {
        cv::VideoCapture cap(video_path);
        if (!cap.isOpened())
        {
            std::cerr << "Failed to open video." << std::endl;
            return 1;
        }

        const std::string win_name = "YOLO11 Video";
        bool window_ready = false;
        if (!no_display)
            cv::namedWindow(win_name, cv::WINDOW_NORMAL);

        cv::VideoWriter writer;
        bool writer_ready = false;
        double fps_src = cap.get(cv::CAP_PROP_FPS);
        if (fps_src <= 0.0)
            fps_src = 30.0;

        int frame_count = 0;
        double fps = 0.0;
        double fps_min = std::numeric_limits<double>::max();
        double fps_max = 0.0;
        double infer_min_ms = std::numeric_limits<double>::max();
        double infer_max_ms = 0.0;
        double gpu_min_ms = std::numeric_limits<double>::max();
        double gpu_max_ms = 0.0;
        auto t0 = std::chrono::steady_clock::now();

        if (video_batch != 1)
            std::cout << "[INFO] Video mode uses batch=" << video_batch
                      << " (ignoring --batch " << max_batch << ")." << std::endl;
        else if (max_batch > 1)
            std::cout << "[INFO] Video mode uses batch=1 (ignoring --batch " << max_batch << ")." << std::endl;

        while (true)
        {
            cv::Mat frame;
            if (!cap.read(frame) || frame.empty())
                break;

            if (!video_out_path.empty() && !writer_ready)
            {
                int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
                writer.open(video_out_path, fourcc, fps_src, frame.size(), true);
                if (!writer.isOpened())
                {
                    std::cerr << "Failed to open video writer." << std::endl;
                    return 1;
                }
                writer_ready = true;
            }

            if (!no_display && !window_ready)
            {
                const int max_w = 1280;
                const int max_h = 720;
                double scale_w = max_w / static_cast<double>(frame.cols);
                double scale_h = max_h / static_cast<double>(frame.rows);
                double scale = std::min(1.0, std::min(scale_w, scale_h));
                int win_w = static_cast<int>(frame.cols * scale);
                int win_h = static_cast<int>(frame.rows * scale);
                cv::resizeWindow(win_name, win_w, win_h);
                window_ready = true;
            }

            std::vector<cv::Mat> frames;
            frames.reserve(video_batch);
            frames.emplace_back(frame);
            while (static_cast<int>(frames.size()) < video_batch)
            {
                cv::Mat next;
                if (!cap.read(next) || next.empty())
                    break;
                frames.emplace_back(next);
            }
            if (static_cast<int>(frames.size()) < video_batch)
                break;

            auto infer_start = std::chrono::steady_clock::now();
            auto results = myInfer.doInference(frames, conf_thres);
            auto infer_end = std::chrono::steady_clock::now();
            if (results.empty())
                continue;

            double infer_ms = std::chrono::duration<double, std::milli>(infer_end - infer_start).count();
            infer_min_ms = std::min(infer_min_ms, infer_ms);
            infer_max_ms = std::max(infer_max_ms, infer_ms);
            double per_frame_ms = infer_ms / static_cast<double>(frames.size());
            double gpu_ms = myInfer.getLastGpuInferMs();
            double gpu_per_frame_ms = gpu_ms / static_cast<double>(frames.size());
            gpu_min_ms = std::min(gpu_min_ms, gpu_per_frame_ms);
            gpu_max_ms = std::max(gpu_max_ms, gpu_per_frame_ms);

            for (size_t bi = 0; bi < frames.size(); ++bi)
            {
                int dets = static_cast<int>(results[bi].size());
                for (const auto &det : results[bi])
                {
                    cv::rectangle(frames[bi], det.rect, cv::Scalar(0, 255, 0), 2);
                    std::ostringstream oss;
                    oss << det.cls << ":" << std::fixed << std::setprecision(2) << det.conf;
                    cv::putText(frames[bi], oss.str(), det.rect.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 0}, 2);
                }

                frame_count++;
                auto t1 = std::chrono::steady_clock::now();
                std::chrono::duration<double> elapsed = t1 - t0;
                if (elapsed.count() >= 1.0)
                {
                    fps = frame_count / elapsed.count();
                    fps_min = std::min(fps_min, fps);
                    fps_max = std::max(fps_max, fps);
                    frame_count = 0;
                    t0 = t1;
                }

                std::ostringstream info;
                info << "FPS: " << std::fixed << std::setprecision(1) << fps
                     << "  Infer: " << std::setprecision(2) << per_frame_ms << " ms"
                     << "  GPU: " << std::setprecision(2) << gpu_per_frame_ms << " ms"
                     << "  Dets: " << dets;
                cv::putText(frames[bi], info.str(), {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1.2, {0, 0, 255}, 3);

                if (writer_ready)
                    writer.write(frames[bi]);

                if (!no_display)
                {
                    cv::imshow(win_name, frames[bi]);
                    if (cv::waitKey(1) == 'q')
                        return 0;
                }
                else if (frame_count == 0)
                {
                    std::cout << "[INFO] " << info.str() << std::endl;
                }
            }
        }
        if (fps_min < std::numeric_limits<double>::max())
        {
            std::cout << "[INFO] Video summary: FPS(min/max)="
                      << std::fixed << std::setprecision(1) << fps_min << "/" << fps_max
                      << " infer(ms)(min/max)=" << std::setprecision(2)
                      << infer_min_ms << "/" << infer_max_ms
                      << " gpu(ms)(min/max)=" << std::setprecision(2)
                      << gpu_min_ms << "/" << gpu_max_ms << std::endl;
        }
        return 0;
    }

    if (use_camera)
    {
        cv::VideoCapture cap(camera_id);
        if (!cap.isOpened())
        {
            std::cerr << "Failed to open camera." << std::endl;
            return 1;
        }

        const std::string win_name = "YOLO11 Camera";
        bool window_ready = false;
        if (!no_display)
            cv::namedWindow(win_name, cv::WINDOW_NORMAL);

        int frame_count = 0;
        double fps = 0.0;
        double fps_min = std::numeric_limits<double>::max();
        double fps_max = 0.0;
        double infer_min_ms = std::numeric_limits<double>::max();
        double infer_max_ms = 0.0;
        double gpu_min_ms = std::numeric_limits<double>::max();
        double gpu_max_ms = 0.0;
        auto t0 = std::chrono::steady_clock::now();

        if (max_batch > 1)
            std::cout << "[INFO] Camera mode uses batch=1 (ignoring --batch " << max_batch << ")." << std::endl;

        while (true)
        {
            cv::Mat frame;
            if (!cap.read(frame) || frame.empty())
                break;

            if (!no_display && !window_ready)
            {
                const int max_w = 1280;
                const int max_h = 720;
                double scale_w = max_w / static_cast<double>(frame.cols);
                double scale_h = max_h / static_cast<double>(frame.rows);
                double scale = std::min(1.0, std::min(scale_w, scale_h));
                int win_w = static_cast<int>(frame.cols * scale);
                int win_h = static_cast<int>(frame.rows * scale);
                cv::resizeWindow(win_name, win_w, win_h);
                window_ready = true;
            }

            std::vector<cv::Mat> frames;
            frames.emplace_back(frame);

            auto infer_start = std::chrono::steady_clock::now();
            auto results = myInfer.doInference(frames, conf_thres);
            auto infer_end = std::chrono::steady_clock::now();
            if (results.empty())
                continue;

            int dets = static_cast<int>(results[0].size());
            for (const auto &det : results[0])
            {
                cv::rectangle(frame, det.rect, cv::Scalar(0, 255, 0), 2);
                std::ostringstream oss;
                oss << det.cls << ":" << std::fixed << std::setprecision(2) << det.conf;
                cv::putText(frame, oss.str(), det.rect.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 0}, 2);
            }

            frame_count++;
            auto t1 = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = t1 - t0;
            if (elapsed.count() >= 1.0)
            {
                fps = frame_count / elapsed.count();
                fps_min = std::min(fps_min, fps);
                fps_max = std::max(fps_max, fps);
                frame_count = 0;
                t0 = t1;
            }

            double infer_ms = std::chrono::duration<double, std::milli>(infer_end - infer_start).count();
            infer_min_ms = std::min(infer_min_ms, infer_ms);
            infer_max_ms = std::max(infer_max_ms, infer_ms);
            double gpu_ms = myInfer.getLastGpuInferMs();
            gpu_min_ms = std::min(gpu_min_ms, gpu_ms);
            gpu_max_ms = std::max(gpu_max_ms, gpu_ms);
            std::ostringstream info;
            info << "FPS: " << std::fixed << std::setprecision(1) << fps
                 << "  Infer: " << std::setprecision(2) << infer_ms << " ms"
                 << "  GPU: " << std::setprecision(2) << gpu_ms << " ms"
                 << "  Dets: " << dets;
            cv::putText(frame, info.str(), {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1.2, {0, 0, 255}, 3);

            if (!no_display)
            {
                cv::imshow(win_name, frame);
                if (cv::waitKey(1) == 'q')
                    break;
            }
            else if (frame_count == 0)
            {
                std::cout << "[INFO] " << info.str() << std::endl;
            }
        }
        if (fps_min < std::numeric_limits<double>::max())
        {
            std::cout << "[INFO] Camera summary: FPS(min/max)="
                      << std::fixed << std::setprecision(1) << fps_min << "/" << fps_max
                      << " infer(ms)(min/max)=" << std::setprecision(2)
                      << infer_min_ms << "/" << infer_max_ms
                      << " gpu(ms)(min/max)=" << std::setprecision(2)
                      << gpu_min_ms << "/" << gpu_max_ms << std::endl;
        }
        return 0;
    }

    cv::Mat img = cv::imread(image_path);
    if (img.empty())
    {
        std::cerr << "Failed to load image." << std::endl;
        return 1;
    }

    std::vector<cv::Mat> frames;
    frames.emplace_back(img);
    if (max_batch > 1)
        frames.resize(max_batch, img);

    auto results = myInfer.doInference(frames, conf_thres);
    if (results.empty())
    {
        std::cerr << "Inference failed or returned empty results." << std::endl;
        return 1;
    }
    for (const auto &det : results[0])
    {
        cv::rectangle(img, det.rect, cv::Scalar(0, 255, 0), 2);
        std::ostringstream oss;
        oss << det.cls << ":" << std::fixed << std::setprecision(2) << det.conf;
        cv::putText(img, oss.str(), det.rect.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 0}, 2);
    }

    if (!output_path.empty())
        cv::imwrite(output_path, img);
    if (!no_display)
    {
        cv::namedWindow("YOLO11", cv::WINDOW_NORMAL);
        cv::imshow("YOLO11", img);
        cv::waitKey(0);
    }
    return 0;
}
