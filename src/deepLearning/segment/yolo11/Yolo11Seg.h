#ifndef YOLO11_SEG_H
#define YOLO11_SEG_H

#include <fstream>

#include "NvInferPlugin.h"
#include "common.hpp"

using namespace segYolo11;

class Yolo11Seg {
public:
    explicit Yolo11Seg(const std::string &engine_file_path);
    ~Yolo11Seg();

    void make_pipe(bool warmup = true);
    void copy_from_Mat(const cv::Mat &image);
    void copy_from_Mat(const cv::Mat &image, cv::Size &size);
    void letterbox(const cv::Mat &image, cv::Mat &out, cv::Size &size);
    void infer();
    void postprocess(std::vector<ObjectYolo11Seg> &objs, float score_thres = 0.25f, float iou_thres = 0.65f, int topk = 100, int seg_channels = 32,
                     int seg_h = 160, int seg_w = 160);
    void draw_objects(const cv::Mat &image, cv::Mat &res, const std::vector<ObjectYolo11Seg> &objs, const std::vector<std::string> &CLASS_NAMES,
                      const std::vector<std::vector<unsigned int>> &COLORS, const std::vector<std::vector<unsigned int>> &MASK_COLORS);
    int num_bindings;
    int num_inputs = 0;
    int num_outputs = 0;
    std::vector<BindingYolo11Seg> input_bindings;
    std::vector<BindingYolo11Seg> output_bindings;
    std::vector<void *> host_ptrs;
    std::vector<void *> device_ptrs;
    PreParamYolo11Seg pparam;

    cv::Mat maskOnly;  // 推理得到的掩膜

private:
    nvinfer1::ICudaEngine *engine = nullptr;
    nvinfer1::IRuntime *runtime = nullptr;
    nvinfer1::IExecutionContext *context = nullptr;
    cudaStream_t stream = nullptr;
    LoggerYOLO11Seg gLogger{nvinfer1::ILogger::Severity::kERROR};
};

#endif  // YOLO11_SEG_H
