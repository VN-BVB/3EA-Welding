#ifndef YOLO11OBJDEC_H
#define YOLO11OBJDEC_H

#include <math.h>

#include <fstream>

#include "NvInferPlugin.h"
#include "common.hpp"

using namespace det;

const std::vector<std::string> CLASS_NAMES = {"BackType1", "FrontType1", "FrontType3", "BackType2", "FrontType2"};

const std::vector<std::vector<unsigned int>> COLORS = {
    {0,   114, 189},
    {217, 83,  25 },
    {237, 177, 32 },
    {126, 47,  142},
    {119, 172, 48 }
};

class Yolo11ObjDet {
public:
    explicit Yolo11ObjDet(const std::string& engine_file_path);
    ~Yolo11ObjDet();

    void make_pipe(bool warmup = true);
    void copy_from_Mat(const cv::Mat& image);
    void copy_from_Mat(const cv::Mat& image, cv::Size& size);
    void letterbox(const cv::Mat& image, cv::Mat& out, cv::Size& size);
    void infer();
    void postprocess(std::vector<Object>& objs, float score_thres = 0.25f, float iou_thres = 0.65f, int topk = 100, int num_labels = 80);
    static void draw_objects(const cv::Mat& image, cv::Mat& res, const std::vector<Object>& objs, const std::vector<std::string>& CLASS_NAMES,
                             const std::vector<std::vector<unsigned int>>& COLORS);
    int num_bindings;
    int num_inputs = 0;
    int num_outputs = 0;
    std::vector<Binding> input_bindings;
    std::vector<Binding> output_bindings;
    std::vector<void*> host_ptrs;
    std::vector<void*> device_ptrs;

    PreParam pparam;

private:
    nvinfer1::ICudaEngine* engine = nullptr;
    nvinfer1::IRuntime* runtime = nullptr;
    nvinfer1::IExecutionContext* context = nullptr;
    cudaStream_t stream = nullptr;
    Logger gLogger{nvinfer1::ILogger::Severity::kERROR};
};

#endif  // YOLO11OBJDEC_H
