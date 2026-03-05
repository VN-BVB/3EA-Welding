#ifndef YOLO11SEGINFERENCE_H
#define YOLO11SEGINFERENCE_H

#include "deepLearning/segment/AbstractSegment.h"

class Yolo11Seg;

class Yolo11SegInference : public AbstractSegment {
public:
    Yolo11SegInference();

    void initialization() override;                                      // 初始化推理类
    void inference(cv::Mat &img, std::vector<SegResult> &res) override;  // 推理函数

private:
    Yolo11Seg *yolo11Seg = nullptr;

    cv::Mat res;
    int topk = 100;
};

#endif  // YOLO11SEGINFERENCE_H
