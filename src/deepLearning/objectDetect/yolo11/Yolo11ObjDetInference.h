#ifndef YOLO11OBJDECINFERENCE_H
#define YOLO11OBJDECINFERENCE_H

#include "deepLearning/objectDetect/AbstractObjectDetect.h"

class Yolo11ObjDet;

class Yolo11ObjDetInference : public AbstractObjectDetect {
public:
    Yolo11ObjDetInference();

    void initialization() override;                                      // 初始化推理类
    void inference(cv::Mat& img, std::vector<DetResult>& res) override;  // 推理函数

private:
    Yolo11ObjDet* yolo11ObjDet = nullptr;
    cv::Mat res;
    int topk = 100;
};

#endif  // YOLO11OBJDECINFERENCE_H
