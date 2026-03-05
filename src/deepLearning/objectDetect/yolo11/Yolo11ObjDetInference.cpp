#include "Yolo11ObjDetInference.h"

#include "yolo11objdet.h"

Yolo11ObjDetInference::Yolo11ObjDetInference() {}

// 初始化推理类
void Yolo11ObjDetInference::initialization() {
    yolo11ObjDet = new Yolo11ObjDet(this->engine_path);
    cudaSetDevice(0);  // 使用显卡推理
    yolo11ObjDet->make_pipe(true);
}

// 推理函数
void Yolo11ObjDetInference::inference(cv::Mat &img, std::vector<DetResult> &res) {
    res.clear();
    std::vector<Object> objs;

    if (yolo11ObjDet) {
        yolo11ObjDet->copy_from_Mat(img, this->size);                                                      // 传入图像
        yolo11ObjDet->infer();                                                                             // 推理
        yolo11ObjDet->postprocess(objs, this->score_thres, this->iou_thres, this->topk, this->labelsNum);  // 后处理
        yolo11ObjDet->draw_objects(img, this->res, objs, this->classNames, this->colors);                  // 绘制检测结果
        for (auto &o : objs) {
            o.rect = o.rect & cv::Rect_<float>(0, 0, img.cols, img.rows);
            DetResult detResult(o.label, o.prob, o.rect.x, o.rect.y, o.rect.x + o.rect.width, o.rect.y + o.rect.height, this->res);
            res.push_back(detResult);
        }
    }
}
