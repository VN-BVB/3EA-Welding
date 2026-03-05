#include "Yolo11SegInference.h"

#include "Yolo11Seg.h"

Yolo11SegInference::Yolo11SegInference() {}

// 初始化推理类
void Yolo11SegInference::initialization() {
    yolo11Seg = new Yolo11Seg(this->engine_path);
    cudaSetDevice(0);  // 使用显卡推理
    yolo11Seg->make_pipe(true);
}

// 推理函数
void Yolo11SegInference::inference(cv::Mat &img, std::vector<SegResult> &res) {
    res.clear();
    std::vector<ObjectYolo11Seg> objs;

    static int a = 0;

    if (yolo11Seg) {
        yolo11Seg->copy_from_Mat(img, this->size);
        yolo11Seg->infer();
        yolo11Seg->postprocess(objs, this->score_thres, this->iou_thres, this->topk, this->seg_channels, this->seg_h, this->seg_w);
        yolo11Seg->draw_objects(img, this->res, objs, this->classNames, this->colors, this->colors);  // 绘制检测结果
        for (auto &o : objs) {
            o.rect = o.rect & cv::Rect_<float>(0, 0, img.cols, img.rows);  // 限制在图像范围内
            SegResult segResult(o.label, o.prob, o.rect.x, o.rect.y, o.rect.x + o.rect.width, o.rect.y + o.rect.height, this->res,
                                o.boxMask);
            res.push_back(segResult);
        }
    }
}
