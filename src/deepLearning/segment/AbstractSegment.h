#ifndef ABSTRACTSEGMENT_H
#define ABSTRACTSEGMENT_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class SegResult {
public:
    SegResult() = default;
    SegResult(int id, float score, float x1, float y1, float x2, float y2, cv::Mat res, cv::Mat mask);

    int classId;
    float score;
    float topLeftX;
    float topLeftY;
    float bottomRightX;
    float bottomRightY;

    cv::Mat segRes;
    cv::Mat maskOnly;
};

class AbstractSegment {
public:
    AbstractSegment();

    virtual void initialization() = 0;                                      // 初始化推理类
    virtual void inference(cv::Mat &img, std::vector<SegResult> &res) = 0;  // 推理函数

    std::string getEngine_path() const;
    void setEngine_path(const std::string &newEngine_path);
    std::vector<std::string> getClassNames() const;
    void setClassNames(const std::vector<std::string> &newClassNames);
    std::vector<std::vector<unsigned int>> getColors() const;
    void setColors(const std::vector<std::vector<unsigned int>> &newColors);
    float getScore_thres() const;
    void setScore_thres(float newScore_thres);
    float getIou_thres() const;
    void setIou_thres(float newIou_thres);
    cv::Size getSize() const;
    void setSize(const cv::Size &newSize);
    int getLabelsNum() const;
    void setLabelsNum(int newLabelsNum);
    int getSeg_channels() const;
    void setSeg_channels(int newSeg_channels);

protected:
    std::string engine_path;
    std::vector<std::string> classNames = {"Class1", "Class2", "Class3", "Class4", "Class5", "Class6", "Class7", "Class8", "Class9"};
    std::vector<std::vector<unsigned int>> colors = {
        {0,   114, 189},
        {217, 83,  25 },
        {237, 177, 32 },
        {126, 47,  142},
        {119, 172, 48 },
        {255, 0,   127},
        {0,   255, 204},
        {255, 105, 0  },
        {127, 0,   255}
    };  // 目标框和掩膜颜色相同
    float score_thres = 0.25f;
    float iou_thres = 0.65f;
    cv::Size size = cv::Size{640, 640};
    int seg_h = 160;
    int seg_w = 160;
    int seg_channels = 32;

private:
};

#endif  // ABSTRACTSEGMENT_H
