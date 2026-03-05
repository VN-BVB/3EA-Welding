#include "AbstractObjectDetect.h"

DetResult::DetResult(int id, float score, float x1, float y1, float x2, float y2, cv::Mat res)
    : classId(id), score(score), topLeftX(x1), topLeftY(y1), bottomRightX(x2), bottomRightY(y2), detRes(res) {}

AbstractObjectDetect::AbstractObjectDetect() {}

std::string AbstractObjectDetect::getEngine_path() const { return engine_path; }
void AbstractObjectDetect::setEngine_path(const std::string &newEngine_path) { engine_path = newEngine_path; }

std::vector<std::string> AbstractObjectDetect::getClassNames() const { return classNames; }
void AbstractObjectDetect::setClassNames(const std::vector<std::string> &newClassNames) { classNames = newClassNames; }

std::vector<std::vector<unsigned int> > AbstractObjectDetect::getColors() const { return colors; }
void AbstractObjectDetect::setColors(const std::vector<std::vector<unsigned int> > &newColors) { colors = newColors; }

float AbstractObjectDetect::getScore_thres() const { return score_thres; }
void AbstractObjectDetect::setScore_thres(float newScore_thres) { score_thres = newScore_thres; }

float AbstractObjectDetect::getIou_thres() const { return iou_thres; }
void AbstractObjectDetect::setIou_thres(float newIou_thres) { iou_thres = newIou_thres; }

cv::Size AbstractObjectDetect::getSize() const { return size; }
void AbstractObjectDetect::setSize(const cv::Size &newSize) { size = newSize; }

int AbstractObjectDetect::getLabelsNum() const { return labelsNum; }
void AbstractObjectDetect::setLabelsNum(int newLabelsNum) { labelsNum = newLabelsNum; }
