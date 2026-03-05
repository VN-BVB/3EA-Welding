#include "AbstractSegment.h"

SegResult::SegResult(int id, float score, float x1, float y1, float x2, float y2, cv::Mat res, cv::Mat mask)
    : classId(id), score(score), topLeftX(x1), topLeftY(y1), bottomRightX(x2), bottomRightY(y2), segRes(res), maskOnly(mask) {}

AbstractSegment::AbstractSegment() {}

std::string AbstractSegment::getEngine_path() const { return engine_path; }
void AbstractSegment::setEngine_path(const std::string &newEngine_path) { engine_path = newEngine_path; }

std::vector<std::string> AbstractSegment::getClassNames() const { return classNames; }
void AbstractSegment::setClassNames(const std::vector<std::string> &newClassNames) { classNames = newClassNames; }

std::vector<std::vector<unsigned int> > AbstractSegment::getColors() const { return colors; }
void AbstractSegment::setColors(const std::vector<std::vector<unsigned int> > &newColors) { colors = newColors; }

float AbstractSegment::getScore_thres() const { return score_thres; }
void AbstractSegment::setScore_thres(float newScore_thres) { score_thres = newScore_thres; }

float AbstractSegment::getIou_thres() const { return iou_thres; }
void AbstractSegment::setIou_thres(float newIou_thres) { iou_thres = newIou_thres; }

cv::Size AbstractSegment::getSize() const { return size; }
void AbstractSegment::setSize(const cv::Size &newSize) {
    size = newSize;
    seg_h = newSize.height / 4;
    seg_w = newSize.width / 4;
}

int AbstractSegment::getSeg_channels() const { return seg_channels; }
void AbstractSegment::setSeg_channels(int newSeg_channels) { seg_channels = newSeg_channels; }
