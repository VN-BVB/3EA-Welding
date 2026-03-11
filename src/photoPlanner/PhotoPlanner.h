#ifndef PHOTO_PLANNER_H
#define PHOTO_PLANNER_H

#include <plog/Log.h>

#include <opencv2/opencv.hpp>
#include <set>
#include <stdexcept>
#include <vector>

class PhotoPlanner {
public:
    PhotoPlanner(double photoWidth, double photoHeight);

    // 根据粗定位框规划拍照位置
    std::vector<cv::Point2d> plan(const std::vector<cv::Rect_<double>>& roughRegions);

    // 判断一个矩形框是否完全在另一个框的内部
    bool isFullyCovered(const cv::Rect_<double>& outer, const cv::Rect_<double>& inner) const;

private:
    double photoWidth_;
    double photoHeight_;

    // 获取已覆盖区域索引
    std::vector<size_t> getCoveredIndices(const cv::Rect_<double>& photoArea, const std::vector<cv::Rect_<double>>& roughRegions,
                                          const std::vector<size_t>& uncoveredIndices) const;

    // 根据所有粗定位框和已经覆盖的粗定位框索引, 确定下一个拍照中心点
    cv::Point2d getBestPhotoCenter(const std::vector<cv::Rect_<double>>& roughRegions, const std::vector<size_t>& uncoveredIndices) const;

    // 合并拍照位置结果, 将『能用一个拍照位置同时拍到的多个位置』合并为一个拍照位置
    std::vector<cv::Point2d> mergePhotoCenters(const std::vector<cv::Point2d>& centers, const std::vector<cv::Rect_<double>>& roughRegions) const;

    friend class RailWeldingSystem;
};

#endif  // PHOTO_PLANNER_H
