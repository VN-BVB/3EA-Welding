#ifndef POSECALCULATION_H
#define POSECALCULATION_H

#include <vector>

#include <opencv2/core.hpp>

namespace PoseCalculation {

void splitTransform(cv::Mat& transform, cv::Mat& rotation, cv::Mat& translation);
cv::Mat composeTransform(cv::Mat& rotation, cv::Mat& translation);
void solveBoardPoses(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<cv::Point3f>& objectPoints,
                     std::vector<std::vector<cv::Point2f>>& imagePoints, std::vector<cv::Mat>& transforms);

}  // PoseCalculation 命名空间

#endif  // POSECALCULATION_H
