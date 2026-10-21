#ifndef BOARDDETECTION_H
#define BOARDDETECTION_H

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace BoardDetection {

void createObjectPoints(int boardWidth, int boardHeight, double spacing, std::vector<cv::Point3f>& points);
bool findChessboardCornersInFolder(const std::string& path, cv::Size boardSize, std::vector<std::vector<cv::Point2f>>& imagePoints);
// 从文件夹读取圆点板图片；检测失败时保持原行为，将对应 .bmp 重命名为 *_unfind.bmp。
bool findCirclesInFolder(const std::string& path, cv::Size boardSize, std::vector<std::vector<cv::Point2f>>& imagePoints);
bool findCirclesInImages(std::vector<cv::Mat>& images, cv::Size boardSize, std::vector<std::vector<cv::Point2f>>& imagePoints);

}  // BoardDetection 命名空间

#endif  // BOARDDETECTION_H
