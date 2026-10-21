#ifndef CAMERAPLANEALGORITHMS_H
#define CAMERAPLANEALGORITHMS_H

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace CameraPlaneAlgorithms {
// 错误度量结构
struct ErrorMetrics {
    double meanErrorX;
    double meanErrorY;
    double meanErrorZ;
    double meanTotalError;
};

// 图像处理：获取、校正和预处理
void undistortImage(cv::Mat& srcImage, cv::Mat& dstImage, cv::Mat& cameraMatrix, cv::Mat& distCoeffs);  // 图像畸变校正
void removeSmallRegions(cv::Mat& inputImage, cv::Mat& outputImage, int pixel);                          // 去除小面积连通域

// 相机标定：相机参数计算
void calibrateCamera(std::vector<std::string>& files, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<cv::Mat>& tvecsMat,
                     std::vector<cv::Mat>& rvecsMat, int& imageCount,
                     double& totalErr);  // 相机标定主函数
// 小圆点检测：保留原有灰度反色和面积 300～3000 像素的检测参数。
bool detectCircleGridPoints(cv::Mat imageInput, cv::Size boardSize, std::vector<cv::Point2f>& imagePoints);
// 大圆点检测：保留原有面积参数、亚像素精化及行列顺序整理，不与上一个版本合并。
bool detectRefinedCircleGridPoints(cv::Mat imageInput, cv::Size boardSize, std::vector<cv::Point2f>& imagePoints);
void detectAndAnnotateBoardPoints(int i, const cv::Mat& imageInput, cv::Size boardSize,
                                  std::vector<cv::Point2f>& imagePointsBuf);  // 图像角点计算辅助函数
void solveBoardExtrinsics(cv::Mat& Kc, cv::Mat& distCoeffs, std::vector<cv::Point3f>& objPoints, std::vector<std::vector<cv::Point2f>>& imagePoints,
                          std::vector<cv::Mat>& vecHc);                // 外参矩阵求解
bool pointLessByXThenY(const cv::Point2d pt1, const cv::Point2d pt2);  // 点坐标排序规则

// 坐标转换与分离
void separatePlaneAndObjectPoints(std::vector<cv::Point2d>& P, std::vector<cv::Point2d>& pPlane,
                                  std::vector<cv::Point2d>& pObject);  // 分离平面和物体点
void pixelsToPointsOnPlane(const std::vector<double> plane, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
                           const std::vector<cv::Point2d>& pixelPoints,
                           std::vector<cv::Point3d>& cameraPoints);  // 2D转3D坐标
std::vector<cv::Point3d> transformCameraToBase(const std::vector<cv::Point3d>& cameraPoints,
                                               const cv::Mat& extrinsicMatrix);  // 相机坐标系转基坐标系
void savePointCloud(const std::vector<std::vector<cv::Point3d>>& objectCornerPoints,
                    const std::string& filename);  // 保存点云数据
// 平面拟合与评估
std::vector<double> planeLeastSquareFitting(std::vector<cv::Point3d>& pts);                                   // 最小二乘法拟合平面
cv::Vec4d fitPlaneToPoints(const std::vector<cv::Point3d>& points);                                           // 点集拟合平面
void evaluatePointToPlaneDistances(const std::vector<cv::Point3d>& cameraPoints, std::vector<double> plane);  // 点到平面精度评估
std::vector<double> fitPlaneToMultipleEquations(const std::vector<std::vector<double>>& planes);              // 多平面方程拟合
void computePlaneEquations(std::vector<cv::Mat>& extrinsicMatrices, std::vector<cv::Point3f>& objectCornerPoints,
                           std::vector<double>& globalPlane);                                 // 计算全局平面方程
cv::Vec4f calculatePlaneSquareWithVecs(std::vector<double> plane, cv::Mat& extrinsicMatrix);  // 计算平面方程系数

// 外参矩阵计算
void calculateExtrinsicMatrices(int imageCount, std::vector<cv::Mat>& tvecsMat, std::vector<cv::Mat>& rvecsMat,
                                std::vector<cv::Mat>& extrinsicMatrices);  // 计算外参矩阵

// 平面标定与误差评估
void calibratePlane(std::vector<std::string>& files, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<double>& globalPlane,
                    CameraPlaneAlgorithms::ErrorMetrics& errorMetrics);  // 平面标定主函数
ErrorMetrics evaluatePlaneFittingError(const std::vector<std::vector<cv::Point2f>>& imageCornerPoints,
                                       const std::vector<cv::Point3f>& objectCornerPoints, const std::vector<cv::Mat>& extrinsicMatrices,
                                       const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
                                       const std::vector<double>& globalPlane);  // 平面拟合误差评估
}  // CameraPlaneAlgorithms 命名空间

#endif  // CAMERAPLANEALGORITHMS_H
