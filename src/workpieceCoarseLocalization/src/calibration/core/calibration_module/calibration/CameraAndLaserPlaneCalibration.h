#ifndef CAMERAANDLASERPLANECALIBRATION_H
#define CAMERAANDLASERPLANECALIBRATION_H
#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <filesystem>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
// #define findChessboardCorner

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#define BOARD_SCALE 60  // 棋盘格边长（mm）35
#define BOARD_HEIGHT 9  // 棋盘格高度方向角点个数
#define BOARD_WIDTH 6   // 棋盘格宽度方向角点个数
// #define BOARD_SCALE 60    // 棋盘格边长（mm）35
// #define BOARD_HEIGHT 5  // 棋盘格高度方向角点个数
// #define BOARD_WIDTH 8   // 棋盘格宽度方向角点个数
// #define BOARD_SCALE 35    // 棋盘格边长（mm）35
// #define BOARD_HEIGHT 6  // 棋盘格高度方向角点个数
// #define BOARD_WIDTH 8   // 棋盘格宽度方向角点个数
// #define BOARD_SCALE 15    // 棋盘格边长（mm）35
// #define BOARD_HEIGHT 9  // 棋盘格高度方向角点个数
// #define BOARD_WIDTH 11   // 棋盘格宽度方向角点个数//黑白圆形棋盘格

namespace CameraAndLaserPlaneCalibration {
// 错误度量结构
struct ErrorMetrics {
    double meanErrorX;
    double meanErrorY;
    double meanErrorZ;
    double meanTotalError;
};

// 图像处理：获取、校正和预处理
void GetImage(std::string imgSrc, cv::Mat& srcImage);                                               // 读取图像文件
void Correction(cv::Mat& srcImage, cv::Mat& dstImage, cv::Mat& cameraMatrix, cv::Mat& distCoeffs);  // 图像畸变校正
void RemoveSmallRegion(cv::Mat& InputImage, cv::Mat& OutputImage, int pixel);                       // 去除小面积连通域

// 相机标定：相机参数计算
void cameraCalibration(std::vector<std::string>& files, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                       std::vector<cv::Mat>& tvecsMat, std::vector<cv::Mat>& rvecsMat, int& imageCount,
                       double& totalErr);  // 相机标定主函数
bool calculate_Image_Points(cv::Mat imageInput, cv::Size boardSize, std::vector<cv::Point2f>& imagePoints);  // 计算图像角点
bool calculate_Image_Points2(cv::Mat imageInput, cv::Size boardSize, std::vector<cv::Point2f>& imagePoints);  // 计算图像角点
void whenCalculateImagePoints(int i, const cv::Mat& imageInput, cv::Size boardSize,
                              std::vector<cv::Point2f>& imagePointsBuf);  // 图像角点计算辅助函数
void calibrationSolveExtrinsics(cv::Mat& Kc, cv::Mat& distCoeffs, std::vector<cv::Point3f>& objPoints,
                                std::vector<std::vector<cv::Point2f>>& imagePoints, std::vector<cv::Mat>& vecHc);  // 外参矩阵求解
bool PointSortRule(const cv::Point2d pt1, const cv::Point2d pt2);  // 点坐标排序规则

// 坐标转换与分离
void Point2dSperate(std::vector<cv::Point2d>& P, std::vector<cv::Point2d>& pPlane,
                    std::vector<cv::Point2d>& pObject);  // 分离平面和物体点
void Point2dto3d(const std::vector<double> plane, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
                 const std::vector<cv::Point2d>& Pt2ds,
                 std::vector<cv::Point3d>& Pt3ds);  // 2D转3D坐标
std::vector<cv::Point3d> transformCameraToBase(const std::vector<cv::Point3d>& cameraPoints,
                                               const cv::Mat& extrinsicMatrix);  // 相机坐标系转基坐标系
void savePointCloud(const std::vector<std::vector<cv::Point3d>>& objectCornerPoints,
                    const std::string& filename);  // 保存点云数据
// 平面拟合与评估
std::vector<double> planeLeastSquareFitting(std::vector<cv::Point3d>& pts);                       // 最小二乘法拟合平面
cv::Vec4d fitPlaneToPoints(const std::vector<cv::Point3d>& points);                               // 点集拟合平面
void PointtoPlaneEvaluation(const std::vector<cv::Point3d>& Pt3ds, std::vector<double> plane);    // 点到平面精度评估
std::vector<double> fitPlaneToMultipleEquations(const std::vector<std::vector<double>>& planes);  // 多平面方程拟合
void computePlaneEquations(std::vector<cv::Mat>& extrinsicMatrices, std::vector<cv::Point3f>& objectCornerPoints,
                           std::vector<double>& globalPlane);                                 // 计算全局平面方程
cv::Vec4f calculatePlaneSquareWithVecs(std::vector<double> plane, cv::Mat& extrinsicMatrix);  // 计算平面方程系数

// 外参矩阵计算
void calculateExtrinsicMatrices(int imageCount, std::vector<cv::Mat>& tvecsMat, std::vector<cv::Mat>& rvecsMat,
                                std::vector<cv::Mat>& extrinsicMatrices);  // 计算外参矩阵

// 平面标定与误差评估
void planeCalibration(std::vector<std::string>& files, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                      std::vector<double>& globalPlane,
                      CameraAndLaserPlaneCalibration::ErrorMetrics& errorMetrics);  // 平面标定主函数
ErrorMetrics evaluatePlaneFittingError(const std::vector<std::vector<cv::Point2f>>& imageCornerPoints,
                                       const std::vector<cv::Point3f>& objectCornerPoints,
                                       const std::vector<cv::Mat>& extrinsicMatrices, const cv::Mat& cameraMatrix,
                                       const cv::Mat& distCoeffs,
                                       const std::vector<double>& globalPlane);  // 平面拟合误差评估
}  // namespace CameraAndLaserPlaneCalibration

#endif  // CAMERAANDLASERPLANECALIBRATION_H
