#pragma once

#include <pcl/visualization/pcl_plotter.h>
#include <stdio.h>

#include <fstream>
#include <iostream>
#include <numeric>

#include "opencv2/calib3d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/core/utils/filesystem.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

// using namespace cv;
using namespace std;

extern std::vector<double> vecWorldX, vecWorldY, vecWorldZ;

extern void calc_stdev(vector<double> &vecWorld, double &stdev, double &mean);
// RT转R和T  从RT中把 R和T整出来
extern void RT2R_T(cv::Mat &RT, cv::Mat &R, cv::Mat &T);

// R和T转RT
extern cv::Mat R_T2RT(cv::Mat &R, cv::Mat &T);

// 根据标定板信息，输出坐标点
extern void calculate_Object_Points(int board_width, int board_heignt, double distance, vector<cv::Point3f> &objP);

// 根据读取的图片，计算各张图片的圆心集合
extern bool calculate_Image_Points_ChessboardCorners(const std::string &path, cv::Size boardSize,
                                                     std::vector<std::vector<cv::Point2f>> &imagePoints);
extern bool calculate_Image_Points(std::string &path, cv::Size boardSize, std::vector<std::vector<cv::Point2f>> &imagePoints);
extern bool calculate_Image_Points_V(std::vector<cv::Mat> &dirImages, cv::Size boardSize,
                                     std::vector<std::vector<cv::Point2f>> &imagePoints);
// 已知内参标定外参
extern void Calibration_Solve_Extrinsics(cv::Mat &Kc, cv::Mat &distCoeffs, vector<cv::Point3f> &objPoints,
                                         std::vector<std::vector<cv::Point2f>> &imagePoints, std::vector<cv::Mat> &vecHc);

// 绘制折线图
extern void draw_line_chart(vector<double> &X_data, vector<double> &Y_data, string string_title);

extern void draw_line_chart_visualization(pcl::visualization::PCLPlotter *plot_, vector<double> &X_data, vector<double> &Y_data,
                                          string string_title);
