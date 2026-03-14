#ifndef POINTCLOUDFUNC_H
#define POINTCLOUDFUNC_H

#include <omp.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <plog/Log.h>
#include <stdlib.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <cmath>
#include <eigen/SVD>
#include <iostream>
#include <iterator>
#include <limits>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>

namespace MyToolFunc {

// ###################################### 点 ######################################
// 对点做矩阵变换
pcl::PointXYZ transformSinglePoint(const pcl::PointXYZ& point, const Eigen::Matrix4f& transform);
pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                        const Eigen::Matrix4f& transform);
void scalePointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, double scaleX, double transX, double scaleY,
                      double transY);  // 点云放缩与平移

// ###################################### 直线 ######################################
// 计算两直线夹角
double getLineAngle(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);

// ###################################### 点与直线 ######################################
// 点到直线距离
double getPoint2LineDis(Eigen::Vector4f Point, pcl::ModelCoefficients::Ptr line_coff);
double getPoint2LineDis(pcl::PointXYZ Point, pcl::ModelCoefficients::Ptr line_coff);
// 点投影到直线
Eigen::Vector4f projPoint2Line(Eigen::Vector4f& point, pcl::ModelCoefficients::Ptr& line_coff);
pcl::PointXYZ projPoint2Line(pcl::PointXYZ& p, pcl::ModelCoefficients::Ptr& line_coff);
Eigen::Vector4f projPoint2Line(Eigen::Vector4f& point, Eigen::Vector4f& line_pt, Eigen::Vector4f& line_dir);
// 计算直线内点端点
void lineCloudEndPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::VectorXf& line_coff_vector,
                        Eigen::Vector4f specified_start_point, std::vector<Eigen::Vector4f>& two_endpoints);
std::vector<pcl::PointXYZ> lineCloudEndPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr lineCloud,
                                              pcl::ModelCoefficients::Ptr coefficients);
// 计算点到直线垂线
void Solve_ProjectVerticalLine(Eigen::Vector4f& point, pcl::ModelCoefficients::Ptr& line_coff,
                               Eigen::Vector4f& VerticalLine_vector);

// ###################################### 点与平面 ######################################
// 点投影到平面
void projPoint2Plane(const pcl::PointXYZ& point, const pcl::ModelCoefficients& coefficients, pcl::PointXYZ& projection);

// ###################################### 点云算法 ######################################
// 直通滤波
void passthroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered,
                       double min, double max);
// 统计滤波
void statisticalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered, int nr_k,
                       float std_mul);
// 计算向量的标准差, v为输入向量, avg为均值
float calcSigma(std::vector<float>& v, float& avg);
// 计算高斯聚类最大点集
void myFastMaxCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double cluster_tolerance_);
// 计算高曲率点
void highCurvaturePointsDetect(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_detect, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               double K_Radius, double sm_ratio,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr high_curvature_scatter_points);

void pointcloudUniformDownsampling(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float leafSize,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudResult);
}  // namespace MyToolFunc

#endif  // POINTCLOUDFUNC_H
