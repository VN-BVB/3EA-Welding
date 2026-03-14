#ifndef CORNERBUTTSEAMSDET_H
#define CORNERBUTTSEAMSDET_H

#include "seamDetWithPointCloud/AbstractSeamDet.h"

class CornerButtSeamsDet : public AbstractSeamDet {
public:
    explicit CornerButtSeamsDet(QObject *parent = nullptr);

    std::vector<std::shared_ptr<WeldSeamInfo>> solveSeamsEndPoints(std::vector<std::shared_ptr<WeldSeamInfo>> seamsInfo) override;  // 求解焊缝

    // Ransac拟合平面，，并输出平面的内点集合
    void Ransac_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud);
    // 体素滤波
    void Statistic_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
    // 点云投影至指定平面
    void Project_ToPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud);
    // 筛选焊缝候选点集
    void Identify_boundary(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
    // 求点云聚类中各点之间的最大距离
    double getMaxDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
    // 提取焊缝端点，方法为拟合直线，求取直线交点
    bool Sovle_SeamEndpoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundaries);
    // 对每条ransac的直线，进行1.排序 3.投影 4.计算端点
    std::vector<pcl::PointXYZ> Solve_LineEndoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line, pcl::ModelCoefficients::Ptr coefficients);
    // 仅用于实验观测。Ransac提取多条直线参数，并提取端点
    void Ransac_MultipleLines(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundaries);
    // 单条焊缝检测完成后，变量重新初始化
    void SingleSeam_Reinitialize();
    // 运行焊缝识别的整个流程
    void Seam_extraction_run(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &BoxResults_Pointclouds);

signals:

public slots:

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;                  // ROI输入点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_interior;   // 焊缝所在平面内点
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_projected;  // 将点云投影到平面
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary;         // AlphaShape识别的边界
    // pcl::ModelCoefficients::Ptr plane_coeffs;                   // 焊缝所在平面系数

    std::vector<double> Six_BoundaryLines_Length;                                     // 各条边界线的长度
    std::vector<pcl::PointCloud<pcl::PointXYZ>> Six_BoundaryLinesInliers;             // 提取的6条边界线的内点集合
    std::vector<pcl::ModelCoefficients::Ptr> Six_BoundaryLinesCoff;                   // 提取的6条边界线的参数
    std::vector<pcl::PointCloud<pcl::PointXYZ>> SortedSix_BoundaryLinesInliers;       // 按长度排序后的6条边界线的内点集合
    std::vector<pcl::ModelCoefficients::Ptr> SortedSix_BoundaryLinesCoff;             // 按长度排序后的6条边界线的参数
    std::vector<pcl::PointCloud<pcl::PointXYZ>> MaxFourLength_BoundaryLines_inliers;  // 长度最大的4条边界线的内点集合
    std::vector<std::vector<pcl::PointXYZ>> ALL_BoundaryLines_endpoints;              // 各边界线端点，仅用于实验观测
    std::vector<pcl::ModelCoefficients::Ptr> MaxFourLength_BoundaryLinesCoff;         // 长度最大的4条边界线的参数
    std::vector<pcl::PointXYZ> CurrentSeam_endpoints;                                 // 当前焊缝的端点

    const double Max_Cluster_radius = 8;                // 欧式聚类提取最大点集半径
    const int Ransac_Plane_Iterations = 500;            // Ransac拟合平面的迭代数
    const double Ransac_plane_Dth = 2;                  // ransac拟合平面的距离阈值
    const int Statistic_NeighPoints = 20;               // 统计滤波近邻点数
    const double Statistic_sigma = 3;                   // 统计滤波系数
    const double Alpha_Radius = 2;                      // AlphaShape识别边界的半径参数
    const double Ransac_BoundaryLine_Dth = 1;           // Ransac拟合边界直线的距离阈值
    const double EucSeg_Dth = 10;                       // 每一Ransac内点，使用欧式聚类提取最大点集
    const double Angle_ParalleThreshold = 10;           // 平行线角度判断阈值
    const double Angle_Vertical_1st2st_Threshold = 85;  // 最长直线和次长直线的垂直夹角阈值
    const double Min_Dth_IntersectionToEndpoint = 1;  // 交点到边界端点的距离最小阈值，用于判断焊缝区域点云是否拍摄完整，以决定是否输出结果
    const double Max_Dth_IntersectionToEndpoint = 20;  // 交点到边界端点的距离最大阈值，用于判断焊缝区域点云是否拍摄完整，以决定是否输出结果
    const bool Bool_IdentifyCurrentSeam = false;  // 当前焊缝是否完成识别

    // bool saveAndOutputDebugInformation = true;  // 是否保存用于调试的中间过程点云，以及输出用于调试的信息
    bool saveAndOutputDebugInformation = false;  // 是否保存用于调试的中间过程点云，以及输出用于调试的信息

    std::vector<pcl::PointXYZ> cornerButtSeams;           // 各条对接焊缝的端点
    pcl::ModelCoefficients::Ptr planeCoeffsWithWeldSeam;  // 焊缝所在平面系数
    pcl::ModelCoefficients::Ptr seamsLineToVal;  // 通过外侧边缘线求角平分线得到的焊缝近似直线，用于验证焊缝检测正确性
};

#endif  // CORNERBUTTSEAMSDET_H
