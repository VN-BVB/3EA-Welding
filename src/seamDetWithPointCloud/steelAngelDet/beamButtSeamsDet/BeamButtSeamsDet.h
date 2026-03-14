#ifndef BEAMBUTTSEAMSDET_H
#define BEAMBUTTSEAMSDET_H

#include "seamDetWithPointCloud/AbstractSeamDet.h"
#include "utils/common/WeldSeamInfo.h"

class BeamButtSeamsDet : public AbstractSeamDet {
public:
    explicit BeamButtSeamsDet(QObject *parent = nullptr);

    // 求解焊缝
    std::vector<std::shared_ptr<WeldSeamInfo>> solveSeamsEndPoints(std::vector<std::shared_ptr<WeldSeamInfo>> seamsInfo) override;

    // 提取横梁焊缝端点
    bool Solve_BeamSeam_Endpoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudBoundaries);
    // Ransac拟合平面，并输出平面的内点集合
    void Ransac_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud);
    // 欧式聚类聚类，提取最大点集
    void maxEuclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, double radius);
    // 欧式聚类聚类，提取最大和次大点集的总和
    void maxAndSecondMaxEuclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, double radius);
    // 保留内点索引数量靠前的索引
    void reserveIndexAtFrontOfInlierNum(std::vector<int> &indexOfLines, int reserveNum);
    // 保留直线长度靠前的索引
    void reserveIndexAtFrontOfLength(std::vector<int> &indexOfLines, int reserveNum);
    // 对每条ransac的直线，进行 1.排序 3.投影 4.计算端点
    std::vector<pcl::PointXYZ> Solve_LineEndoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line, pcl::ModelCoefficients::Ptr coefficients);
    // AlphaShape宽窄边界对比筛选。1.宽窄边界对比筛选 2.输出粗边界  3.输出粗边界点云质心
    void Solve_CloudFineAwayRough(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
    // 单条焊缝检测完成后，变量重新初始化
    void SingleSeam_Reinitialize();
    // 体素滤波
    void Statistic_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
    // 点云投影至指定平面
    void Project_ToPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud);
    // 欧式聚类聚类，提取最大点集
    void Max_EuclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, double radius);

signals:

public slots:

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInObjectDetectBox;    // ROI输入点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOnPlaneWithWeldSeam;  // 焊缝所在平面内点
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudProjected2Plane;      // 将点云投影到平面
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOnBigBoundary;        // AlphaShape识别的主平面边界
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOnHugeBoundary;       // AlphaShape识别的主平面巨大半径边界
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOnSeamBoundary;       // AlphaShape识别的焊缝处边界点云
    Eigen::Vector4f Centroid_FineAwayRough;                        // AlphaShape宽窄质心

    std::vector<double> boundaryLines_Length;                                         // 各条边界线的长度
    std::vector<pcl::PointCloud<pcl::PointXYZ>> boundaryLinesInliers;                 // 提取的6条边界线的内点集合
    std::vector<pcl::ModelCoefficients::Ptr> boundaryLinesCoff;                       // 提取的6条边界线的参数
    std::vector<pcl::PointCloud<pcl::PointXYZ>> SortedVector_BoundaryLinesInliers;    // 按长度排序后的6条边界线的内点集合
    std::vector<pcl::ModelCoefficients::Ptr> SortedVector_BoundaryLinesCoff;          // 按长度排序后的6条边界线的参数
    std::vector<pcl::PointCloud<pcl::PointXYZ>> MaxFourLength_BoundaryLines_inliers;  // 长度最大的4条边界线的内点集合
    std::vector<std::vector<pcl::PointXYZ>> ALL_BoundaryLines_endpoints;              // 各边界线端点，仅用于实验观测
    std::vector<pcl::ModelCoefficients::Ptr> MaxFourLength_BoundaryLinesCoff;         // 长度最大的4条边界线的参数
    std::vector<pcl::PointXYZ> CurrentSeam_endpoints;                                 // 焊缝的端点

    double Max_Cluster_radius = 8;            // 欧式聚类提取最大点集半径
    int Ransac_Plane_Iterations = 500;        // Ransac拟合平面的迭代数
    double Ransac_plane_Dth = 2;              // ransac拟合平面的距离阈值  2
    int Statistic_NeighPoints = 20;           // 统计滤波近邻点数   20
    double Statistic_sigma = 3;               // 统计滤波系数	 3
    double alphaBigRidus = 5;                 // AlphaShape识别边界的半径参数 2
    double LineFittingDistanceThreshold = 1;  // Ransac拟合边界直线的距离阈值  1
    double EucSeg_Dth = 10;                   // 每一Ransac内点，使用欧式聚类提取最大点集
    double paralleThreshold = 10;             // 平行线角度判断阈值
    double verticalThreshold = 80;            // 垂直线角度判断阈值
    double Min_Dth_IntersectionToEndpoint = 2;  // 交点到边界端点的距离最小阈值，用于判断焊缝区域点云是否拍摄完整，以决定是否输出结果
    double Max_Dth_IntersectionToEndpoint = 20;  // 交点到边界端点的距离最大阈值，用于判断焊缝区域点云是否拍摄完整，以决定是否输出结果

    // bool saveAndOutputDebugInformation = true;  // 是否保存用于调试的中间过程点云，以及输出用于调试的信息
    bool saveAndOutputDebugInformation = false;  // 是否保存用于调试的中间过程点云，以及输出用于调试的信息

    int seamSide = -1;                                       // 焊缝正反
    std::vector<pcl::PointXYZ> beamButtSeams;                // 各条横梁对接焊缝的端点
    pcl::ModelCoefficients::Ptr planeCoeffsWithWeldSeam;     // 焊缝所在平面系数
    pcl::ModelCoefficients::Ptr lineCoeffsWithWeldSeam2Val;  // 焊缝所在直线系数 (用于验证)
};

#endif  // BEAMBUTTSEAMSDET_H
