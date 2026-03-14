#ifndef UPBEAMFILLETSEAMSDET_H
#define UPBEAMFILLETSEAMSDET_H

#include "seamDetWithPointCloud/AbstractSeamDet.h"

class UpBeamFilletSeamsDet : public AbstractSeamDet {
public:
    explicit UpBeamFilletSeamsDet(QObject *parent = nullptr);

    std::vector<std::shared_ptr<WeldSeamInfo>> solveSeamsEndPoints(std::vector<std::shared_ptr<WeldSeamInfo>> seamsInfo) override;  // 求解焊缝

    void Ransac_Multiple_planes(int plane_nums);  // Ransac分割拟合多个平面
    bool Solve_Fillet_Endpoints(int m);           // 提取焊缝
    void SingleSeam_Reinitialize();               // 单区域焊缝检测完成后，变量重新初始化

signals:

public slots:

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;  // 原始输入点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;        // 待运算点云

    std::vector<pcl::ModelCoefficients> CoefficientsList;         // 各平面系数列表
    std::vector<pcl::PointCloud<pcl::PointXYZ>> planeCloudsList;  // 各平面提取的内点

    pcl::PointXYZ ThreePlanesIntersectionPoint;         // 三面交点
    std::vector<pcl::PointXYZ> HorizonSeam_EndPoints;   // 横梁处水平角接焊缝端点
    std::vector<pcl::PointXYZ> VerticalSeam_EndPoints;  // 横梁处竖直角接焊缝端点

    // 参数
    std::vector<pcl::PointXYZ> ButtSeam_endpoints;  // 用于辅助计算角接焊缝端点的对接焊缝端点

    int Ransac_Plane_Iterations = 500;      // Ransac拟合平面的迭代数
    double Max_Cluster_radius = 8;          // 欧式聚类提取最大点集半径
    double Ransac_plane_Dth = 1;            // ransac拟合平面的距离阈值  2
    double NearLine_DistanceThreshold = 1;  // 提取近线点云的距离阈值
    float Distance_Vertical_Offset = 3;     // 端点偏移距离
    float Distance_Horizon_Offset = 3;      // 端点偏移距离
    double angleParalleThreshold = 10;      // 平行线角度判断阈值

    bool saveAndOutputDebugInformation = true;  // 是否保存用于调试的中间过程点云，以及输出用于调试的信息
    // bool saveAndOutputDebugInformation = false;  // 是否保存用于调试的中间过程点云，以及输出用于调试的信息

    std::vector<pcl::PointXYZ> beamUpHorizonFilletSeams;   // 横梁水平角接焊缝的端点
    std::vector<pcl::PointXYZ> beamUpVerticalFilletSeams;  // 横梁竖直角接焊缝的端点
};

#endif  // UPBEAMFILLETSEAMSDET_H
