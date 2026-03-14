#ifndef DOWNBEAMFILLETSEAMSDET_H
#define DOWNBEAMFILLETSEAMSDET_H

#include "seamDetWithPointCloud/AbstractSeamDet.h"

class DownBeamFilletSeamsDet : public AbstractSeamDet {
public:
    explicit DownBeamFilletSeamsDet(QObject *parent = nullptr);

    std::vector<std::shared_ptr<WeldSeamInfo>> solveSeamsEndPoints(std::vector<std::shared_ptr<WeldSeamInfo>> seamsInfo) override;  // 求解焊缝

    void Ransac_Multiple_planes(int plane_nums);  // Ransac分割拟合多个平面
    bool Solve_Fillet_Endpoints();                // 提取焊缝
    void SingleSeam_Reinitialize();               // 单条焊缝检测完成后，变量重新初始化

signals:

public slots:

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;  // 待运算点云

    std::vector<pcl::ModelCoefficients> Coefficients_list;          // 各平面系数列表
    std::vector<pcl::PointCloud<pcl::PointXYZ>> plane_clouds_list;  // 各平面提取的内点

    pcl::PointXYZ ThreePlanes_IntersectionPoint;        // 三面交点
    std::vector<pcl::PointXYZ> HorizonSeam_EndPoints;   // 横梁处水平角接焊缝端点
    std::vector<pcl::PointXYZ> VerticalSeam_EndPoints;  // 横梁处数值角接焊缝端点

    // 参数
    int Ransac_Plane_Iterations = 500;      // Ransac拟合平面的迭代数
    double Ransac_plane_Dth = 1;            // ransac拟合平面的距离阈值
    double NearLine_DistanceThreshold = 1;  // 提取近线点云的距离阈值
    float Distance_Vertical_Offset = 3;     // 端点偏移距离
    float Distance_Horizon_Offset = 3;      // 端点偏移距离

    // bool saveAndOutputDebugInformation = true;  // 是否保存用于调试的中间过程点云，以及输出用于调试的信息
    bool saveAndOutputDebugInformation = false;  // 是否保存用于调试的中间过程点云，以及输出用于调试的信息

    std::vector<pcl::PointXYZ> beamDownHorizonFilletSeams;   // 横梁水平角接焊缝的端点
    std::vector<pcl::PointXYZ> beamDownVerticalFilletSeams;  // 横梁竖直角接焊缝的端点
};

#endif  // DOWNBEAMFILLETSEAMSDET_H
