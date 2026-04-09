#ifndef TUBEPLATEFILLETSEAMSDET_H
#define TUBEPLATEFILLETSEAMSDET_H

#include <QObject>

#include "seamDetWithPointCloud/AbstractSeamDet.h"
#include "utils/common/WeldSeamInfo.h"
class TubePlateFilletSeamsDet : public AbstractSeamDet {
public:
    explicit TubePlateFilletSeamsDet(QObject* parent = nullptr);
    // 求解焊缝
    std::vector<std::shared_ptr<WeldSeamInfo>> solveSeamsEndPoints(std::vector<std::shared_ptr<WeldSeamInfo>> seamsInfo) override;
    struct PtTheta {
        pcl::PointXYZ pt;
        float theta;
    };

private:
    void singleSeamReinitialize();
    void statisticFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
    void ransacCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_non_cylinder);
    void ransacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::ModelCoefficients::Ptr planeCoeff);

    void extractSeamPointsFromCylinderPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud,
                                            pcl::ModelCoefficients::Ptr plane_coeff, double thresh_plane);
    void removePlanePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    bool solveSeamEndPoints();
    bool moveAlongOrdered(const std::vector<PtTheta>& ordered, float offset, bool from_start, PtTheta& result, int& cut_idx);

private:
    bool saveFlag = false;
    bool detectSuccFlag = false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInWeldArea;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCylinderInWeldAreaWithSeam;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNoCylinderInWeldArea;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPlaneInWeldArea;
    pcl::PointCloud<pcl::PointXYZ>::Ptr seamEndPoints;
    pcl::PointCloud<pcl::PointXYZ>::Ptr axisRangeCloud;

    double Max_cluster_radius = 8;           // 欧式聚类提取最大点集半径
    int Ransac_plane_Iterations = 10000;     // Ransac拟合平面的迭代数
    int Ransac_cylinder_Iterations = 10000;  // Ransac拟合圆柱的迭代数
    double Ransac_plane_Dth = 1.0;           // ransac拟合平面的距离阈值
    double Ransac_cylinder_Dth = 1.5;        // ransac拟合圆柱面的距离阈值

    int Statistic_NeighPoints = 20;           // 统计滤波近邻点数
    double Statistic_sigma = 6.0;             // 统计滤波系数
    double extendCylinderInPlaneArea = 15.0;  // 选取焊缝区域衍生
    float t_step = 2.0f;                      // 轴向分辨率（mm）
    float theta_step = 2.0f * M_PI / 180.0f;  // n°一格
    double widthThreshRatio = 0.75;           // 筛选残留点云宽度阈值比例
    int sample_num = 3 + 3 * 2;               // 交线采样点数量，3是起点中点终点，乘2是中点两边

    std::vector<pcl::PointXYZ> filletSeamsTP;                // 焊缝的端点
    pcl::ModelCoefficients::Ptr cylinderCoeffsWithWeldSeam;  // 焊缝所在母材系数
    pcl::ModelCoefficients::Ptr planeCoeffsInWeldArea;       // 焊接区域中母材平面系数
    pcl::ModelCoefficients::Ptr lineCoeffsWithWeldSeam2Val;  // 焊缝所在直线系数 (用于验证)
};

#endif  // TUBEPLATEFILLETSEAMSDET_H
