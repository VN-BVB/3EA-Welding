#ifndef PLATEPLATEFILLETSEAMSDET_H
#define PLATEPLATEFILLETSEAMSDET_H

#include <QObject>

#include "seamDetWithPointCloud/AbstractSeamDet.h"
#include "utils/common/WeldSeamInfo.h"
enum WELD_TYPE;
class PlatePlateFilletSeamsDet : public AbstractSeamDet {
public:
    explicit PlatePlateFilletSeamsDet(QObject* parent = nullptr);
    // 求解焊缝
    std::vector<std::shared_ptr<WeldSeamInfo>> solveSeamsEndPoints(std::vector<std::shared_ptr<WeldSeamInfo>> seamsInfo) override;

private:
    void singleSeamReinitialize();
    void statisticFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
    float computePlaneArea(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::ModelCoefficients::Ptr coeff);
    void ransacMultiPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud,
                          pcl::ModelCoefficients::Ptr& mainPlaneCoeffs, pcl::ModelCoefficients::Ptr& otherPlaneCoeffs);
    void computePlaneIntersectionLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr line_cloud);
    void filterCloudByPlaneThenMorph(pcl::PointCloud<pcl::PointXYZ>::Ptr axisRangeCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud);
    bool solveSeamEndPoints();
signals:

private:
    bool saveFlag = false;
    bool detectSuccFlag = false;
    WELD_TYPE weldType;
    Eigen::Vector3f mainPlaneAxis = Eigen::Vector3f(0, 1, 0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInWeldArea;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPlaneInWeldAreaWithSeam;
    pcl::PointCloud<pcl::PointXYZ>::Ptr seamEndPoints;
    pcl::PointCloud<pcl::PointXYZ>::Ptr axisRangeCloud;

    double Max_cluster_radius = 8;       // 欧式聚类提取最大点集半径
    int Ransac_plane_Iterations = 1000;  // Ransac拟合平面的迭代数
    double Ransac_plane_Dth = 1.0;       // ransac拟合平面的距离阈值

    double resolution = 1.0;                  // 二维栅格大小
    int Statistic_NeighPoints = 20;           // 统计滤波近邻点数
    double Statistic_sigma = 3.0;             // 统计滤波系数
    double extendCylinderInPlaneArea = 10.0;  // 选取焊缝区域衍生
    float max_dist = 5.0f;                    // 拟合端点衍生

    std::vector<pcl::PointXYZ> filletSeamsTSP;               // 焊缝的端点
    pcl::ModelCoefficients::Ptr planeCoeffsWithWeldSeam;     // 焊缝所在平面系数
    pcl::ModelCoefficients::Ptr otherPlaneCoeffsInWeldArea;  // 焊接区域中其余母材面系数
    pcl::ModelCoefficients::Ptr lineCoeffsWithWeldSeam2Val;  // 焊缝所在直线系数
};

#endif  // PLATEPLATEFILLETSEAMSDET_H
