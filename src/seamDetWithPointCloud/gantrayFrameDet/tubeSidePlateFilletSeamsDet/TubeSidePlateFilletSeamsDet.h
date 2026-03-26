#ifndef TUBESIDEPLATEFILLETSEAMSDET_H
#define TUBESIDEPLATEFILLETSEAMSDET_H

#include <QObject>

#include "seamDetWithPointCloud/AbstractSeamDet.h"
#include "utils/common/WeldSeamInfo.h"
class TubeSidePlateFilletSeamsDet : public AbstractSeamDet {
public:
    explicit TubeSidePlateFilletSeamsDet(QObject *parent = nullptr);
    // 求解焊缝
    std::vector<std::shared_ptr<WeldSeamInfo>> solveSeamsEndPoints(std::vector<std::shared_ptr<WeldSeamInfo>> seamsInfo) override;

private:
    void singleSeamReinitialize();
    void statisticFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
    void ransacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_noplane);
    void ransacCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud);
    void removeCylinderPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    bool solveSeamEndPoints();
signals:

private:
    bool saveFlag = false;
    bool detectSuccFlag = false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInWeldArea;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPlaneInWeldAreaWithSeam;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNoPlaneInWeldArea;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCylinderInWeldArea;
    pcl::PointCloud<pcl::PointXYZ>::Ptr seamEndPoints;
    pcl::PointCloud<pcl::PointXYZ>::Ptr axisRangeCloud;

    double Max_cluster_radius = 8;           // 欧式聚类提取最大点集半径
    int Ransac_plane_Iterations = 10000;     // Ransac拟合平面的迭代数
    int Ransac_cylinder_Iterations = 10000;  // Ransac拟合圆柱的迭代数
    double Ransac_plane_Dth = 1.0;           // ransac拟合平面的距离阈值
    double Ransac_cylinder_Dth = 1.5;        // ransac拟合平面的距离阈值
    double resolution = 1.0;                 // 腐蚀膨胀中二维栅格大小
    int Statistic_NeighPoints = 20;          // 统计滤波近邻点数
    double Statistic_sigma = 3.0;            // 统计滤波系数
    double extendCylinderInPlaneArea = 3.0;  // 选取焊缝区域衍生 R+extendCylinderInPlaneArea

    std::vector<pcl::PointXYZ> filletSeamsTSP;               // 焊缝的端点
    pcl::ModelCoefficients::Ptr planeCoeffsWithWeldSeam;     // 焊缝所在平面系数
    pcl::ModelCoefficients::Ptr cylinderCoeffsInWeldArea;    // 焊接区域中圆柱面系数
    pcl::ModelCoefficients::Ptr lineCoeffsWithWeldSeam2Val;  // 焊缝所在直线系数 (用于验证)
};

#endif  // TUBESIDEPLATEFILLETSEAMSDET_H
