#ifndef TUBEPLATEBUTTSEAM_H
#define TUBEPLATEBUTTSEAM_H

#include <QObject>

#include "seamDetWithPointCloud/AbstractSeamDet.h"
#include "utils/common/WeldSeamInfo.h"
class TubeSidePlateFilletSeamsDet : public AbstractSeamDet {
public:
    explicit TubeSidePlateFilletSeamsDet(QObject *parent = nullptr);
    // 求解焊缝
    std::vector<std::shared_ptr<WeldSeamInfo>> solveSeamsEndPoints(std::vector<std::shared_ptr<WeldSeamInfo>> seamsInfo) override;

private:
    // void SingleSeam_Reinitialize();

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInWeldArea;

    double Max_Cluster_radius = 8;  // 欧式聚类提取最大点集半径
signals:
};

#endif  // TUBEPLATEBUTTSEAM_H
