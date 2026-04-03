#ifndef ABSTRACTSEAMDET_H
#define ABSTRACTSEAMDET_H

#include <pcl/ModelCoefficients.h>
#include <pcl/common/distances.h>
#include <pcl/common/intersections.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <plog/Log.h>

#include <QObject>
#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <iostream>
#include <memory>

#include "src/utils/common/CommonFunc.h"

class WeldSeamInfo;

class AbstractSeamDet : public QObject {
    Q_OBJECT
public:
    explicit AbstractSeamDet(QObject *parent = nullptr);

    virtual std::vector<std::shared_ptr<WeldSeamInfo>> solveSeamsEndPoints(std::vector<std::shared_ptr<WeldSeamInfo>> seamsInfo) = 0;  // 求解焊缝

signals:

public slots:

protected:
    std::vector<std::shared_ptr<WeldSeamInfo>> tempWeldSeamsInfo;  // 临时焊缝信息

private:
};

#endif  // ABSTRACTSEAMDET_H
