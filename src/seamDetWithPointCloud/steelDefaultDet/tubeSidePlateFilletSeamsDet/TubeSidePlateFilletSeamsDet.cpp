#include "TubeSidePlateFilletSeamsDet.h"

#include "seamDetWithPointCloud/QhullLock.h"
#include "utils/pointCloud/PointCloudFunc.h"
TubeSidePlateFilletSeamsDet::TubeSidePlateFilletSeamsDet(QObject* parent) : AbstractSeamDet{parent} {}

std::vector<std::shared_ptr<WeldSeamInfo> > TubeSidePlateFilletSeamsDet::solveSeamsEndPoints(
    std::vector<std::shared_ptr<WeldSeamInfo> > seamsInfo) {
    tempWeldSeamsInfo = seamsInfo;
    for (int i = 0; i < tempWeldSeamsInfo.size(); i++) {
        // SingleSeam_Reinitialize();
        cloudInWeldArea = tempWeldSeamsInfo[i]->weldAreaPointCloud;  // 获取焊缝区域点云
        if (cloudInWeldArea->size() == 0) {
            tempWeldSeamsInfo[i]->detectSuccFlag = false;
            continue;
        }

        // pcl::io::savePCDFile("./data/seamDetWithPointCloud/before_cluster.pcd", *cloudInWeldArea);

        MyToolFunc::myFastMaxCluster(cloudInWeldArea, Max_Cluster_radius);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCopy(new pcl::PointCloud<pcl::PointXYZ>);
        *cloudCopy = *cloudInWeldArea;  // 深拷贝

        pcl::io::savePCDFileBinary("./data/seamDetWithPointCloud/tubeSidePlateFilletSeamsDet/after_cluster.pcd", *cloudCopy);
        // pcl::ModelCoefficients planeCoefficients;
        // pcl::PointCloud<pcl::PointXYZ>::Ptr planeInliers(new pcl::PointCloud<pcl::PointXYZ>);
        // std::vector<pcl::ModelCoefficients> cylinderCoefficients;
        // fitPlaneAnd2Cylinders(cloud, planeCoefficients, planeInliers, cylinderCoefficients);
    }
    // tempWeldSeamsInfo.clear();
    return tempWeldSeamsInfo;
}
// void TubePlateButtSeamsDet::fitPlaneAnd2Cylinders(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
//                                                   pcl::ModelCoefficients& planeCoefficients,
//                                                   pcl::PointCloud<pcl::PointXYZ>::Ptr& planeInliers,
//                                                   std::vector<pcl::ModelCoefficients>& cylinderCoefficients) {
//     if (!cloud || cloud->empty()) {
//         PLOGE << "点云为空";
//         return;
//     }

//     cylinderCoefficients.clear();
//     planeInliers.reset(new pcl::PointCloud<pcl::PointXYZ>);

//     // ---------- 下采样 ----------
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFit(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloudExtract(new pcl::PointCloud<pcl::PointXYZ>);
//     pointcloudUniformDownsampling(cloud, 2.5f, cloudFit);
//     pointcloudUniformDownsampling(cloud, 1.0f, cloudExtract);

//     pcl::PointCloud<pcl::PointXYZ>::Ptr remainingCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudFit));

//     // ---------- 法向量估计 ----------
//     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

//     ne.setSearchMethod(tree);
//     ne.setInputCloud(remainingCloud);
//     ne.setKSearch(100);
//     ne.compute(*normals);

//     pcl::ExtractIndices<pcl::PointXYZ> extract;

//     std::vector<pcl::ModelCoefficients> cylCoeff(2);
//     std::vector<pcl::PointIndices::Ptr> cylIdx(2);

//     // ---------- 顺序拟合两个圆柱 ----------
//     for (int k = 0; k < 2; ++k) {
//         pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
//         cylIdx[k].reset(new pcl::PointIndices);

//         seg.setOptimizeCoefficients(true);
//         seg.setModelType(pcl::SACMODEL_CYLINDER);
//         seg.setMethodType(pcl::SAC_RANSAC);
//         seg.setNormalDistanceWeight(0.1);
//         seg.setMaxIterations(10000);
//         seg.setDistanceThreshold(0.5);
//         seg.setRadiusLimits(10, 150);
//         seg.setInputCloud(remainingCloud);
//         seg.setInputNormals(normals);

//         seg.segment(*cylIdx[k], cylCoeff[k]);
//         if (cylIdx[k]->indices.empty()) break;

//         // 移除当前圆柱内点，避免影响下一个圆柱
//         extract.setInputCloud(remainingCloud);
//         extract.setIndices(cylIdx[k]);
//         extract.setNegative(true);

//         pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
//         extract.filter(*tmp);
//         remainingCloud = tmp;

//         // 重新计算剩余点云的法向量
//         normals.reset(new pcl::PointCloud<pcl::Normal>);
//         ne.setInputCloud(remainingCloud);
//         ne.compute(*normals);
//     }

//     if (cylIdx[0]->indices.empty() || cylIdx[1]->indices.empty()) return;

//     // ---------- 拟合平面 ----------
//     pcl::SACSegmentation<pcl::PointXYZ> planeSeg;
//     pcl::PointIndices::Ptr planeIdx(new pcl::PointIndices);

//     planeSeg.setOptimizeCoefficients(true);
//     planeSeg.setModelType(pcl::SACMODEL_PLANE);
//     planeSeg.setMethodType(pcl::SAC_RANSAC);
//     planeSeg.setMaxIterations(1000);
//     planeSeg.setDistanceThreshold(1.0);
//     planeSeg.setInputCloud(remainingCloud);
//     planeSeg.segment(*planeIdx, planeCoefficients);

//     if (planeIdx->indices.empty()) return;

//     // ---------- 提取平面与圆柱交线区域 ----------
//     Eigen::Vector3d P0_1(cylCoeff[0].values[0], cylCoeff[0].values[1], cylCoeff[0].values[2]);
//     Eigen::Vector3d A1(cylCoeff[0].values[3], cylCoeff[0].values[4], cylCoeff[0].values[5]);
//     A1.normalize();
//     double r1 = cylCoeff[0].values[6];

//     Eigen::Vector3d P0_2(cylCoeff[1].values[0], cylCoeff[1].values[1], cylCoeff[1].values[2]);
//     Eigen::Vector3d A2(cylCoeff[1].values[3], cylCoeff[1].values[4], cylCoeff[1].values[5]);
//     A2.normalize();
//     double r2 = cylCoeff[1].values[6];

//     double a = planeCoefficients.values[0];
//     double b = planeCoefficients.values[1];
//     double c = planeCoefficients.values[2];
//     double d = planeCoefficients.values[3];
//     double invPlaneNorm = 1.0 / std::sqrt(a * a + b * b + c * c);

//     const double planeTol = 1.0;
//     const double radialTol = 15;

//     for (const auto& p : cloudExtract->points) {
//         Eigen::Vector3d pt(p.x, p.y, p.z);

//         // 点到平面的距离
//         double planeDist = std::abs(a * p.x + b * p.y + c * p.z + d) * invPlaneNorm;
//         if (planeDist > planeTol) continue;

//         bool inIntersection = false;

//         // 到第一个圆柱轴线的径向距离
//         {
//             Eigen::Vector3d v = pt - P0_1;
//             double t = v.dot(A1);
//             double r = (v - t * A1).norm();
//             if (std::abs(r - r1) < radialTol) inIntersection = true;
//         }

//         // 到第二个圆柱轴线的径向距离
//         if (!inIntersection) {
//             Eigen::Vector3d v = pt - P0_2;
//             double t = v.dot(A2);
//             double r = (v - t * A2).norm();
//             if (std::abs(r - r2) < radialTol) inIntersection = true;
//         }

//         if (inIntersection) planeInliers->push_back(p);
//     }

//     cylinderCoefficients.push_back(cylCoeff[0]);
//     cylinderCoefficients.push_back(cylCoeff[1]);
// }
