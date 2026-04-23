#include "TubeTubeFilletSeamsDet.h"

#include "seamDetWithPointCloud/QhullLock.h"
#include "settingPara/SettingPara.h"
#include "utils/pointCloud/PointCloudFunc.h"
#include "utils/pointCloud/SeamConcavityExtractor.h"
TubeTubeFilletSeamsDet::TubeTubeFilletSeamsDet(QObject* parent) : AbstractSeamDet{parent} {}

std::vector<std::shared_ptr<WeldSeamInfo>> TubeTubeFilletSeamsDet::solveSeamsEndPoints(std::vector<std::shared_ptr<WeldSeamInfo>> seamsInfo) {
    tempWeldSeamsInfo = seamsInfo;
    for (size_t i = 0; i < tempWeldSeamsInfo.size(); i++) {
        singleSeamReinitialize();
        areaNum = tempWeldSeamsInfo[i]->areaNum;

        cloudInWeldArea = tempWeldSeamsInfo[i]->weldAreaPointCloudInCamera;  // 获取焊缝区域点云
        if (cloudInWeldArea->size() == 0) {
            PLOGE << "焊缝区域为0";
            detectSuccFlag = false;
            continue;
        }
        {
            // ScopedTimer t("myFastMaxCluster");
            MyToolFunc::myFastMaxCluster(cloudInWeldArea, Max_cluster_radius);
            if (saveFlag) {
                cloudInWeldArea->height = 1;
                cloudInWeldArea->width = static_cast<uint32_t>(cloudInWeldArea->size());
                pcl::io::savePCDFile("./data/seamDetWithPointCloud/tubeTubeFilletSeamsDet/after_cluster_" + std::to_string(areaNum) + ".pcd",
                                     *cloudInWeldArea);
            }
        }
        {
            // ScopedTimer t("ransacCylinder");
            ransacCylinder();
            if (!cylinderCloudPrimary || cylinderCloudPrimary->empty() || !cylinderCloudSecondary || cylinderCloudSecondary->empty()) {
                PLOGE << "Ransac_cylinder: 圆柱点云为空";
                detectSuccFlag = false;
                continue;
            }
            cylinderCloudPrimary->height = 1;
            cylinderCloudPrimary->width = static_cast<uint32_t>(cylinderCloudPrimary->size());
            if (saveFlag) {
                pcl::io::savePCDFile("./data/seamDetWithPointCloud/tubeTubeFilletSeamsDet/cylinderCloudPrimary" + std::to_string(areaNum) + ".pcd",
                                     *cylinderCloudPrimary);
                // cylinderCloudSecondary->height = 1;
                // cylinderCloudSecondary->width = static_cast<uint32_t>(cylinderCloudSecondary->size());
                // pcl::io::savePCDFile("./data/seamDetWithPointCloud/tubeTubeFilletSeamsDet/cylinderCloudSecondary" + std::to_string(areaNum) +
                // ".pcd",
                //                      *cylinderCloudSecondary);
            }
        }
        {
            // ScopedTimer t("myFastMaxCluster");
            MyToolFunc::myFastMaxCluster(cylinderCloudPrimary, 3);
            if (saveFlag) {
                cylinderCloudPrimary->height = 1;
                cylinderCloudPrimary->width = static_cast<uint32_t>(cylinderCloudPrimary->size());
                pcl::io::savePCDFile("./data/seamDetWithPointCloud/tubeTubeFilletSeamsDet/myFastMaxCluster_" + std::to_string(areaNum) + ".pcd",
                                     *cylinderCloudPrimary);
            }
        }
        {
            // ScopedTimer t("statisticFilter");
            // 统计滤波
            MyToolFunc::statisticFilter(cylinderCloudPrimary, cylinderCloudPrimary, Statistic_NeighPoints, Statistic_sigma);

            if (saveFlag) {
                cylinderCloudPrimary->height = 1;
                cylinderCloudPrimary->width = static_cast<uint32_t>(cylinderCloudPrimary->size());
                pcl::io::savePCDFile("./data/seamDetWithPointCloud/tubeTubeFilletSeamsDet/statisticFilter_" + std::to_string(areaNum) + ".pcd",
                                     *cylinderCloudPrimary);
            }
        }
        {
            // ScopedTimer t("projectCloudToCylinder");
            MyToolFunc::projectCloudToCylinder(cylinderCloudPrimary, axisRangeCloud, cylinderCoeffsPrimary);
            if (saveFlag) {
                axisRangeCloud->height = 1;
                axisRangeCloud->width = static_cast<uint32_t>(axisRangeCloud->size());
                pcl::io::savePCDFile("./data/seamDetWithPointCloud/tubeTubeFilletSeamsDet/projectCloudToCylinder_" + std::to_string(areaNum) + ".pcd",
                                     *axisRangeCloud);
            }
        }

        detectSuccFlag = solveSeamEndPoints();
        if (saveFlag) {
            seamEndPoints->height = 1;
            seamEndPoints->width = static_cast<uint32_t>(seamEndPoints->size());
            pcl::io::savePCDFile("./data/seamDetWithPointCloud/tubeTubeFilletSeamsDet/seamEndPoints" + std::to_string(areaNum) + ".pcd",
                                 *seamEndPoints);
        }
        // detectSuccFlag = false;
        // 保存本次检测到的信息
        tempWeldSeamsInfo[i]->detectSuccFlag = detectSuccFlag;
        if (detectSuccFlag) {
            tempWeldSeamsInfo[i]->weldEndPointsInCamera.reset(new std::vector<pcl::PointXYZ>(std::move(filletSeamsTP)));  // 检测结果
            tempWeldSeamsInfo[i]->weldEndPointsInRobot.reset(new std::vector<pcl::PointXYZ>());
            tempWeldSeamsInfo[i]->weldCoeff = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients(*cylinderCoeffsPrimary));
            tempWeldSeamsInfo[i]->otherSurface.emplace_back(pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients(*cylinderCoeffsSecondary)));
            tempWeldSeamsInfo[i]->weldType = Tube_Plate_Fillet;
        }
    }
    PLOGE << "return tempWeldSeamsInfo;";
    return tempWeldSeamsInfo;
}
// 单条焊缝检测前，变量重新初始化
void TubeTubeFilletSeamsDet::singleSeamReinitialize() {
    cloudInWeldArea.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cylinderCloudPrimary.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cylinderCloudSecondary.reset(new pcl::PointCloud<pcl::PointXYZ>);
    seamEndPoints.reset(new pcl::PointCloud<pcl::PointXYZ>);
    axisRangeCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    cylinderCoeffsPrimary.reset(new pcl::ModelCoefficients);
    cylinderCoeffsSecondary.reset(new pcl::ModelCoefficients);

    filletSeamsTP.clear();
    saveFlag = SettingPara::getInstance().bool_save_model;
    // saveFlag = true;
    detectSuccFlag = false;
}
void TubeTubeFilletSeamsDet::ransacCylinder() {
    if (!cloudInWeldArea || cloudInWeldArea->empty()) {
        PLOGE << "ransacCylinder: cloudInWeldArea 为空";
        detectSuccFlag = false;
        return;
    }

    if (!cylinderCoeffsPrimary) {
        cylinderCoeffsPrimary.reset(new pcl::ModelCoefficients);
    }
    if (!cylinderCoeffsSecondary) {
        cylinderCoeffsSecondary.reset(new pcl::ModelCoefficients);
    }
    if (!cylinderCloudPrimary) {
        cylinderCloudPrimary.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }
    if (!cylinderCloudSecondary) {
        cylinderCloudSecondary.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }

    cylinderCoeffsPrimary->values.clear();
    cylinderCoeffsSecondary->values.clear();
    cylinderCloudPrimary->clear();
    cylinderCloudSecondary->clear();

    detectSuccFlag = true;

    // ======================== 0. 原始点云先做体素下采样 ========================
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZ>);
    {
        // ScopedTimer t("voxelDownsample");
        const float voxelLeafSize = 1.0f;  // 你后面可以自己调，先给个常用值
        MyToolFunc::pointcloudVoxelDownsampling(cloudInWeldArea, voxelLeafSize, cloud_ds);
    }

    if (!cloud_ds || cloud_ds->empty()) {
        PLOGE << "ransacCylinder: 体素下采样后点云为空";
        detectSuccFlag = false;
        return;
    }

    // 如果下采样太狠，直接退回原始点云，避免拟合不稳
    pcl::PointCloud<pcl::PointXYZ>::Ptr fitCloud = cloud_ds;
    if (cloud_ds->size() < 200) {
        PLOGW << "ransacCylinder: 下采样后点数过少，退回原始点云拟合";
        fitCloud = cloudInWeldArea;
    }

    // ======================== 1. 在拟合点云上计算法向 ========================
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_all(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_all(new pcl::PointCloud<pcl::Normal>);

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne_all;
    ne_all.setNumberOfThreads(std::max(1u, std::thread::hardware_concurrency() / 2));
    ne_all.setInputCloud(fitCloud);
    ne_all.setSearchMethod(tree_all);

    const int k_all = std::min<int>(100, std::max<int>(10, static_cast<int>(fitCloud->size()) / 20));
    ne_all.setKSearch(k_all);
    ne_all.compute(*normals_all);

    if (normals_all->empty() || normals_all->size() != fitCloud->size()) {
        PLOGE << "ransacCylinder: normals_all 计算失败";
        detectSuccFlag = false;
        return;
    }

    // ======================== 2. 第一次拟合圆柱（在下采样点云上） ========================
    pcl::PointIndices::Ptr inliers_first(new pcl::PointIndices);

    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_first;
    seg_first.setOptimizeCoefficients(true);
    seg_first.setModelType(pcl::SACMODEL_CYLINDER);
    seg_first.setMethodType(pcl::SAC_RANSAC);
    seg_first.setNormalDistanceWeight(0.2);
    seg_first.setMaxIterations(Ransac_cylinder_Iterations);
    seg_first.setDistanceThreshold(Ransac_cylinder_Dth);
    seg_first.setRadiusLimits(20, 150);
    seg_first.setInputCloud(fitCloud);
    seg_first.setInputNormals(normals_all);

    pcl::ModelCoefficients::Ptr coeff_first(new pcl::ModelCoefficients);
    seg_first.segment(*inliers_first, *coeff_first);

    if (inliers_first->indices.empty() || coeff_first->values.size() < 7) {
        PLOGE << "ransacCylinder: 第一个圆柱拟合失败";
        detectSuccFlag = false;
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_first_ds(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract_xyz_first;
    extract_xyz_first.setInputCloud(fitCloud);
    extract_xyz_first.setIndices(inliers_first);
    extract_xyz_first.setNegative(false);
    extract_xyz_first.filter(*cloud_first_ds);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remain_ds(new pcl::PointCloud<pcl::PointXYZ>);
    extract_xyz_first.setNegative(true);
    extract_xyz_first.filter(*cloud_remain_ds);

    if (!cloud_remain_ds || cloud_remain_ds->empty()) {
        PLOGE << "ransacCylinder: 去掉第一个圆柱后剩余点云为空，无法拟合第二个圆柱";
        detectSuccFlag = false;
        return;
    }

    // ======================== 3. 在剩余下采样点云上重新计算法向 ========================
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_remain(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_remain(new pcl::PointCloud<pcl::Normal>);

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne_remain;
    ne_remain.setNumberOfThreads(std::max(1u, std::thread::hardware_concurrency() / 2));
    ne_remain.setInputCloud(cloud_remain_ds);
    ne_remain.setSearchMethod(tree_remain);

    const int k_remain = std::min<int>(100, std::max<int>(10, static_cast<int>(cloud_remain_ds->size()) / 20));
    ne_remain.setKSearch(k_remain);
    ne_remain.compute(*normals_remain);

    if (normals_remain->empty() || normals_remain->size() != cloud_remain_ds->size()) {
        PLOGE << "ransacCylinder: normals_remain 计算失败";
        detectSuccFlag = false;
        return;
    }

    // ======================== 4. 第二次拟合圆柱（在下采样剩余点云上） ========================
    pcl::PointIndices::Ptr inliers_second(new pcl::PointIndices);

    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_second;
    seg_second.setOptimizeCoefficients(true);
    seg_second.setModelType(pcl::SACMODEL_CYLINDER);
    seg_second.setMethodType(pcl::SAC_RANSAC);
    seg_second.setNormalDistanceWeight(0.2);
    seg_second.setMaxIterations(Ransac_cylinder_Iterations);
    seg_second.setDistanceThreshold(Ransac_cylinder_Dth);
    seg_second.setRadiusLimits(20, 150);
    seg_second.setInputCloud(cloud_remain_ds);
    seg_second.setInputNormals(normals_remain);

    pcl::ModelCoefficients::Ptr coeff_second(new pcl::ModelCoefficients);
    seg_second.segment(*inliers_second, *coeff_second);

    if (inliers_second->indices.empty() || coeff_second->values.size() < 7) {
        PLOGE << "ransacCylinder: 第二个圆柱拟合失败";
        detectSuccFlag = false;
        return;
    }

    // ======================== 5. Primary / Secondary 排序 ========================
    // 判据：圆柱轴线到原点的距离，谁更小谁是 Primary
    auto calcAxisLineDistToOrigin = [](const pcl::ModelCoefficients::Ptr& coeff) -> float {
        if (!coeff || coeff->values.size() < 6) {
            return std::numeric_limits<float>::max();
        }

        Eigen::Vector3f axisPoint(coeff->values[0], coeff->values[1], coeff->values[2]);
        Eigen::Vector3f axisDir(coeff->values[3], coeff->values[4], coeff->values[5]);

        const float eps = 1e-6f;
        const float dirNorm = axisDir.norm();
        if (dirNorm < eps) {
            return std::numeric_limits<float>::max();
        }

        axisDir /= dirNorm;

        // 原点到直线 distance = |p x d|
        return axisPoint.cross(axisDir).norm();
    };

    const float dist_first = calcAxisLineDistToOrigin(coeff_first);
    const float dist_second = calcAxisLineDistToOrigin(coeff_second);

    const float dist_eps = 1e-3f;
    if (!std::isfinite(dist_first) || !std::isfinite(dist_second) || dist_first == std::numeric_limits<float>::max() ||
        dist_second == std::numeric_limits<float>::max()) {
        PLOGE << "ransacCylinder: 圆柱轴线到原点距离无效, dist_first = " << dist_first << ", dist_second = " << dist_second;
        detectSuccFlag = false;
        cylinderCoeffsPrimary->values.clear();
        cylinderCoeffsSecondary->values.clear();
        cylinderCloudPrimary->clear();
        cylinderCloudSecondary->clear();
        return;
    }

    if (std::fabs(dist_first - dist_second) <= dist_eps) {
        PLOGE << "ransacCylinder: 两个圆柱轴线到原点距离过于接近，无法稳定区分主次, dist_first = " << dist_first << ", dist_second = " << dist_second
              << ", eps = " << dist_eps;
        detectSuccFlag = false;
        cylinderCoeffsPrimary->values.clear();
        cylinderCoeffsSecondary->values.clear();
        cylinderCloudPrimary->clear();
        cylinderCloudSecondary->clear();
        return;
    }

    // 谁离原点更近，谁是 Primary
    const bool firstIsPrimary = (dist_first < dist_second);

    pcl::ModelCoefficients::Ptr coeff_primary(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr coeff_secondary(new pcl::ModelCoefficients);

    if (firstIsPrimary) {
        *coeff_primary = *coeff_first;
        *coeff_secondary = *coeff_second;
    } else {
        *coeff_primary = *coeff_second;
        *coeff_secondary = *coeff_first;
    }

    *cylinderCoeffsPrimary = *coeff_primary;
    *cylinderCoeffsSecondary = *coeff_secondary;

    // ======================== 6. 用拟合好的两个圆柱参数，在原始点云中重新提取 ========================
    {
        // ScopedTimer t("extractCylinderFromOriginalCloud");

        Eigen::Vector3f axisPoint1(coeff_primary->values[0], coeff_primary->values[1], coeff_primary->values[2]);
        Eigen::Vector3f axisDir1(coeff_primary->values[3], coeff_primary->values[4], coeff_primary->values[5]);
        const float radius1 = coeff_primary->values[6];

        Eigen::Vector3f axisPoint2(coeff_secondary->values[0], coeff_secondary->values[1], coeff_secondary->values[2]);
        Eigen::Vector3f axisDir2(coeff_secondary->values[3], coeff_secondary->values[4], coeff_secondary->values[5]);
        const float radius2 = coeff_secondary->values[6];

        const float eps = 1e-6f;
        if (axisDir1.norm() < eps || axisDir2.norm() < eps) {
            PLOGE << "ransacCylinder: 提取阶段圆柱轴方向无效";
            detectSuccFlag = false;
            cylinderCloudPrimary->clear();
            cylinderCloudSecondary->clear();
            return;
        }

        axisDir1.normalize();
        axisDir2.normalize();

        const float distThresh = static_cast<float>(Ransac_cylinder_Dth);

        cylinderCloudPrimary->clear();
        cylinderCloudSecondary->clear();
        cylinderCloudPrimary->reserve(cloudInWeldArea->size() / 2);
        cylinderCloudSecondary->reserve(cloudInWeldArea->size() / 2);

        const int N = static_cast<int>(cloudInWeldArea->size());

        // 0: 都不属于
        // 1: primary
        // 2: secondary
        std::vector<unsigned char> labels(N, 0);

#pragma omp parallel for schedule(static)
        for (int i = 0; i < N; ++i) {
            const auto& pt = cloudInWeldArea->points[i];
            Eigen::Vector3f P(pt.x, pt.y, pt.z);

            // ---------- 对 primary 圆柱的误差 ----------
            Eigen::Vector3f AP1 = P - axisPoint1;
            const float t1 = AP1.dot(axisDir1);
            Eigen::Vector3f radial1 = AP1 - t1 * axisDir1;
            const float err1 = std::fabs(radial1.norm() - radius1);

            // ---------- 对 secondary 圆柱的误差 ----------
            Eigen::Vector3f AP2 = P - axisPoint2;
            const float t2 = AP2.dot(axisDir2);
            Eigen::Vector3f radial2 = AP2 - t2 * axisDir2;
            const float err2 = std::fabs(radial2.norm() - radius2);

            const bool on1 = (err1 <= distThresh);
            const bool on2 = (err2 <= distThresh);

            if (on1 && on2) {
                labels[i] = (err1 <= err2) ? 1 : 2;
            } else if (on1) {
                labels[i] = 1;
            } else if (on2) {
                labels[i] = 2;
            }
        }

        // 串行收集，避免并发 push_back
        for (int i = 0; i < N; ++i) {
            if (labels[i] == 1) {
                cylinderCloudPrimary->points.push_back(cloudInWeldArea->points[i]);
            } else if (labels[i] == 2) {
                cylinderCloudSecondary->points.push_back(cloudInWeldArea->points[i]);
            }
        }

        cylinderCloudPrimary->width = static_cast<uint32_t>(cylinderCloudPrimary->points.size());
        cylinderCloudPrimary->height = 1;
        cylinderCloudPrimary->is_dense = cloudInWeldArea->is_dense;

        cylinderCloudSecondary->width = static_cast<uint32_t>(cylinderCloudSecondary->points.size());
        cylinderCloudSecondary->height = 1;
        cylinderCloudSecondary->is_dense = cloudInWeldArea->is_dense;
    }

    if (!cylinderCloudPrimary || cylinderCloudPrimary->empty() || !cylinderCloudSecondary || cylinderCloudSecondary->empty()) {
        PLOGE << "ransacCylinder: 在原始点云中提取两个圆柱失败";
        detectSuccFlag = false;
        return;
    }

    PLOGD << "ransacCylinder: 拟合两个圆柱成功";

    // ======================== 7. 继续删除第一中的第二点云 ========================
    if (1) {
        {
            // ScopedTimer t("removePrimaryPtsBelongToSecondaryCylinder");

            if (!cylinderCloudPrimary || cylinderCloudPrimary->empty() || !cylinderCoeffsSecondary || cylinderCoeffsSecondary->values.size() < 7) {
                PLOGW << "debug remove overlap: 输入无效";
            } else {
                Eigen::Vector3f axisPoint(cylinderCoeffsSecondary->values[0], cylinderCoeffsSecondary->values[1], cylinderCoeffsSecondary->values[2]);

                Eigen::Vector3f axisDir(cylinderCoeffsSecondary->values[3], cylinderCoeffsSecondary->values[4], cylinderCoeffsSecondary->values[5]);

                const float radius = cylinderCoeffsSecondary->values[6];
                const float distThresh = static_cast<float>(Ransac_cylinder_Dth);
                const float eps = 1e-6f;

                const float dirNorm = axisDir.norm();
                if (dirNorm < eps) {
                    PLOGW << "debug remove overlap: 第二圆柱轴方向无效";
                } else {
                    axisDir /= dirNorm;

                    const float minR = std::max(0.0f, radius - distThresh);
                    const float maxR = radius + distThresh;
                    const float minR2 = minR * minR;
                    const float maxR2 = maxR * maxR;

                    const int N = static_cast<int>(cylinderCloudPrimary->size());
                    std::vector<unsigned char> keepMask(N, 0);

#pragma omp parallel for schedule(static)
                    for (int i = 0; i < N; ++i) {
                        const auto& pt = cylinderCloudPrimary->points[i];
                        Eigen::Vector3f P(pt.x, pt.y, pt.z);

                        Eigen::Vector3f AP = P - axisPoint;
                        const float t = AP.dot(axisDir);
                        Eigen::Vector3f radial = AP - t * axisDir;
                        const float radialDist2 = radial.squaredNorm();

                        if (radialDist2 < minR2 || radialDist2 > maxR2) {
                            keepMask[i] = 1;
                        }
                    }

                    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
                    filtered->reserve(cylinderCloudPrimary->size());

                    for (int i = 0; i < N; ++i) {
                        if (keepMask[i]) {
                            filtered->points.push_back(cylinderCloudPrimary->points[i]);
                        }
                    }

                    filtered->width = static_cast<uint32_t>(filtered->points.size());
                    filtered->height = 1;
                    filtered->is_dense = cylinderCloudPrimary->is_dense;

                    PLOGD << "debug remove overlap: primary before = " << cylinderCloudPrimary->size() << ", after = " << filtered->size()
                          << ", removed = " << (cylinderCloudPrimary->size() - filtered->size());

                    *cylinderCloudPrimary = *filtered;
                }
            }
        }
    }
}

void TubeTubeFilletSeamsDet::extractSeamPointsFromCylinderPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                                                pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud,
                                                                pcl::ModelCoefficients::Ptr plane_coeff, double thresh_plane) {
    if (!input_cloud || input_cloud->empty() || !output_cloud || !plane_coeff || plane_coeff->values.size() != 4) {
        PLOGE << "extractSeamPointsFromCylinderPlane: 参数错误";
        return;
    }

    // ---------- 平面参数 ----------
    float a = plane_coeff->values[0];
    float b = plane_coeff->values[1];
    float c = plane_coeff->values[2];
    float d = plane_coeff->values[3];

    // 是否归一化
    float norm_n = std::sqrt(a * a + b * b + c * c);

    output_cloud->clear();
    output_cloud->points.reserve(input_cloud->size());

    for (const auto& pt : input_cloud->points) {
        float x = pt.x;
        float y = pt.y;
        float z = pt.z;

        // ---------- 点到平面距离 ----------
        float dist = std::fabs(a * x + b * y + c * z + d) / norm_n;

        if (dist < thresh_plane) {
            output_cloud->points.push_back(pt);
        }
    }

    output_cloud->width = output_cloud->points.size();
    output_cloud->height = 1;
    output_cloud->is_dense = true;
}
void TubeTubeFilletSeamsDet::removePlanePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    if (!cloud || cloud->empty() || !cylinderCoeffsPrimary) {
        PLOGE << "removePlanePoints: 参数错误";
        return;
    }

    // ================= 1. 圆柱参数 =================
    Eigen::Vector3f C(cylinderCoeffsPrimary->values[0], cylinderCoeffsPrimary->values[1], cylinderCoeffsPrimary->values[2]);

    Eigen::Vector3f axis(cylinderCoeffsPrimary->values[3], cylinderCoeffsPrimary->values[4], cylinderCoeffsPrimary->values[5]);

    axis.normalize();

    // ================= 2. θ参考方向 =================
    Eigen::Vector3f z_world(0, 0, -1);
    if (fabs(axis.dot(z_world)) > 0.95f) {
        z_world = Eigen::Vector3f(1, 0, 0);
    }

    Eigen::Vector3f u = z_world - (z_world.dot(axis)) * axis;
    u.normalize();

    Eigen::Vector3f v_dir = axis.cross(u).normalized();

    // ================= 3. 计算 t 和 θ =================
    std::vector<float> t_vals, theta_vals;

    float t_min = 1e9f, t_max = -1e9f;

    t_vals.resize(cloud->size());
    theta_vals.resize(cloud->size());

    float t_min_local = 1e9f;
    float t_max_local = -1e9f;

#pragma omp parallel
    {
        float t_min_thread = 1e9f;
        float t_max_thread = -1e9f;

#pragma omp for
        for (int i = 0; i < static_cast<int>(cloud->size()); ++i) {
            const auto& pt = cloud->points[i];
            Eigen::Vector3f P(pt.x, pt.y, pt.z);

            float t = (P - C).dot(axis);

            Eigen::Vector3f P_axis = C + t * axis;
            Eigen::Vector3f v = P - P_axis;

            float x = v.dot(u);
            float y = v.dot(v_dir);

            float theta = atan2(y, x);

            t_vals[i] = t;
            theta_vals[i] = theta;

            t_min_thread = std::min(t_min_thread, t);
            t_max_thread = std::max(t_max_thread, t);
        }

#pragma omp critical
        {
            t_min_local = std::min(t_min_local, t_min_thread);
            t_max_local = std::max(t_max_local, t_max_thread);
        }
    }

    t_min = t_min_local;
    t_max = t_max_local;
    // ================= 4. 栅格参数 =================
    int t_bins = static_cast<int>((t_max - t_min) / t_step) + 1;
    int theta_bins = static_cast<int>(2 * M_PI / theta_step);

    std::vector<std::vector<bool>> grid(theta_bins, std::vector<bool>(t_bins, false));

    // ================= 5. 填充栅格 =================
    for (int i = 0; i < static_cast<int>(cloud->size()); ++i) {
        int t_idx = static_cast<int>((t_vals[i] - t_min) / t_step);
        int theta_idx = static_cast<int>((theta_vals[i] + M_PI) / theta_step);
        if (theta_idx == theta_bins) theta_idx = theta_bins - 1;
        if (t_bins <= 0 || t_bins > 50000) {
            PLOGE << "t_bins异常: " << t_bins;
            return;
        }
        if (t_idx >= 0 && t_idx < t_bins && theta_idx >= 0 && theta_idx < theta_bins) {
            grid[theta_idx][t_idx] = true;  // 幂等写，无需锁
        }
    }

    // ================= 6. 密度宽度 =================
    std::vector<double> theta_plot, width_plot;
    std::vector<int> theta_valid_idx;
    // 总占据长度函数
    for (int th = 0; th < theta_bins; ++th) {
        int valid_count = 0;

        for (int t = 0; t < t_bins; ++t) {
            if (grid[th][t]) {
                valid_count++;
            }
        }

        float width = valid_count * t_step;
        float theta = -M_PI + th * theta_step;

        theta_plot.push_back(theta * 180.0 / M_PI);
        width_plot.push_back(width);
        theta_valid_idx.push_back(th);
    }
    /*// 最长连续段长度
    // 记录每个 theta 上最长连续段的起止 t 索引
    std::vector<int> longest_run_start(theta_bins, -1);
    std::vector<int> longest_run_end(theta_bins, -1);

    // 最长连续段长度
    for (int th = 0; th < theta_bins; ++th) {
        int max_run = 0;      // 最大连续段长度
        int current_run = 0;  // 当前连续段长度

        int current_start = -1;
        int best_start = -1;
        int best_end = -1;

        for (int t = 0; t < t_bins; ++t) {
            if (grid[th][t]) {
                if (current_run == 0) {
                    current_start = t;
                }

                current_run++;

                if (current_run > max_run) {
                    max_run = current_run;
                    best_start = current_start;
                    best_end = t;
                }
            } else {
                current_run = 0;
                current_start = -1;
            }
        }

        longest_run_start[th] = best_start;
        longest_run_end[th] = best_end;

        float width = max_run * t_step;
        float theta = -M_PI + th * theta_step;

        theta_plot.push_back(theta * 180.0 / M_PI);
        width_plot.push_back(width);
        theta_valid_idx.push_back(th);
    }*/
    if (width_plot.empty()) {
        PLOGE << "宽度统计为空";
        return;
    }

    // ================= 7. 阈值 =================
    double sum = 0.0;
    int count = 0;
    for (auto w : width_plot) {
        if (w != 0) {
            sum += w;
            count++;
        }
    }
    if (count == 0) {
        PLOGE << "有效宽度为0";
        return;
    }
    double avg_width = sum / count;
    double thresh = avg_width * widthThreshRatio;

    // ================= 8. 标记删除θ =================
    std::vector<bool> theta_remove(theta_bins, false);

    for (size_t i = 0; i < width_plot.size(); ++i) {
        if (width_plot[i] < thresh) {
            int th = theta_valid_idx[i];
            theta_remove[th] = true;
        }
    }

    // ================= 9. 删除点 =================
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    filtered->reserve(cloud->size());

    for (size_t i = 0; i < cloud->size(); ++i) {
        int theta_idx = static_cast<int>((theta_vals[i] + M_PI) / theta_step);

        if (theta_idx >= 0 && theta_idx < theta_bins) {
            if (!theta_remove[theta_idx]) {
                filtered->points.push_back(cloud->points[i]);
            }
        }
    }

    cloud->swap(*filtered);
    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = true;
    // ================= 10. 画图 =================
    if (saveFlag) {
        std::vector<double> thresh_line(theta_plot.size(), thresh);

        pcl::visualization::PCLPlotter plotter;

        plotter.addPlotData(theta_plot, width_plot, "Density Width", vtkChart::LINE);
        plotter.addPlotData(theta_plot, thresh_line, "Threshold", vtkChart::LINE);

        plotter.setTitle("Density Width Distribution");
        plotter.setXTitle("Theta (deg)");
        plotter.setYTitle("Density Width");

        plotter.plot();
    }
}

bool TubeTubeFilletSeamsDet::moveAlongOrdered(const std::vector<PtTheta>& ordered, float offset, bool from_start, PtTheta& result, int& cut_idx) {
    if (ordered.size() < 2) return false;

    float remain = offset;

    if (from_start) {
        for (size_t i = 1; i < ordered.size(); ++i) {
            const auto& p0 = ordered[i - 1];
            const auto& p1 = ordered[i];

            float dx = p1.pt.x - p0.pt.x;
            float dy = p1.pt.y - p0.pt.y;
            float dz = p1.pt.z - p0.pt.z;

            float len = std::sqrt(dx * dx + dy * dy + dz * dz);

            if (remain <= len) {
                float t = (len > 1e-6f) ? remain / len : 0.0f;

                result.pt.x = p0.pt.x + t * dx;
                result.pt.y = p0.pt.y + t * dy;
                result.pt.z = p0.pt.z + t * dz;

                result.theta = p0.theta + t * (p1.theta - p0.theta);

                cut_idx = (int)i;  // 切在 i-1 和 i 之间
                return true;
            }

            remain -= len;
        }

        result = ordered.back();
        cut_idx = (int)ordered.size() - 1;
        return true;
    } else {
        for (int i = (int)ordered.size() - 1; i > 0; --i) {
            const auto& p0 = ordered[i];
            const auto& p1 = ordered[i - 1];

            float dx = p1.pt.x - p0.pt.x;
            float dy = p1.pt.y - p0.pt.y;
            float dz = p1.pt.z - p0.pt.z;

            float len = std::sqrt(dx * dx + dy * dy + dz * dz);

            if (remain <= len) {
                float t = (len > 1e-6f) ? remain / len : 0.0f;

                result.pt.x = p0.pt.x + t * dx;
                result.pt.y = p0.pt.y + t * dy;
                result.pt.z = p0.pt.z + t * dz;

                result.theta = p0.theta + t * (p1.theta - p0.theta);

                cut_idx = i - 1;  // 切在 i 和 i-1 之间
                return true;
            }

            remain -= len;
        }

        result = ordered.front();
        cut_idx = 0;
        return true;
    }
}
bool TubeTubeFilletSeamsDet::buildCylinderCache(const pcl::ModelCoefficients::Ptr& coeffs, CylinderModelCache& cache) {
    if (!coeffs || coeffs->values.size() < 7) {
        PLOGE << "buildCylinderCache: coeffs 无效";
        return false;
    }

    cache.center = Eigen::Vector3f(coeffs->values[0], coeffs->values[1], coeffs->values[2]);

    cache.axis = Eigen::Vector3f(coeffs->values[3], coeffs->values[4], coeffs->values[5]);

    if (cache.axis.norm() < 1e-6f) {
        PLOGE << "buildCylinderCache: axis 无效";
        return false;
    }
    cache.axis.normalize();

    cache.radius = coeffs->values[6];
    if (cache.radius <= 1e-6f) {
        PLOGE << "buildCylinderCache: radius 无效";
        return false;
    }

    Eigen::Vector3f ref(0.0f, 0.0f, 1.0f);
    if (std::fabs(cache.axis.dot(ref)) > 0.95f) {
        ref = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
    }

    cache.u = ref - ref.dot(cache.axis) * cache.axis;
    if (cache.u.norm() < 1e-6f) {
        PLOGE << "buildCylinderCache: u 构造失败";
        return false;
    }
    cache.u.normalize();

    cache.v = cache.axis.cross(cache.u);
    if (cache.v.norm() < 1e-6f) {
        PLOGE << "buildCylinderCache: v 构造失败";
        return false;
    }
    cache.v.normalize();

    return true;
}

bool TubeTubeFilletSeamsDet::solveSeamEndPoints() {
    if (!cylinderCloudPrimary || cylinderCloudPrimary->empty() || !cylinderCoeffsPrimary || !cylinderCoeffsSecondary) {
        PLOGE << "solveSeamEndPoints: 输入参数无效";
        return false;
    }

    std::vector<pcl::PointXYZ> filletSeamsTheoryTP;
    std::vector<pcl::PointXYZ> filletSeamsActualTP;

    filletSeamsTheoryTP.clear();
    filletSeamsActualTP.clear();
    filletSeamsTP.clear();

    // ================= 0. 统一缓存几何参数 =================
    if (!buildCylinderCache(cylinderCoeffsPrimary, seamCtx_.primary)) {
        PLOGE << "solveSeamEndPoints: 主圆柱缓存失败";
        return false;
    }

    if (!buildCylinderCache(cylinderCoeffsSecondary, seamCtx_.secondary)) {
        PLOGE << "solveSeamEndPoints: 次圆柱缓存失败";
        return false;
    }

    seamCtx_.axisDot = seamCtx_.primary.axis.dot(seamCtx_.secondary.axis);
    seamCtx_.axesNearlyParallel = (std::fabs(std::fabs(seamCtx_.axisDot) - 1.0f) < 1e-3f);

    // ================= 1. 理论点求解 =================
    {
        ScopedTimer t("solveTheorySeamEndPoints");
        if (!solveTheorySeamEndPoints(filletSeamsTheoryTP)) {
            PLOGE << "solveSeamEndPoints: 理论点求解失败";
            return false;
        }
    }

    // 理论点保存
    if (saveFlag && seamEndPoints) {
        seamEndPoints->clear();
        seamEndPoints->points.assign(filletSeamsTheoryTP.begin(), filletSeamsTheoryTP.end());
        seamEndPoints->height = 1;
        seamEndPoints->width = static_cast<uint32_t>(seamEndPoints->size());

        pcl::io::savePCDFileBinary("./data/seamDetWithPointCloud/tubeTubeFilletSeamsDet/seamTheoryEndPoints_" + std::to_string(areaNum) + ".pcd",
                                   *seamEndPoints);
    }

    // ================= 2. 实际点求解接口 =================
    {
        ScopedTimer t("solveActualSeamPoints");
        if (!solveActualSeamPoints(filletSeamsTheoryTP, filletSeamsActualTP)) {
            PLOGE << "solveSeamEndPoints: 实际点求解失败";
            return false;
        }
    }

    // ================= 3. 最终输出 =================
    filletSeamsTP = filletSeamsActualTP;

    if (saveFlag && seamEndPoints) {
        seamEndPoints->clear();
        seamEndPoints->points.assign(filletSeamsTP.begin(), filletSeamsTP.end());
        seamEndPoints->height = 1;
        seamEndPoints->width = static_cast<uint32_t>(seamEndPoints->size());

        pcl::io::savePCDFileBinary("./data/seamDetWithPointCloud/tubeTubeFilletSeamsDet/seamEndPoints_" + std::to_string(areaNum) + ".pcd",
                                   *seamEndPoints);
    }

    return true;
}
bool TubeTubeFilletSeamsDet::solveTheorySeamEndPoints(std::vector<pcl::PointXYZ>& filletSeamsTheoryTP) {
    filletSeamsTheoryTP.clear();

    if (!axisRangeCloud || axisRangeCloud->empty()) {
        PLOGE << "solveTheorySeamEndPoints: axisRangeCloud 为空";
        return false;
    }

    const auto& cyl1 = seamCtx_.primary;
    const auto& cyl2 = seamCtx_.secondary;

    const Eigen::Vector3f& C1 = cyl1.center;
    const Eigen::Vector3f& a1 = cyl1.axis;
    const float R1 = cyl1.radius;
    const Eigen::Vector3f& u = cyl1.u;
    const Eigen::Vector3f& v = cyl1.v;

    const Eigen::Vector3f& C2 = cyl2.center;
    const Eigen::Vector3f& a2 = cyl2.axis;
    const float R2 = cyl2.radius;

    const float EPS = 1e-6f;

    // ================= 1. 把 axisRangeCloud 上的点映射到“双圆柱理论交线” =================
    std::vector<PtTheta> pts;
    pts.reserve(axisRangeCloud->size());

    for (const auto& p : axisRangeCloud->points) {
        Eigen::Vector3f P(p.x, p.y, p.z);

        // 先求该点在主圆柱横截面上的角度 theta
        float t0 = (P - C1).dot(a1);
        Eigen::Vector3f foot1 = C1 + t0 * a1;
        Eigen::Vector3f dvec = P - foot1;

        float x = dvec.dot(u);
        float y = dvec.dot(v);
        float norm_xy = std::sqrt(x * x + y * y);
        if (norm_xy < EPS) {
            continue;
        }

        float theta = std::atan2(y, x);

        // 主圆柱上固定 theta 的母线：
        // X(t) = base + t * a1
        Eigen::Vector3f radial = R1 * (std::cos(theta) * u + std::sin(theta) * v);
        Eigen::Vector3f base = C1 + radial;

        // 令该点也满足第二个圆柱方程：
        // dist^2(X(t), axis2) = R2^2
        Eigen::Vector3f m = base - C2;
        float k = a1.dot(a2);

        float alpha = 1.0f - k * k;
        float beta = 2.0f * (m.dot(a1) - m.dot(a2) * k);
        float gamma = m.squaredNorm() - std::pow(m.dot(a2), 2.0f) - R2 * R2;

        std::vector<float> roots;

        if (std::fabs(alpha) < EPS) {
            // 两圆柱轴近似平行，退化
            if (std::fabs(beta) < EPS) {
                continue;
            }
            roots.push_back(-gamma / beta);
        } else {
            float delta = beta * beta - 4.0f * alpha * gamma;
            if (delta < 0.0f) {
                continue;
            }

            delta = std::max(delta, 0.0f);
            float sqrtDelta = std::sqrt(delta);

            float t1 = (-beta - sqrtDelta) / (2.0f * alpha);
            float t2 = (-beta + sqrtDelta) / (2.0f * alpha);
            roots.push_back(t1);

            if (std::fabs(t2 - t1) > 1e-5f) {
                roots.push_back(t2);
            }
        }

        if (roots.empty()) {
            continue;
        }

        // 有两个根时，选离当前点 P 最近的那个理论点
        float bestDist2 = std::numeric_limits<float>::max();
        Eigen::Vector3f bestX = Eigen::Vector3f::Zero();

        for (float t : roots) {
            Eigen::Vector3f X = base + t * a1;
            float d2 = (X - P).squaredNorm();
            if (d2 < bestDist2) {
                bestDist2 = d2;
                bestX = X;
            }
        }

        PtTheta item;
        item.theta = theta;
        item.pt.x = bestX.x();
        item.pt.y = bestX.y();
        item.pt.z = bestX.z();
        pts.push_back(item);
    }

    if (pts.size() < 10) {
        PLOGE << "solveTheorySeamEndPoints: 理论交线点过少";
        return false;
    }

    // ================= 2. 按 theta 排序 =================
    std::sort(pts.begin(), pts.end(), [](const PtTheta& a, const PtTheta& b) { return a.theta < b.theta; });

    if (pts.empty()) {
        PLOGE << "solveTheorySeamEndPoints: pts 为空";
        return false;
    }

    // ================= 3. 处理 theta 跨 ±pi 断裂 =================
    std::vector<PtTheta> ordered = pts;

    float max_gap = 0.0f;
    int split_idx = 0;
    for (int i = 1; i < static_cast<int>(pts.size()); ++i) {
        float gap = pts[i].theta - pts[i - 1].theta;
        if (gap > max_gap) {
            max_gap = gap;
            split_idx = i;
        }
    }

    if (max_gap > static_cast<float>(M_PI)) {
        ordered.clear();
        ordered.reserve(pts.size());

        for (int i = split_idx; i < static_cast<int>(pts.size()); ++i) {
            ordered.push_back(pts[i]);
        }
        for (int i = 0; i < split_idx; ++i) {
            ordered.push_back(pts[i]);
        }
    }

    if (ordered.size() < 2) {
        PLOGE << "solveTheorySeamEndPoints: ordered 点数不足";
        return false;
    }

    // ================= 4. 保证 x 小的一端作为起点 =================
    if (ordered.front().pt.x > ordered.back().pt.x) {
        std::reverse(ordered.begin(), ordered.end());
    }

    // ================= 5. 根据 offset 收缩首尾 =================
    PtTheta new_start, new_end;
    // TODO 管管角接收缩
    // float start_offset = SettingPara::getInstance().TubeTubeFilletStartOffset;
    // float end_offset = SettingPara::getInstance().TubeTubeFilletEndOffset;
    float start_offset = 0.0f;
    float end_offset = 0.0f;
    if (start_offset < 0.0f) start_offset = 0.0f;
    if (end_offset < 0.0f) end_offset = 0.0f;

    float total_len_check = 0.0f;
    for (int i = 1; i < static_cast<int>(ordered.size()); ++i) {
        float dx = ordered[i].pt.x - ordered[i - 1].pt.x;
        float dy = ordered[i].pt.y - ordered[i - 1].pt.y;
        float dz = ordered[i].pt.z - ordered[i - 1].pt.z;
        total_len_check += std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    if (total_len_check < 1e-6f) {
        PLOGE << "solveTheorySeamEndPoints: 理论交线总长度过小";
        return false;
    }

    if (start_offset + end_offset >= total_len_check) {
        PLOGE << "solveTheorySeamEndPoints: 收缩过大，超过焊缝长度";
        return false;
    }

    int start_idx = 0;
    int end_idx = static_cast<int>(ordered.size()) - 1;

    moveAlongOrdered(ordered, start_offset, true, new_start, start_idx);
    moveAlongOrdered(ordered, end_offset, false, new_end, end_idx);

    std::vector<PtTheta> trimmed;
    trimmed.reserve(ordered.size());

    trimmed.push_back(new_start);

    for (int i = start_idx; i <= end_idx; ++i) {
        trimmed.push_back(ordered[i]);
    }

    trimmed.push_back(new_end);

    ordered.swap(trimmed);

    if (ordered.size() < 2) {
        PLOGE << "solveTheorySeamEndPoints: 裁剪后点数不足";
        return false;
    }

    // 再次确保 x 小的是起点
    if (ordered.front().pt.x > ordered.back().pt.x) {
        std::reverse(ordered.begin(), ordered.end());
    }

    // ================= 6. 计算累计弧长 =================
    const int total = static_cast<int>(ordered.size());
    std::vector<float> arc_len(total, 0.0f);

    for (int i = 1; i < total; ++i) {
        const auto& p0 = ordered[i - 1].pt;
        const auto& p1 = ordered[i].pt;

        float dx = p1.x - p0.x;
        float dy = p1.y - p0.y;
        float dz = p1.z - p0.z;

        float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        arc_len[i] = arc_len[i - 1] + dist;
    }

    float total_len = arc_len.back();
    if (total_len < 1e-6f) {
        PLOGE << "solveTheorySeamEndPoints: 弧长过小";
        return false;
    }

    // ================= 7. 均匀采样 =================
    int N = sample_num;
    if (N < 2) {
        PLOGE << "solveTheorySeamEndPoints: sample_num < 2";
        return false;
    }

    filletSeamsTheoryTP.reserve(N);

    float step = total_len / static_cast<float>(N - 1);

    filletSeamsTheoryTP.push_back(ordered.front().pt);

    int curr_idx = 1;
    for (int i = 1; i < N - 1; ++i) {
        float target_len = i * step;

        while (curr_idx < total && arc_len[curr_idx] < target_len) {
            curr_idx++;
        }

        if (curr_idx >= total) {
            filletSeamsTheoryTP.push_back(ordered.back().pt);
            continue;
        }

        int idx1 = curr_idx - 1;
        int idx2 = curr_idx;

        const auto& p1 = ordered[idx1].pt;
        const auto& p2 = ordered[idx2].pt;

        float len1 = arc_len[idx1];
        float len2 = arc_len[idx2];

        float t = 0.0f;
        if (len2 > len1) {
            t = (target_len - len1) / (len2 - len1);
        }

        pcl::PointXYZ interp_pt;
        interp_pt.x = p1.x + t * (p2.x - p1.x);
        interp_pt.y = p1.y + t * (p2.y - p1.y);
        interp_pt.z = p1.z + t * (p2.z - p1.z);

        filletSeamsTheoryTP.push_back(interp_pt);
    }

    filletSeamsTheoryTP.push_back(ordered.back().pt);

    return true;
}

bool TubeTubeFilletSeamsDet::solveActualSeamPoints(const std::vector<pcl::PointXYZ>& filletSeamsTheoryTP,
                                                   std::vector<pcl::PointXYZ>& filletSeamsActualTP) {
    // ================= 后续你自己接这里 =================
    // 这里先留接口，暂时直接复制理论点，确保流程能通
    filletSeamsActualTP = filletSeamsTheoryTP;
    return true;
}
bool TubeTubeFilletSeamsDet::extractLocalVoxelRegionAroundSeamSamples(const pcl::PointCloud<pcl::PointXYZ>::Ptr& srcCloud,
                                                                      const std::vector<pcl::PointXYZ>& seamSamples,
                                                                      pcl::PointCloud<pcl::PointXYZI>::Ptr& outCloud) {
    ScopedTimer t("extractLocalVoxelRegionAroundSeamSamples");
    if (!srcCloud || srcCloud->empty()) {
        PLOGE << "srcCloud 为空";
        return false;
    }

    if (seamSamples.empty()) {
        PLOGE << "seamSamples 为空";
        return false;
    }

    if (!cylinderCoeffsPrimary || cylinderCoeffsPrimary->values.size() < 7) {
        PLOGE << "cylinderCoeffsWithWeldSeam 无效";
        return false;
    }

    Eigen::Vector3f axis(cylinderCoeffsPrimary->values[3], cylinderCoeffsPrimary->values[4], cylinderCoeffsPrimary->values[5]);
    if (axis.norm() < 1e-6f) {
        PLOGE << "axis 无效";
        return false;
    }
    axis.normalize();

    Eigen::Vector3f cylC(cylinderCoeffsPrimary->values[0], cylinderCoeffsPrimary->values[1], cylinderCoeffsPrimary->values[2]);
    const float cylRadius = cylinderCoeffsPrimary->values[6];

    // 最终焊缝点快速欧式聚类参数
    const float finalClusterTolerance = 1.5f;

    // 圆柱表面筛选容差
    const float cylinderSurfaceTol = 1.0f;

    if (!outCloud) {
        outCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
    outCloud->clear();

    // 保存每个 seam sample 对应的局部区域
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> finalSeamClouds(seamSamples.size());

    // ================= 小工具：保存 PointXYZI 强度点云 =================
    auto saveIntensityCloud = [](const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std::string& savePath) -> bool {
        if (!cloud || cloud->empty()) {
            PLOGE << "saveIntensityCloud: 点云为空, " << savePath;
            return false;
        }

        int ret = pcl::io::savePCDFileBinary(savePath, *cloud);
        if (ret != 0) {
            PLOGE << "saveIntensityCloud: 保存失败 " << savePath;
            return false;
        }

        float minI = std::numeric_limits<float>::max();
        float maxI = -std::numeric_limits<float>::max();
        for (const auto& p : cloud->points) {
            minI = std::min(minI, p.intensity);
            maxI = std::max(maxI, p.intensity);
        }

        PLOGD << "saveIntensityCloud: 已保存 " << savePath << ", 点数 = " << cloud->size() << ", intensity范围 = [" << minI << ", " << maxI << "]";
        return true;
    };
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());
    kdtree->setInputCloud(srcCloud);
    const float axialHalfLen = 10.0f;  // 轴向 ±10mm
    const float radialRadius = 10.0f;  // 径向 10mm
    const float radialRadius2 = radialRadius * radialRadius;
    const float localSearchRadius = std::sqrt(axialHalfLen * axialHalfLen + radialRadius * radialRadius);
    const float queryRadius = 10.0f;
    const float queryRadius2 = queryRadius * queryRadius;
    const int maxThreads = std::max(1, omp_get_max_threads());
    const bool parallelBySeam = static_cast<int>(seamSamples.size()) >= maxThreads * 2;
#pragma omp parallel for schedule(dynamic) if (parallelBySeam)
    for (int seamdex = 0; seamdex < static_cast<int>(seamSamples.size()); ++seamdex) {
        const auto& seamPt = seamSamples[seamdex];
        Eigen::Vector3f S(seamPt.x, seamPt.y, seamPt.z);

        std::vector<int> candidateIdx;
        std::vector<float> candidateDist2;
        kdtree->radiusSearch(seamPt, localSearchRadius, candidateIdx, candidateDist2);

        pcl::PointCloud<pcl::PointXYZ>::Ptr localCloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr queryCloud(new pcl::PointCloud<pcl::PointXYZ>());

        localCloud->reserve(candidateIdx.size());
        queryCloud->reserve(candidateIdx.size() / 4 + 8);

        for (int idx : candidateIdx) {
            const auto& p = srcCloud->points[idx];
            Eigen::Vector3f P(p.x, p.y, p.z);

            Eigen::Vector3f d = P - S;

            float axialDist = d.dot(axis);
            if (std::abs(axialDist) > axialHalfLen) continue;

            Eigen::Vector3f radialVec = d - axialDist * axis;
            if (radialVec.squaredNorm() > radialRadius2) continue;

            localCloud->points.push_back(p);

            float dist2 = d.squaredNorm();
            if (dist2 <= queryRadius2) {
                queryCloud->points.push_back(p);
            }
        }

        localCloud->width = static_cast<uint32_t>(localCloud->size());
        localCloud->height = 1;
        localCloud->is_dense = false;

        queryCloud->width = static_cast<uint32_t>(queryCloud->size());
        queryCloud->height = 1;
        queryCloud->is_dense = false;
        if (queryCloud->empty()) {
            PLOGW << "seamdex = " << seamdex << " 的 queryCloud 为空";
            continue;
        }

        if (/*seamdex == 5 && */ !localCloud->empty()) {
            ScopedTimer t(std::string("seamdex" + std::to_string(seamdex)));

            SeamConcavityExtractor extractor;
            extractor.setAreadex(seamdex);
            extractor.setInputCloud(localCloud);
            extractor.setQueryPoints(queryCloud);
            extractor.setNeighborRadius(2.0f);
            extractor.setDebug(false);

            bool ok = extractor.run();

            if (ok) {
                const auto& featureResults = extractor.getAllFeatureResults();

                // 1. 提取最终焊缝点强度图
                pcl::PointCloud<pcl::PointXYZI>::Ptr finalRawIntensityCloud(new pcl::PointCloud<pcl::PointXYZI>());
                finalRawIntensityCloud->reserve(featureResults.size());

                for (const auto& r : featureResults) {
                    if (!r.valid) continue;
                    if (!r.isFinalSeamPoint) continue;
                    if (!std::isfinite(r.absConcavityScore)) continue;

                    pcl::PointXYZI p;
                    p.x = r.queryPoint.x;
                    p.y = r.queryPoint.y;
                    p.z = r.queryPoint.z;
                    p.intensity = r.absConcavityScore;
                    finalRawIntensityCloud->points.push_back(p);
                }

                finalRawIntensityCloud->width = static_cast<uint32_t>(finalRawIntensityCloud->size());
                finalRawIntensityCloud->height = 1;
                finalRawIntensityCloud->is_dense = false;

                if (!finalRawIntensityCloud->empty()) {
                    // std::string rawPath =
                    //     "./data/seamDetWithPointCloud/tubeTubeFilletSeamsDet/final_raw_intensity_" + std::to_string(seamdex) + ".pcd";
                    // saveIntensityCloud(finalRawIntensityCloud, rawPath);

                    // 2. 转成 XYZ 做快速欧式聚类
                    pcl::PointCloud<pcl::PointXYZ>::Ptr finalRawXYZ(new pcl::PointCloud<pcl::PointXYZ>());
                    finalRawXYZ->reserve(finalRawIntensityCloud->size());

                    for (const auto& p : finalRawIntensityCloud->points) {
                        pcl::PointXYZ q;
                        q.x = p.x;
                        q.y = p.y;
                        q.z = p.z;
                        finalRawXYZ->points.push_back(q);
                    }
                    finalRawXYZ->width = static_cast<uint32_t>(finalRawXYZ->size());
                    finalRawXYZ->height = 1;
                    finalRawXYZ->is_dense = false;

                    MyToolFunc::myFastMaxCluster(finalRawXYZ, finalClusterTolerance);

                    // 3. 聚类后的 XYZ 重新映射回 XYZI
                    pcl::PointCloud<pcl::PointXYZI>::Ptr clusteredIntensityCloud(new pcl::PointCloud<pcl::PointXYZI>());
                    clusteredIntensityCloud->reserve(finalRawXYZ->size());

                    std::vector<bool> used(finalRawIntensityCloud->size(), false);
                    const float matchTol2 = 1e-8f;

                    for (const auto& q : finalRawXYZ->points) {
                        int matchedIdx = -1;

                        for (size_t k = 0; k < finalRawIntensityCloud->size(); ++k) {
                            if (used[k]) continue;

                            const auto& p = finalRawIntensityCloud->points[k];
                            float dx = p.x - q.x;
                            float dy = p.y - q.y;
                            float dz = p.z - q.z;
                            float dist2 = dx * dx + dy * dy + dz * dz;

                            if (dist2 <= matchTol2) {
                                matchedIdx = static_cast<int>(k);
                                break;
                            }
                        }

                        if (matchedIdx >= 0) {
                            clusteredIntensityCloud->points.push_back(finalRawIntensityCloud->points[matchedIdx]);
                            used[matchedIdx] = true;
                        }
                    }

                    clusteredIntensityCloud->width = static_cast<uint32_t>(clusteredIntensityCloud->size());
                    clusteredIntensityCloud->height = 1;
                    clusteredIntensityCloud->is_dense = false;

                    if (!clusteredIntensityCloud->empty()) {
                        // std::string clusteredPath =
                        //     "./data/seamDetWithPointCloud/tubeTubeFilletSeamsDet/final_clustered_intensity_" + std::to_string(seamdex) +
                        //     ".pcd";
                        // saveIntensityCloud(clusteredIntensityCloud, clusteredPath);

                        // 4. 筛选圆柱表面附近点，结果仍然保持 XYZI
                        pcl::PointCloud<pcl::PointXYZI>::Ptr onCylinderIntensityCloud(new pcl::PointCloud<pcl::PointXYZI>());
                        onCylinderIntensityCloud->reserve(clusteredIntensityCloud->size());

                        for (const auto& p : clusteredIntensityCloud->points) {
                            Eigen::Vector3f P(p.x, p.y, p.z);
                            Eigen::Vector3f CP = P - cylC;

                            float axial = CP.dot(axis);
                            Eigen::Vector3f radialVec2 = CP - axial * axis;
                            float radialDist = radialVec2.norm();

                            float err = std::fabs(radialDist - cylRadius);
                            if (err <= cylinderSurfaceTol) {
                                onCylinderIntensityCloud->points.push_back(p);
                            }
                        }

                        onCylinderIntensityCloud->width = static_cast<uint32_t>(onCylinderIntensityCloud->size());
                        onCylinderIntensityCloud->height = 1;
                        onCylinderIntensityCloud->is_dense = false;

                        if (!onCylinderIntensityCloud->empty()) {
                            // std::string cylPath = "./data/seamDetWithPointCloud/tubeTubeFilletSeamsDet/final_on_cylinder_intensity_" +
                            //                       std::to_string(seamdex) + ".pcd";
                            // saveIntensityCloud(onCylinderIntensityCloud, cylPath);

                            // 这里直接保存 XYZI，不要再转 XYZ
                            finalSeamClouds[seamdex] = onCylinderIntensityCloud;
                        }

                        PLOGD << "seamdex = " << seamdex << ", finalRaw = " << finalRawIntensityCloud->size()
                              << ", clustered = " << clusteredIntensityCloud->size() << ", onCylinder = " << onCylinderIntensityCloud->size();
                    }
                }
            }
        }
    }

    // 串行合并
    size_t totalSize = 0;
    for (const auto& c : finalSeamClouds) {
        if (c) totalSize += c->size();
    }

    outCloud->clear();
    outCloud->points.reserve(totalSize);

    for (const auto& c : finalSeamClouds) {
        if (!c || c->empty()) continue;

        outCloud->points.insert(outCloud->points.end(), c->points.begin(), c->points.end());
    }

    outCloud->width = static_cast<uint32_t>(outCloud->size());
    outCloud->height = 1;
    outCloud->is_dense = false;

    PLOGD << "局部点数: " << outCloud->size();

    return !outCloud->empty();
}
void TubeTubeFilletSeamsDet::removePointsNearPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::ModelCoefficients::Ptr& planeCoeffs,
                                                   float distThresh) {
    if (!cloud || cloud->empty()) {
        PLOGW << "removePointsNearPlane: cloud 为空";
        return;
    }

    if (!planeCoeffs || planeCoeffs->values.size() < 4) {
        PLOGE << "removePointsNearPlane: planeCoeffs 无效";
        return;
    }

    // 平面参数
    float A = planeCoeffs->values[0];
    float B = planeCoeffs->values[1];
    float C = planeCoeffs->values[2];
    float D = planeCoeffs->values[3];

    float norm = std::sqrt(A * A + B * B + C * C);
    if (norm < 1e-6f) {
        PLOGE << "removePointsNearPlane: 法向无效";
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    filtered->reserve(cloud->size());

    for (const auto& p : cloud->points) {
        float dist = std::fabs(A * p.x + B * p.y + C * p.z + D) / norm;

        // 只保留远离平面的点
        if (dist > distThresh) {
            filtered->points.push_back(p);
        }
    }

    filtered->width = static_cast<uint32_t>(filtered->size());
    filtered->height = 1;
    filtered->is_dense = false;

    PLOGD << "removePointsNearPlane: 原始点数 = " << cloud->size() << ", 过滤后 = " << filtered->size();

    cloud.swap(filtered);
}
bool TubeTubeFilletSeamsDet::refineTheoryPointsToActualPoints(const std::vector<pcl::PointXYZ>& theoryPts,
                                                              const pcl::PointCloud<pcl::PointXYZ>::Ptr& refCloud,
                                                              const pcl::ModelCoefficients::Ptr& planeCoeffs, const Eigen::Vector3f& refDirInput,
                                                              std::vector<pcl::PointXYZ>& actualPts) {
    // #define DEBUG_REFINE_BEFORE_PROJECTION
    if (!refCloud || refCloud->empty()) {
        PLOGE << "refineTheoryPointsToActualPoints: refCloud 为空";
        return false;
    }

    if (theoryPts.empty()) {
        PLOGE << "refineTheoryPointsToActualPoints: theoryPts 为空";
        return false;
    }

    if (!planeCoeffs || planeCoeffs->values.size() < 4) {
        PLOGE << "refineTheoryPointsToActualPoints: planeCoeffs 无效";
        return false;
    }

    Eigen::Vector3f n(planeCoeffs->values[0], planeCoeffs->values[1], planeCoeffs->values[2]);
    float d = planeCoeffs->values[3];

    float nNorm = n.norm();
    if (nNorm < 1e-6f) {
        PLOGE << "平面法向无效";
        return false;
    }
    n.normalize();

    Eigen::Vector3f refDir = refDirInput;
    if (refDir.norm() < 1e-6f) {
        PLOGE << "refDir 无效";
        return false;
    }
    refDir.normalize();

    actualPts.clear();
    actualPts.reserve(theoryPts.size());

#ifdef DEBUG_REFINE_BEFORE_PROJECTION
    pcl::PointCloud<pcl::PointXYZ>::Ptr beforeProjectionCloud(new pcl::PointCloud<pcl::PointXYZ>());
    beforeProjectionCloud->reserve(theoryPts.size());
#endif

    for (const auto& theoryPt : theoryPts) {
        Eigen::Vector3f T(theoryPt.x, theoryPt.y, theoryPt.z);

        bool found = false;
        float bestScore = std::numeric_limits<float>::max();
        Eigen::Vector3f bestRealPoint = T;

        for (const auto& p : refCloud->points) {
            Eigen::Vector3f P(p.x, p.y, p.z);
            Eigen::Vector3f diff = P - T;

            float dirOffset = diff.dot(refDir);
            if (std::fabs(dirOffset) > maxNormalOffset) continue;

            Eigen::Vector3f perpVec = diff - dirOffset * refDir;
            float perpDist = perpVec.norm();
            if (perpDist > maxTangentialDist) continue;

            float euclidDist = diff.norm();
            if (euclidDist > maxEuclidDist) continue;

            float score = perpDist + 0.2f * std::fabs(dirOffset);

            if (score < bestScore) {
                bestScore = score;
                bestRealPoint = P;
                found = true;
            }
        }

        Eigen::Vector3f pointBeforeProjection;

        if (!found) {
            // fallback：理论点
            pointBeforeProjection = T;
        } else {
            // 找到真实点
            pointBeforeProjection = bestRealPoint;
        }

#ifdef DEBUG_REFINE_BEFORE_PROJECTION
        {
            pcl::PointXYZ debugPt;
            debugPt.x = pointBeforeProjection.x();
            debugPt.y = pointBeforeProjection.y();
            debugPt.z = pointBeforeProjection.z();
            beforeProjectionCloud->push_back(debugPt);
        }
#endif

        // ===== 投影回平面 =====
        float signedDist = n.dot(pointBeforeProjection) + d;
        Eigen::Vector3f proj = pointBeforeProjection - signedDist * n;

        pcl::PointXYZ actualPt;
        actualPt.x = proj.x();
        actualPt.y = proj.y();
        actualPt.z = proj.z();
        actualPts.push_back(actualPt);
    }

#ifdef DEBUG_REFINE_BEFORE_PROJECTION
    if (!beforeProjectionCloud->empty()) {
        beforeProjectionCloud->width = static_cast<uint32_t>(beforeProjectionCloud->size());
        beforeProjectionCloud->height = 1;
        beforeProjectionCloud->is_dense = false;

        std::string path = "./data/seamDetWithPointCloud/tubeTubeFilletSeamsDet/beforeProjectionCloud.pcd";

        pcl::io::savePCDFileBinary(path, *beforeProjectionCloud);

        PLOGD << "保存投影前点云: " << path << ", 点数: " << beforeProjectionCloud->size();
    }
#endif

    return !actualPts.empty();
}
