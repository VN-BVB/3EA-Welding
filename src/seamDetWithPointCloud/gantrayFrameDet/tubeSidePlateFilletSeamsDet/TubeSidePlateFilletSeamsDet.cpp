#include "TubeSidePlateFilletSeamsDet.h"

#include "seamDetWithPointCloud/QhullLock.h"
#include "settingPara/SettingPara.h"
#include "utils/pointCloud/PointCloudFunc.h"
TubeSidePlateFilletSeamsDet::TubeSidePlateFilletSeamsDet(QObject* parent) : AbstractSeamDet{parent} {}

std::vector<std::shared_ptr<WeldSeamInfo>> TubeSidePlateFilletSeamsDet::solveSeamsEndPoints(std::vector<std::shared_ptr<WeldSeamInfo>> seamsInfo) {
    tempWeldSeamsInfo = seamsInfo;
    for (size_t i = 0; i < tempWeldSeamsInfo.size(); i++) {
        singleSeamReinitialize();

        cloudInWeldArea = tempWeldSeamsInfo[i]->weldAreaPointCloudInCamera;  // 获取焊缝区域点云
        if (cloudInWeldArea->size() == 0) {
            PLOGE << "焊缝区域为0";
            detectSuccFlag = false;
            continue;
        }

        MyToolFunc::myFastMaxCluster(cloudInWeldArea, Max_cluster_radius);
        if (saveFlag) {
            cloudInWeldArea->height = 1;
            cloudInWeldArea->width = static_cast<uint32_t>(cloudInWeldArea->size());
            pcl::io::savePCDFile("./data/seamDetWithPointCloud/tubeSidePlateFilletSeamsDet/after_cluster.pcd", *cloudInWeldArea);
        }

        ransacPlane(cloudInWeldArea, cloudPlaneInWeldAreaWithSeam, cloudNoPlaneInWeldArea);
        if (!cloudPlaneInWeldAreaWithSeam || cloudPlaneInWeldAreaWithSeam->empty()) {
            PLOGE << "Ransac_plane: 平面点云为空";
            detectSuccFlag = false;
            continue;
        }
        cloudPlaneInWeldAreaWithSeam->height = 1;
        cloudPlaneInWeldAreaWithSeam->width = static_cast<uint32_t>(cloudPlaneInWeldAreaWithSeam->size());
        if (saveFlag) {
            pcl::io::savePCDFile("./data/seamDetWithPointCloud/tubeSidePlateFilletSeamsDet/cloudPlaneInWeldArea.pcd", *cloudPlaneInWeldAreaWithSeam);
            cloudNoPlaneInWeldArea->height = 1;
            cloudNoPlaneInWeldArea->width = static_cast<uint32_t>(cloudNoPlaneInWeldArea->size());
            pcl::io::savePCDFile("./data/seamDetWithPointCloud/tubeSidePlateFilletSeamsDet/cloudNoPlaneInWeldArea.pcd", *cloudNoPlaneInWeldArea);
        }

        ransacCylinder(cloudNoPlaneInWeldArea, cloudCylinderInWeldArea);
        if (!cloudCylinderInWeldArea || cloudCylinderInWeldArea->empty()) {
            PLOGE << "Ransac_cylinder: 圆柱点云为空";
            detectSuccFlag = false;
            continue;
        }

        if (saveFlag) {
            cloudCylinderInWeldArea->height = 1;
            cloudCylinderInWeldArea->width = static_cast<uint32_t>(cloudCylinderInWeldArea->size());
            pcl::io::savePCDFile("./data/seamDetWithPointCloud/tubeSidePlateFilletSeamsDet/cloudCylinderInWeldArea.pcd", *cloudCylinderInWeldArea);
        }
        removeCylinderPoints(cloudPlaneInWeldAreaWithSeam);
        if (saveFlag) {
            cloudPlaneInWeldAreaWithSeam->height = 1;
            cloudPlaneInWeldAreaWithSeam->width = static_cast<uint32_t>(cloudPlaneInWeldAreaWithSeam->size());
            pcl::io::savePCDFile("./data/seamDetWithPointCloud/tubeSidePlateFilletSeamsDet/cloudPlaneInWeldArea2.pcd", *cloudPlaneInWeldAreaWithSeam);
        }
        // 统计滤波
        statisticFilter(cloudPlaneInWeldAreaWithSeam);
        if (saveFlag) {
            cloudPlaneInWeldAreaWithSeam->height = 1;
            cloudPlaneInWeldAreaWithSeam->width = static_cast<uint32_t>(cloudPlaneInWeldAreaWithSeam->size());
            pcl::io::savePCDFile("./data/seamDetWithPointCloud/tubeSidePlateFilletSeamsDet/cloudPlaneInWeldArea3.pcd", *cloudPlaneInWeldAreaWithSeam);
        }
        MyToolFunc::myFastMaxCluster(cloudPlaneInWeldAreaWithSeam, 2);
        if (saveFlag) {
            cloudPlaneInWeldAreaWithSeam->height = 1;
            cloudPlaneInWeldAreaWithSeam->width = static_cast<uint32_t>(cloudPlaneInWeldAreaWithSeam->size());
            pcl::io::savePCDFile("./data/seamDetWithPointCloud/tubeSidePlateFilletSeamsDet/cloudPlaneInWeldArea4.pcd", *cloudPlaneInWeldAreaWithSeam);
        }
        detectSuccFlag = solveSeamEndPoints();
        // 保存本次检测到的信息
        tempWeldSeamsInfo[i]->detectSuccFlag = detectSuccFlag;
        if (detectSuccFlag) {
            tempWeldSeamsInfo[i]->weldEndPointsInCamera.reset(new std::vector<pcl::PointXYZ>(std::move(filletSeamsTSP)));  // 检测结果
            tempWeldSeamsInfo[i]->weldEndPointsInRobot.reset(new std::vector<pcl::PointXYZ>());
            tempWeldSeamsInfo[i]->weldCoeff = planeCoeffsWithWeldSeam;
            tempWeldSeamsInfo[i]->otherSurface.emplace_back(cylinderCoeffsInWeldArea);
            tempWeldSeamsInfo[i]->weldType = TubeSide_Plate_F_H;
            tempWeldSeamsInfo[i]->seamsLineToVal = lineCoeffsWithWeldSeam2Val;
        }
    }
    return tempWeldSeamsInfo;
}
// 单条焊缝检测前，变量重新初始化
void TubeSidePlateFilletSeamsDet::singleSeamReinitialize() {
    cloudInWeldArea.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloudPlaneInWeldAreaWithSeam.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloudNoPlaneInWeldArea.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloudCylinderInWeldArea.reset(new pcl::PointCloud<pcl::PointXYZ>);
    seamEndPoints.reset(new pcl::PointCloud<pcl::PointXYZ>);
    axisRangeCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    planeCoeffsWithWeldSeam.reset(new pcl::ModelCoefficients);
    cylinderCoeffsInWeldArea.reset(new pcl::ModelCoefficients);
    lineCoeffsWithWeldSeam2Val.reset(new pcl::ModelCoefficients);

    filletSeamsTSP.clear();
    saveFlag = SettingPara::getInstance().bool_save_model;
    // saveFlag = true;
    detectSuccFlag = false;
}
void TubeSidePlateFilletSeamsDet::statisticFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input_cloud);       // 设置待滤波的点云
    sor.setMeanK(Statistic_NeighPoints);  // 设置在进行统计时考虑查询点邻近点数
    sor.setStddevMulThresh(Statistic_sigma);  // 设置判断是否为离群点的阈值，里边的数字表示标准差的倍数，1个标准差以上就是离群点。
    sor.filter(*input_cloud);  // 存储内点
}
// Ransac拟合平面，并输出平面的内点集合
void TubeSidePlateFilletSeamsDet::ransacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_plane,
                                              pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_noplane) {
    // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);              // 开启最小二乘系数优化
    seg.setModelType(pcl::SACMODEL_PLANE);          // 设置模型类型
    seg.setMethodType(pcl::SAC_RANSAC);             // 设置算法类型
    seg.setMaxIterations(Ransac_plane_Iterations);  // 设置最大迭代次数
    seg.setDistanceThreshold(Ransac_plane_Dth);     // 设置距离阈值。
    seg.setInputCloud(input_cloud);                 // 输入点云
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.segment(*inliers, *planeCoeffsWithWeldSeam);  // 实现分割，并存储分割结果到点集合inliers及存储平面模型系数coefficients
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*output_cloud_plane);
    extract.setNegative(true);
    extract.filter(*output_cloud_noplane);
}
// Ransac拟合平面，并输出平面的内点集合
void TubeSidePlateFilletSeamsDet::ransacCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud) {
    if (!input_cloud || input_cloud->empty() || !output_cloud) {
        PLOGE << "Ransac_cylinder: 输入参数无效";
        return;
    }

    // ---------- 1 计算法向量 (OMP并行) ----------
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads(std::thread::hardware_concurrency() / 2);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    ne.setSearchMethod(tree);
    ne.setInputCloud(input_cloud);
    ne.setKSearch(100);
    ne.compute(*normals);
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    // ---------- 2 RANSAC拟合圆柱 ----------
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);

    seg.setNormalDistanceWeight(0.2);                  // 法向量权重
    seg.setMaxIterations(Ransac_cylinder_Iterations);  // 最大迭代次数
    seg.setDistanceThreshold(Ransac_cylinder_Dth);     // 点到圆柱距离阈值
    seg.setRadiusLimits(20, 150);                      // 圆柱半径范围

    seg.setInputCloud(input_cloud);
    seg.setInputNormals(normals);

    seg.segment(*inliers_cylinder, *cylinderCoeffsInWeldArea);

    if (inliers_cylinder->indices.size() == 0) {
        PLOGD << "圆柱面提取失败";
        detectSuccFlag = false;
        return;
    }

    // ---------- 3 提取圆柱点 ----------
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud(input_cloud);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);  // 提取内点
    extract.filter(*output_cloud);

    // ---------- 4 输出圆柱参数 ----------
    std::cout << u8"cylinderCoeffsInWeldArea: ";
    for (size_t i = 0; i < cylinderCoeffsInWeldArea->values.size(); ++i) std::cout << cylinderCoeffsInWeldArea->values[i] << " ";
    std::cout << std::endl;
}

void TubeSidePlateFilletSeamsDet::removeCylinderPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    if (!planeCoeffsWithWeldSeam || !cylinderCoeffsInWeldArea || !cloud || cloud->empty()) {
        PLOGE << "removeCylinderPoints: 输入参数无效";
        return;
    }

    /* ================== 1 投影到平面 ================== */

    pcl::PointCloud<pcl::PointXYZ>::Ptr projectedCloud(new pcl::PointCloud<pcl::PointXYZ>);

    MyToolFunc::projectCloudToPlane(cloud, projectedCloud, planeCoeffsWithWeldSeam);

    if (projectedCloud->empty()) return;

    /* ================== 2 构建局部坐标系 ================== */

    Eigen::Vector3d cylinderDir(cylinderCoeffsInWeldArea->values[3], cylinderCoeffsInWeldArea->values[4], cylinderCoeffsInWeldArea->values[5]);

    cylinderDir.normalize();

    Eigen::Vector3d planeNormal(planeCoeffsWithWeldSeam->values[0], planeCoeffsWithWeldSeam->values[1], planeCoeffsWithWeldSeam->values[2]);

    planeNormal.normalize();

    /* 圆柱轴在平面上的投影 */

    Eigen::Vector3d lineDir = cylinderDir - cylinderDir.dot(planeNormal) * planeNormal;

    lineDir.normalize();

    /* 垂直方向 */

    Eigen::Vector3d vDir = planeNormal.cross(lineDir);
    vDir.normalize();

    Eigen::Vector3d origin(projectedCloud->points[0].x, projectedCloud->points[0].y, projectedCloud->points[0].z);

    /* ================== 3 计算uv坐标 ================== */

    size_t N = projectedCloud->size();

    std::vector<double> u(N);
    std::vector<double> v(N);

    double uMin = 1e9, uMax = -1e9;
    double vMin = 1e9, vMax = -1e9;

#pragma omp parallel for num_threads(12)
    for (int i = 0; i < (int)N; ++i) {
        Eigen::Vector3d p(projectedCloud->points[i].x, projectedCloud->points[i].y, projectedCloud->points[i].z);

        Eigen::Vector3d d = p - origin;

        u[i] = d.dot(lineDir);
        v[i] = d.dot(vDir);
    }

    for (size_t i = 0; i < N; ++i) {
        uMin = std::min(uMin, u[i]);
        uMax = std::max(uMax, u[i]);
        vMin = std::min(vMin, v[i]);
        vMax = std::max(vMax, v[i]);
    }

    /* ================== 4 构建栅格 ================== */

    int W = (uMax - uMin) / resolution + 1;
    int H = (vMax - vMin) / resolution + 1;

    std::vector<uint8_t> mask(W * H, 0);

    std::vector<int> px(N);
    std::vector<int> py(N);

    /* 每个栅格存原始点索引 */

    std::vector<std::vector<std::vector<int>>> grid(H, std::vector<std::vector<int>>(W));

#pragma omp parallel for num_threads(12)
    for (int i = 0; i < (int)N; ++i) {
        int x = (u[i] - uMin) / resolution;
        int y = (v[i] - vMin) / resolution;

        px[i] = x;
        py[i] = y;

        if (x >= 0 && x < W && y >= 0 && y < H) {
#pragma omp critical
            {
                grid[y][x].push_back(i);
                mask[y * W + x] = 1;
            }
        }
    }

    /* ================== 5 v方向腐蚀 ================== */

    int radius = 1;

    std::vector<uint8_t> eroded(W * H, 0);

#pragma omp parallel for num_threads(12)
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            bool ok = true;

            for (int dy = -radius; dy <= radius; ++dy) {
                int yy = y + dy;

                if (yy < 0 || yy >= H) {
                    ok = false;
                    break;
                }

                if (!mask[yy * W + x]) {
                    ok = false;
                    break;
                }
            }

            if (ok) eroded[y * W + x] = 1;
        }
    }

    /* ================== 6 v方向膨胀 ================== */

    std::vector<uint8_t> opened(W * H, 0);

#pragma omp parallel for num_threads(12)
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            bool ok = false;

            for (int dy = -radius; dy <= radius; ++dy) {
                int yy = y + dy;

                if (yy < 0 || yy >= H) continue;

                if (eroded[yy * W + x]) {
                    ok = true;
                    break;
                }
            }

            if (ok) opened[y * W + x] = 1;
        }
    }

    /* ================== 7 恢复原始点 ================== */

    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            if (opened[y * W + x]) {
                for (int idx : grid[y][x]) {
                    result->push_back(projectedCloud->points[idx]);
                }
            }
        }
    }

    /* ================== 8 输出 ================== */

    result->height = 1;
    result->width = result->size();

    cloud->swap(*result);
}

bool TubeSidePlateFilletSeamsDet::solveSeamEndPoints() {
    if (!cloudPlaneInWeldAreaWithSeam || cloudPlaneInWeldAreaWithSeam->empty() || !cylinderCoeffsInWeldArea || !planeCoeffsWithWeldSeam) {
        PLOGE << "SolveBeamButtSeamEndPoints: 输入参数无效";
        return false;
    }

    /* ================== 平面参数 ================== */
    Eigen::Vector3d planeN(planeCoeffsWithWeldSeam->values[0], planeCoeffsWithWeldSeam->values[1], planeCoeffsWithWeldSeam->values[2]);
    planeN.normalize();
    double planeD = planeCoeffsWithWeldSeam->values[3];

    /* ================== 圆柱参数 ================== */
    Eigen::Vector3d p0(cylinderCoeffsInWeldArea->values[0], cylinderCoeffsInWeldArea->values[1], cylinderCoeffsInWeldArea->values[2]);

    Eigen::Vector3d ez(cylinderCoeffsInWeldArea->values[3], cylinderCoeffsInWeldArea->values[4], cylinderCoeffsInWeldArea->values[5]);
    ez.normalize();

    double R = cylinderCoeffsInWeldArea->values[6];

    /* ================== 构造局部坐标系 ================== */
    Eigen::Vector3d tmp(1, 0, 0);
    if (fabs(ez.dot(tmp)) > 0.9) tmp = Eigen::Vector3d(0, 1, 0);

    Eigen::Vector3d ex = ez.cross(tmp).normalized();
    Eigen::Vector3d ey = ez.cross(ex).normalized();

    /* ================== 估计轴向范围 ================== */
    double zmin = std::numeric_limits<double>::max();
    double zmax = -std::numeric_limits<double>::max();
    double threshold = R + extendCylinderInPlaneArea;
#pragma omp parallel
    {
        double local_zmin = std::numeric_limits<double>::max();
        double local_zmax = -std::numeric_limits<double>::max();

        std::vector<pcl::PointXYZ> local_pts;

#pragma omp for nowait
        for (int i = 0; i < (int)cloudPlaneInWeldAreaWithSeam->size(); ++i) {
            const auto& pt = cloudPlaneInWeldAreaWithSeam->points[i];

            Eigen::Vector3d pw(pt.x, pt.y, pt.z);
            Eigen::Vector3d pl = pw - p0;

            double distAxis = (pl - pl.dot(ez) * ez).norm();

            if (distAxis <= threshold) {
                double z = pl.dot(ez);

                local_zmin = std::min(local_zmin, z);
                local_zmax = std::max(local_zmax, z);

                local_pts.push_back(pt);
            }
        }

        // 合并 zmin/zmax（线程安全）
#pragma omp critical
        {
            zmin = std::min(zmin, local_zmin);
            zmax = std::max(zmax, local_zmax);

            axisRangeCloud->points.insert(axisRangeCloud->points.end(), local_pts.begin(), local_pts.end());
        }
    }
    if (zmax <= zmin) {
        PLOGE << "SolveBeamButtSeamEndPoints: 轴向范围计算失败 (zmax <= zmin)";
        return false;
    }
    if (saveFlag) {
        axisRangeCloud->height = 1;
        axisRangeCloud->width = static_cast<uint32_t>(axisRangeCloud->size());
        pcl::io::savePCDFile("./data/seamDetWithPointCloud/tubeSidePlateFilletSeamsDet/axisRangeCloud.pcd", *axisRangeCloud);
    }

    /* ================== KDTree ================== */
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(axisRangeCloud);

    std::vector<int> nnIdx(1);
    std::vector<float> nnDist(1);

    /* ================== 计算平面在局部坐标的表达 ================== */
    double nx = planeN.dot(ex);
    double ny = planeN.dot(ey);
    double nz = planeN.dot(ez);
    double d0 = planeN.dot(p0) + planeD;

    /* ================== 只取两端点 ================== */

    for (int i = 0; i < 2; ++i) {
        double z = (i == 0) ? zmin : zmax;

        double rhs = -(nz * z + d0);

        double a = nx;
        double b = ny;
        double c = rhs;

        double denom = a * a + b * b;
        if (denom < 1e-10) {
            PLOGE << "SolveBeamButtSeamEndPoints: 交线点计算失败 (denom < 1e-10)";
            continue;
        }

        double d2 = c * c / denom;
        if (d2 > R * R) {
            PLOGE << "SolveBeamButtSeamEndPoints: 交线点超出圆柱范围 (d2 > R*R)";
            continue;
        }

        double h = std::sqrt(R * R - d2);

        double x0 = a * c / denom;
        double y0 = b * c / denom;

        double norm_ab = std::sqrt(denom);

        double dx = -b * h / norm_ab;
        double dy = a * h / norm_ab;

        Eigen::Vector3d p1 = p0 + (x0 + dx) * ex + (y0 + dy) * ey + z * ez;
        Eigen::Vector3d p2 = p0 + (x0 - dx) * ex + (y0 - dy) * ey + z * ez;

        float d1 = std::numeric_limits<float>::max();
        float d2n = std::numeric_limits<float>::max();

        if (kdtree.nearestKSearch(pcl::PointXYZ(p1.x(), p1.y(), p1.z()), 1, nnIdx, nnDist) > 0) d1 = nnDist[0];

        if (kdtree.nearestKSearch(pcl::PointXYZ(p2.x(), p2.y(), p2.z()), 1, nnIdx, nnDist) > 0) d2n = nnDist[0];

        float threshold = 1.0f * 1.0f;  // 1mm，对应平方距离

        Eigen::Vector3d best = (d1 < d2n) ? p1 : p2;
        float bestDist = (d1 < d2n) ? d1 : d2n;

        if (bestDist < threshold) {
            if (saveFlag) seamEndPoints->push_back(pcl::PointXYZ(best.x(), best.y(), best.z()));
            filletSeamsTSP.emplace_back(best.x(), best.y(), best.z());
        } else {
            PLOGE << "端点距离过大，丢弃该点";
        }
    }
    if (filletSeamsTSP.size() < 2) {
        PLOGE << "SolveBeamButtSeamEndPoints: 焊缝端点计算失败 (seamEndPoints->size() < 2)";
        return false;
    }

    lineCoeffsWithWeldSeam2Val->values.resize(6);

    // 取两个端点
    Eigen::Vector3d p_start(filletSeamsTSP[0].x, filletSeamsTSP[0].y, filletSeamsTSP[0].z);

    Eigen::Vector3d p_end(filletSeamsTSP[1].x, filletSeamsTSP[1].y, filletSeamsTSP[1].z);

    // 方向向量
    Eigen::Vector3d dir = (p_end - p_start).normalized();

    // 填充
    lineCoeffsWithWeldSeam2Val->values[0] = p_start.x();
    lineCoeffsWithWeldSeam2Val->values[1] = p_start.y();
    lineCoeffsWithWeldSeam2Val->values[2] = p_start.z();

    lineCoeffsWithWeldSeam2Val->values[3] = dir.x();
    lineCoeffsWithWeldSeam2Val->values[4] = dir.y();
    lineCoeffsWithWeldSeam2Val->values[5] = dir.z();

    if (saveFlag) {
        seamEndPoints->height = 1;
        seamEndPoints->width = static_cast<uint32_t>(seamEndPoints->size());
        pcl::io::savePCDFile("./data/seamDetWithPointCloud/tubeSidePlateFilletSeamsDet/seamEndPoints.pcd", *seamEndPoints);
    }

    PLOGD << "Seam endpoints computed and saved.";
    return true;
}
