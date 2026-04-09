#include "TubePlateFilletSeamsDet.h"

#include "seamDetWithPointCloud/QhullLock.h"
#include "settingPara/SettingPara.h"
#include "utils/pointCloud/PointCloudFunc.h"
TubePlateFilletSeamsDet::TubePlateFilletSeamsDet(QObject* parent) : AbstractSeamDet{parent} {}

std::vector<std::shared_ptr<WeldSeamInfo>> TubePlateFilletSeamsDet::solveSeamsEndPoints(std::vector<std::shared_ptr<WeldSeamInfo>> seamsInfo) {
    tempWeldSeamsInfo = seamsInfo;
    for (size_t i = 0; i < tempWeldSeamsInfo.size(); i++) {
        singleSeamReinitialize();

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
                pcl::io::savePCDFile(
                    "./data/seamDetWithPointCloud/tubePlateFilletSeamsDet/after_cluster_" + std::to_string(tempWeldSeamsInfo[i]->areaNum) + ".pcd",
                    *cloudInWeldArea);
            }
        }
        {
            // ScopedTimer t("ransacCylinder");
            ransacCylinder(cloudInWeldArea, cloudCylinderInWeldAreaWithSeam, cloudNoCylinderInWeldArea);
            if (!cloudCylinderInWeldAreaWithSeam || cloudCylinderInWeldAreaWithSeam->empty()) {
                PLOGE << "Ransac_cylinder: 圆柱点云为空";
                detectSuccFlag = false;
                continue;
            }
            cloudCylinderInWeldAreaWithSeam->height = 1;
            cloudCylinderInWeldAreaWithSeam->width = static_cast<uint32_t>(cloudCylinderInWeldAreaWithSeam->size());
            if (saveFlag) {
                pcl::io::savePCDFile("./data/seamDetWithPointCloud/tubePlateFilletSeamsDet/cloudCylinderInWeldAreaWithSeam_" +
                                         std::to_string(tempWeldSeamsInfo[i]->areaNum) + ".pcd",
                                     *cloudCylinderInWeldAreaWithSeam);
                cloudNoCylinderInWeldArea->height = 1;
                cloudNoCylinderInWeldArea->width = static_cast<uint32_t>(cloudNoCylinderInWeldArea->size());
                pcl::io::savePCDFile("./data/seamDetWithPointCloud/tubePlateFilletSeamsDet/cloudNoPlaneInWeldArea_" +
                                         std::to_string(tempWeldSeamsInfo[i]->areaNum) + ".pcd",
                                     *cloudNoCylinderInWeldArea);
            }
        }
        {
            // ScopedTimer t("ransacPlane");
            ransacPlane(cloudNoCylinderInWeldArea, planeCoeffsInWeldArea);
            if (!planeCoeffsInWeldArea || planeCoeffsInWeldArea->values.empty()) {
                PLOGE << "Ransac_plane: 平面点云为空";
                detectSuccFlag = false;
                continue;
            }
        }
        {
            // ScopedTimer t("projectCloudToCylinder");
            MyToolFunc::projectCloudToCylinder(cloudCylinderInWeldAreaWithSeam, cloudCylinderInWeldAreaWithSeam, cylinderCoeffsWithWeldSeam);
            if (saveFlag) {
                cloudCylinderInWeldAreaWithSeam->height = 1;
                cloudCylinderInWeldAreaWithSeam->width = static_cast<uint32_t>(cloudCylinderInWeldAreaWithSeam->size());
                pcl::io::savePCDFile("./data/seamDetWithPointCloud/tubePlateFilletSeamsDet/projectCloudToCylinder_" +
                                         std::to_string(tempWeldSeamsInfo[i]->areaNum) + ".pcd",
                                     *cloudCylinderInWeldAreaWithSeam);
            }
        }
        {
            // ScopedTimer t("extractSeamPointsFromCylinderPlane");
            extractSeamPointsFromCylinderPlane(cloudCylinderInWeldAreaWithSeam, axisRangeCloud, planeCoeffsInWeldArea, extendCylinderInPlaneArea);
            if (saveFlag) {
                axisRangeCloud->height = 1;
                axisRangeCloud->width = static_cast<uint32_t>(axisRangeCloud->size());
                pcl::io::savePCDFile(
                    "./data/seamDetWithPointCloud/tubePlateFilletSeamsDet/axisRangeCloud_" + std::to_string(tempWeldSeamsInfo[i]->areaNum) + ".pcd",
                    *axisRangeCloud);
            }
        }
        {
            // ScopedTimer t("statisticFilter");
            // 统计滤波
            statisticFilter(axisRangeCloud);
            if (saveFlag) {
                axisRangeCloud->height = 1;
                axisRangeCloud->width = static_cast<uint32_t>(axisRangeCloud->size());
                pcl::io::savePCDFile(
                    "./data/seamDetWithPointCloud/tubePlateFilletSeamsDet/statisticFilter_" + std::to_string(tempWeldSeamsInfo[i]->areaNum) + ".pcd",
                    *axisRangeCloud);
            }
        }
        {
            // ScopedTimer t("removePlanePoints");
            // 去除平面母材残留点云
            removePlanePoints(axisRangeCloud);
            PLOGD << "axisRangeCloud" << axisRangeCloud->size();
            if (saveFlag) {
                axisRangeCloud->height = 1;
                axisRangeCloud->width = static_cast<uint32_t>(axisRangeCloud->size());
                pcl::io::savePCDFile("./data/seamDetWithPointCloud/tubePlateFilletSeamsDet/removePlanePoints_" +
                                         std::to_string(tempWeldSeamsInfo[i]->areaNum) + ".pcd",
                                     *axisRangeCloud);
            }
        }
        {
            // ScopedTimer t("myFastMaxCluster");
            MyToolFunc::myFastMaxCluster(axisRangeCloud, 3);
            if (saveFlag) {
                axisRangeCloud->height = 1;
                axisRangeCloud->width = static_cast<uint32_t>(axisRangeCloud->size());
                pcl::io::savePCDFile(
                    "./data/seamDetWithPointCloud/tubePlateFilletSeamsDet/myFastMaxCluster_" + std::to_string(tempWeldSeamsInfo[i]->areaNum) + ".pcd",
                    *axisRangeCloud);
            }
        }
        detectSuccFlag = solveSeamEndPoints();
        if (saveFlag) {
            seamEndPoints->height = 1;
            seamEndPoints->width = static_cast<uint32_t>(seamEndPoints->size());
            pcl::io::savePCDFile(
                "./data/seamDetWithPointCloud/tubePlateFilletSeamsDet/seamEndPoints" + std::to_string(tempWeldSeamsInfo[i]->areaNum) + ".pcd",
                *seamEndPoints);
        }
        // 保存本次检测到的信息
        tempWeldSeamsInfo[i]->detectSuccFlag = detectSuccFlag;
        if (detectSuccFlag) {
            tempWeldSeamsInfo[i]->weldEndPointsInCamera.reset(new std::vector<pcl::PointXYZ>(std::move(filletSeamsTP)));  // 检测结果
            tempWeldSeamsInfo[i]->weldEndPointsInRobot.reset(new std::vector<pcl::PointXYZ>());
            tempWeldSeamsInfo[i]->weldCoeff = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients(*cylinderCoeffsWithWeldSeam));
            tempWeldSeamsInfo[i]->otherSurface.emplace_back(pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients(*planeCoeffsInWeldArea)));
            tempWeldSeamsInfo[i]->weldType = Tube_Plate_Fillet;
            tempWeldSeamsInfo[i]->seamsLineToVal = lineCoeffsWithWeldSeam2Val;
        }
    }
    PLOGE << "return tempWeldSeamsInfo;";
    return tempWeldSeamsInfo;
}
// 单条焊缝检测前，变量重新初始化
void TubePlateFilletSeamsDet::singleSeamReinitialize() {
    cloudInWeldArea.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloudCylinderInWeldAreaWithSeam.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloudNoCylinderInWeldArea.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloudPlaneInWeldArea.reset(new pcl::PointCloud<pcl::PointXYZ>);
    seamEndPoints.reset(new pcl::PointCloud<pcl::PointXYZ>);
    axisRangeCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    cylinderCoeffsWithWeldSeam.reset(new pcl::ModelCoefficients);
    planeCoeffsInWeldArea.reset(new pcl::ModelCoefficients);
    lineCoeffsWithWeldSeam2Val.reset(new pcl::ModelCoefficients);

    filletSeamsTP.clear();
    saveFlag = SettingPara::getInstance().bool_save_model;
    detectSuccFlag = false;
}
void TubePlateFilletSeamsDet::statisticFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input_cloud);       // 设置待滤波的点云
    sor.setMeanK(Statistic_NeighPoints);  // 设置在进行统计时考虑查询点邻近点数
    sor.setStddevMulThresh(Statistic_sigma);  // 设置判断是否为离群点的阈值，里边的数字表示标准差的倍数，1个标准差以上就是离群点。
    sor.filter(*input_cloud);  // 存储内点
}
void TubePlateFilletSeamsDet::ransacCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_non_cylinder) {
    if (!input_cloud || input_cloud->empty() || !cloud_cylinder || !cloud_non_cylinder) {
        PLOGE << "Ransac_cylinder: 输入参数无效";
        detectSuccFlag = false;
        return;
    }

    // ---------- 1 计算法向量 (OMP并行) ----------
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads(std::max(1u, std::thread::hardware_concurrency() / 2));

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    ne.setSearchMethod(tree);
    ne.setInputCloud(input_cloud);
    ne.setKSearch(100);
    ne.compute(*normals);

    // ---------- 2 RANSAC ----------
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);

    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);

    seg.setNormalDistanceWeight(0.2);
    seg.setMaxIterations(Ransac_cylinder_Iterations);
    seg.setDistanceThreshold(Ransac_cylinder_Dth);
    seg.setRadiusLimits(20, 150);

    seg.setInputCloud(input_cloud);
    seg.setInputNormals(normals);

    seg.segment(*inliers_cylinder, *cylinderCoeffsWithWeldSeam);

    if (inliers_cylinder->indices.empty()) {
        PLOGD << "圆柱面提取失败";
        detectSuccFlag = false;
        return;
    }

    // ---------- 3 提取圆柱 ----------
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input_cloud);
    extract.setIndices(inliers_cylinder);

    extract.setNegative(false);
    extract.filter(*cloud_cylinder);

    // ---------- 4 提取非圆柱（关键：焊缝候选） ----------
    extract.setNegative(true);
    extract.filter(*cloud_non_cylinder);

    // // ---------- 5 输出参数 ----------
    // std::cout << u8"cylinderCoeffs: ";
    // for (auto v : cylinderCoeffsWithWeldSeam->values) std::cout << v << " ";
    // std::cout << std::endl;
}
void TubePlateFilletSeamsDet::ransacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::ModelCoefficients::Ptr planeCoeff) {
    if (!input_cloud || input_cloud->empty() || !planeCoeff) {
        PLOGE << "Ransac_plane: 输入参数无效";
        detectSuccFlag = false;
        return;
    }

    // ---------- 1 RANSAC ----------
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(Ransac_plane_Iterations);
    seg.setDistanceThreshold(Ransac_plane_Dth);
    seg.setInputCloud(input_cloud);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    seg.segment(*inliers, *planeCoeff);

    if (inliers->indices.empty()) {
        PLOGD << "平面提取失败";
        detectSuccFlag = false;
        return;
    }

    // // ---------- 2 输出参数 ----------
    // std::cout << u8"planeCoeff: ";
    // for (auto v : planeCoeff->values) std::cout << v << " ";
    // std::cout << std::endl;
}
//
void TubePlateFilletSeamsDet::extractSeamPointsFromCylinderPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
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
void TubePlateFilletSeamsDet::removePlanePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    if (!cloud || cloud->empty() || !cylinderCoeffsWithWeldSeam) {
        PLOGE << "removePlanePoints: 参数错误";
        return;
    }

    // ================= 1. 圆柱参数 =================
    Eigen::Vector3f C(cylinderCoeffsWithWeldSeam->values[0], cylinderCoeffsWithWeldSeam->values[1], cylinderCoeffsWithWeldSeam->values[2]);

    Eigen::Vector3f axis(cylinderCoeffsWithWeldSeam->values[3], cylinderCoeffsWithWeldSeam->values[4], cylinderCoeffsWithWeldSeam->values[5]);

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
bool TubePlateFilletSeamsDet::solveSeamEndPoints() {
    if (!cloudCylinderInWeldAreaWithSeam || cloudCylinderInWeldAreaWithSeam->empty() || !planeCoeffsInWeldArea || !cylinderCoeffsWithWeldSeam) {
        PLOGE << "SolveSeamEndPoints: 输入参数无效";
        return false;
    }

    filletSeamsTP.clear();

    // ================= 1. 圆柱参数 =================
    Eigen::Vector3f C(cylinderCoeffsWithWeldSeam->values[0], cylinderCoeffsWithWeldSeam->values[1], cylinderCoeffsWithWeldSeam->values[2]);

    Eigen::Vector3f axis(cylinderCoeffsWithWeldSeam->values[3], cylinderCoeffsWithWeldSeam->values[4], cylinderCoeffsWithWeldSeam->values[5]);

    axis.normalize();

    float R = cylinderCoeffsWithWeldSeam->values[6];

    // ================= 2. 平面参数 =================
    Eigen::Vector3f n(planeCoeffsInWeldArea->values[0], planeCoeffsInWeldArea->values[1], planeCoeffsInWeldArea->values[2]);

    float d = planeCoeffsInWeldArea->values[3];

    // ================= 3. 构造局部坐标系 =================
    Eigen::Vector3f ref(0, 0, -1);
    if (fabs(axis.dot(ref)) > 0.95f) ref = Eigen::Vector3f(1, 0, 0);

    Eigen::Vector3f u = (ref - ref.dot(axis) * axis).normalized();
    Eigen::Vector3f v = axis.cross(u).normalized();

    // ================= 4. 构造交线函数 =================
    float A = n.dot(axis);

    if (fabs(A) < 1e-6) {
        PLOGE << "平面与轴接近平行，交线退化";
        return false;
    }

    auto computePointOnCurve = [&](float theta) -> Eigen::Vector3f {
        float B = n.dot(C) + R * (cos(theta) * n.dot(u) + sin(theta) * n.dot(v)) + d;

        float t = -B / A;

        return C + t * axis + R * (cos(theta) * u + sin(theta) * v);
    };

    // ================= 5. 点云投影到交线 =================
    struct PtTheta {
        pcl::PointXYZ pt;
        float theta;
    };

    std::vector<PtTheta> pts;
    pts.reserve(axisRangeCloud->size());

    for (auto& p : axisRangeCloud->points) {
        Eigen::Vector3f P(p.x, p.y, p.z);

        float t0 = (P - C).dot(axis);
        Eigen::Vector3f proj = C + t0 * axis;

        Eigen::Vector3f d_vec = P - proj;

        float x = d_vec.dot(u);
        float y = d_vec.dot(v);

        float theta = atan2(y, x);

        Eigen::Vector3f Pc = computePointOnCurve(theta);

        pcl::PointXYZ p_new;
        p_new.x = Pc.x();
        p_new.y = Pc.y();
        p_new.z = Pc.z();

        pts.push_back({p_new, theta});
    }

    if (pts.size() < 10) {
        PLOGE << "点太少";
        return false;
    }

    // ================= 6. 按 θ 排序 =================
    std::sort(pts.begin(), pts.end(), [](const PtTheta& a, const PtTheta& b) { return a.theta < b.theta; });

    if (pts.empty()) {
        PLOGE << "pts 为空，无法计算端点";
        return false;
    }

    // ================= 7. 构造 ordered（仅用于采样） =================
    std::vector<PtTheta> ordered = pts;

    //  可选：只有跨π才做重排（推荐）
    float max_gap = 0;
    int split_idx = 0;

    for (int i = 1; i < pts.size(); ++i) {
        float gap = pts[i].theta - pts[i - 1].theta;
        if (gap > max_gap) {
            max_gap = gap;
            split_idx = i;
        }
    }

    // 如果存在明显断裂才重排
    if (max_gap > M_PI) {
        ordered.clear();
        ordered.reserve(pts.size());

        for (int i = split_idx; i < pts.size(); ++i) ordered.push_back(pts[i]);
        for (int i = 0; i < split_idx; ++i) ordered.push_back(pts[i]);
    }
    // ================= 8. 三个关键点 =================
    int total = ordered.size();
    if (total < 2) return false;

    filletSeamsTP.clear();

    // ================= 9. 计算累计弧长 =================
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
    if (total_len < 1e-6f) return false;

    // ================= 10. 均匀采样 =================
    int N = sample_num;  // 你想要的采样点数（包含首尾）
    filletSeamsTP.reserve(N);

    // 步长
    float step = total_len / (N - 1);

    // 起点
    filletSeamsTP.push_back(ordered.front().pt);

    int curr_idx = 1;

    for (int i = 1; i < N - 1; ++i) {
        float target_len = i * step;

        // 找到 target 所在区间
        while (curr_idx < total && arc_len[curr_idx] < target_len) {
            curr_idx++;
        }

        if (curr_idx >= total) {
            filletSeamsTP.push_back(ordered.back().pt);
            continue;
        }

        // 区间两端点
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

        // 线性插值
        pcl::PointXYZ interp_pt;
        interp_pt.x = p1.x + t * (p2.x - p1.x);
        interp_pt.y = p1.y + t * (p2.y - p1.y);
        interp_pt.z = p1.z + t * (p2.z - p1.z);

        filletSeamsTP.push_back(interp_pt);
    }

    // 终点
    filletSeamsTP.push_back(ordered.back().pt);

    // ================= 10. 保存 =================
    if (saveFlag && seamEndPoints) {
        seamEndPoints->points.assign(filletSeamsTP.begin(), filletSeamsTP.end());
    }

    return true;
}
