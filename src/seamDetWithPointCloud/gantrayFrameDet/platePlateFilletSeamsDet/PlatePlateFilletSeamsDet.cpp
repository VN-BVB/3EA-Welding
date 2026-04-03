#include "PlatePlateFilletSeamsDet.h"

#include "seamDetWithPointCloud/QhullLock.h"
#include "settingPara/SettingPara.h"
#include "utils/pointCloud/PointCloudFunc.h"

PlatePlateFilletSeamsDet::PlatePlateFilletSeamsDet(QObject* parent) : AbstractSeamDet{parent} {}

std::vector<std::shared_ptr<WeldSeamInfo>> PlatePlateFilletSeamsDet::solveSeamsEndPoints(std::vector<std::shared_ptr<WeldSeamInfo>> seamsInfo) {
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
            pcl::io::savePCDFile(
                "./data/seamDetWithPointCloud/PlatePlateFilletSeamsDet/after_cluster_" + std::to_string(tempWeldSeamsInfo[i]->areaNum) + ".pcd",
                *cloudInWeldArea);
        }
        // 拟合多个平面，并输出主平面点云与母材系数
        ransacMultiPlane(cloudInWeldArea, cloudPlaneInWeldAreaWithSeam, planeCoeffsWithWeldSeam, otherPlaneCoeffsInWeldArea);
        if (!cloudPlaneInWeldAreaWithSeam || cloudPlaneInWeldAreaWithSeam->empty()) {
            PLOGE << "Ransac_plane: 平面点云为空";
            detectSuccFlag = false;
            continue;
        }
        cloudPlaneInWeldAreaWithSeam->height = 1;
        cloudPlaneInWeldAreaWithSeam->width = static_cast<uint32_t>(cloudPlaneInWeldAreaWithSeam->size());
        if (saveFlag) {
            pcl::io::savePCDFile("./data/seamDetWithPointCloud/PlatePlateFilletSeamsDet/cloudPlaneInWeldAreaWithSeam" +
                                     std::to_string(tempWeldSeamsInfo[i]->areaNum) + ".pcd",
                                 *cloudPlaneInWeldAreaWithSeam);
        }

        // 平面投影
        MyToolFunc::projectCloudToPlane(cloudPlaneInWeldAreaWithSeam, cloudPlaneInWeldAreaWithSeam, planeCoeffsWithWeldSeam);
        cloudPlaneInWeldAreaWithSeam->height = 1;
        cloudPlaneInWeldAreaWithSeam->width = static_cast<uint32_t>(cloudPlaneInWeldAreaWithSeam->size());
        if (saveFlag) {
            pcl::io::savePCDFile("./data/seamDetWithPointCloud/PlatePlateFilletSeamsDet/cloudPlaneInWeldAreaWithSeam2D" +
                                     std::to_string(tempWeldSeamsInfo[i]->areaNum) + ".pcd",
                                 *cloudPlaneInWeldAreaWithSeam);
        }
        // 求交线，并获取焊缝区域点云
        computePlaneIntersectionLine(cloudPlaneInWeldAreaWithSeam, axisRangeCloud);
        if (!axisRangeCloud || axisRangeCloud->empty() || !lineCoeffsWithWeldSeam2Val || lineCoeffsWithWeldSeam2Val->values.size() < 6) {
            PLOGE << "求交线区域点云为空";
            detectSuccFlag = false;
            continue;
        }
        if (saveFlag) {
            axisRangeCloud->height = 1;
            axisRangeCloud->width = static_cast<uint32_t>(axisRangeCloud->size());
            pcl::io::savePCDFile(
                "./data/seamDetWithPointCloud/PlatePlateFilletSeamsDet/axisRangeCloud" + std::to_string(tempWeldSeamsInfo[i]->areaNum) + ".pcd",
                *axisRangeCloud);
        }

        // 去除其他母材表面点云
        filterCloudByPlaneThenMorph(axisRangeCloud, cloudPlaneInWeldAreaWithSeam);
        if (!cloudPlaneInWeldAreaWithSeam || cloudPlaneInWeldAreaWithSeam->empty()) {
            PLOGE << "去除其他母材表面点云将点云全部去除。";
            detectSuccFlag = false;
            continue;
        }
        if (saveFlag) {
            cloudPlaneInWeldAreaWithSeam->height = 1;
            cloudPlaneInWeldAreaWithSeam->width = static_cast<uint32_t>(cloudPlaneInWeldAreaWithSeam->size());
            pcl::io::savePCDFile(
                "./data/seamDetWithPointCloud/PlatePlateFilletSeamsDet/filterCloudByPlane" + std::to_string(tempWeldSeamsInfo[i]->areaNum) + ".pcd",
                *cloudPlaneInWeldAreaWithSeam);
        }
        MyToolFunc::myFastMaxCluster(cloudPlaneInWeldAreaWithSeam, Max_cluster_radius);

        statisticFilter(cloudPlaneInWeldAreaWithSeam);
        cloudPlaneInWeldAreaWithSeam->height = 1;
        cloudPlaneInWeldAreaWithSeam->width = static_cast<uint32_t>(cloudPlaneInWeldAreaWithSeam->size());
        if (saveFlag) {
            pcl::io::savePCDFile(
                "./data/seamDetWithPointCloud/PlatePlateFilletSeamsDet/currentEnd" + std::to_string(tempWeldSeamsInfo[i]->areaNum) + ".pcd",
                *cloudPlaneInWeldAreaWithSeam);
        }
        detectSuccFlag = solveSeamEndPoints();
        if (saveFlag) {
            seamEndPoints->height = 1;
            seamEndPoints->width = static_cast<uint32_t>(seamEndPoints->size());
            pcl::io::savePCDFile(
                "./data/seamDetWithPointCloud/PlatePlateFilletSeamsDet/seamEndPoints" + std::to_string(tempWeldSeamsInfo[i]->areaNum) + ".pcd",
                *seamEndPoints);
        }
        // 保存本次检测到的信息
        tempWeldSeamsInfo[i]->detectSuccFlag = detectSuccFlag;
        if (detectSuccFlag) {
            tempWeldSeamsInfo[i]->weldEndPointsInCamera.reset(new std::vector<pcl::PointXYZ>(std::move(filletSeamsTSP)));  // 检测结果
            tempWeldSeamsInfo[i]->weldCoeff = planeCoeffsWithWeldSeam;
            tempWeldSeamsInfo[i]->otherSurface.emplace_back(otherPlaneCoeffsInWeldArea);
            tempWeldSeamsInfo[i]->weldType = weldType;
            tempWeldSeamsInfo[i]->seamsLineToVal = lineCoeffsWithWeldSeam2Val;
        } else {
            PLOGD << "板板角接焊缝检测失败";
        }
        PLOGD << "区域" << std::to_string(tempWeldSeamsInfo[i]->areaNum) << "为" << weldType;
    }
    return tempWeldSeamsInfo;
}
// 单条焊缝检测前，变量重新初始化
void PlatePlateFilletSeamsDet::singleSeamReinitialize() {
    cloudInWeldArea.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloudPlaneInWeldAreaWithSeam.reset(new pcl::PointCloud<pcl::PointXYZ>);
    seamEndPoints.reset(new pcl::PointCloud<pcl::PointXYZ>);
    axisRangeCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    planeCoeffsWithWeldSeam.reset(new pcl::ModelCoefficients);
    otherPlaneCoeffsInWeldArea.reset(new pcl::ModelCoefficients);
    lineCoeffsWithWeldSeam2Val.reset(new pcl::ModelCoefficients);

    filletSeamsTSP.clear();
    saveFlag = SettingPara::getInstance().bool_save_model;  // saveFlag = true;
    detectSuccFlag = false;
    weldType = Plate_Plate_Fillet_V;
}
void PlatePlateFilletSeamsDet::statisticFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input_cloud);       // 设置待滤波的点云
    sor.setMeanK(Statistic_NeighPoints);  // 设置在进行统计时考虑查询点邻近点数
    sor.setStddevMulThresh(Statistic_sigma);  // 设置判断是否为离群点的阈值，里边的数字表示标准差的倍数，1个标准差以上就是离群点。
    sor.filter(*input_cloud);  // 存储内点
}

// ===== 计算平面面积（辅助函数）=====
float PlatePlateFilletSeamsDet::computePlaneArea(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::ModelCoefficients::Ptr coeff) {
    if (!cloud || cloud->empty()) return 0.0f;

    Eigen::Vector3f normal(coeff->values[0], coeff->values[1], coeff->values[2]);
    normal.normalize();

    // 构建局部坐标系
    Eigen::Vector3f u = normal.unitOrthogonal();
    Eigen::Vector3f v = normal.cross(u);

    float min_u = FLT_MAX, max_u = -FLT_MAX;
    float min_v = FLT_MAX, max_v = -FLT_MAX;

    for (const auto& pt : cloud->points) {
        Eigen::Vector3f p(pt.x, pt.y, pt.z);

        float pu = p.dot(u);
        float pv = p.dot(v);

        min_u = std::min(min_u, pu);
        max_u = std::max(max_u, pu);
        min_v = std::min(min_v, pv);
        max_v = std::max(max_v, pv);
    }

    float width = max_u - min_u;
    float height = max_v - min_v;

    return width * height;
}
void PlatePlateFilletSeamsDet::ransacMultiPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_plane, pcl::ModelCoefficients::Ptr& mainPlaneCoeffs,
                                                pcl::ModelCoefficients::Ptr& otherPlaneCoeffs) {
    // ===== 初始化 =====
    output_cloud_plane->clear();
    if (!input_cloud || input_cloud->empty()) {
        PLOGE << "输入点云为空";
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remain(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_remain = *input_cloud;

    std::vector<pcl::ModelCoefficients::Ptr> plane_coeffs;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> plane_clouds;

    int max_planes = 3;    // 至少拟合3个（你要求）
    int min_points = 100;  // 防止过小

    // ===== 多平面提取 =====
    for (int i = 0; i < max_planes; ++i) {
        if (cloud_remain->size() < min_points) break;

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(Ransac_plane_Iterations);
        seg.setDistanceThreshold(Ransac_plane_Dth);
        seg.setInputCloud(cloud_remain);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);

        seg.segment(*inliers, *coeff);

        if (inliers->indices.size() < min_points) break;

        // 提取当前平面
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_remain);
        extract.setIndices(inliers);
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setNegative(false);
        extract.filter(*plane);

        // 剩余点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr new_remain(new pcl::PointCloud<pcl::PointXYZ>);
        new_remain->points.reserve(cloud_remain->size());

        extract.setNegative(true);
        extract.filter(*new_remain);

        plane_coeffs.push_back(coeff);
        plane_clouds.push_back(plane);

        cloud_remain.swap(new_remain);
    }

    if (plane_coeffs.empty()) {
        // 没拟合到
        PLOGD << "没有拟合到平面";
        return;
    }

    // ===== 选择主平面（法向最接近Y轴）=====

    int best_idx = -1;
    float best_score = -1.0f;

    for (size_t i = 0; i < plane_coeffs.size(); ++i) {
        auto& c = plane_coeffs[i]->values;

        Eigen::Vector3f normal(c[0], c[1], c[2]);
        normal.normalize();

        float score = std::abs(normal.dot(mainPlaneAxis));  // 越接近1越平行

        if (score > best_score) {
            best_score = score;
            best_idx = static_cast<int>(i);
        }
    }
    if (best_idx < 0) {
        PLOGD << "没有找到主平面";
        return;
    }
    // ===== 输出主平面 =====
    mainPlaneCoeffs = plane_coeffs[best_idx];
    float a = mainPlaneCoeffs->values[0];
    float b = mainPlaneCoeffs->values[1];
    float c = mainPlaneCoeffs->values[2];
    float d = mainPlaneCoeffs->values[3];

    float norm = std::sqrt(a * a + b * b + c * c);
    float th = Ransac_plane_Dth * norm;

    for (const auto& pt : input_cloud->points) {
        float dist = std::fabs(a * pt.x + b * pt.y + c * pt.z + d);

        if (dist < th) {
            output_cloud_plane->points.push_back(pt);
        }
    }

    output_cloud_plane->width = static_cast<uint32_t>(output_cloud_plane->size());
    output_cloud_plane->height = 1;
    output_cloud_plane->is_dense = false;

    // ===== 找垂直且面积最大的平面 =====
    pcl::ModelCoefficients::Ptr best_other_plane = nullptr;
    float max_area = 0.0f;

    Eigen::Vector3f main_normal(a, b, c);
    main_normal.normalize();

    float cos_thresh = 0.2f;

    for (size_t i = 0; i < plane_coeffs.size(); ++i) {
        if (i == best_idx) continue;

        auto& coeff = plane_coeffs[i];

        Eigen::Vector3f normal(coeff->values[0], coeff->values[1], coeff->values[2]);
        normal.normalize();

        float dot = std::abs(main_normal.dot(normal));

        if (dot < cos_thresh) {
            float area = computePlaneArea(plane_clouds[i], coeff);

            if (area > max_area) {
                max_area = area;
                best_other_plane = coeff;
            }
        }
    }

    if (best_other_plane) {
        otherPlaneCoeffs = best_other_plane;
    } else {
        PLOGD << "没找到第二平面";
    }
}
void PlatePlateFilletSeamsDet::computePlaneIntersectionLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                            pcl::PointCloud<pcl::PointXYZ>::Ptr line_cloud) {
    line_cloud->clear();

    if (!planeCoeffsWithWeldSeam || planeCoeffsWithWeldSeam->values.size() < 4) {
        PLOGE << "主平面无效";
        return;
    }

    if (!otherPlaneCoeffsInWeldArea || otherPlaneCoeffsInWeldArea->values.size() < 4) {
        PLOGE << "第二平面无效";
        return;
    }

    // ===== 取平面参数 =====
    auto& p1 = planeCoeffsWithWeldSeam->values;
    auto& p2 = otherPlaneCoeffsInWeldArea->values;

    Eigen::Vector3f n1(p1[0], p1[1], p1[2]);
    Eigen::Vector3f n2(p2[0], p2[1], p2[2]);
    float d1 = p1[3];
    float d2 = p2[3];

    // ===== 求方向向量 =====
    Eigen::Vector3f dir = n1.cross(n2);

    if (dir.norm() < 1e-6) {
        PLOGE << "两个平面平行或接近平行";
        return;
    }

    dir.normalize();

    // ===== 求交线上一点（稳定解法）=====
    Eigen::Vector3f point;

    // 选一个分量最小的轴固定为0
    Eigen::Vector3f abs_dir = dir.cwiseAbs();
    // 求解焊缝类型--后续会在轨迹规划类沿着
    int max_idx;
    abs_dir.maxCoeff(&max_idx);

    if (max_idx == 2) {
        weldType = Plate_Plate_Fillet_V;  // Z方向主导--竖直焊缝
    } else {
        weldType = Plate_Plate_Fillet_H;  // 水平焊缝
    }

    if (abs_dir.x() <= abs_dir.y() && abs_dir.x() <= abs_dir.z()) {
        // x = 0，解 y z
        Eigen::Matrix2f A;
        A << n1.y(), n1.z(), n2.y(), n2.z();

        Eigen::Vector2f b(-d1, -d2);

        Eigen::Vector2f yz = A.colPivHouseholderQr().solve(b);
        point = Eigen::Vector3f(0, yz[0], yz[1]);
    } else if (abs_dir.y() <= abs_dir.x() && abs_dir.y() <= abs_dir.z()) {
        // y = 0
        Eigen::Matrix2f A;
        A << n1.x(), n1.z(), n2.x(), n2.z();

        Eigen::Vector2f b(-d1, -d2);

        Eigen::Vector2f xz = A.colPivHouseholderQr().solve(b);
        point = Eigen::Vector3f(xz[0], 0, xz[1]);
    } else {
        // z = 0
        Eigen::Matrix2f A;
        A << n1.x(), n1.y(), n2.x(), n2.y();

        Eigen::Vector2f b(-d1, -d2);

        Eigen::Vector2f xy = A.colPivHouseholderQr().solve(b);
        point = Eigen::Vector3f(xy[0], xy[1], 0);
    }

    // ===== 保存直线系数 =====
    lineCoeffsWithWeldSeam2Val.reset(new pcl::ModelCoefficients);
    lineCoeffsWithWeldSeam2Val->values.resize(6);

    lineCoeffsWithWeldSeam2Val->values[0] = point.x();
    lineCoeffsWithWeldSeam2Val->values[1] = point.y();
    lineCoeffsWithWeldSeam2Val->values[2] = point.z();

    lineCoeffsWithWeldSeam2Val->values[3] = dir.x();
    lineCoeffsWithWeldSeam2Val->values[4] = dir.y();
    lineCoeffsWithWeldSeam2Val->values[5] = dir.z();

    // ===== 从点云中提取靠近该直线的点 =====
    for (const auto& pt : cloud->points) {
        Eigen::Vector3f p(pt.x, pt.y, pt.z);

        Eigen::Vector3f v = p - point;

        // 点到直线距离
        float dist = (v.cross(dir)).norm();
        if (dist < extendCylinderInPlaneArea) {
            line_cloud->points.push_back(pt);
        }
    }

    line_cloud->width = static_cast<uint32_t>(line_cloud->size());
    line_cloud->height = 1;
    line_cloud->is_dense = false;
}
// void PlatePlateFilletSeamsDet::filterCloudByPlaneThenMorph(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
//                                                            pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud) {
//     output_cloud->clear();
//     if (!input_cloud || input_cloud->empty()) {
//         PLOGE << "filterCloudByPlane: 输入点云为空";
//         return;
//     }

//     if (!otherPlaneCoeffsInWeldArea || otherPlaneCoeffsInWeldArea->values.size() < 4) {
//         PLOGE << "filterCloudByPlane: 第二母材平面参数无效";
//         return;
//     }
//     if (!lineCoeffsWithWeldSeam2Val || lineCoeffsWithWeldSeam2Val->values.size() < 6) {
//         PLOGE << "交线参数无效";
//         return;
//     }

//     // ===== 平面参数 =====
//     const auto& coeff = otherPlaneCoeffsInWeldArea->values;

//     float a = coeff[0];
//     float b = coeff[1];
//     float c = coeff[2];
//     float d = coeff[3];

//     // ===== 阈值（你要求 /2）=====
//     float norm = std::sqrt(a * a + b * b + c * c);
//     float th = (Ransac_plane_Dth * 0.5f) * norm;

//     std::vector<Eigen::Vector3f> pts;
//     pts.reserve(input_cloud->size());

//     for (const auto& pt : input_cloud->points) {
//         float dist = std::fabs(a * pt.x + b * pt.y + c * pt.z + d);

//         if (dist > th) {
//             pts.emplace_back(pt.x, pt.y, pt.z);
//         }
//     }

//     if (pts.empty()) {
//         PLOGW << "filterCloudByPlane: 过滤后点云为空";
//         return;
//     }
//     // ================== Step2：构建“定向坐标系” ==================

//     // 主平面法向
//     Eigen::Vector3f plane_normal(planeCoeffsWithWeldSeam->values[0], planeCoeffsWithWeldSeam->values[1], planeCoeffsWithWeldSeam->values[2]);
//     plane_normal.normalize();

//     // 交线方向（焊缝方向）
//     const auto& line = lineCoeffsWithWeldSeam2Val->values;
//     Eigen::Vector3f u_dir(line[3], line[4], line[5]);
//     u_dir.normalize();

//     // 垂直焊缝方向（腐蚀方向）
//     Eigen::Vector3f v_dir = u_dir.cross(plane_normal);

//     if (v_dir.norm() < 1e-6) {
//         PLOGE << "v_dir异常（交线与平面法向异常）";
//         return;
//     }
//     v_dir.normalize();

//     int N = pts.size();

//     std::vector<float> u(N), v(N);

// #pragma omp parallel for
//     for (int i = 0; i < N; ++i) {
//         u[i] = pts[i].dot(u_dir);
//         v[i] = pts[i].dot(v_dir);
//     }
//     PLOGD << "开始栅格化";
//     // ================== Step3：栅格 ==================
//     float uMin = *std::min_element(u.begin(), u.end());
//     float uMax = *std::max_element(u.begin(), u.end());
//     float vMin = *std::min_element(v.begin(), v.end());
//     float vMax = *std::max_element(v.begin(), v.end());
//     // 检查范围是否合理，防止栅格尺寸过大
//     if ((uMax - uMin) / resolution > 10000 || (vMax - vMin) / resolution > 10000) {
//         PLOGE << "栅格尺寸过大，可能导致内存问题";
//         return;
//     }

//     int W = static_cast<int>((uMax - uMin) / resolution) + 1;
//     int H = static_cast<int>((vMax - vMin) / resolution) + 1;

//     std::vector<uint8_t> mask(W * H, 0);
//     std::vector<std::vector<std::vector<int>>> grid(H, std::vector<std::vector<int>>(W));
// #pragma omp parallel for
//     for (int i = 0; i < N; ++i) {
//         int x = static_cast<int>((u[i] - uMin) / resolution);
//         int y = static_cast<int>((v[i] - vMin) / resolution);

//         if (x >= 0 && x < W && y >= 0 && y < H) {
// #pragma omp critical
//             {
//                 grid[y][x].push_back(i);
//                 mask[y * W + x] = 1;
//             }
//         }
//     }

//     // ================== Step4：定向腐蚀（沿 v 方向） ==================
//     int radius = 1;
//     std::vector<uint8_t> eroded(W * H, 0);

// #pragma omp parallel for
//     for (int y = 0; y < H; ++y) {
//         for (int x = 0; x < W; ++x) {
//             bool ok = true;

//             for (int dy = -radius; dy <= radius; ++dy) {
//                 int yy = y + dy;

//                 if (yy < 0 || yy >= H || !mask[yy * W + x]) {
//                     ok = false;
//                     break;
//                 }
//             }

//             if (ok) eroded[y * W + x] = 1;
//         }
//     }
//     // ================== Step5：定向膨胀 ==================
//     std::vector<uint8_t> opened(W * H, 0);

// #pragma omp parallel for
//     for (int y = 0; y < H; ++y) {
//         for (int x = 0; x < W; ++x) {
//             bool ok = false;

//             for (int dy = -radius; dy <= radius; ++dy) {
//                 int yy = y + dy;

//                 if (yy >= 0 && yy < H && eroded[yy * W + x]) {
//                     ok = true;
//                     break;
//                 }
//             }

//             if (ok) opened[y * W + x] = 1;
//         }
//     }

//     // ================== Step6：恢复点云 ==================
//     output_cloud->points.reserve(pts.size());

//     for (int y = 0; y < H; ++y) {
//         for (int x = 0; x < W; ++x) {
//             if (opened[y * W + x]) {
//                 for (int idx : grid[y][x]) {
//                     output_cloud->push_back(pcl::PointXYZ(pts[idx].x(), pts[idx].y(), pts[idx].z()));
//                 }
//             }
//         }
//     }
//     output_cloud->width = static_cast<uint32_t>(output_cloud->size());
//     output_cloud->height = 1;
//     output_cloud->is_dense = false;
// }
void PlatePlateFilletSeamsDet::filterCloudByPlaneThenMorph(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                                           pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud) {
    output_cloud->clear();

    if (!input_cloud || input_cloud->empty()) {
        PLOGE << "输入点云为空";
        return;
    }

    if (!otherPlaneCoeffsInWeldArea || otherPlaneCoeffsInWeldArea->values.size() < 4) {
        PLOGE << "第二母材平面参数无效";
        return;
    }

    if (!lineCoeffsWithWeldSeam2Val || lineCoeffsWithWeldSeam2Val->values.size() < 6) {
        PLOGE << "交线参数无效";
        return;
    }

    // ================== Step1：平面过滤 ==================
    const auto& coeff = otherPlaneCoeffsInWeldArea->values;

    float a = coeff[0];
    float b = coeff[1];
    float c = coeff[2];
    float d = coeff[3];

    float norm = std::sqrt(a * a + b * b + c * c);
    float th = (Ransac_plane_Dth)*norm;

    std::vector<Eigen::Vector3f> pts;
    pts.reserve(input_cloud->size());

    for (const auto& pt : input_cloud->points) {
        float dist = std::fabs(a * pt.x + b * pt.y + c * pt.z + d);

        if (dist > th) {
            pts.emplace_back(pt.x, pt.y, pt.z);
        }
    }

    if (pts.empty()) {
        PLOGW << "平面过滤后为空";
        return;
    }

    // ================== Step2：坐标系 ==================
    Eigen::Vector3f plane_normal(planeCoeffsWithWeldSeam->values[0], planeCoeffsWithWeldSeam->values[1], planeCoeffsWithWeldSeam->values[2]);
    plane_normal.normalize();

    const auto& line = lineCoeffsWithWeldSeam2Val->values;
    Eigen::Vector3f u_dir(line[3], line[4], line[5]);
    u_dir.normalize();

    Eigen::Vector3f v_dir = u_dir.cross(plane_normal);
    if (v_dir.norm() < 1e-6) {
        PLOGE << "v_dir异常";
        return;
    }
    v_dir.normalize();

    int N = pts.size();

    std::vector<float> u(N), v(N);

#pragma omp parallel for
    for (int i = 0; i < N; ++i) {
        u[i] = pts[i].dot(u_dir);
        v[i] = pts[i].dot(v_dir);
    }

    // ================== Step3：分桶 ==================
    float uMin = *std::min_element(u.begin(), u.end());
    float uMax = *std::max_element(u.begin(), u.end());

    int W = static_cast<int>((uMax - uMin) / resolution) + 1;

    std::vector<std::vector<int>> bins(W);

    for (int i = 0; i < N; ++i) {
        int x = static_cast<int>((u[i] - uMin) / resolution);
        if (x >= 0 && x < W) {
            bins[x].push_back(i);
        }
    }

    // ================== Step4：宽度 ==================
    std::vector<float> widths(W, 0);

    for (int x = 0; x < W; ++x) {
        if (bins[x].empty()) continue;

        float vmin = 1e9f, vmax = -1e9f;

        for (int idx : bins[x]) {
            vmin = std::min(vmin, v[idx]);
            vmax = std::max(vmax, v[idx]);
        }

        widths[x] = vmax - vmin;
    }

    // ================== Step5.1：平滑 ==================
    std::vector<float> smooth(W, 0);

    int k = 2;  // 平滑窗口

    for (int i = 0; i < W; ++i) {
        float sum = 0;
        int cnt = 0;

        for (int j = -k; j <= k; ++j) {
            int idx = i + j;
            if (idx >= 0 && idx < W && widths[idx] > 0) {
                sum += widths[idx];
                cnt++;
            }
        }

        if (cnt > 0) smooth[i] = sum / cnt;
    }

    // ================== Step5.2：找有效区域 ==================
    float max_w = 0;
    for (float w : smooth) {
        if (w > max_w) max_w = w;
    }

    if (max_w <= 0) {
        PLOGW << "宽度异常";
        return;
    }
    float mth = max_w * 0.5f;

    std::vector<uint8_t> valid(W, 0);

    for (int i = 0; i < W; ++i) {
        if (smooth[i] >= mth) {
            valid[i] = 1;
        }
    }

    // ================== Step5.3：最长连续段 ==================
    int best_start = 0, best_len = 0;
    int cur_start = 0, cur_len = 0;

    for (int i = 0; i < W; ++i) {
        if (valid[i]) {
            if (cur_len == 0) cur_start = i;
            cur_len++;
        } else {
            if (cur_len > best_len) {
                best_len = cur_len;
                best_start = cur_start;
            }
            cur_len = 0;
        }
    }

    if (cur_len > best_len) {
        best_len = cur_len;
        best_start = cur_start;
    }

    if (best_len == 0) {
        PLOGW << "没有找到连续焊缝段";
        return;
    }

    int bestleft = best_start;
    int bestright = best_start + best_len - 1;

    // ================== Step5.4：连续段平均宽度 ==================
    float ref_width = 0.0f;
    int cnt = 0;

    for (int i = bestleft; i <= bestright; ++i) {
        if (smooth[i] > 0) {
            ref_width += smooth[i];
            cnt++;
        }
    }

    if (cnt == 0) {
        PLOGW << "连续段宽度统计失败";
        return;
    }

    ref_width /= cnt;

    // ================== Step5.5：输出阈值 ==================
    float width_min = ref_width * 0.9f;
    float width_max = ref_width * 2.0f;

    // ================== Step6：从两端收缩 ==================

    int left = 0;
    while (left < W) {
        if (widths[left] >= width_min && widths[left] <= width_max) {
            break;
        }
        left++;
    }

    int right = W - 1;
    while (right >= 0) {
        if (widths[right] >= width_min && widths[right] <= width_max) {
            break;
        }
        right--;
    }

    if (left >= right) {
        PLOGW << "没有找到有效焊缝区间";
        return;
    }
    int expand = 2;
    left = std::max(0, left - expand);
    right = std::min(W - 1, right + expand);

    // ================== Step7：恢复点云 ==================
    output_cloud->points.reserve(N);

    for (int x = left; x <= right; ++x) {
        for (int idx : bins[x]) {
            output_cloud->points.emplace_back(pts[idx].x(), pts[idx].y(), pts[idx].z());
        }
    }

    output_cloud->width = static_cast<uint32_t>(output_cloud->size());
    output_cloud->height = 1;
    output_cloud->is_dense = false;
}
bool PlatePlateFilletSeamsDet::solveSeamEndPoints() {
    filletSeamsTSP.clear();

    if (!cloudPlaneInWeldAreaWithSeam || cloudPlaneInWeldAreaWithSeam->empty()) {
        PLOGE << "点云为空";
        return false;
    }

    if (!lineCoeffsWithWeldSeam2Val || lineCoeffsWithWeldSeam2Val->values.size() < 6) {
        PLOGE << "直线参数无效";
        return false;
    }

    // ================== Step1：直线参数 ==================
    const auto& line = lineCoeffsWithWeldSeam2Val->values;

    Eigen::Vector3f P0(line[0], line[1], line[2]);   // 直线上一点
    Eigen::Vector3f dir(line[3], line[4], line[5]);  // 方向
    dir.normalize();

    std::vector<float> proj_vals;
    proj_vals.reserve(cloudPlaneInWeldAreaWithSeam->size());

    std::vector<Eigen::Vector3f> proj_pts;
    proj_pts.reserve(cloudPlaneInWeldAreaWithSeam->size());

    // ================== Step2：筛选 + 投影 ==================
    for (const auto& pt : cloudPlaneInWeldAreaWithSeam->points) {
        Eigen::Vector3f P(pt.x, pt.y, pt.z);

        // 点到直线距离
        Eigen::Vector3f v = P - P0;
        Eigen::Vector3f proj = v.dot(dir) * dir;
        Eigen::Vector3f perp = v - proj;

        float dist = perp.norm();

        if (dist <= max_dist) {
            float t = v.dot(dir);  // 投影标量（沿直线坐标）

            proj_vals.push_back(t);
            proj_pts.push_back(P0 + t * dir);
        }
    }

    if (proj_vals.size() < 10) {
        PLOGW << "有效点太少";
        return false;
    }

    // ================== Step3：找端点 ==================
    auto minmax = std::minmax_element(proj_vals.begin(), proj_vals.end());

    float t_min = *minmax.first;
    float t_max = *minmax.second;

    Eigen::Vector3f pt_start = P0 + t_min * dir;
    Eigen::Vector3f pt_end = P0 + t_max * dir;

    // ================== Step4：保存 ==================
    filletSeamsTSP.emplace_back(pt_start.x(), pt_start.y(), pt_start.z());
    filletSeamsTSP.emplace_back(pt_end.x(), pt_end.y(), pt_end.z());
    if (saveFlag) {
        seamEndPoints->points.emplace_back(pt_start.x(), pt_start.y(), pt_start.z());
        seamEndPoints->points.emplace_back(pt_end.x(), pt_end.y(), pt_end.z());
    }
    return true;
}
