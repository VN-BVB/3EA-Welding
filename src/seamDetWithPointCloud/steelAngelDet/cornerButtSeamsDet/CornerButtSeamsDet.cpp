#include "CornerButtSeamsDet.h"

#include "seamDetWithPointCloud/QhullLock.h"
#include "utils/common/CommonFunc.h"
#include "utils/common/WeldSeamInfo.h"
#include "utils/pointCloud/PointCloudFunc.h"

CornerButtSeamsDet::CornerButtSeamsDet(QObject* parent) : AbstractSeamDet{parent} {}

// 求解焊缝
std::vector<std::shared_ptr<WeldSeamInfo>> CornerButtSeamsDet::solveSeamsEndPoints(
    std::vector<std::shared_ptr<WeldSeamInfo>> seamsInfo) {
    PLOGD << "CornerButtSeamsDet::solveSeamsEndPoints In";
    tempWeldSeamsInfo = seamsInfo;  // 传入的焊缝信息

    // 焊缝求解
    for (int i = 0; i < tempWeldSeamsInfo.size(); i++) {
        SingleSeam_Reinitialize();                         // 单条焊缝检测前，变量重新初始化
        cloud = tempWeldSeamsInfo[i]->weldAreaPointCloudInCamera;  // 获取焊缝区域点云

        if (cloud->size() == 0) {
            tempWeldSeamsInfo[i]->detectSuccFlag = false;
            continue;
        }

        MyToolFunc::myFastMaxCluster(cloud, Max_Cluster_radius);  // 快速欧式聚类提取最大点集
        Ransac_plane(cloud, cloud_plane_interior);                // Ransac拟合平面，并输出平面的内点集合
        Statistic_filter(cloud_plane_interior);  // 统计滤波易造成小空洞，从而造成伪焊缝点的生成，因此参数sigma应设置的大些
        Project_ToPlane(cloud_plane_interior, cloud_plane_projected);  // 将平面点云集合进行平面投影
        if (saveAndOutputDebugInformation == true) {
            cloud_plane_projected->height = 1;  // 解决由于height和width未被自动赋值导致的程序崩溃问题
            cloud_plane_projected->width = static_cast<uint32_t>(cloud_plane_projected->size());
            pcl::io::savePCDFile("./data/seamDetWithPointCloud/Corner/cloud_plane_projected.pcd", *cloud_plane_projected);
        }

        Identify_boundary(cloud_plane_projected);                      // 筛选焊缝候选点集
        bool bool_IdentifySeam = Sovle_SeamEndpoints(cloud_boundary);  // 提取焊缝端点，方法为拟合直线，求取直线交点

        // 打印检测结果
        PLOGD << "边角对接焊缝端点识别结果: " << bool_IdentifySeam;
        if (cornerButtSeams.size() == 2) {
            PLOGD << "焊缝端点坐标: (" << cornerButtSeams[0].x << " " << cornerButtSeams[0].y << " " << cornerButtSeams[0].z
                  << ") (" << cornerButtSeams[1].x << " " << cornerButtSeams[1].y << " " << cornerButtSeams[1].z << ")";
        }

        // 保存本次检测到的信息
        tempWeldSeamsInfo[i]->detectSuccFlag = bool_IdentifySeam;  // 检测是否成功标志位
        if (bool_IdentifySeam == true) {
            tempWeldSeamsInfo[i]->weldEndPointsInCamera.reset(
                new std::vector<pcl::PointXYZ>(std::move(cornerButtSeams)));  // 检测结果
            tempWeldSeamsInfo[i]->weldPlane = planeCoeffsWithWeldSeam;        // 焊缝所在平面
            if (tempWeldSeamsInfo[i]->weldAreaType == WELD_AREA_TYPE::BACK_CORNER) {
                tempWeldSeamsInfo[i]->weldType = WELD_TYPE::BACK_CORNER_BUTT;  // 焊缝类型
            } else if (tempWeldSeamsInfo[i]->weldAreaType == WELD_AREA_TYPE::FRONT_CORNER) {
                tempWeldSeamsInfo[i]->weldType = WELD_TYPE::FRONT_CORNER_BUTT;  // 焊缝类型
            }
            tempWeldSeamsInfo[i]->seamsLineToVal = seamsLineToVal;  // 用于验证正确性的直线
        }
    }

    PLOGD << "CornerButtSeamsDet::solveSeamsEndPoints Out";

    return tempWeldSeamsInfo;
}

// Ransac拟合平面，并输出平面的内点集合
void CornerButtSeamsDet::Ransac_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud) {
    // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(Ransac_Plane_Iterations);
    seg.setDistanceThreshold(Ransac_plane_Dth);
    // 距离阈值表示点到估计模型的距离最大值。
    seg.setInputCloud(input_cloud);  // 输入点云
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.segment(*inliers, *planeCoeffsWithWeldSeam);  // 实现分割，并存储分割结果到点集合inliers及存储平面模型系数coefficients
    PLOGD << "inliers->indices.size(): " << inliers->indices.size();
    pcl::copyPointCloud(*input_cloud, inliers->indices, *output_cloud);
}

// 统计滤波
void CornerButtSeamsDet::Statistic_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input_cloud);       // 设置待滤波的点云
    sor.setMeanK(Statistic_NeighPoints);  // 设置在进行统计时考虑查询点邻近点数
    sor.setStddevMulThresh(Statistic_sigma);  // 设置判断是否为离群点的阈值，里边的数字表示标准差的倍数，1个标准差以上就是离群点。
    sor.filter(*input_cloud);  // 存储内点
}

// 点云投影至指定平面
void CornerButtSeamsDet::Project_ToPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud) {
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(input_cloud);
    proj.setModelCoefficients(planeCoeffsWithWeldSeam);
    proj.filter(*output_cloud);
}

// 筛选焊缝候选点集
void CornerButtSeamsDet::Identify_boundary(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
    // 创建alpha_shapes对象
    pcl::ConcaveHull<pcl::PointXYZ> alpha_shapes;  // alpha shapes
    alpha_shapes.setInputCloud(input_cloud);

    {
        std::lock_guard<std::mutex> lock(getQhullMutex());  // 锁作用域仅限此块

        // 检测边界
        alpha_shapes.setAlpha(Alpha_Radius);
        alpha_shapes.reconstruct(*cloud_boundary);
    }
}

// 求点云聚类中各点之间的最大距离
double CornerButtSeamsDet::getMaxDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
    double max_dist = std::numeric_limits<double>::min();
    int i_min = -1, i_max = -1;
#pragma omp parallel for
    for (int i = 0; i < input_cloud->points.size(); ++i) {
        for (int j = i; j < input_cloud->points.size(); ++j) {
            // Compute the distance
            double dist = (input_cloud->points[i].getVector4fMap() - input_cloud->points[j].getVector4fMap()).squaredNorm();
            if (dist > max_dist) {
                max_dist = dist;
                i_min = i;
                i_max = j;
            }
        }
    }
    if (i_min == -1 || i_max == -1) return (max_dist = std::numeric_limits<double>::min());
    return (std::sqrt(max_dist));
}

// 提取焊缝端点，方法为拟合直线，求取直线交点
bool CornerButtSeamsDet::Sovle_SeamEndpoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundaries) {
    if (saveAndOutputDebugInformation == true) {
        cloud_boundaries->height = 1;  // 解决由于height和width未被自动赋值导致的程序崩溃问题
        cloud_boundaries->width = static_cast<uint32_t>(cloud_boundaries->size());
        pcl::io::savePCDFile("./data/seamDetWithPointCloud/Corner/cloud_boundaries.pcd", *cloud_boundaries);
    }

    // 直线Ransac参数
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZ> seg;            // 创建拟合对象
    seg.setOptimizeCoefficients(true);                  // 设置对估计模型参数进行优化处理
    seg.setModelType(pcl::SACMODEL_LINE);               // 设置拟合模型为直线模型
    seg.setMethodType(pcl::SAC_RANSAC);                 // 设置拟合方法为RANSAC
    seg.setMaxIterations(1000);                         // 设置最大迭代次数
    seg.setDistanceThreshold(Ransac_BoundaryLine_Dth);  // 判断是否为模型内点的距离阀值/设置误差容忍范围

    // 拟合6条直线
    // int nr_points = cloud_boundaries->points.size();
    for (int i = 0; i < 6; i++) {  // 执行6次
        if (cloud_boundaries->points.size() == 0) {
            PLOGE << "错误1，焊缝区域拍摄不完整，无法准确提取焊缝";
            return false;
        }
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        seg.setInputCloud(cloud_boundaries);   // 输入点云
        seg.segment(*inliers, *coefficients);  // 内点的索引，模型系数

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr outside(new pcl::PointCloud<pcl::PointXYZ>);

        // 提取内点
        pcl::ExtractIndices<pcl::PointXYZ> extract;  // 创建点云提取对象
        extract.setInputCloud(cloud_boundaries);
        extract.setIndices(inliers);
        extract.setNegative(false);  // 设置为false，表示提取内点
        extract.filter(*cloud_line);

        // 欧式聚类，提取内点的最大点集
        MyToolFunc::myFastMaxCluster(cloud_line, EucSeg_Dth);

        // 计算内点集合的最大距离
        pcl::PointXYZ pmin, pmax;
        double line_distance = pcl::getMaxSegment(*cloud_line, pmin, pmax);
        Six_BoundaryLinesInliers.push_back(*cloud_line);
        Six_BoundaryLinesCoff.push_back(coefficients);
        Six_BoundaryLines_Length.push_back(line_distance);

        // 将剩余的外点，重新赋值给初值，用于再次循环拟合直线
        extract.setNegative(true);       // true提取外点（该直线之外的点）
        extract.filter(*outside);        // outside为外点点云
        cloud_boundaries.swap(outside);  // 将cloud_f中的点云赋值给cloud_boundaries
    }

    // 暂存原本的直线长度顺序
    std::vector<double> Six_BoundaryLines_Length_Temp;
    if (saveAndOutputDebugInformation == true) {
        Six_BoundaryLines_Length_Temp = Six_BoundaryLines_Length;
    }

    // 对各边界线段根据长度进行排序，并返回其索引值
    std::vector<int> sort_idxes = MyToolFunc::sortVetorIndexMax2Min(Six_BoundaryLines_Length);

    // 保存检测到的直线，用于可视化
    // for (int i = 0; i < 6; i++) {
    //     lineCoeffForDebug.push_back(Six_BoundaryLinesCoff[i]);
    // }

    // 根据6条直线的长度，压入sorted_vector
    for (int i = 0; i < 6; i++) {
        SortedSix_BoundaryLinesCoff.push_back(Six_BoundaryLinesCoff[sort_idxes[i]]);
        SortedSix_BoundaryLinesInliers.push_back(Six_BoundaryLinesInliers[sort_idxes[i]]);
    }

    // 打印直线参数
    if (saveAndOutputDebugInformation == true) {
        for (int i = 0; i < 6; i++) {
            std::cout << "SortedSix_BoundaryLinesCoff: " << SortedSix_BoundaryLinesCoff[i]->values[0] << "  "
                      << SortedSix_BoundaryLinesCoff[i]->values[1] << "  " << SortedSix_BoundaryLinesCoff[i]->values[2] << "  "
                      << SortedSix_BoundaryLinesCoff[i]->values[3] << "  " << SortedSix_BoundaryLinesCoff[i]->values[4] << "  "
                      << SortedSix_BoundaryLinesCoff[i]->values[5] << "    L:" << Six_BoundaryLines_Length_Temp[sort_idxes[i]]
                      << std::endl;
        }
    }

    // // 最长直线的方向
    // Eigen::Vector3f Line_1st_Dir(SortedSix_BoundaryLinesCoff[0]->values[3], SortedSix_BoundaryLinesCoff[0]->values[4],
    //                              SortedSix_BoundaryLinesCoff[0]->values[5]);
    // // 次长直线的方向
    // Eigen::Vector3f Line_2st_Dir(SortedSix_BoundaryLinesCoff[1]->values[3], SortedSix_BoundaryLinesCoff[1]->values[4],
    //                              SortedSix_BoundaryLinesCoff[1]->values[5]);

    // 直线方向向量（根据直线长度由长到短排序）
    std::vector<Eigen::Vector3f> lineDirSortedByLength;
    for (int i = 0; i < 6; i++) {
        lineDirSortedByLength.push_back(Eigen::Vector3f(SortedSix_BoundaryLinesCoff[i]->values[3],
                                                        SortedSix_BoundaryLinesCoff[i]->values[4],
                                                        SortedSix_BoundaryLinesCoff[i]->values[5]));
    }

    // 计算最长直线和其它直线的夹角
    std::vector<double> angleWith1st;
    for (int i = 0; i < 6; i++) {
        // 计算最长直线与其它直线的夹角
        angleWith1st.push_back(MyToolFunc::getLineAngle(lineDirSortedByLength[0], lineDirSortedByLength[i]));
    }
    if (saveAndOutputDebugInformation == true) {
        for (int i = 0; i < 6; i++) {
            std::cout << "angleWith1st: " << angleWith1st[i] << std::endl;
        }
    }

    // 与 最长直线 垂直的直线 中的最长直线 的序号（也就是外侧直线中短的那条）
    int IndexOfOutsideShortLine = 0;

    // 最长直线和次长直线的垂直夹角阈值判断
    if (angleWith1st[1] < Angle_Vertical_1st2st_Threshold) {
        // 优化算法：如果最长的两条线不是最外侧的两条，则继续寻找最外侧直线
        for (int i = 2; i < 6; i++) {
            std::cout << "原本的算法失效，开始执行优化算法..." << std::endl;
            if (angleWith1st[i] > Angle_Vertical_1st2st_Threshold) {
                IndexOfOutsideShortLine = i;
                break;
            }
        }

        if (IndexOfOutsideShortLine == 0) {
            PLOGE << "错误2，焊缝区域拍摄不完整，或最长的两条线不是最外侧的两条，无法准确提取焊缝，angleWith1st[1]："
                  << angleWith1st[1];
            return false;
        }
    } else {
        IndexOfOutsideShortLine = 1;
    }

    if (saveAndOutputDebugInformation == true) {
        std::cout << "外侧边界线中短的那条的序号，IndexOf1stAmongLinesPerpendicularTo1st：" << IndexOfOutsideShortLine
                  << std::endl;
    }

    int Index_NearestParallelLineTo1st = 0;  // 与 最长 边界线最近的平行线序号
    int Index_NearestParallelLineTo2st = 0;  // 与 次长 边界线最近的平行线序号

    // 求外侧两条以外的直线中，与外侧长边界线的最近平行线序号
    double DisTo1st_temp = 10000000;  // 距离最长直线的临时比较值
    for (int i = 1; i < 6; i++) {
        Eigen::Vector3f Line_i_Dir(SortedSix_BoundaryLinesCoff[i]->values[3], SortedSix_BoundaryLinesCoff[i]->values[4],
                                   SortedSix_BoundaryLinesCoff[i]->values[5]);
        double angle = MyToolFunc::getLineAngle(Line_i_Dir, lineDirSortedByLength[0]);  // 计算当前直线和最长直线的夹角

        if (angle < Angle_ParalleThreshold) {  // 平行线判断阈值 10°
            // 求出当前直线内点的质心
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(SortedSix_BoundaryLinesInliers[i], centroid);
            // 计算质心到最长直线的距离
            double distance = MyToolFunc::getPoint2LineDis(centroid, SortedSix_BoundaryLinesCoff[0]);
            if (distance < DisTo1st_temp) {
                Index_NearestParallelLineTo1st = i;  // 更新最长直线的最近平行线序号
                DisTo1st_temp = distance;            // 更新最长直线的最近平行线距离
            }
        }
    }
    // 求外侧两条以外的直线中，与外侧短边界线的最近平行线序号
    double DisTo2st_temp = 10000000;  // 距离次长直线的临时比较值
    for (int i = 1; i < 6; i++) {
        if (i != IndexOfOutsideShortLine) {
            Eigen::Vector3f Line_i_Dir(SortedSix_BoundaryLinesCoff[i]->values[3], SortedSix_BoundaryLinesCoff[i]->values[4],
                                       SortedSix_BoundaryLinesCoff[i]->values[5]);
            // double angle = getLineAngle(Line_i_Dir, lineDirSortedByLength[1]);  // 计算当前直线和外侧直线中短的那条的夹角
            double angle = MyToolFunc::getLineAngle(
                Line_i_Dir,
                lineDirSortedByLength[IndexOfOutsideShortLine]);  // 计算当前直线和外侧直线中短的那条的夹角

            if (angle < Angle_ParalleThreshold) {  // 平行线判断阈值 10°
                // 求出当前直线内点的质心
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(SortedSix_BoundaryLinesInliers[i], centroid);
                // 计算质心到次长直线的距离
                double distance = MyToolFunc::getPoint2LineDis(centroid, SortedSix_BoundaryLinesCoff[1]);
                if (distance < DisTo2st_temp) {
                    Index_NearestParallelLineTo2st = i;  // 更新次长直线的最近平行线序号
                    DisTo2st_temp = distance;            // 更新次长直线的最近平行线距离
                }
            }
        }
    }
    // std::cout << "Index_NearestParallelLineTo1st" << Index_NearestParallelLineTo1st <<std::endl;
    // std::cout << "Index_NearestParallelLineTo2st" << Index_NearestParallelLineTo2st <<std::endl;

    // 判断焊缝是否可以被正确识别
    if ((Index_NearestParallelLineTo1st == 0) || (Index_NearestParallelLineTo2st == 0) ||
        (Index_NearestParallelLineTo1st == Index_NearestParallelLineTo2st)) {
        PLOGE << "错误3，焊缝区域拍摄不完整，无法准确提取焊缝，Index_NearestParallelLineTo1st：" << Index_NearestParallelLineTo1st
              << "  Index_NearestParallelLineTo2st:" << Index_NearestParallelLineTo2st;
        return false;
    }

    // 两个外侧的边界线相交、两个内侧的边界线相交
    Eigen::Vector4f Intersection_VecPoint1, Intersection_VecPoint2;
    pcl::lineWithLineIntersection(*SortedSix_BoundaryLinesCoff[0], *SortedSix_BoundaryLinesCoff[IndexOfOutsideShortLine],
                                  Intersection_VecPoint1);
    pcl::lineWithLineIntersection(*SortedSix_BoundaryLinesCoff[Index_NearestParallelLineTo1st],
                                  *SortedSix_BoundaryLinesCoff[Index_NearestParallelLineTo2st], Intersection_VecPoint2);
    pcl::PointXYZ intersectionPoint1 = {Intersection_VecPoint1[0], Intersection_VecPoint1[1], Intersection_VecPoint1[2]};
    pcl::PointXYZ intersectionPoint2 = {Intersection_VecPoint2[0], Intersection_VecPoint2[1], Intersection_VecPoint2[2]};

    // 求解最长边界的两个端点，并求两个端点和边界线交点的距离，获取最小距离，作为延伸基准距离
    // vector<pcl::PointXYZ> MaxLenBoundary_Endpoints = Solve_LineEndoints(SortedSix_BoundaryLinesInliers[0].makeShared(),
    // SortedSix_BoundaryLinesCoff[0]); double dis_1 = pcl::euclideanDistance(Intersection_Point1, MaxLenBoundary_Endpoints[0]);
    // double dis_2 = pcl::euclideanDistance(Intersection_Point1, MaxLenBoundary_Endpoints[1]);
    // double Distance_IntersectionToEndpoint = (dis_1 < dis_2) ? dis_1 : dis_2;
    // std::cout << "Distance_IntersectionToEndpoint:" << Distance_IntersectionToEndpoint <<std::endl;

    // 求解最外侧两个边界的四个端点，求四个端点和和边界线交点的距离，获取最小距离，作为延伸基准距离
    std::vector<pcl::PointXYZ> outsideLongLineEndpoints =
        Solve_LineEndoints(SortedSix_BoundaryLinesInliers[0].makeShared(), SortedSix_BoundaryLinesCoff[0]);
    std::vector<pcl::PointXYZ> outsideShortLineEndpoints =
        Solve_LineEndoints(SortedSix_BoundaryLinesInliers[IndexOfOutsideShortLine].makeShared(),
                           SortedSix_BoundaryLinesCoff[IndexOfOutsideShortLine]);
    std::vector<double> disVector;
    disVector.push_back(pcl::euclideanDistance(intersectionPoint1, outsideLongLineEndpoints[0]));  // 放入外侧长边界的两个端点
    disVector.push_back(pcl::euclideanDistance(intersectionPoint1, outsideLongLineEndpoints[1]));
    disVector.push_back(pcl::euclideanDistance(intersectionPoint1, outsideShortLineEndpoints[0]));  // 放入外侧短边界的两个端点
    disVector.push_back(pcl::euclideanDistance(intersectionPoint1, outsideShortLineEndpoints[1]));
    double Distance_IntersectionToEndpoint = *std::min_element(disVector.begin(), disVector.end());
    // std::cout << "Distance_IntersectionToEndpoint:" << Distance_IntersectionToEndpoint << std::endl;

    // 计算外侧两个边界的角平分线，用于验证焊缝计算的正确性
    pcl::ModelCoefficients::Ptr lineCoeff(new pcl::ModelCoefficients());
    if (disVector.size() == 4) {
        pcl::PointXYZ vectorEnd1, vectorEnd2;  // 方向向量端点
        if (disVector[0] > disVector[1]) {
            vectorEnd1 = outsideLongLineEndpoints[0];
        } else {
            vectorEnd1 = outsideLongLineEndpoints[1];
        }
        if (disVector[2] > disVector[3]) {
            vectorEnd2 = outsideShortLineEndpoints[0];
        } else {
            vectorEnd2 = outsideShortLineEndpoints[1];
        }

        // 获得交点指向远端端点的向量
        Eigen::Vector3d vector1(vectorEnd1.x - intersectionPoint1.x, vectorEnd1.y - intersectionPoint1.y,
                                vectorEnd1.z - intersectionPoint1.z);
        Eigen::Vector3d vector2(vectorEnd2.x - intersectionPoint1.x, vectorEnd2.y - intersectionPoint1.y,
                                vectorEnd2.z - intersectionPoint1.z);
        vector1 /= vector1.norm();  // 单位化
        vector2 /= vector2.norm();

        // 获得角平分线
        Eigen::Vector3d vectorLine = vector1 + vector2;
        vectorLine /= vectorLine.norm();

        // 生成直线参数

        lineCoeff->values.resize(6);
        lineCoeff->values[0] = intersectionPoint1.x;
        lineCoeff->values[1] = intersectionPoint1.y;
        lineCoeff->values[2] = intersectionPoint1.z;
        lineCoeff->values[3] = vectorLine[0];
        lineCoeff->values[4] = vectorLine[1];
        lineCoeff->values[5] = vectorLine[2];

        // seamsLineToVal = lineCoeff;
        // seamsLineToVal.push_back(lineCoeff);  // 在下方与焊缝检测结果一起保存
    }

    // 判断焊缝区域点云是否拍摄完整，以决定是否输出焊缝结果
    if ((Distance_IntersectionToEndpoint > Min_Dth_IntersectionToEndpoint) &&
        (Distance_IntersectionToEndpoint < Max_Dth_IntersectionToEndpoint)) {
        // 焊缝端点确认，方法为沿着焊缝直线方向进行伸缩,即以基准距离的1.2倍数(实验所得)，从交点沿焊缝进行缩减
        Eigen::Vector3f lineDir(intersectionPoint2.x - intersectionPoint1.x, intersectionPoint2.y - intersectionPoint1.y,
                                intersectionPoint2.z - intersectionPoint1.z);
        lineDir = lineDir / lineDir.norm();
        Eigen::Vector3f VecEndpoint1 = Intersection_VecPoint1.head<3>() + lineDir * Distance_IntersectionToEndpoint * 1.2;
        // Eigen::Vector3f VecEndpoint1 = Intersection_VecPoint1.head<3>();  // 原始交点连成的线段
        pcl::PointXYZ Seam_endpoint1, Seam_endpoint2;
        Seam_endpoint1.getVector3fMap() = VecEndpoint1;
        Seam_endpoint2 = intersectionPoint2;
        CurrentSeam_endpoints.push_back(Seam_endpoint1);  // 当前焊缝的端点
        CurrentSeam_endpoints.push_back(Seam_endpoint2);
        cornerButtSeams = CurrentSeam_endpoints;  // 各条焊缝的端点
        // DetectedSeamAreas_Pointclouds.push_back(cloud);    // 焊缝区域点云
        seamsLineToVal = lineCoeff;  // 用于验证的外侧两边界线的角平分线
        return true;
    } else {
        PLOGE << "错误4，焊缝区域拍摄不完整，无法准确提取焊缝 ";
        PLOGE << "具体原因: Distance_IntersectionToEndpoint:" << Distance_IntersectionToEndpoint;
        return false;
    }
}

// 对每条ransac的直线，进行 1.排序 3.投影 4.计算端点
std::vector<pcl::PointXYZ> CornerButtSeamsDet::Solve_LineEndoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line,
                                                                  pcl::ModelCoefficients::Ptr coefficients) {
    //-------------------对提取的每一线段点集，进行排序和直线投影
    // 1.排序
    std::vector<std::pair<int, double>> idxSorted(cloud_line->size());
    for (int j = 0; j < cloud_line->size(); j++) {
        idxSorted[j].first = j;
        idxSorted[j].second = cloud_line->points[j].getVector3fMap().dot(
            Eigen::Map<Eigen::Vector3f>(const_cast<float*>(coefficients->values.data() + 3), 3));
    }
    std::sort(idxSorted.begin(), idxSorted.end(),
              [](const std::pair<int, double>& lhs, const std::pair<int, double>& rhs) { return lhs.second < rhs.second; });
    pcl::PointCloud<pcl::PointXYZ>::Ptr sorted_cloud_line(new pcl::PointCloud<pcl::PointXYZ>);
    for (int j = 0; j < cloud_line->size(); j++) {
        sorted_cloud_line->push_back(cloud_line->points[idxSorted[j].first]);  // 获取排序后的点云
    }
    // 2.投影
    pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud_line(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ProjectInliers<pcl::PointXYZ> projector;
    projector.setModelType(pcl::SACMODEL_LINE);
    projector.setInputCloud(sorted_cloud_line);
    projector.setModelCoefficients(coefficients);
    projector.filter(*projected_cloud_line);
    // 输出每条线段的两个端点
    std::vector<pcl::PointXYZ> line_endpoints;  // 每条直线的两个端点
    line_endpoints.push_back(projected_cloud_line->points.front());
    line_endpoints.push_back(projected_cloud_line->points.back());
    return line_endpoints;
}

// Ransac提取多条直线参数，并提取端点，仅用于实验观测
void CornerButtSeamsDet::Ransac_MultipleLines(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundaries) {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZ> seg;            // 创建拟合对象
    seg.setOptimizeCoefficients(true);                  // 设置对估计模型参数进行优化处理
    seg.setModelType(pcl::SACMODEL_LINE);               // 设置拟合模型为直线模型
    seg.setMethodType(pcl::SAC_RANSAC);                 // 设置拟合方法为RANSAC
    seg.setMaxIterations(1000);                         // 设置最大迭代次数
    seg.setDistanceThreshold(Ransac_BoundaryLine_Dth);  // 判断是否为模型内点的距离阀值/设置误差容忍范围

    int nr_points = cloud_boundaries->points.size();
    for (int i = 0; i < 6; i++)  // 执行4次
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        seg.setInputCloud(cloud_boundaries);   // 输入点云
        seg.segment(*inliers, *coefficients);  // 内点的索引，模型系数

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr outside(new pcl::PointCloud<pcl::PointXYZ>);

        // 提取内点
        pcl::ExtractIndices<pcl::PointXYZ> extract;  // 创建点云提取对象
        extract.setInputCloud(cloud_boundaries);
        extract.setIndices(inliers);
        extract.setNegative(false);  // 设置为false，表示提取内点
        extract.filter(*cloud_line);

        // 对每条ransac的直线，进行2.排序 3.投影 4.计算端点
        // 欧式聚类聚类，将ransac提取的直线，按欧式聚类进行分割
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_line);                    // 桌子平面上其他的点云
        std::vector<pcl::PointIndices> cluster_indices;     // 点云团索引
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;  // 欧式聚类对象
        ec.setClusterTolerance(EucSeg_Dth);                 // 设置近邻搜索的搜索半径
        ec.setMinClusterSize(5);                            // 设置一个聚类需要的最少的点数目
        ec.setMaxClusterSize(9999999);                      // 设置一个聚类需要的最大点数目
        ec.setSearchMethod(tree);                           // 设置点云的搜索机制
        ec.setInputCloud(cloud_line);
        ec.extract(cluster_indices);

        // 寻找点云最多的聚类
        std::vector<int> max_cluster;
        int max_size = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
            if (it->indices.size() > max_size) {
                max_size = it->indices.size();
                max_cluster = it->indices;
            }
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr max_cluster_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = max_cluster.begin(); pit != max_cluster.end(); ++pit)
            max_cluster_pointcloud->points.push_back(cloud_line->points[*pit]);
        cloud_line->swap(*max_cluster_pointcloud);

        //-------------------对提取的每一线段点集，进行排序和直线投影
        // 1.排序
        std::vector<std::pair<int, double>> idxSorted(cloud_line->size());
        for (int j = 0; j < cloud_line->size(); j++) {
            idxSorted[j].first = j;
            idxSorted[j].second = cloud_line->points[j].getVector3fMap().dot(
                Eigen::Map<Eigen::Vector3f>(const_cast<float*>(coefficients->values.data() + 3), 3));
        }
        std::sort(idxSorted.begin(), idxSorted.end(),
                  [](const std::pair<int, double>& lhs, const std::pair<int, double>& rhs) { return lhs.second < rhs.second; });
        pcl::PointCloud<pcl::PointXYZ>::Ptr sorted_cloud_line(new pcl::PointCloud<pcl::PointXYZ>);
        for (int j = 0; j < cloud_line->size(); j++) {
            sorted_cloud_line->push_back(cloud_line->points[idxSorted[j].first]);  // 获取排序后的点云
        }
        // 2.投影
        pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud_line(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ProjectInliers<pcl::PointXYZ> projector;
        projector.setModelType(pcl::SACMODEL_LINE);
        projector.setInputCloud(sorted_cloud_line);
        projector.setModelCoefficients(coefficients);
        projector.filter(*projected_cloud_line);
        // 输出每条线段的两个端点
        std::vector<pcl::PointXYZ> line_endpoints;  // 每条直线的两个端点
        line_endpoints.push_back(projected_cloud_line->points.front());
        line_endpoints.push_back(projected_cloud_line->points.back());
        ALL_BoundaryLines_endpoints.push_back(line_endpoints);  // 多条直线的端点集合

        // 将剩余的外点，重新赋值给初值，用于再次循环拟合直线
        extract.setNegative(true);       // true提取外点（该直线之外的点）
        extract.filter(*outside);        // outside为外点点云
        cloud_boundaries.swap(outside);  // 将cloud_f中的点云赋值给cloud_boundaries
    }
}

// 单条焊缝检测完成后，变量重新初始化
void CornerButtSeamsDet::SingleSeam_Reinitialize() {
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);                  // ROI输入点云
    cloud_plane_interior.reset(new pcl::PointCloud<pcl::PointXYZ>);   // 焊缝所在平面内点
    cloud_plane_projected.reset(new pcl::PointCloud<pcl::PointXYZ>);  // 将点云投影到平面
    cloud_boundary.reset(new pcl::PointCloud<pcl::PointXYZ>);         // 边界点
    // plane_coeffs.reset(new pcl::ModelCoefficients);                   // 焊缝所在平面系数
    planeCoeffsWithWeldSeam.reset(new pcl::ModelCoefficients);  // 焊缝所在平面系数

    // vector清空
    Six_BoundaryLines_Length.clear();  // 各条边界线的长度
    Six_BoundaryLinesInliers.clear();  // 提取的6条边界线的内点集合
    Six_BoundaryLinesCoff.clear();
    ;                                        ////提取的6条边界线的参数
    SortedSix_BoundaryLinesInliers.clear();  // 按长度排序后的6条边界线的内点集合
    SortedSix_BoundaryLinesCoff.clear();
    ;                                             ////按长度排序后的6条边界线的参数
    MaxFourLength_BoundaryLines_inliers.clear();  // 长度最大的4条边界线的内点集合
    ALL_BoundaryLines_endpoints.clear();          // 各边界线端点，仅用于实验观测
    MaxFourLength_BoundaryLinesCoff.clear();      // 长度最大的4条边界线的参数
    CurrentSeam_endpoints.clear();                // 当前焊缝的端点
}
