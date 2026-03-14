#include "BeamButtSeamsDet.h"

#include "seamDetWithPointCloud/QhullLock.h"
#include "utils/common/WeldSeamInfo.h"
#include "utils/pointCloud/PointCloudFunc.h"

BeamButtSeamsDet::BeamButtSeamsDet(QObject* parent) : AbstractSeamDet{parent} {}

// 求解焊缝
std::vector<std::shared_ptr<WeldSeamInfo>> BeamButtSeamsDet::solveSeamsEndPoints(
    std::vector<std::shared_ptr<WeldSeamInfo>> seamsInfo) {
    PLOGD << "BeamButtSeamsDet::solveSeamsEndPoints In";
    tempWeldSeamsInfo = seamsInfo;  // 传入的焊缝信息

    // 焊缝判面
    if (tempWeldSeamsInfo.size() >= 1) {
        if (tempWeldSeamsInfo[0]->weldAreaType == WELD_AREA_TYPE::BACK_BEAM ||
            tempWeldSeamsInfo[0]->weldAreaType == WELD_AREA_TYPE::BACK_CORNER) {
            seamSide = SEAM_SIDE::BACK;
        } else {
            seamSide = SEAM_SIDE::FRONT;
        }
    }

    // 焊缝求解
    for (int i = 0; i < tempWeldSeamsInfo.size(); i++) {
        SingleSeam_Reinitialize();                                          // 单条焊缝检测前，变量重新初始化
        cloudInObjectDetectBox = tempWeldSeamsInfo[i]->weldAreaPointCloud;  // 获取焊缝区域点云

        if (cloudInObjectDetectBox->size() == 0) {
            tempWeldSeamsInfo[i]->detectSuccFlag = false;
            continue;
        }

        MyToolFunc::myFastMaxCluster(cloudInObjectDetectBox, Max_Cluster_radius);  // 快速欧式聚类提取最大点集
        Ransac_plane(cloudInObjectDetectBox, cloudOnPlaneWithWeldSeam);  // Ransac拟合平面，并输出平面的内点集合
        Statistic_filter(cloudOnPlaneWithWeldSeam);  // 统计滤波易造成小空洞，从而造成伪焊缝点的生成，因此参数sigma应设置的大些
        Project_ToPlane(cloudOnPlaneWithWeldSeam, cloudProjected2Plane);  // 将平面点云集合进行平面投影
        if (saveAndOutputDebugInformation == true && cloudProjected2Plane->size() != 0) {
            cloudProjected2Plane->height = 1;
            cloudProjected2Plane->width = static_cast<uint32_t>(cloudProjected2Plane->size());
            pcl::io::savePCDFile("./data/seamDetWithPointCloud/BeamButt/cloudProjected2Plane.pcd", *cloudProjected2Plane);
        }

        MyToolFunc::myFastMaxCluster(cloudProjected2Plane,
                                     7);  // 目标框会框到横梁角钢对面的一部分工件，使用欧式聚类先去除那一部分的点云
        // Max_EuclideanCluster(cloudProjected2Plane, 7);
        if (saveAndOutputDebugInformation == true && cloudProjected2Plane->size() != 0) {
            cloudProjected2Plane->height = 1;
            cloudProjected2Plane->width = static_cast<uint32_t>(cloudProjected2Plane->size());
            pcl::io::savePCDFile("./data/seamDetWithPointCloud/BeamButt/cloudProjected2Plane_after_euclidean.pcd",
                                 *cloudProjected2Plane);
        }

        Solve_CloudFineAwayRough(cloudProjected2Plane);  // 筛选焊缝候选点集 1.宽窄边界对比筛选 2.输出粗边界 3.输出粗边界点云质心
        bool bool_IdentifySeam = Solve_BeamSeam_Endpoints(cloudOnBigBoundary);  // 提取焊缝端点

        // 打印检测结果
        PLOGD << "横梁对接焊缝端点识别结果: " << bool_IdentifySeam;
        if (beamButtSeams.size() == 2) {
            PLOGD << "焊缝端点坐标: (" << beamButtSeams[0].x << " " << beamButtSeams[0].y << " " << beamButtSeams[0].z << ") ("
                  << beamButtSeams[1].x << " " << beamButtSeams[1].y << " " << beamButtSeams[1].z << ")";
        }

        // 保存本次检测到的信息
        tempWeldSeamsInfo[i]->detectSuccFlag = bool_IdentifySeam;  // 检测是否成功标志位
        if (bool_IdentifySeam == true) {
            tempWeldSeamsInfo[i]->weldEndPointsInCamera.reset(
                new std::vector<pcl::PointXYZ>(std::move(beamButtSeams)));  // 检测结果
            tempWeldSeamsInfo[i]->weldPlane = planeCoeffsWithWeldSeam;      // 焊缝所在平面
            if (seamSide == SEAM_SIDE::BACK) {                              // 焊缝类型
                tempWeldSeamsInfo[i]->weldType = WELD_TYPE::BACK_BEAM_BUTT;
            } else if (seamSide == SEAM_SIDE::FRONT) {
                tempWeldSeamsInfo[i]->weldType = WELD_TYPE::FRONT_BEAM_BUTT;
            }
            tempWeldSeamsInfo[i]->seamsLineToVal = lineCoeffsWithWeldSeam2Val;  // 用于验证正确性的直线
        }
    }

    PLOGD << "BeamButtSeamsDet::solveSeamsEndPoints Out";

    return tempWeldSeamsInfo;
}

// Ransac拟合平面，并输出平面的内点集合
void BeamButtSeamsDet::Ransac_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud) {
    // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);              // 开启最小二乘系数优化
    seg.setModelType(pcl::SACMODEL_PLANE);          // 设置模型类型
    seg.setMethodType(pcl::SAC_RANSAC);             // 设置算法类型
    seg.setMaxIterations(Ransac_Plane_Iterations);  // 设置最大迭代次数
    seg.setDistanceThreshold(Ransac_plane_Dth);     // 设置距离阈值。
    seg.setInputCloud(input_cloud);                 // 输入点云
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.segment(*inliers, *planeCoeffsWithWeldSeam);  // 实现分割，并存储分割结果到点集合inliers及存储平面模型系数coefficients
    copyPointCloud(*input_cloud, inliers->indices, *output_cloud);
}

// AlphaShape  1.宽窄边界对比筛选  2.输出粗边界  3.输出粗边界点云质心
void BeamButtSeamsDet::Solve_CloudFineAwayRough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudProjected2Plane) {
    pcl::ConcaveHull<pcl::PointXYZ> alpha_shapes;  // 创建alpha_shapes对象
    alpha_shapes.setInputCloud(cloudProjected2Plane);
    double alphaSmallRadius = 0.7;  // AlphaShape半径阈值设置 需将结构光重建时的调制度下调至2
    double FineRough_Dth = 4;       // 焊缝对应的边缘点距离粗边界距离阈值
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudWithSmallAlphaRidus(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTemp(new pcl::PointCloud<pcl::PointXYZ>);

    {
        std::lock_guard<std::mutex> lock(getQhullMutex());  // 锁作用域仅限此块

        alpha_shapes.setAlpha(alphaSmallRadius);  // 检测细边界(小alpha半径)
        alpha_shapes.reconstruct(*cloudWithSmallAlphaRidus);
        alpha_shapes.setAlpha(alphaBigRidus);  // 检测宽边界(大alpha半径)
        alpha_shapes.reconstruct(*cloudOnBigBoundary);
        alpha_shapes.setAlpha(70);  // 进一步获取巨大alpha shape半径点云，以备后期焊缝处点云没有找到
        alpha_shapes.reconstruct(*cloudOnHugeBoundary);
    }

    // 对比大半径边界和小半径边界，生成焊缝候选点集
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(cloudOnBigBoundary);                       // 树中放入宽边界
    for (int i = 0; i < cloudWithSmallAlphaRidus->size(); ++i) {  // 通过宽边界点集和细边界点集的计算距离，筛选焊缝候选点集
        std::vector<int> indices(1);
        std::vector<float> sqr_distances(1);
        tree.nearestKSearch(cloudWithSmallAlphaRidus->points[i], 1, indices, sqr_distances);  // 查找最近点
        if (sqrt(sqr_distances[0]) > FineRough_Dth) {  // 距离大于阈值，才有可能为焊缝点
            cloudOnSeamBoundary->push_back(cloudWithSmallAlphaRidus->points[i]);
        }
    }
    // 对比大半径边界和巨大半径边界，生成关键边界线筛选候选点集
    for (int i = 0; i < cloudOnHugeBoundary->size(); ++i) {  // 通过宽边界点集和细边界点集的计算距离，筛选焊缝候选点集
        std::vector<int> indices(2);
        std::vector<float> sqr_distances(2);
        tree.nearestKSearch(cloudOnHugeBoundary->points[i], 2, indices, sqr_distances);  // 查找最近的2个点
        cloudTemp->push_back(cloudOnBigBoundary->points[indices[1]]);
    }
    for (int i = 0; i < cloudTemp->size(); ++i) {  // 将临时点云中的点纳入巨大alpha半径点云中
        cloudOnHugeBoundary->push_back(cloudTemp->points[i]);
    }

    if (saveAndOutputDebugInformation == true && cloudOnSeamBoundary->size() != 0) {
        cloudOnSeamBoundary->height = 1;  // 解决由于height和width未被自动赋值导致的程序崩溃问题
        cloudOnSeamBoundary->width = static_cast<uint32_t>(cloudOnSeamBoundary->size());
        pcl::io::savePCDFile("./data/seamDetWithPointCloud/BeamButt/cloudOnSeamBoundary_beforeeeeeeeeeeeee.pcd",
                             *cloudOnSeamBoundary);
    }
    if (saveAndOutputDebugInformation == true && cloudOnHugeBoundary->size() != 0) {
        cloudOnHugeBoundary->height = 1;  // 解决由于height和width未被自动赋值导致的程序崩溃问题
        cloudOnHugeBoundary->width = static_cast<uint32_t>(cloudOnHugeBoundary->size());
        pcl::io::savePCDFile("./data/seamDetWithPointCloud/BeamButt/cloudOnHugeBoundary.pcd", *cloudOnHugeBoundary);
    }
    // 欧式聚类筛选最大点集
    // 原本的算法（可能受到焊点处点云的影响）
    // double Euclidean_Distance = 20;
    // maxEuclideanCluster(cloud_FineAwayRough, Euclidean_Distance);
    // 新算法
    double Euclidean_Distance = 3;
    maxAndSecondMaxEuclideanCluster(cloudOnSeamBoundary, Euclidean_Distance);

    pcl::compute3DCentroid(*cloudOnSeamBoundary, Centroid_FineAwayRough);  // 求解质心
}

// 欧式聚类聚类，提取最大点集
void BeamButtSeamsDet::maxEuclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, double radius) {
    if (input_cloud->size() > 0) {  // 必须满足最低点数要求
        // 欧式聚类聚类，将ransac提取的直线，按欧式聚类进行分割
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(input_cloud);                   // 桌子平面上其他的点云
        std::vector<pcl::PointIndices> cluster_indices;     // 点云团索引
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;  // 欧式聚类对象
        ec.setClusterTolerance(radius);                     // 欧式聚类搜索半径
        ec.setMinClusterSize(10);                           // 设置一个聚类需要的最少的点数目
        ec.setMaxClusterSize(99999999);                     // 设置一个聚类需要的最大点数目
        ec.setSearchMethod(tree);                           // 设置点云的搜索机制
        ec.setInputCloud(input_cloud);
        ec.extract(cluster_indices);  // 从点云中提取聚类，并将点云索引保存在cluster_indices中

        // 寻找点云最多的聚类
        std::vector<int> max_cluster;
        int max_size = 0;
        //        std::cout << "**********cluster_indices.size()" << cluster_indices.size() << std::endl;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
            if (it->indices.size() > max_size) {
                max_size = it->indices.size();
                max_cluster = it->indices;
                //                std::cout << "**********max_cluster.size()" << max_cluster.size() << std::endl;
            }
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr max_cluster_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*input_cloud, max_cluster, *max_cluster_pointcloud);
        input_cloud->swap(*max_cluster_pointcloud);
    } else {
        std::cout << "欧式聚类点数为0" << std::endl;
    }
}

// 欧式聚类聚类，提取最大和次大点集的总和
void BeamButtSeamsDet::maxAndSecondMaxEuclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, double radius) {
    if (inputCloud->size() > 0) {  // 必须满足最低点数要求
        // 欧式聚类聚类，将ransac提取的直线，按欧式聚类进行分割
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(inputCloud);                    // 桌子平面上其他的点云
        std::vector<pcl::PointIndices> clusterIndices;      // 点云团索引
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;  // 欧式聚类对象
        ec.setClusterTolerance(radius);                     // 欧式聚类搜索半径
        ec.setMinClusterSize(10);                           // 设置一个聚类需要的最少的点数目
        ec.setMaxClusterSize(99999999);                     // 设置一个聚类需要的最大点数目
        ec.setSearchMethod(tree);                           // 设置点云的搜索机制
        ec.setInputCloud(inputCloud);
        ec.extract(clusterIndices);  // 从点云中提取聚类，并将点云索引保存在cluster_indices中

        // 寻找点云最多和次多的聚类
        std::vector<int> maxCluster;
        int maxSize = 0;
        std::vector<int> secondMaxCluster;
        int secondMaxSize = 0;
        // std::cout << "**********clusterIndices.size()" << clusterIndices.size() << std::endl;
        for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
            if (it->indices.size() > maxSize) {
                maxSize = it->indices.size();
                maxCluster = it->indices;
            } else if (it->indices.size() > secondMaxSize) {
                secondMaxSize = it->indices.size();
                secondMaxCluster = it->indices;
            }
        }
        // std::cout << "**********maxCluster.size()" << maxCluster.size() << std::endl;
        // std::cout << "**********secondMaxCluster.size()" << secondMaxCluster.size() << std::endl;

        // 将最大的点云簇和第二大的点云簇合并为一个点云簇，舍弃其他的点云
        if (secondMaxCluster.size() > 20) {
            for (auto& i : secondMaxCluster) {
                maxCluster.push_back(i);
            }
        }
        // std::cout << "**********maxCluster.size()" << maxCluster.size() << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr maxAndSecondMaxClusterPointcloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*inputCloud, maxCluster, *maxAndSecondMaxClusterPointcloud);
        inputCloud->swap(*maxAndSecondMaxClusterPointcloud);
    } else {
        std::cout << "欧式聚类点数为0" << std::endl;
    }
}

/**
 * @brief BeamButtSeamsDet::reserveIndexAtFrontOfInlierNum 保留内点索引数量靠前的索引
 * @param indexOfLines 直线索引
 * @param reserveNum 保留索引数量
 */
void BeamButtSeamsDet::reserveIndexAtFrontOfInlierNum(std::vector<int>& indexOfLines, int reserveNum) {
    // 创建一个临时容器，存储索引及其对应的大小
    std::vector<std::pair<int, size_t>> indexAndInliersNum;
    for (int idx : indexOfLines) {
        indexAndInliersNum.emplace_back(idx, boundaryLinesInliers[idx].size());
    }
    // 按照 size 从大到小排序
    std::sort(indexAndInliersNum.begin(), indexAndInliersNum.end(),
              [](const std::pair<int, size_t>& a, const std::pair<int, size_t>& b) { return a.second > b.second; });
    // 保留前 reserveNum 个索引
    indexOfLines.clear();
    for (size_t i = 0; i < reserveNum; ++i) {
        indexOfLines.push_back(indexAndInliersNum[i].first);
    }
}

/**
 * @brief BeamButtSeamsDet::reserveIndexAtFrontOfLength 保留直线长度靠前的索引
 * @param indexOfLines 直线索引
 * @param reserveNum 保留索引数量
 */
void BeamButtSeamsDet::reserveIndexAtFrontOfLength(std::vector<int>& indexOfLines, int reserveNum) {
    // 创建一个临时容器，存储索引及其对应的直线长度
    std::vector<std::pair<int, size_t>> indexAndInliersNum;
    for (int idx : indexOfLines) {
        indexAndInliersNum.emplace_back(idx, boundaryLines_Length[idx]);
    }
    // 按照直线长度从大到小排序
    std::sort(indexAndInliersNum.begin(), indexAndInliersNum.end(),
              [](const std::pair<int, size_t>& a, const std::pair<int, size_t>& b) { return a.second > b.second; });
    // 保留前 reserveNum 个索引
    indexOfLines.clear();
    for (size_t i = 0; i < reserveNum; ++i) {
        indexOfLines.push_back(indexAndInliersNum[i].first);
    }
}

// 提取横梁焊缝端点
bool BeamButtSeamsDet::Solve_BeamSeam_Endpoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudBoundaries) {
    if (saveAndOutputDebugInformation == true && cloudBoundaries->size() != 0) {
        cloudBoundaries->height = 1;  // 解决由于height和width未被自动赋值导致的程序崩溃问题
        cloudBoundaries->width = static_cast<uint32_t>(cloudBoundaries->size());
        pcl::io::savePCDFile("./data/seamDetWithPointCloud/BeamButt/cloud_boundaries.pcd", *cloudBoundaries);
    }

    // 直线Ransac参数
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZ> seg;                 // 创建拟合对象
    seg.setOptimizeCoefficients(true);                       // 设置对估计模型参数进行优化处理
    seg.setModelType(pcl::SACMODEL_LINE);                    // 设置拟合模型为直线模型
    seg.setMethodType(pcl::SAC_RANSAC);                      // 设置拟合方法为RANSAC
    seg.setMaxIterations(500);                               // 设置最大迭代次数
    seg.setDistanceThreshold(LineFittingDistanceThreshold);  // 判断是否为模型内点的距离阀值/设置误差容忍范围

    // 拟合7条直线 Beam背面需要拟合7条直线，Beam正面拟合6条即可，可统一设置为拟合7条
    // int nr_points = cloud_boundaries->points.size();
    // (现在增加到8次，因为当工件放置不正时，目标框可能框到工件表面，进而导致表面可以拟合出8条线，为了不遗漏关键直线，直接拟合8条)
    int segLineNum = 7;
    if (seamSide == SEAM_SIDE::BACK) {  // 背面(平整面)
        segLineNum = 7;
    } else if (seamSide == SEAM_SIDE::FRONT) {  // 正面(非平整面)
        segLineNum = 6;
    } else {
        PLOGE << "错误4, 传入的焊缝类型错误, 传入的类型: " << seamSide;
        return false;
    }
    for (int i = 0; i < segLineNum; i++) {  // 执行segLineNum次
        if (cloudBoundaries->points.size() > 3) {
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
            seg.setInputCloud(cloudBoundaries);    // 输入点云
            seg.segment(*inliers, *coefficients);  // 内点的索引，模型系数（直线参数）
            pcl::PointCloud<pcl::PointXYZ>::Ptr lineCloud(new pcl::PointCloud<pcl::PointXYZ>);  // 直线内点
            pcl::PointCloud<pcl::PointXYZ>::Ptr outside(new pcl::PointCloud<pcl::PointXYZ>);

            // 打印直线参数
            if (saveAndOutputDebugInformation == true) {
                std::cout << "lineCoefficients: " << coefficients->values[0] << "  " << coefficients->values[1] << "  "
                          << coefficients->values[2] << "  " << coefficients->values[3] << "  " << coefficients->values[4] << "  "
                          << coefficients->values[5] << std::endl;
            }

            // 提取内点
            pcl::ExtractIndices<pcl::PointXYZ> extract;  // 创建点云提取对象
            extract.setInputCloud(cloudBoundaries);
            extract.setIndices(inliers);
            extract.setNegative(false);  // 设置为false，表示提取内点
            extract.filter(*lineCloud);

            // 计算内点集合的最大距离
            pcl::PointXYZ pmin, pmax;
            double lineDistance = pcl::getMaxSegment(*lineCloud, pmin, pmax);  // 计算点云中最长分段的长度
            boundaryLinesInliers.push_back(*lineCloud);                        // 各条直线的内点
            boundaryLinesCoff.push_back(coefficients);                         // 各条直线的系数（点+方向向量）
            boundaryLines_Length.push_back(lineDistance);                      // 各条直线的长度

            // 将剩余的外点，重新赋值给初值，用于再次循环拟合直线
            extract.setNegative(true);      // true提取外点（该直线之外的点）
            extract.filter(*outside);       // outside为外点点云
            cloudBoundaries.swap(outside);  // 将cloud_f中的点云赋值给cloudBoundaries
            // std::cout << "剩余点数：" << cloud_boundaries->size() << " 当前i值：" << i << std::endl;
        }
    }

    PLOGD << "焊缝边缘点云点数: " << cloudOnSeamBoundary->size();
    if (true /*cloudOnSeamBoundary->size() < 30*/) {  // 焊缝边界点云数量不够，采取其它算法
        // 找到焊缝求解的关键直线(在巨大alpha半径的边界点云中找每个直线内点，取内点最少的三条)
        std::vector<std::pair<int, int>> linePointCounts;  // 存储直线索引及其对应的内点数量
        for (int i = 0; i < boundaryLinesCoff.size(); ++i) {
            int pointNumNearLine = 0;
#pragma omp parallel for reduction(+ : pointNumNearLine) schedule(dynamic)
            for (int j = 0; j < cloudOnHugeBoundary->size(); ++j) {
                Eigen::Vector4f point(cloudOnHugeBoundary->points[j].x, cloudOnHugeBoundary->points[j].y,
                                      cloudOnHugeBoundary->points[j].z, 0);
                if (MyToolFunc::getPoint2LineDis(point, boundaryLinesCoff[i]) < LineFittingDistanceThreshold) {
                    ++pointNumNearLine;  // 累加符合条件的点数
                }
            }
            PLOGD << "巨大alpha半径得到的点云与索引为 " << i << " 的直线相近的点数：" << pointNumNearLine;
            linePointCounts.emplace_back(i, pointNumNearLine);  // 存储直线索引及其内点数量
        }
        // 按内点数量从小到大排序
        std::sort(linePointCounts.begin(), linePointCounts.end(),
                  [](const std::pair<int, int>& a, const std::pair<int, int>& b) { return a.second < b.second; });
        // 只保留内点数量最少的若干条直线索引-与巨大 alpha 边界“最不接近”的直线。
        std::vector<int> indexOfKeyLines;
        if (seamSide == SEAM_SIDE::BACK) {  // 背面(平整面)
            for (int i = 0; i < std::min(3, static_cast<int>(linePointCounts.size())); ++i) {
                indexOfKeyLines.push_back(linePointCounts[i].first);
            }
        } else if (seamSide == SEAM_SIDE::FRONT) {  // 正面(非平整面)
            for (int i = 0; i < std::min(2, static_cast<int>(linePointCounts.size())); ++i) {
                indexOfKeyLines.push_back(linePointCounts[i].first);
            }
        }

        // 采用阈值进行判断的逻辑
        //         std::vector<int> indexOfKeyLines;
        //         for (int i = 0; i < boundaryLinesCoff.size(); ++i) {
        //             int pointNumNearLine = 0;
        // #pragma omp parallel for reduction(+ : pointNumNearLine) schedule(dynamic)
        //             for (int j = 0; j < cloudOnHugeBoundary->size(); ++j) {
        //                 Eigen::Vector4f point(cloudOnHugeBoundary->points[j].x, cloudOnHugeBoundary->points[j].y,
        //                                       cloudOnHugeBoundary->points[j].z, 0);
        //                 if (getPoint2LineDistance(point, boundaryLinesCoff[i]) < LineFittingDistanceThreshold) {
        //                     ++pointNumNearLine;  // 累加符合条件的点数
        //                 }
        //             }
        //             PLOGD << "巨大alpha半径得到的点云与索引为 " << i << L" 的直线相近的点数：" << pointNumNearLine;
        //             if (pointNumNearLine < 10) {  // 内点数量少，为关键直线
        //                 indexOfKeyLines.push_back(i);
        //             }
        //         }

        if (seamSide == SEAM_SIDE::BACK) {     // 背面(平整面)
            if (indexOfKeyLines.size() > 3) {  // 背面只需要三条(取内点最多的三条，以排除干扰直线)
                reserveIndexAtFrontOfInlierNum(indexOfKeyLines, 3);
            }
            if (indexOfKeyLines.size() != 3) {  // 关键直线数量不满足要求
                PLOGE << "错误5, 背面横梁对接处没有找到足够的关键直线, 找到条数: " << indexOfKeyLines.size();
                return false;
            } else {  // 关键直线数量满足要求，对直线进行区分
                // 取出关键直线的方向向量
                Eigen::Vector3f lineWithSeam(boundaryLinesCoff[indexOfKeyLines[0]]->values[3],
                                             boundaryLinesCoff[indexOfKeyLines[0]]->values[4],
                                             boundaryLinesCoff[indexOfKeyLines[0]]->values[5]);
                Eigen::Vector3f lineVerticalWithSeam1(boundaryLinesCoff[indexOfKeyLines[1]]->values[3],
                                                      boundaryLinesCoff[indexOfKeyLines[1]]->values[4],
                                                      boundaryLinesCoff[indexOfKeyLines[1]]->values[5]);
                Eigen::Vector3f lineVerticalWithSeam2(boundaryLinesCoff[indexOfKeyLines[2]]->values[3],
                                                      boundaryLinesCoff[indexOfKeyLines[2]]->values[4],
                                                      boundaryLinesCoff[indexOfKeyLines[2]]->values[5]);

                double angle1 = MyToolFunc::getLineAngle(lineWithSeam, lineVerticalWithSeam1);
                double angle2 = MyToolFunc::getLineAngle(lineWithSeam, lineVerticalWithSeam2);
                double angle3 = MyToolFunc::getLineAngle(lineVerticalWithSeam1, lineVerticalWithSeam2);
                // 将关键直线正确排序
                if (angle1 > verticalThreshold && angle2 > verticalThreshold) {
                    // do nothing
                } else if (angle1 > verticalThreshold && angle3 > verticalThreshold) {  // LineVerticalWithSeam1是焊缝所在直线
                    std::swap(lineWithSeam, lineVerticalWithSeam1);
                    std::swap(indexOfKeyLines[0], indexOfKeyLines[1]);
                } else if (angle2 > verticalThreshold && angle3 > verticalThreshold) {  // LineVerticalWithSeam2是焊缝所在直线
                    std::swap(lineWithSeam, lineVerticalWithSeam2);
                    std::swap(indexOfKeyLines[0], indexOfKeyLines[2]);
                } else {
                    PLOGE << "错误6, 横梁处对接找到的关键直线不符合角度关系要求: " << angle1 << " " << angle2 << " " << angle3;
                    return false;
                }
                lineCoeffsWithWeldSeam2Val = boundaryLinesCoff[indexOfKeyLines[0]];  // 保存用于验证的直线

                // 求解焊缝直线与其它两个垂直直线的交点
                Eigen::Vector4f seamPoint1, seamPoint2;
                pcl::lineWithLineIntersection(*boundaryLinesCoff[indexOfKeyLines[0]], *boundaryLinesCoff[indexOfKeyLines[1]],
                                              seamPoint1);
                pcl::lineWithLineIntersection(*boundaryLinesCoff[indexOfKeyLines[0]], *boundaryLinesCoff[indexOfKeyLines[2]],
                                              seamPoint2);
                pcl::PointXYZ PointXYZ_Seam_endPoint1 = {seamPoint1[0], seamPoint1[1], seamPoint1[2]};
                pcl::PointXYZ PointXYZ_Seam_endPoint2 = {seamPoint2[0], seamPoint2[1], seamPoint2[2]};
                CurrentSeam_endpoints.push_back(PointXYZ_Seam_endPoint1);
                CurrentSeam_endpoints.push_back(PointXYZ_Seam_endPoint2);
                beamButtSeams = CurrentSeam_endpoints;  // 各条焊缝的端点
                // DetectedSeamAreas_Pointclouds.push_back(cloudInObjectDetectBox);  // 焊缝区域点云

                return true;
            }
        } else if (seamSide == SEAM_SIDE::FRONT) {  // 正面(非平整面)
            if (indexOfKeyLines.size() > 2) {       // 正面只需要两条(取内点最多的两条，以排除干扰直线)
                reserveIndexAtFrontOfInlierNum(indexOfKeyLines, 2);
            }
            if (indexOfKeyLines.size() != 2) {  // 关键直线数量不满足要求
                PLOGE << "错误5, 正面横梁对接处没有找到足够的关键直线, 找到条数: " << indexOfKeyLines.size();
                return false;
            } else {  // 关键直线数量满足要求，对直线进行区分
                // 取出关键直线的方向向量
                Eigen::Vector3f lineWithSeam(boundaryLinesCoff[indexOfKeyLines[0]]->values[3],
                                             boundaryLinesCoff[indexOfKeyLines[0]]->values[4],
                                             boundaryLinesCoff[indexOfKeyLines[0]]->values[5]);
                Eigen::Vector3f lineVerticalWithSeam(boundaryLinesCoff[indexOfKeyLines[1]]->values[3],
                                                     boundaryLinesCoff[indexOfKeyLines[1]]->values[4],
                                                     boundaryLinesCoff[indexOfKeyLines[1]]->values[5]);
                double angle = MyToolFunc::getLineAngle(lineWithSeam, lineVerticalWithSeam);
                // 将关键直线正确排序（分出水平线和数值线）
                if (angle > verticalThreshold) {
                    if (abs(lineWithSeam[0]) < abs(lineWithSeam[1]) &&
                        abs(lineVerticalWithSeam[0]) > abs(lineVerticalWithSeam[1])) {
                        // do nothing
                    } else if (abs(lineWithSeam[0]) > abs(lineWithSeam[1]) &&
                               abs(lineVerticalWithSeam[0]) < abs(lineVerticalWithSeam[1])) {
                        std::swap(lineWithSeam, lineVerticalWithSeam);
                        std::swap(indexOfKeyLines[0], indexOfKeyLines[1]);
                    }
                } else {
                    PLOGE << "错误6, 横梁处对接找到的关键直线不符合角度关系要求: " << angle;
                    return false;
                }
                lineCoeffsWithWeldSeam2Val = boundaryLinesCoff[indexOfKeyLines[0]];  // 保存用于验证的直线

                // 找到另一条与焊缝直线垂直的关键直线
                std::vector<int> indexOfVerticalWithSeamLines;
                for (int i = 0; i < boundaryLinesCoff.size(); ++i) {
                    if (i != indexOfKeyLines[0] && i != indexOfKeyLines[1]) {
                        Eigen::Vector3f line(boundaryLinesCoff[i]->values[3], boundaryLinesCoff[i]->values[4],
                                             boundaryLinesCoff[i]->values[5]);
                        if (MyToolFunc::getLineAngle(line, lineWithSeam) > verticalThreshold) {
                            indexOfVerticalWithSeamLines.push_back(i);
                        }
                    }
                }
                if (indexOfVerticalWithSeamLines.size() > 1) {  // 取长度较长的一条，即为需要的那条直线
                    reserveIndexAtFrontOfLength(indexOfVerticalWithSeamLines, 1);
                }
                if (indexOfVerticalWithSeamLines.size() != 1) {
                    PLOGE << "错误7, 横梁处对接没有找到的第二条与焊缝直线垂直的关键直线, indexOfVerticalWithSeamLines.size(): "
                          << indexOfVerticalWithSeamLines.size();
                    return false;
                }
                indexOfKeyLines.push_back(indexOfVerticalWithSeamLines[0]);

                // 求解焊缝直线与其它两个垂直直线的交点
                Eigen::Vector4f seamPoint1, seamPoint2;
                pcl::lineWithLineIntersection(*boundaryLinesCoff[indexOfKeyLines[0]], *boundaryLinesCoff[indexOfKeyLines[1]],
                                              seamPoint1);
                pcl::lineWithLineIntersection(*boundaryLinesCoff[indexOfKeyLines[0]], *boundaryLinesCoff[indexOfKeyLines[2]],
                                              seamPoint2);
                pcl::PointXYZ PointXYZ_Seam_endPoint1 = {seamPoint1[0], seamPoint1[1], seamPoint1[2]};
                pcl::PointXYZ PointXYZ_Seam_endPoint2 = {seamPoint2[0], seamPoint2[1], seamPoint2[2]};
                CurrentSeam_endpoints.push_back(PointXYZ_Seam_endPoint1);
                CurrentSeam_endpoints.push_back(PointXYZ_Seam_endPoint2);
                beamButtSeams = CurrentSeam_endpoints;  // 各条焊缝的端点
                // DetectedSeamAreas_Pointclouds.push_back(cloudInObjectDetectBox);  // 焊缝区域点云

                return true;
            }
        } else {
            PLOGE << "错误4, 传入的焊缝类型错误, 传入的类型: " << seamSide;
            return false;
        }
    } else {  // 焊缝边界点云数量足够
        // 确认焊缝所在直线，方法为: 求解点云cloudOnSeamBoundary的质心，距离各条Ransac直线的距离，距离最小的即为所求
        double DisTemp = 10000000;                  // 焊缝质心到与焊缝最近直线的距离
        std::vector<double> VecDistanceToCentroid;  // cloudOnSeamBoundary的质心距离各条直线的距离
        int IndexLineWithSeam;                      // 焊缝所在直线的索引
        for (int i = 0; i < boundaryLinesCoff.size(); i++) {
            if (saveAndOutputDebugInformation == true) {
                std::cout << "i: " << i << "  Vector_BoundaryLines_Length[i]: " << boundaryLines_Length[i] << std::endl;
            }
            if (boundaryLines_Length[i] > 7) {  // 设置最近长度阈值，防止噪声干扰
                double distance = MyToolFunc::getPoint2LineDis(Centroid_FineAwayRough, boundaryLinesCoff[i]);
                VecDistanceToCentroid.push_back(distance);
                // std::cout << "distance_i: " << distance <<std::endl;
                if (distance < DisTemp) {
                    IndexLineWithSeam = i;
                    DisTemp = distance;
                }
            }
        }
        // std::cout << "Index_LineWithSeam:" << Index_LineWithSeam <<std::endl;

        // 求解与焊缝所在直线垂直的直线，并求解其中距离焊缝中心最近的三条直线
        Eigen::Vector3f SeamLineDir(boundaryLinesCoff[IndexLineWithSeam]->values[3],
                                    boundaryLinesCoff[IndexLineWithSeam]->values[4],
                                    boundaryLinesCoff[IndexLineWithSeam]->values[5]);         // 焊缝所在直线的方向
        lineCoeffsWithWeldSeam2Val = boundaryLinesCoff[IndexLineWithSeam];                    // 保存用于验证的直线
        double min_1stDistance = 999999, min_2stDistance = 999999, min_3stDistance = 999999;  // 最近、次近、第三近的距离
        int MinDisLineIndex_1st = -1, MinDisLineIndex_2st = -1, MinDisLineIndex_3st = -1;  // 距离最近、次近、第三近的直线索引
        for (int i = 0; i < boundaryLinesCoff.size(); i++) {
            if (boundaryLines_Length[i] > 15) {  // 设置最短长度，防止噪声干扰
                Eigen::Vector3f LineIDir(boundaryLinesCoff[i]->values[3], boundaryLinesCoff[i]->values[4],
                                         boundaryLinesCoff[i]->values[5]);
                double angle = MyToolFunc::getLineAngle(LineIDir, SeamLineDir);  // 计算当前直线和焊缝所在直线的夹角
                if (saveAndOutputDebugInformation == true) {
                    std::cout << "VecDistanceToCentroid[i]:" << VecDistanceToCentroid[i] << std::endl;
                    // std::cout << "  min_1stDistance:" << min_1stDistance <<std::endl;
                    // std::cout << "  min_2stDistance:" << min_2stDistance <<std::endl;
                    // std::cout << "  min_3stDistance:" << min_3stDistance <<std::endl;
                    std::cout << "   i:" << i << "   angle: " << angle << std::endl;
                }
                if (angle > 85) {  // 垂直判断阈值
                    // 求解距离质心距离最短的三条直线  VecDistanceToCentroid
                    if (VecDistanceToCentroid[i] < min_1stDistance) {
                        MinDisLineIndex_3st = MinDisLineIndex_2st;
                        MinDisLineIndex_2st = MinDisLineIndex_1st;
                        MinDisLineIndex_1st = i;
                        min_3stDistance = min_2stDistance;
                        min_2stDistance = min_1stDistance;
                        min_1stDistance = VecDistanceToCentroid[i];
                    } else if (VecDistanceToCentroid[i] < min_2stDistance) {
                        MinDisLineIndex_3st = MinDisLineIndex_2st;
                        MinDisLineIndex_2st = i;
                        min_3stDistance = min_2stDistance;
                        min_2stDistance = VecDistanceToCentroid[i];
                    } else if (VecDistanceToCentroid[i] < min_3stDistance) {
                        MinDisLineIndex_3st = i;
                        min_3stDistance = VecDistanceToCentroid[i];
                    }
                }
            }
        }
        if (saveAndOutputDebugInformation == true) {
            std::cout << "MinDisLineIndex_1st: " << MinDisLineIndex_1st << std::endl;
            std::cout << "MinDisLineIndex_2st: " << MinDisLineIndex_2st << std::endl;
            std::cout << "MinDisLineIndex_3st: " << MinDisLineIndex_3st << std::endl;
        }

        // 焊缝与选定的两个垂直边界线的交点
        Eigen::Vector4f Intersection_VecPoint1, Intersection_VecPoint2;
        int Index_SecondLength = -1;  // 选定两条垂直边界线后，进行长度比较，求出长度短的边界线索引

        // 求解焊缝质心到前两条最近垂线的投影点
        if (MinDisLineIndex_1st == -1 || MinDisLineIndex_2st == -1) {
            std::cout << "错误3，无法准确提取焊缝（横梁对接焊缝检测到的垂线不足），MinDisLineIndex_2st：" << MinDisLineIndex_2st
                      << std::endl;
            return false;
        }
        Eigen::Vector4f Vertical_Foot1 =
            MyToolFunc::projPoint2Line(Centroid_FineAwayRough, boundaryLinesCoff[MinDisLineIndex_1st]);
        Eigen::Vector4f Vertical_Foot2 =
            MyToolFunc::projPoint2Line(Centroid_FineAwayRough, boundaryLinesCoff[MinDisLineIndex_2st]);

        // 如果垂足1和垂足2在焊缝质心的不同方向，则将垂足1和垂足2视为焊缝的两个端点
        if ((Vertical_Foot1 - Centroid_FineAwayRough).dot(Vertical_Foot2 - Centroid_FineAwayRough) < 0) {
            pcl::lineWithLineIntersection(*boundaryLinesCoff[IndexLineWithSeam], *boundaryLinesCoff[MinDisLineIndex_1st],
                                          Intersection_VecPoint1);
            pcl::lineWithLineIntersection(*boundaryLinesCoff[IndexLineWithSeam], *boundaryLinesCoff[MinDisLineIndex_2st],
                                          Intersection_VecPoint2);
            // 选定两条垂直边界线后，进行长度比较，求出长度短的边界线索引
            if (boundaryLines_Length[MinDisLineIndex_1st] > boundaryLines_Length[MinDisLineIndex_2st]) {
                Index_SecondLength = MinDisLineIndex_2st;
            } else {
                Index_SecondLength = MinDisLineIndex_1st;
            }
        } else {  // 如果垂足1和垂足2在焊缝质心的同一方向，则明显错误，垂足3取代垂足2成为焊缝的一个端点
            if (MinDisLineIndex_3st != -1) {  // 需保证存在第三条垂线
                pcl::lineWithLineIntersection(*boundaryLinesCoff[IndexLineWithSeam], *boundaryLinesCoff[MinDisLineIndex_1st],
                                              Intersection_VecPoint1);
                pcl::lineWithLineIntersection(*boundaryLinesCoff[IndexLineWithSeam], *boundaryLinesCoff[MinDisLineIndex_3st],
                                              Intersection_VecPoint2);
                // 选定两条垂直边界线后，进行长度比较，求出长度短的边界线索引
                if (boundaryLines_Length[MinDisLineIndex_1st] > boundaryLines_Length[MinDisLineIndex_3st]) {
                    Index_SecondLength = MinDisLineIndex_3st;
                } else {
                    Index_SecondLength = MinDisLineIndex_1st;
                }
            } else {
                std::cout << "错误1，无法准确提取焊缝（横梁对接焊缝所在直线的垂线不符合算法条件），MinDisLineIndex_3st："
                          << MinDisLineIndex_3st << std::endl;
                return false;
            }
        }
        std::cout << "Index_SecondLength: " << Index_SecondLength << std::endl;

        // 不选择偏移的焊缝输出：
        /*PointXYZ Intersection_Point1 = { Intersection_VecPoint1[0], Intersection_VecPoint1[1], Intersection_VecPoint1[2] };
        PointXYZ Intersection_Point2 = { Intersection_VecPoint2[0], Intersection_VecPoint2[1], Intersection_VecPoint2[2] };
        Seam_endpoints.push_back(Intersection_Point1);
        Seam_endpoints.push_back(Intersection_Point2);*/

        // 对上述所求的两个交点，沿焊缝垂直方向向焊缝内部移动一定距离(焊缝半径，即焊缝宽度的一半)，即可得到焊缝的准确位置
        // 求解焊缝的半径，计算方法为：cloud_FineAwayRough点云到焊缝所在直线的平均距离
        double Seam_radius = 0;  // 焊缝半径(也就是焊缝宽度的一半)
        if (saveAndOutputDebugInformation == true && cloudOnSeamBoundary->size() != 0) {
            cloudOnSeamBoundary->height = 1;  // 解决由于height和width未被自动赋值导致的程序崩溃问题
            cloudOnSeamBoundary->width = static_cast<uint32_t>(cloudOnSeamBoundary->size());
            pcl::io::savePCDFile("./data/seamDetWithPointCloud/BeamButt/cloudOnSeamBoundary.pcd", *cloudOnSeamBoundary);
        }
        if (cloudOnSeamBoundary->size() != 0) {
            for (int i = 0; i < cloudOnSeamBoundary->size(); i++) {
                Eigen::Vector4f point = {cloudOnSeamBoundary->points[i].x, cloudOnSeamBoundary->points[i].y,
                                         cloudOnSeamBoundary->points[i].z, 0};
                double distance = MyToolFunc::getPoint2LineDis(point, boundaryLinesCoff[IndexLineWithSeam]);
                Seam_radius = Seam_radius + distance;
            }
            Seam_radius = Seam_radius / cloudOnSeamBoundary->size();
        }
        // std::cout << "Seam_radius:" << Seam_radius <<std::endl;
        if (Seam_radius > 5) {  // 焊缝半径(也就是焊缝宽度的一半)需要小于设定的阈值
            std::cout << "错误2，无法准确提取焊缝（横梁对接焊缝算法检测到的焊缝宽度过宽，认为检测错误），Seam_radius："
                      << Seam_radius << std::endl;
            return false;
        }
        // 求解焊缝的垂线，计算方法：首先求解与焊缝近似垂直的某条边界线的质心，求解直线
        Eigen::Vector4f Centord_SecondLength;  // 定义长度较短的垂线质心
        pcl::compute3DCentroid(boundaryLinesInliers[Index_SecondLength], Centord_SecondLength);  // 求解质心
        // 求解单个点到直线投影的垂线向量
        Eigen::Vector4f VerticalLine_vector;
        MyToolFunc::Solve_ProjectVerticalLine(Centord_SecondLength, boundaryLinesCoff[IndexLineWithSeam], VerticalLine_vector);
        // 两个焊缝交点偏移
        Eigen::Vector4f vec4f_Seam_endPoint1, vec4f_Seam_endPoint2;

        vec4f_Seam_endPoint1 = Intersection_VecPoint1 - Seam_radius * VerticalLine_vector / VerticalLine_vector.norm();
        vec4f_Seam_endPoint2 = Intersection_VecPoint2 - Seam_radius * VerticalLine_vector / VerticalLine_vector.norm();
        pcl::PointXYZ PointXYZ_Seam_endPoint1 = {vec4f_Seam_endPoint1[0], vec4f_Seam_endPoint1[1], vec4f_Seam_endPoint1[2]};
        pcl::PointXYZ PointXYZ_Seam_endPoint2 = {vec4f_Seam_endPoint2[0], vec4f_Seam_endPoint2[1], vec4f_Seam_endPoint2[2]};
        CurrentSeam_endpoints.push_back(PointXYZ_Seam_endPoint1);
        CurrentSeam_endpoints.push_back(PointXYZ_Seam_endPoint2);
        beamButtSeams = CurrentSeam_endpoints;  // 各条焊缝的端点
        // DetectedSeamAreas_Pointclouds.push_back(cloudInObjectDetectBox);  // 焊缝区域点云
        return true;
    }
}

// 统计滤波
void BeamButtSeamsDet::Statistic_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input_cloud);       // 设置待滤波的点云
    sor.setMeanK(Statistic_NeighPoints);  // 设置在进行统计时考虑查询点邻近点数
    sor.setStddevMulThresh(Statistic_sigma);  // 设置判断是否为离群点的阈值，里边的数字表示标准差的倍数，1个标准差以上就是离群点。
    sor.filter(*input_cloud);  // 存储内点
}

// 点云投影至指定平面
void BeamButtSeamsDet::Project_ToPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud) {
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(input_cloud);
    proj.setModelCoefficients(planeCoeffsWithWeldSeam);
    proj.filter(*output_cloud);
}

// 欧式聚类聚类，提取最大点集
void BeamButtSeamsDet::Max_EuclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, double radius) {
    if (input_cloud->size() > 0) {  // 必须满足最低点数要求
        // 欧式聚类聚类，将ransac提取的直线，按欧式聚类进行分割
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(input_cloud);                   // 桌子平面上其他的点云
        std::vector<pcl::PointIndices> cluster_indices;     // 点云团索引
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;  // 欧式聚类对象
        ec.setClusterTolerance(radius);                     // 欧式聚类搜索半径
        ec.setMinClusterSize(10);                           // 设置一个聚类需要的最少的点数目
        ec.setMaxClusterSize(99999999);                     // 设置一个聚类需要的最大点数目
        ec.setSearchMethod(tree);                           // 设置点云的搜索机制
        ec.setInputCloud(input_cloud);
        ec.extract(cluster_indices);  // 从点云中提取聚类，并将点云索引保存在cluster_indices中

        // 寻找点云最多的聚类
        std::vector<int> max_cluster;
        int max_size = 0;
        // std::cout << "**********cluster_indices.size()" << cluster_indices.size() << std::endl;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
            if (it->indices.size() > max_size) {
                max_size = it->indices.size();
                max_cluster = it->indices;
                // std::cout << "**********max_cluster.size()" << max_cluster.size() << std::endl;
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr max_cluster_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*input_cloud, max_cluster, *max_cluster_pointcloud);
        input_cloud->swap(*max_cluster_pointcloud);
    } else {
        std::cout << "欧式聚类点数为0" << std::endl;
    }
}

// 单条焊缝检测前，变量重新初始化
void BeamButtSeamsDet::SingleSeam_Reinitialize() {
    cloudInObjectDetectBox.reset(new pcl::PointCloud<pcl::PointXYZ>);    // ROI输入点云
    cloudOnPlaneWithWeldSeam.reset(new pcl::PointCloud<pcl::PointXYZ>);  // 焊缝所在平面内点
    cloudProjected2Plane.reset(new pcl::PointCloud<pcl::PointXYZ>);      // 将点云投影到平面
    planeCoeffsWithWeldSeam.reset(new pcl::ModelCoefficients);           // 焊缝所在平面系数
    cloudOnBigBoundary.reset(new pcl::PointCloud<pcl::PointXYZ>);        // AlphaShape识别的边界
    cloudOnHugeBoundary.reset(new pcl::PointCloud<pcl::PointXYZ>);       // AlphaShape识别的主平面巨大半径边界
    cloudOnSeamBoundary.reset(new pcl::PointCloud<pcl::PointXYZ>);       // AlphaShape宽窄边界对比筛选

    lineCoeffsWithWeldSeam2Val.reset(new pcl::ModelCoefficients);  // 焊缝所在直线系数

    // vector清空
    boundaryLines_Length.clear();                 // 各条边界线的长度
    boundaryLinesInliers.clear();                 // 提取的6条边界线的内点集合
    boundaryLinesCoff.clear();                    // 提取的6条边界线的参数
    SortedVector_BoundaryLinesInliers.clear();    // 按长度排序后的6条边界线的内点集合
    SortedVector_BoundaryLinesCoff.clear();       // 按长度排序后的6条边界线的参数
    MaxFourLength_BoundaryLines_inliers.clear();  // 长度最大的4条边界线的内点集合
    ALL_BoundaryLines_endpoints.clear();          // 各边界线端点，仅用于实验观测
    MaxFourLength_BoundaryLinesCoff.clear();      // 长度最大的4条边界线的参数
    CurrentSeam_endpoints.clear();                // 焊缝的端点
}
