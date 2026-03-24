#include "AccuratePositioning.h"

AccuratePositioning::AccuratePositioning(QObject* parent) : QObject(parent) {}

/**
 * 函数：extractNeighbors
 * 作用：从点云中提取指定点周围半径内的邻居点
 * 输入：
 *   - point: 中心点坐标
 *   - radius: 搜索半径
 *   - tree: KD树搜索对象（已构建索引）
 *   - cloud: 原始点云数据
 * 输出：邻居点集合
 * 原理：使用KD树进行半径搜索，快速找到指定范围内的所有点
 */
std::vector<pcl::PointXYZ> AccuratePositioning::extractNeighbors(const pcl::PointXYZ& point, double radius,
                                                                 const pcl::search::KdTree<pcl::PointXYZ>::Ptr& tree,
                                                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    std::vector<pcl::PointXYZ> neighbors;
    if (tree->radiusSearch(point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) >= 0) {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
            neighbors.push_back(cloud->points[pointIdxRadiusSearch[i]]);
        }
    }

    return neighbors;
}

/**
 * 函数：computeCovariance
 * 作用：计算协方差矩阵
 * 输入：三个维度的数据向量
 * 输出：3x3协方差矩阵
 * 原理：协方差矩阵描述各维度间的线性关系
 *      对角线元素是方差，非对角线元素是协方差
 */
Eigen::Matrix3d AccuratePositioning::computeCovariance(std::vector<double> xVector, std::vector<double> yVector,
                                                       std::vector<double> zVector) {
    int size = xVector.size();

    // 计算各维度均值
    double xAverage;
    double yAverage;
    double zAverage;
    double xx;
    double xy;
    double xz;
    double yx;
    double yy;
    double yz;
    double zx;
    double zy;
    double zz;

    double total = 0;
    for (const auto& value : xVector) {
        total += value;
    }
    xAverage = total / xVector.size();

    total = 0;
    for (const auto& value : yVector) {
        total += value;
    }
    yAverage = total / yVector.size();

    total = 0;
    for (const auto& value : zVector) {
        total += value;
    }
    zAverage = total / zVector.size();

    double tempxx = 0;
    double tempxy = 0;
    double tempxz = 0;
    double tempyx = 0;
    double tempyy = 0;
    double tempyz = 0;
    double tempzx = 0;
    double tempzy = 0;
    double tempzz = 0;
    for (int i = 0; i < size; i++) {
        tempxx += (xVector[i] - xAverage) * (xVector[i] - xAverage);
        tempxy += (xVector[i] - xAverage) * (yVector[i] - yAverage);
        tempxz += (xVector[i] - xAverage) * (zVector[i] - zAverage);

        tempyx += (yVector[i] - yAverage) * (xVector[i] - xAverage);
        tempyy += (yVector[i] - yAverage) * (yVector[i] - yAverage);
        tempyz += (yVector[i] - yAverage) * (zVector[i] - zAverage);

        tempzx += (zVector[i] - zAverage) * (xVector[i] - xAverage);
        tempzy += (zVector[i] - zAverage) * (yVector[i] - yAverage);
        tempzz += (zVector[i] - zAverage) * (zVector[i] - zAverage);
    }

    xx = tempxx / (size - 1);
    xy = tempxy / (size - 1);
    xz = tempxz / (size - 1);
    yx = tempyx / (size - 1);
    yy = tempyy / (size - 1);
    yz = tempyz / (size - 1);
    zx = tempzx / (size - 1);
    zy = tempzy / (size - 1);
    zz = tempzz / (size - 1);

    Eigen::Matrix3d mat;
    mat << xx, xy, xz, yx, yy, yz, zx, zy, zz;

    return mat;
}

/**
 * 函数：computeLOBBFromNeighbors
 * 作用：计算局部定向边界框
 * 输入：邻居点集合
 * 输出：l,w,h - 边界框的长宽高（按从大到小排序）
 * 原理：通过主成分分析找到局部坐标系，计算点在三个主方向上的投影范围
 */
void AccuratePositioning::computeLOBBFromNeighbors(const std::vector<pcl::PointXYZ>& neighbors, double& l, double& w, double& h) {
    // 计算质心
    Eigen::Vector3d centroid(0, 0, 0);
    for (const auto& p : neighbors) {
        centroid += Eigen::Vector3d(p.x, p.y, p.z);
    }
    centroid /= neighbors.size();

    // 计算协方差矩阵
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : neighbors) {
        Eigen::Vector3d d(p.x, p.y, p.z);
        d -= centroid;
        cov += d * d.transpose();
    }
    cov /= (neighbors.size() - 1);

    // 特征分解找到主方向
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    Eigen::Matrix3d eigVec = solver.eigenvectors();

    // 特征向量按特征值从大到小排列
    Eigen::Vector3d e1 = eigVec.col(2).normalized();
    Eigen::Vector3d e2 = eigVec.col(1).normalized();
    Eigen::Vector3d e3 = eigVec.col(0).normalized();

    // 计算点在三个主方向上的投影范围
    double min1 = std::numeric_limits<double>::max();
    double max1 = -std::numeric_limits<double>::max();
    double min2 = min1, max2 = -min1;
    double min3 = min1, max3 = -min1;

    for (const auto& p : neighbors) {
        Eigen::Vector3d d(p.x, p.y, p.z);
        d -= centroid;

        double p1 = d.dot(e1);
        double p2 = d.dot(e2);
        double p3 = d.dot(e3);

        min1 = std::min(min1, p1);
        max1 = std::max(max1, p1);
        min2 = std::min(min2, p2);
        max2 = std::max(max2, p2);
        min3 = std::min(min3, p3);
        max3 = std::max(max3, p3);
    }

    // 计算三个方向的长度并排序
    double L1 = max1 - min1;
    double L2 = max2 - min2;
    double L3 = max3 - min3;

    std::vector<double> dims = {L1, L2, L3};
    std::sort(dims.begin(), dims.end(), std::greater<double>());

    l = dims[0];
    w = dims[1];
    h = dims[2];
}

/**
 * 函数：buildFDN
 * 作用：构建特征检测邻域
 * 输入：
 *   - p: 当前点
 *   - seed: 种子点集
 *   - candidates: 候选点集
 *   - tau_d: 距离阈值
 *   - tau_r: 比率阈值
 * 输出：满足条件的特征邻域点集
 * 原理：基于局部几何约束筛选特征点
 */
std::vector<pcl::PointXYZ> AccuratePositioning::buildFDN(const pcl::PointXYZ& p, const std::vector<pcl::PointXYZ>& seed,
                                                         const std::vector<pcl::PointXYZ>& candidates, double tau_d,
                                                         double tau_r) {
    // 计算种子点集的质心和协方差矩阵
    Eigen::Vector3d c(0, 0, 0);
    for (auto& q : seed) c += Eigen::Vector3d(q.x, q.y, q.z);
    c /= seed.size();

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (auto& q : seed) {
        Eigen::Vector3d d(q.x, q.y, q.z);
        d -= c;
        cov += d * d.transpose();
    }

    // 特征分解得到局部坐标系
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    Eigen::Vector3d u = es.eigenvectors().col(2).normalized();
    Eigen::Vector3d v = es.eigenvectors().col(1).normalized();
    Eigen::Vector3d n = es.eigenvectors().col(0).normalized();

    Eigen::Vector3d pi(p.x, p.y, p.z);

    std::vector<pcl::PointXYZ> fdn;
    for (const auto& q : candidates) {
        Eigen::Vector3d d(q.x, q.y, q.z);
        d -= pi;

        if (std::abs(d.dot(n)) > tau_d) continue;

        // 切向距离比率约束：排除切向距离比率过大的点
        double du = std::abs(d.dot(u));
        double dv = std::abs(d.dot(v));
        if (dv < 1e-6) continue;
        if (du / dv > tau_r) continue;

        fdn.push_back(q);
    }
    return fdn;
}

/**
 * 函数：calculateF
 * 作用：计算点云中每个点的局部几何特征值
 * 输入：
 *   - cloud: 输入点云
 *   - fVector: 输出特征值向量
 *   - radius: 局部邻域半径
 * 原理：基于局部协方差分析，计算点的扁平度特征
 */
void AccuratePositioning::calculateF(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<double>& fVector,
                                     double radius) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    pcl::PointXYZ point;
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        std::vector<double> xVector;
        std::vector<double> yVector;
        std::vector<double> zVector;

        point = cloud->points[i];
        // 提取当前点的邻域
        std::vector<pcl::PointXYZ> neighbors = extractNeighbors(point, radius, tree, cloud);
        // 计算邻域点的协方差矩阵
        for (const auto& p : neighbors) {
            xVector.push_back(p.x);
            yVector.push_back(p.y);
            zVector.push_back(p.z);
        }

        Eigen::Matrix3d covMatrix = computeCovariance(xVector, yVector, zVector);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covMatrix);
        Eigen::Matrix3d vecMatrix = eigensolver.eigenvectors();
        Eigen::Vector3d u = vecMatrix.col(2);
        u.normalize();
        Eigen::Vector3d v = vecMatrix.col(1);
        v.normalize();
        Eigen::Vector3d w = u.cross(v);
        w.normalize();

        // 计算邻域质心
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (const auto& p : neighbors) {
            centroid(0) += p.x;
            centroid(1) += p.y;
            centroid(2) += p.z;
        }
        centroid /= neighbors.size();

        double min_u = std::numeric_limits<double>::max(), max_u = -min_u;
        double min_v = min_u, max_v = -min_u;
        double min_w = min_u, max_w = -min_u;

        for (const auto& p : neighbors) {
            Eigen::Vector3d vec;
            vec(0) = p.x - centroid(0);
            vec(1) = p.y - centroid(1);
            vec(2) = p.z - centroid(2);
            double proj_u = vec.dot(u);
            double proj_v = vec.dot(v);
            double proj_w = vec.dot(w);

            min_u = std::min(min_u, proj_u);
            max_u = std::max(max_u, proj_u);
            min_v = std::min(min_v, proj_v);
            max_v = std::max(max_v, proj_v);
            min_w = std::min(min_w, proj_w);
            max_w = std::max(max_w, proj_w);
        }
        Eigen::Matrix3d rotation;
        rotation.col(0) = u;
        rotation.col(1) = v;
        rotation.col(2) = w;

        // 计算特征值：法向范围与切向范围的比值（反映局部扁平度）
        Eigen::Vector3d size = Eigen::Vector3d(max_u - min_u, max_v - min_v, max_w - min_w);

        double f = size(2) / sqrt(size(0) * size(0) + size(1) * size(1));
        fVector.push_back(f);
    }
}

/**
 * 函数：nonlinearActivation
 * 作用：对特征向量进行非线性激活（tanh函数）
 * 输入：原始特征向量
 * 输出：激活后的特征向量
 * 原理：使用双曲正切函数增强特征区分度
 */
void AccuratePositioning::nonlinearActivation(std::vector<double> vector, std::vector<double>& activatedVector) {
    // 将向量转换为Eigen格式并归一化
    Eigen::VectorXd vec(vector.size());
    for (int i = 0; i < vector.size(); i++) {
        vec(i) = vector[i];
    }
    vec.normalize();

    // 应用tanh激活函数：tanh(x) = (e^x - e^{-x})/(e^x + e^{-x})
    Eigen::VectorXd projectedVector = vec.array() * 2;

    Eigen::VectorXd expX = projectedVector.array().exp();
    Eigen::VectorXd expNegX = (-projectedVector).array().exp();

    Eigen::VectorXd numerator = expX - expNegX;
    Eigen::VectorXd denominator = expX + expNegX;

    Eigen::VectorXd result = numerator.array() / denominator.array();
    result.normalize();

    for (int i = 0; i < result.size(); ++i) {
        activatedVector.push_back(result[i]);
    }
}

/**
 * 函数：kmeansPlusPlusInit
 * 作用：K-means++算法初始化聚类中心
 * 输入：
 *   - data: 数据点集合
 *   - k: 聚类数量
 * 输出：初始聚类中心
 * 原理：基于概率密度选择初始中心，避免K-means陷入局部最优
 */
std::vector<double> AccuratePositioning::kmeansPlusPlusInit(const std::vector<double>& data, int k) {
    std::vector<double> centers;
    if (data.empty() || k <= 0 || k > data.size()) return centers;

    std::random_device rd;
    std::mt19937 gen(rd());

    // 随机选择第一个中心
    std::uniform_int_distribution<int> initDist(0, data.size() - 1);
    centers.push_back(data[initDist(gen)]);

    // 选择后续中心：距离现有中心越远的点被选中的概率越大
    for (int i = 1; i < k; ++i) {
        std::vector<double> distances;
        double totalDistance = 0.0;

        // 计算每个点到最近中心的距离
        for (double x : data) {
            double minDist = std::numeric_limits<double>::max();
            for (double c : centers) {
                double d = pow(x - c, 2);
                if (d < minDist) minDist = d;
            }
            distances.push_back(minDist);
            totalDistance += minDist;
        }

        // 基于距离的概率分布选择下一个中心
        std::uniform_real_distribution<double> probDist(0.0, totalDistance);
        double threshold = probDist(gen);

        double cumulative = 0.0;
        int selected = 0;
        for (; selected < data.size(); ++selected) {
            cumulative += distances[selected];
            if (cumulative >= threshold) break;
        }

        selected = std::min(selected, (int)data.size() - 1);
        centers.push_back(data[selected]);
    }

    return centers;
}

std::vector<int> AccuratePositioning::kmeansPP(const std::vector<double>& data, int k, int maxIter,
                                               std::vector<int>& clusteringCounts) {
    std::vector<double> centers = kmeansPlusPlusInit(data, k);
    std::vector<int> assignments(data.size(), -1);
    bool changed;
    int iter = 0;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> randIndex(0, static_cast<int>(data.size()) - 1);

    do {
        changed = false;

        for (int i = 0; i < data.size(); ++i) {
            double minDist = std::numeric_limits<double>::max();
            int bestCluster = 0;

            for (int j = 0; j < centers.size(); ++j) {
                double dist = abs(data[i] - centers[j]);
                if (dist < minDist) {
                    minDist = dist;
                    bestCluster = j;
                }
            }

            if (assignments[i] != bestCluster) {
                assignments[i] = bestCluster;
                changed = true;
            }
        }

        std::vector<double> newCenters(k, 0.0);
        std::vector<int> counts(k, 0);

        for (int i = 0; i < data.size(); ++i) {
            int cluster = assignments[i];
            newCenters[cluster] += data[i];
            counts[cluster]++;
        }

        for (int j = 0; j < k; ++j) {
            if (counts[j] > 0) {
                newCenters[j] /= counts[j];
            } else {
                newCenters[j] = data[randIndex(gen)];
            }
        }

        clusteringCounts = counts;
        centers = newCenters;
        iter++;
    } while (changed && iter < maxIter);

    return assignments;
}

/**
 * 函数：compute
 * 作用：检测点云中的折痕特征（用于焊缝检测）
 * 输入：
 *   - cloud: 输入点云
 *   - creaseCloud: 输出折痕点云
 *   - radius: 局部邻域半径
 * 原理：基于局部几何特征聚类，识别具有高曲率变化的区域
 */
void AccuratePositioning::compute(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr& creaseCloud, double radius) {
    std::vector<double> fVector;  // 原始特征值

    std::vector<double> fActivatedVector;  // 激活后的特征值

    // 计算每个点的局部几何特征
    calculateF(cloud, fVector, radius);

    // 非线性激活增强特征区分度
    nonlinearActivation(fVector, fActivatedVector);

    // 使用K-means聚类将点分为两类（折痕和非折痕）
    std::vector<int> clusteringCounts1;
    std::vector<int> assignments1 = kmeansPP(fActivatedVector, 2, 50000, clusteringCounts1);
    if (clusteringCounts1[0] > clusteringCounts1[1]) {
        for (int i = 0; i < assignments1.size(); i++) {
            if (assignments1[i] == 1) {
                creaseCloud->points.push_back(cloud->points[i]);
            }
        }
    } else if (clusteringCounts1[0] <= clusteringCounts1[1]) {
        for (int i = 0; i < assignments1.size(); i++) {
            if (assignments1[i] == 0) {
                creaseCloud->points.push_back(cloud->points[i]);
            }
        }
    }
}

/**
 * 函数：pointcloudUniformDownsampling
 * 作用：均匀下采样点云
 * 输入：
 *   - cloud: 输入点云
 *   - leafSize: 采样半径
 *   - cloudResult: 输出下采样点云
 * 原理：在球体邻域内保留一个点，避免密度不均
 */
void AccuratePositioning::pointcloudUniformDownsampling(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float leafSize,
                                                        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudResult) {
    if (!cloud || cloud->points.empty()) {
        PLOGE << "点云为空，无法进行下采样";
        return;
    }
    pcl::UniformSampling<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.setRadiusSearch(leafSize);
    filter.filter(*cloudResult);
    if (cloudResult->points.empty()) {
        PLOGE << "下采样后点云为空";
    }
}

/**
 * 函数：pointcloudVoxelDownsampling
 * 作用：体素下采样点云
 * 输入：
 *   - cloud: 输入点云
 *   - leafSize: 体素尺寸
 *   - cloudResult: 输出下采样点云
 * 原理：将空间划分为体素网格，每个体素内保留一个点
 */
void AccuratePositioning::pointcloudVoxelDownsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leafSize,
                                                      pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudResult) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(leafSize, leafSize, leafSize);
    voxel.filter(*cloudResult);
}

/**
 * 函数：planeFitting
 * 作用：从点云中提取多个平面（RANSAC算法）
 * 输入：
 *   - cloud: 输入点云
 *   - cloudResult: 输出平面点云（合并）
 *   - planeEquations: 输出平面方程系数
 *   - planeClouds: 输出各个平面的点云
 *   - maxPlanes: 最大平面数量
 * 原理：迭代使用RANSAC算法拟合平面，逐步移除已拟合的点
 */
void AccuratePositioning::planeFitting(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudResult,
                                       std::vector<pcl::ModelCoefficients>& planeEquations,
                                       std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& planeClouds, int maxPlanes) {
    if (!cloud || cloud->points.empty()) {
        PLOGE << "点云为空，无法进行平面拟合";
        return;
    }

    planeEquations.clear();
    planeClouds.clear();

    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *tempCloud);  // 复制点云用于迭代处理

    int numPlanesFound = 0;

    // 迭代拟合多个平面
    while (numPlanesFound < maxPlanes) {
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        // RANSAC参数设置
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000);     // 待定参数
        seg.setDistanceThreshold(1.0);  // 待定参数

        seg.setInputCloud(tempCloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0) {
            PLOGE << "未能找到平面";
            break;
        }

        // 提取平面内点
        pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(tempCloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*planeCloud);

        // 保存结果
        planeEquations.push_back(*coefficients);
        planeClouds.push_back(planeCloud);

        // 移除已拟合的点，继续处理剩余点云
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ>::Ptr remainingCloud(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*remainingCloud);
        tempCloud = remainingCloud;

        numPlanesFound++;
    }

    // 合并所有平面点云作为最终结果
    if (!planeClouds.empty()) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr mergedPlaneCloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (size_t i = 0; i < planeClouds.size(); i++) {
            *mergedPlaneCloud += *planeClouds[i];
        }

        *cloudResult = *mergedPlaneCloud;
    } else {
        PLOGE << "未能提取到平面";
        return;
    }
}

/**
 * 函数：L1Median
 * 作用：使用L1中值滤波对点云进行去噪
 * 输入：
 *   - cloudInput: 输入点云
 *   - radius: 滤波半径
 *   - cloudOutput: 输出滤波后点云
 *   - maxIterations: 最大迭代次数
 * 原理：基于加权中值滤波，对每个点用邻域点的加权平均进行更新
 */
void AccuratePositioning::L1Median(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudInput, float radius,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudOutput, int maxIterations) {
    const float eps = 1e-6f;
    const float convergenceThreshold = 1e-5f;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPrev(new pcl::PointCloud<pcl::PointXYZ>(*cloudInput));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNext(new pcl::PointCloud<pcl::PointXYZ>(*cloudInput));

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);

    for (int iter = 0; iter < maxIterations; ++iter) {
        kdtree->setInputCloud(cloudPrev);

        double totalDisplacement = 0.0;

        for (size_t i = 0; i < cloudPrev->size(); ++i) {
            const pcl::PointXYZ& xi = cloudPrev->points[i];

            std::vector<int> indices;
            std::vector<float> distances;

            // 半径搜索获取邻域点
            if (kdtree->radiusSearch(xi, radius, indices, distances) <= 1) {
                cloudNext->points[i] = xi;  // 无足够邻域点，保持原位置
                continue;
            }

            Eigen::Vector3d numerator(0, 0, 0);
            double denominator = 0.0;

            bool singular = false;

            // 计算加权平均：权重为距离的倒数
            for (int idx : indices) {
                if (idx == static_cast<int>(i)) continue;

                const pcl::PointXYZ& pj = cloudPrev->points[idx];

                Eigen::Vector3d diff(xi.x - pj.x, xi.y - pj.y, xi.z - pj.z);

                double dist = diff.norm();

                if (dist < eps) {
                    singular = true;
                    cloudNext->points[i] = xi;
                    break;
                }

                double w = 1.0 / dist;  // 距离越近权重越大
                numerator += w * Eigen::Vector3d(pj.x, pj.y, pj.z);
                denominator += w;
            }

            if (!singular && denominator > eps) {
                // 更新点位置为邻域点的加权平均
                Eigen::Vector3d newPos = numerator / denominator;
                pcl::PointXYZ& dst = cloudNext->points[i];

                // 计算位移量用于收敛判断
                totalDisplacement +=
                    std::sqrt((dst.x - newPos.x()) * (dst.x - newPos.x()) + (dst.y - newPos.y()) * (dst.y - newPos.y()) +
                              (dst.z - newPos.z()) * (dst.z - newPos.z()));

                dst.x = static_cast<float>(newPos.x());
                dst.y = static_cast<float>(newPos.y());
                dst.z = static_cast<float>(newPos.z());
            }
        }

        // 收敛判断：平均位移小于阈值
        double meanDisp = totalDisplacement / cloudPrev->size();

        if (meanDisp < convergenceThreshold) {
            break;
        }

        *cloudPrev = *cloudNext;
    }

    cloudOutput = cloudNext;
}

/**
 * 函数：calculateDistance
 * 作用：计算两点之间的欧氏距离
 * 输入：两个三维点
 * 输出：距离值
 */
double AccuratePositioning::calculateDistance(const pcl::PointXYZ& point1, const pcl::PointXYZ& point2) {
    return std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2) + std::pow(point1.z - point2.z, 2));
}

/**
 * 函数：findFarthestPoints
 * 作用：在点云中找到距离最远的两个点
 * 输入：点云数据
 * 输出：最远点对
 * 原理：暴力搜索所有点对的距离，找到最大值
 */
void AccuratePositioning::findFarthestPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& inliers, pcl::PointXYZ& farthestPoint1,
                                             pcl::PointXYZ& farthestPoint2) {
    double maxDistance = std::numeric_limits<double>::min();
    size_t index1 = 0, index2 = 1;
    for (size_t i = 0; i < inliers->points.size(); ++i) {
        for (size_t j = i + 1; j < inliers->points.size(); ++j) {
            double distance = calculateDistance(inliers->points[i], inliers->points[j]);
            if (distance > maxDistance) {
                maxDistance = distance;
                index1 = i;
                index2 = j;
            }
        }
    }
    farthestPoint1 = inliers->points[index1];
    farthestPoint2 = inliers->points[index2];
}

/**
 * 函数：pointcloudProjectLine
 * 作用：将点云投影到直线上
 * 输入：
 *   - cloud: 输入点云
 *   - coefficients: 直线方程系数
 *   - cloudResult: 输出投影点云
 * 原理：计算点到直线的垂足作为投影点
 */
void AccuratePositioning::pointcloudProjectLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::ModelCoefficients coefficients,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudResult) {
    pcl::PointXYZ point;
    pcl::PointXYZ projectPoint;
    pcl::PointCloud<pcl::PointXYZ>::Ptr projectCloud(new pcl::PointCloud<pcl::PointXYZ>);

    double dx = coefficients.values[3];
    double dy = coefficients.values[4];
    double dz = coefficients.values[5];
    double px;
    double py;
    double pz;
    double x1 = coefficients.values[0];
    double y1 = coefficients.values[1];
    double z1 = coefficients.values[2];
    double t;

    for (size_t i = 0; i < cloud->points.size(); i++) {
        point = cloud->points[i];

        px = point.x;
        py = point.y;
        pz = point.z;

        t = -(dx * (x1 - px) + dy * (y1 - py) + dz * (z1 - pz)) / (dx * dx + dy * dy + dz * dz);

        projectPoint.x = x1 + t * dx;
        projectPoint.y = y1 + t * dy;
        projectPoint.z = z1 + t * dz;

        projectCloud->push_back(projectPoint);
    }
    *cloudResult = *projectCloud;
}

/**
 * 函数：extractLine
 * 作用：从点云中提取多条直线
 * 输入：
 *   - cloud: 输入点云
 *   - cloudResult: 输出直线点云集合
 *   - extremePoint: 输出每条直线的端点
 *   - modelCoefficients: 输出直线方程系数
 *   - maxLines: 最大直线数量
 * 原理：迭代使用RANSAC拟合直线，结合欧式聚类提取主要直线段
 */
void AccuratePositioning::extractLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cloudResult,
                                      std::vector<std::vector<pcl::PointXYZ>>& extremePoint,
                                      std::vector<pcl::ModelCoefficients>& modelCoefficients, int maxLines) {
    if (!cloud || cloud->points.empty()) {
        PLOGE << "点云为空，无法进行直线拟合";
        return;
    }
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliersPtr(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficientsPtr(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.8);  // 待定参数
    int numLinesFound = 0;
    while (numLinesFound < maxLines) {
        seg.setInputCloud(cloud);
        seg.segment(*inliersPtr, *coefficientsPtr);

        if (inliersPtr->indices.size() == 0) {
            break;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr inlierCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliersPtr);
        extract.setNegative(false);
        extract.filter(*inlierCloud);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(inlierCloud);

        std::vector<pcl::PointIndices> clusterIndices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(50);  // 待定参数
        ec.setMaxClusterSize(400);   // 待定参数
        ec.setSearchMethod(tree);
        ec.setInputCloud(inlierCloud);
        ec.extract(clusterIndices);

        int maxClusterIdx = -1;
        int maxClusterSize = std::numeric_limits<int>::min();
        for (size_t i = 0; i < clusterIndices.size(); i++) {
            int size = clusterIndices[i].indices.size();
            if (size > maxClusterSize) {
                maxClusterSize = size;
                maxClusterIdx = i;
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr largestCluster(new pcl::PointCloud<pcl::PointXYZ>);
        if (maxClusterIdx != -1) {
            for (const auto& idx : clusterIndices[maxClusterIdx].indices) {
                largestCluster->points.push_back(inlierCloud->points[idx]);
            }
            largestCluster->width = largestCluster->points.size();
            largestCluster->height = 1;
            largestCluster->is_dense = true;
            *inlierCloud = *largestCluster;

            cloudResult.push_back(largestCluster);
        }

        pointcloudProjectLine(inlierCloud, *coefficientsPtr, inlierCloud);

        pcl::PointXYZ farthestPoint1, farthestPoint2;
        findFarthestPoints(inlierCloud, farthestPoint1, farthestPoint2);
        std::vector<pcl::PointXYZ> eachExtremePoint;
        eachExtremePoint.push_back(farthestPoint1);
        eachExtremePoint.push_back(farthestPoint2);
        extremePoint.push_back(eachExtremePoint);

        modelCoefficients.push_back(*coefficientsPtr);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
        kdtree->setInputCloud(largestCluster);

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        std::vector<int> indices;
        float radius = 0.0001;

        for (size_t i = 0; i < cloud->size(); i++) {
            if (kdtree->radiusSearch(cloud->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
                indices.push_back(i);
            }
        }

        std::unordered_set<size_t> excludeIndices(indices.begin(), indices.end());

        for (size_t i = 0; i < cloud->size(); i++) {
            if (excludeIndices.find(i) == excludeIndices.end()) {
                cloudFiltered->push_back(cloud->points[i]);
            }
        }

        cloud = cloudFiltered;
        numLinesFound++;
    }
}

/**
 * 函数：computePlaneIntersectionLines
 * 作用：计算多个平面的交线（用于板板焊缝检测）
 * 输入：
 *   - planeEquations: 平面方程系数集合
 *   - planeClouds: 各平面的点云数据
 *   - postures: 输出焊枪姿态
 *   - lineEquations: 输出交线方程
 *   - cloudResult: 输出交线点云
 * 原理：通过平面法向量叉乘得到交线方向，计算交线段
 */
void AccuratePositioning::computePlaneIntersectionLines(const std::vector<pcl::ModelCoefficients>& planeEquations,
                                                        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& planeClouds,
                                                        std::vector<Posture>& postures,
                                                        std::vector<pcl::ModelCoefficients>& lineEquations,
                                                        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cloudResult) {
    if (planeEquations.size() < 2 || planeEquations.size() != planeClouds.size()) {
        PLOGE << "平面数量不足或与点云数量不匹配";
        return;
    }

    postures.clear();
    lineEquations.clear();
    cloudResult.clear();

    const double NearbyDist = 3.0;  // 交线附近的距离阈值（单位：mm）
    const int MinLineSupport = 20;  // 交线最小支撑点数
    const double GridSize = 3.0;    // 栅格化网格大小（待定参数）
    const int OpenRadius = 1.0;     // 形态学操作半径（1 = 3x3核）

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> filteredClouds;

    for (size_t i = 0; i < planeClouds.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr projected(new pcl::PointCloud<pcl::PointXYZ>);
        const auto& coeff = planeEquations[i];

        Eigen::Vector3d n(coeff.values[0], coeff.values[1], coeff.values[2]);
        double d = coeff.values[3];
        double n2 = n.squaredNorm();

        for (const auto& p : planeClouds[i]->points) {
            Eigen::Vector3d pt(p.x, p.y, p.z);
            double dist = (n.dot(pt) + d) / n2;
            Eigen::Vector3d proj = pt - dist * n;
            projected->push_back(pcl::PointXYZ(proj.x(), proj.y(), proj.z()));
        }

        Eigen::Vector3d u = n.unitOrthogonal();
        Eigen::Vector3d v = n.cross(u).normalized();

        std::vector<Eigen::Vector2i> gridIndex(projected->size());

        double minX = 1e9, minY = 1e9;
        double maxX = -1e9, maxY = -1e9;

        for (size_t k = 0; k < projected->size(); ++k) {
            Eigen::Vector3d pt(projected->points[k].x, projected->points[k].y, projected->points[k].z);
            double x = pt.dot(u);  // 点在u方向的坐标
            double y = pt.dot(v);  // 点在v方向的坐标
            minX = std::min(minX, x);
            minY = std::min(minY, y);
            maxX = std::max(maxX, x);
            maxY = std::max(maxY, y);
        }

        int W = static_cast<int>((maxX - minX) / GridSize) + 3;
        int H = static_cast<int>((maxY - minY) / GridSize) + 3;

        std::vector<std::vector<int>> grid(W, std::vector<int>(H, 0));

        for (size_t k = 0; k < projected->size(); ++k) {
            Eigen::Vector3d pt(projected->points[k].x, projected->points[k].y, projected->points[k].z);
            int gx = static_cast<int>((pt.dot(u) - minX) / GridSize);
            int gy = static_cast<int>((pt.dot(v) - minY) / GridSize);
            gridIndex[k] = {gx, gy};
            if (gx >= 0 && gx < W && gy >= 0 && gy < H) grid[gx][gy] = 1;
        }

        std::vector<std::vector<int>> eroded = grid;
        for (int x = OpenRadius; x < W - OpenRadius; ++x) {
            for (int y = OpenRadius; y < H - OpenRadius; ++y) {
                if (grid[x][y] == 1) {
                    for (int dx = -OpenRadius; dx <= OpenRadius; ++dx)
                        for (int dy = -OpenRadius; dy <= OpenRadius; ++dy)
                            if (grid[x + dx][y + dy] == 0) eroded[x][y] = 0;
                }
            }
        }

        std::vector<std::vector<int>> opened = eroded;
        for (int x = OpenRadius; x < W - OpenRadius; ++x) {
            for (int y = OpenRadius; y < H - OpenRadius; ++y) {
                if (eroded[x][y] == 0) {
                    for (int dx = -OpenRadius; dx <= OpenRadius; ++dx)
                        for (int dy = -OpenRadius; dy <= OpenRadius; ++dy)
                            if (eroded[x + dx][y + dy] == 1) opened[x][y] = 1;
                }
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t k = 0; k < projected->size(); ++k) {
            int gx = gridIndex[k].x();
            int gy = gridIndex[k].y();
            if (gx >= 0 && gx < W && gy >= 0 && gy < H && opened[gx][gy] == 1) filtered->push_back(projected->points[k]);
        }

        filteredClouds.push_back(filtered);
    }
    // 计算平面交线
    for (size_t i = 0; i < planeEquations.size(); ++i) {
        for (size_t j = i + 1; j < planeEquations.size(); ++j) {
            Eigen::Vector3d n1(planeEquations[i].values[0], planeEquations[i].values[1], planeEquations[i].values[2]);
            Eigen::Vector3d n2(planeEquations[j].values[0], planeEquations[j].values[1], planeEquations[j].values[2]);

            Eigen::Vector3d dir = n1.cross(n2);
            if (dir.norm() < 1e-6) continue;
            dir.normalize();

            Eigen::Matrix3d A;
            A << n1.transpose(), n2.transpose(), dir.transpose();

            Eigen::Vector3d b(-planeEquations[i].values[3], -planeEquations[j].values[3], 0.0);
            // 使用QR分解求解线性方程组，得到交线上的一点P0
            Eigen::Vector3d P0 = A.colPivHouseholderQr().solve(b);

            auto extractProjectedT = [&](const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<double>& t_values) {
                for (const auto& p : cloud->points) {
                    Eigen::Vector3d pt(p.x, p.y, p.z);
                    Eigen::Vector3d v = pt - P0;
                    Eigen::Vector3d perp = v - v.dot(dir) * dir;
                    if (perp.norm() <= NearbyDist) t_values.push_back(v.dot(dir));
                }
            };

            std::vector<double> t1, t2;
            extractProjectedT(filteredClouds[i], t1);
            extractProjectedT(filteredClouds[j], t2);

            if (t1.size() < MinLineSupport || t2.size() < MinLineSupport) continue;

            double minT = std::max(*std::min_element(t1.begin(), t1.end()), *std::min_element(t2.begin(), t2.end()));
            double maxT = std::min(*std::max_element(t1.begin(), t1.end()), *std::max_element(t2.begin(), t2.end()));

            if (maxT <= minT) continue;

            Eigen::Vector3d e1 = P0 + minT * dir;
            Eigen::Vector3d e2 = P0 + maxT * dir;

            std::vector<pcl::ModelCoefficients> surfaces;
            surfaces.push_back(planeEquations[i]);
            surfaces.push_back(planeEquations[j]);

            std::vector<Point3D> positions;
            positions.push_back(Point3D(e1.x(), e1.y(), e1.z()));
            positions.push_back(Point3D(e2.x(), e2.y(), e2.z()));

            std::vector<std::vector<pcl::ModelCoefficients>> groupedSurfaces;
            for (size_t k = 0; k < positions.size(); ++k) {
                groupedSurfaces.push_back(surfaces);
            }

            std::vector<Eigen::Quaternionf> poses;
            poses.push_back(Eigen::Quaternionf(1, 0, 0, 0));
            poses.push_back(Eigen::Quaternionf(1, 0, 0, 0));

            std::vector<Point3D> tangentVectors;
            if (positions.size() >= 2) {
                Point3D tangent;
                tangent.x = positions[1].x - positions[0].x;
                tangent.y = positions[1].y - positions[0].y;
                tangent.z = positions[1].z - positions[0].z;

                double length = sqrt(tangent.x * tangent.x + tangent.y * tangent.y + tangent.z * tangent.z);
                if (length > 1e-10) {
                    tangent.x /= length;
                    tangent.y /= length;
                    tangent.z /= length;
                }

                tangentVectors.push_back(tangent);
                tangentVectors.push_back(tangent);
            }
            Posture combinedPosture(positions, groupedSurfaces, poses, tangentVectors);
            postures.push_back(combinedPosture);

            pcl::ModelCoefficients line;
            line.values = {(float)P0.x(), (float)P0.y(), (float)P0.z(), (float)dir.x(), (float)dir.y(), (float)dir.z()};
            lineEquations.push_back(line);
        }
    }

    cloudResult = filteredClouds;
}

/**
 * 函数：fitPlaneAnd2Cylinders
 * 作用：
 *   从复杂点云中同时拟合：
 *     1）两个圆柱面
 *     2）一个平面
 * 输入：
 *   - cloud: 原始输入点云
 * 输出：
 *   - planeCoefficients: 平面模型参数
 *   - planeInliers: 位于平面上、且靠近两个圆柱交线区域的点云
 *   - cylinderCoefficients: 两个圆柱模型参数
 * 原理：
 *   1）对点云进行两级下采样：
 *        - 粗采样：提高 RANSAC 拟合稳定性
 *        - 细采样：用于精确点筛选
 *   2）基于法向量约束的 RANSAC：
 *        - 先拟合第一个圆柱
 *        - 剔除其内点
 *        - 再拟合第二个圆柱
 *   3）在剩余点云中拟合平面
 *   4）在原始高分辨率点云中：筛选平面附近点
 */
void AccuratePositioning::fitPlaneAnd2Cylinders(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                pcl::ModelCoefficients& planeCoefficients,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr& planeInliers,
                                                std::vector<pcl::ModelCoefficients>& cylinderCoefficients) {
    if (!cloud || cloud->empty()) {
        PLOGE << "点云为空";
        return;
    }

    cylinderCoefficients.clear();
    planeInliers.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // ---------- 下采样 ----------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFit(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudExtract(new pcl::PointCloud<pcl::PointXYZ>);
    pointcloudUniformDownsampling(cloud, 2.5f, cloudFit);
    pointcloudUniformDownsampling(cloud, 1.0f, cloudExtract);

    pcl::PointCloud<pcl::PointXYZ>::Ptr remainingCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloudFit));

    // ---------- 法向量估计 ----------
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    ne.setSearchMethod(tree);
    ne.setInputCloud(remainingCloud);
    ne.setKSearch(100);
    ne.compute(*normals);

    pcl::ExtractIndices<pcl::PointXYZ> extract;

    std::vector<pcl::ModelCoefficients> cylCoeff(2);
    std::vector<pcl::PointIndices::Ptr> cylIdx(2);

    // ---------- 顺序拟合两个圆柱 ----------
    for (int k = 0; k < 2; ++k) {
        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
        cylIdx[k].reset(new pcl::PointIndices);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight(0.1);
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.5);
        seg.setRadiusLimits(10, 150);
        seg.setInputCloud(remainingCloud);
        seg.setInputNormals(normals);

        seg.segment(*cylIdx[k], cylCoeff[k]);
        if (cylIdx[k]->indices.empty()) break;

        // 移除当前圆柱内点，避免影响下一个圆柱
        extract.setInputCloud(remainingCloud);
        extract.setIndices(cylIdx[k]);
        extract.setNegative(true);

        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*tmp);
        remainingCloud = tmp;

        // 重新计算剩余点云的法向量
        normals.reset(new pcl::PointCloud<pcl::Normal>);
        ne.setInputCloud(remainingCloud);
        ne.compute(*normals);
    }

    if (cylIdx[0]->indices.empty() || cylIdx[1]->indices.empty()) return;

    // ---------- 拟合平面 ----------
    pcl::SACSegmentation<pcl::PointXYZ> planeSeg;
    pcl::PointIndices::Ptr planeIdx(new pcl::PointIndices);

    planeSeg.setOptimizeCoefficients(true);
    planeSeg.setModelType(pcl::SACMODEL_PLANE);
    planeSeg.setMethodType(pcl::SAC_RANSAC);
    planeSeg.setMaxIterations(1000);
    planeSeg.setDistanceThreshold(1.0);
    planeSeg.setInputCloud(remainingCloud);
    planeSeg.segment(*planeIdx, planeCoefficients);

    if (planeIdx->indices.empty()) return;

    // ---------- 提取平面与圆柱交线区域 ----------
    Eigen::Vector3d P0_1(cylCoeff[0].values[0], cylCoeff[0].values[1], cylCoeff[0].values[2]);
    Eigen::Vector3d A1(cylCoeff[0].values[3], cylCoeff[0].values[4], cylCoeff[0].values[5]);
    A1.normalize();
    double r1 = cylCoeff[0].values[6];

    Eigen::Vector3d P0_2(cylCoeff[1].values[0], cylCoeff[1].values[1], cylCoeff[1].values[2]);
    Eigen::Vector3d A2(cylCoeff[1].values[3], cylCoeff[1].values[4], cylCoeff[1].values[5]);
    A2.normalize();
    double r2 = cylCoeff[1].values[6];

    double a = planeCoefficients.values[0];
    double b = planeCoefficients.values[1];
    double c = planeCoefficients.values[2];
    double d = planeCoefficients.values[3];
    double invPlaneNorm = 1.0 / std::sqrt(a * a + b * b + c * c);

    const double planeTol = 1.0;
    const double radialTol = 15;

    for (const auto& p : cloudExtract->points) {
        Eigen::Vector3d pt(p.x, p.y, p.z);

        // 点到平面的距离
        double planeDist = std::abs(a * p.x + b * p.y + c * p.z + d) * invPlaneNorm;
        if (planeDist > planeTol) continue;

        bool inIntersection = false;

        // 到第一个圆柱轴线的径向距离
        {
            Eigen::Vector3d v = pt - P0_1;
            double t = v.dot(A1);
            double r = (v - t * A1).norm();
            if (std::abs(r - r1) < radialTol) inIntersection = true;
        }

        // 到第二个圆柱轴线的径向距离
        if (!inIntersection) {
            Eigen::Vector3d v = pt - P0_2;
            double t = v.dot(A2);
            double r = (v - t * A2).norm();
            if (std::abs(r - r2) < radialTol) inIntersection = true;
        }

        if (inIntersection) planeInliers->push_back(p);
    }

    cylinderCoefficients.push_back(cylCoeff[0]);
    cylinderCoefficients.push_back(cylCoeff[1]);
}

/**
 * 函数：fit2Cylinders
 * 作用：
 *   顺序拟合两个圆柱，并提取各自内点
 * 输入：
 *   - cloud：输入点云
 * 输出：
 *   - cylinderCoefficients：圆柱模型参数
 *   - cylinderInliers：每个圆柱的内点
 */
void AccuratePositioning::fit2Cylinders(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                        std::vector<pcl::ModelCoefficients>& cylinderCoefficients,
                                        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cylinderInliers) {
    if (!cloud || cloud->empty()) {
        PLOGE << "点云为空，无法进行拟合";
        return;
    }

    cylinderCoefficients.clear();
    cylinderInliers.clear();

    // ---------- 法向量估计 ----------
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(100);  // 待定参数
    ne.setNumberOfThreads(omp_get_max_threads());
    ne.compute(*normals);

    pcl::PointCloud<pcl::PointXYZ>::Ptr remainingCloud = cloud;
    pcl::PointCloud<pcl::Normal>::Ptr remainingNormals = normals;

    pcl::ExtractIndices<pcl::PointXYZ> extractPts;
    pcl::ExtractIndices<pcl::Normal> extractNormals;

    // ---------- 圆柱 RANSAC ----------
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);  // 待定参数
    seg.setMaxIterations(8000);        // 待定参数
    seg.setDistanceThreshold(1.5);     // 待定参数
    seg.setRadiusLimits(10, 200);      // 待定参数

    for (int i = 0; i < 2; ++i) {
        if (!remainingCloud || remainingCloud->empty()) break;

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients coeffs;

        seg.setInputCloud(remainingCloud);
        seg.setInputNormals(remainingNormals);
        seg.segment(*inliers, coeffs);

        if (inliers->indices.empty()) {
            if (i == 0)
                PLOGE << "第一个圆柱面拟合失败";
            else
                PLOGE << "第二个圆柱面拟合失败";
            break;
        }

        // 提取当前圆柱内点
        pcl::PointCloud<pcl::PointXYZ>::Ptr cylinderCloud(new pcl::PointCloud<pcl::PointXYZ>);
        extractPts.setInputCloud(remainingCloud);
        extractPts.setIndices(inliers);
        extractPts.setNegative(false);
        extractPts.filter(*cylinderCloud);

        cylinderCoefficients.push_back(coeffs);
        cylinderInliers.push_back(cylinderCloud);

        // 移除已拟合圆柱
        pcl::PointCloud<pcl::PointXYZ>::Ptr newRemainingCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr newRemainingNormals(new pcl::PointCloud<pcl::Normal>);

        extractPts.setNegative(true);
        extractPts.filter(*newRemainingCloud);

        extractNormals.setInputCloud(remainingNormals);
        extractNormals.setIndices(inliers);
        extractNormals.setNegative(true);
        extractNormals.filter(*newRemainingNormals);

        remainingCloud = newRemainingCloud;
        remainingNormals = newRemainingNormals;
    }

    if (cylinderCoefficients.empty()) {
        PLOGE << "未能找到任何圆柱面，请检查点云或参数";
        return;
    } else if (cylinderCoefficients.size() == 1) {
        PLOGW << "只找到一个圆柱面";
    } else {
        PLOGD << "成功找到两个圆柱面";
    }
}

/**
 * 函数：compute2CylinderIntersection
 * 作用：计算两个圆柱面的空间交线点云
 * 输入：
 *   - cylinderCoefficients：两个圆柱参数
 *   - cylinderInliers：对应的圆柱内点
 * 输出：
 *   - cloudResult：真实交线
 */
void AccuratePositioning::compute2CylinderIntersection(const std::vector<pcl::ModelCoefficients>& cylinderCoefficients,
                                                       const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cylinderInliers,
                                                       pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudResult) {
    cloudResult->clear();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudResult1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudResult2(new pcl::PointCloud<pcl::PointXYZ>);

    int mainIdx = 0;
    int branchIdx = 1;

    // 主圆柱
    Eigen::Vector3f C1(cylinderCoefficients[mainIdx].values[0], cylinderCoefficients[mainIdx].values[1],
                       cylinderCoefficients[mainIdx].values[2]);
    Eigen::Vector3f D1(cylinderCoefficients[mainIdx].values[3], cylinderCoefficients[mainIdx].values[4],
                       cylinderCoefficients[mainIdx].values[5]);
    float R1 = cylinderCoefficients[mainIdx].values[6];
    D1.normalize();

    // 支圆柱
    Eigen::Vector3f C2(cylinderCoefficients[branchIdx].values[0], cylinderCoefficients[branchIdx].values[1],
                       cylinderCoefficients[branchIdx].values[2]);
    Eigen::Vector3f D2(cylinderCoefficients[branchIdx].values[3], cylinderCoefficients[branchIdx].values[4],
                       cylinderCoefficients[branchIdx].values[5]);
    float R2 = cylinderCoefficients[branchIdx].values[6] - 9.8;
    D2.normalize();

    Eigen::Vector3f u, v;
    // 构造支圆柱截面正交基
    Eigen::Vector3f arbitrary(1.0f, 0.0f, 0.0f);

    if (std::abs(D2.dot(arbitrary)) > 0.99f) {
        arbitrary = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    }

    u = D2.cross(arbitrary).normalized();
    v = D2.cross(u).normalized();

    // ---------- 扫描圆周，求与主圆柱的交点 ----------
    int steps = 180;

    for (int i = 0; i < steps; ++i) {
        float theta = i * 2.0f * M_PI / steps;

        Eigen::Vector3f circleOffset = R2 * (std::cos(theta) * u + std::sin(theta) * v);
        Eigen::Vector3f PBase = C2 + circleOffset;

        Eigen::Vector3f Delta = PBase - C1;
        Eigen::Vector3f AVec = D2.cross(D1);
        Eigen::Vector3f BVec = Delta.cross(D1);

        double aCoeff = AVec.dot(AVec);
        double bCoeff = 2.0 * AVec.dot(BVec);
        double cCoeff = BVec.dot(BVec) - (R1 * R1);

        double discriminant = bCoeff * bCoeff - 4.0 * aCoeff * cCoeff;

        if (discriminant >= 0 && aCoeff > 1e-6) {
            double t1 = (-bCoeff - std::sqrt(discriminant)) / (2.0 * aCoeff);
            double t2 = (-bCoeff + std::sqrt(discriminant)) / (2.0 * aCoeff);

            Eigen::Vector3f intersection1 = PBase + (float)t1 * D2;
            Eigen::Vector3f intersection2 = PBase + (float)t2 * D2;

            pcl::PointXYZ pt1, pt2;
            pt1.x = intersection1.x();
            pt1.y = intersection1.y();
            pt1.z = intersection1.z();
            pt2.x = intersection2.x();
            pt2.y = intersection2.y();
            pt2.z = intersection2.z();

            cloudResult1->push_back(pt1);
            cloudResult2->push_back(pt2);
        }
    }

    double minDist1 = std::numeric_limits<double>::max();
    for (const auto& pt1 : cloudResult1->points) {
        for (const auto& pt2 : cylinderInliers[branchIdx]->points) {
            double dist = sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) + pow(pt1.z - pt2.z, 2));
            if (dist < minDist1) {
                minDist1 = dist;
            }
        }
    }

    double minDist2 = std::numeric_limits<double>::max();
    for (const auto& pt1 : cloudResult2->points) {
        for (const auto& pt2 : cylinderInliers[branchIdx]->points) {
            double dist = sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) + pow(pt1.z - pt2.z, 2));
            if (dist < minDist2) {
                minDist2 = dist;
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr fullIntersection(new pcl::PointCloud<pcl::PointXYZ>);
    if (minDist1 <= minDist2) {
        *fullIntersection = *cloudResult1;
    } else {
        *fullIntersection = *cloudResult2;
    }

    Eigen::Vector3f ONew = C2;

    Eigen::Vector3f ZNew = D2.normalized();

    Eigen::Vector3f YNew = D2.cross(D1).normalized();

    Eigen::Vector3f XNew = YNew.cross(ZNew).normalized();

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> parts(4);
    for (int k = 0; k < 4; k++) parts[k].reset(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& pt : fullIntersection->points) {
        Eigen::Vector3f PWorld(pt.x, pt.y, pt.z);
        Eigen::Vector3f PVec = PWorld - ONew;

        float xLocal = PVec.dot(XNew);
        float yLocal = PVec.dot(YNew);

        if (xLocal >= 0 && yLocal >= 0)
            parts[0]->push_back(pt);
        else if (xLocal < 0 && yLocal >= 0)
            parts[1]->push_back(pt);
        else if (xLocal < 0 && yLocal < 0)
            parts[2]->push_back(pt);
        else if (xLocal >= 0 && yLocal < 0)
            parts[3]->push_back(pt);
    }

    std::vector<int> votes(4, 0);
    if (cylinderInliers[branchIdx]->size() > 0) {
        for (const auto& pt : cylinderInliers[branchIdx]->points) {
            Eigen::Vector3f PInlier(pt.x, pt.y, pt.z);
            Eigen::Vector3f PVec = PInlier - ONew;

            float xLocal = PVec.dot(XNew);
            float yLocal = PVec.dot(YNew);

            if (xLocal >= 0 && yLocal >= 0)
                votes[0]++;
            else if (xLocal < 0 && yLocal >= 0)
                votes[1]++;
            else if (xLocal < 0 && yLocal < 0)
                votes[2]++;
            else if (xLocal >= 0 && yLocal < 0)
                votes[3]++;
        }
    }

    int bestPartIdx = 0;
    int maxVotes = -1;
    for (int i = 0; i < 4; ++i) {
        if (votes[i] > maxVotes) {
            maxVotes = votes[i];
            bestPartIdx = i;
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr selectedCloud = parts[bestPartIdx];

    std::sort(selectedCloud->points.begin(), selectedCloud->points.end(), [&](const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
        Eigen::Vector3f vecA(a.x - ONew.x(), a.y - ONew.y(), a.z - ONew.z());
        Eigen::Vector3f vecB(b.x - ONew.x(), b.y - ONew.y(), b.z - ONew.z());

        float xLocalA = vecA.dot(XNew);
        float xLocalB = vecB.dot(XNew);

        return std::abs(xLocalA) < std::abs(xLocalB);
    });

    *cloudResult = *selectedCloud;
}

/**
 * 函数：fitPlaneAndCylinder
 * 作用：先拟合平面，再在剩余点云中拟合一个圆柱
 * 输入：- cloud：输入点云
 * 输出：
 *   - planeCoefficients ：平面参数
 *   - planeInliers：平面内点
 *   - cylinderCoefficients：圆柱面参数
 *   - cylinderInliers：圆柱面内点
 */
void AccuratePositioning::fitPlaneAndCylinder(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                              pcl::ModelCoefficients& planeCoefficients,
                                              pcl::PointCloud<pcl::PointXYZ>::Ptr& planeInliers,
                                              pcl::ModelCoefficients& cylinderCoefficients,
                                              pcl::PointCloud<pcl::PointXYZ>::Ptr& cylinderInliers) {
    if (!cloud || cloud->points.empty()) {
        PLOGE << "点云为空，无法进行拟合";
        return;
    }

    planeInliers->clear();
    cylinderInliers->clear();

    pcl::PointCloud<pcl::PointXYZ>::Ptr remainingCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloud));

    // ---------- 拟合平面 ----------
    pcl::SACSegmentation<pcl::PointXYZ> planeSeg;
    pcl::PointIndices::Ptr planeInlierIndices(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr planeCoeff(new pcl::ModelCoefficients);

    planeSeg.setOptimizeCoefficients(true);
    planeSeg.setModelType(pcl::SACMODEL_PLANE);
    planeSeg.setMethodType(pcl::SAC_RANSAC);
    planeSeg.setMaxIterations(1000);
    planeSeg.setDistanceThreshold(1.0);
    planeSeg.setInputCloud(remainingCloud);
    planeSeg.segment(*planeInlierIndices, *planeCoeff);

    if (planeInlierIndices->indices.empty()) {
        PLOGE << "未能找到平面";
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(remainingCloud);
    extract.setIndices(planeInlierIndices);
    extract.setNegative(false);
    extract.filter(*planeInliers);

    planeCoefficients = *planeCoeff;

    // ---------- 移除平面 ----------
    extract.setNegative(true);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*tempCloud);
    remainingCloud = tempCloud;

    if (remainingCloud->points.empty()) {
        PLOGE << "移除平面内点后，剩余点云为空";
    }

    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> cylinderSeg;
    pcl::PointIndices::Ptr cylinderInlierIndices(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr cylinderCoeff(new pcl::ModelCoefficients);

    // ---------- 拟合圆柱 ----------
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);

    ne.setSearchMethod(tree);
    ne.setInputCloud(remainingCloud);
    ne.setKSearch(60);
    ne.compute(*cloudNormals);

    cylinderSeg.setOptimizeCoefficients(true);
    cylinderSeg.setModelType(pcl::SACMODEL_CYLINDER);
    cylinderSeg.setMethodType(pcl::SAC_RANSAC);
    cylinderSeg.setNormalDistanceWeight(0.1);
    cylinderSeg.setMaxIterations(10000);
    cylinderSeg.setDistanceThreshold(1.0);
    cylinderSeg.setRadiusLimits(50.0, 150.0);
    cylinderSeg.setInputCloud(remainingCloud);
    cylinderSeg.setInputNormals(cloudNormals);
    cylinderSeg.segment(*cylinderInlierIndices, *cylinderCoeff);

    if (cylinderInlierIndices->indices.empty()) {
        PLOGE << "未能找到圆柱面";
    }

    extract.setInputCloud(remainingCloud);
    extract.setIndices(cylinderInlierIndices);
    extract.setNegative(false);
    extract.filter(*cylinderInliers);

    cylinderCoefficients = *cylinderCoeff;
}

/**
 * 函数：computefitPlaneAndCylinderIntersection
 * 作用：计算一个平面与一个圆柱面的空间交线，
 * 输入：
 *   - planeCoefficients: 平面参数
 *   - cylinderCoefficients: 圆柱参数
 * 输出：
 *   - cloudResult: 平面与圆柱的空间交线点云
 * 原理：
 *   1）以圆柱轴线为局部坐标系 z 轴
 *   2）将平面方程变换到圆柱局部坐标系
 *   3）在圆柱表面进行角度参数化
 *   4）代入平面方程解析求解 z
 */
void AccuratePositioning::computefitPlaneAndCylinderIntersection(const pcl::ModelCoefficients& planeCoefficients,
                                                                 const pcl::ModelCoefficients& cylinderCoefficients,
                                                                 pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudResult) {
    cloudResult->clear();

    // 提取圆柱轴线与半径
    Eigen::Vector3f cylPt(cylinderCoefficients.values[0], cylinderCoefficients.values[1], cylinderCoefficients.values[2]);
    Eigen::Vector3f cylAxis(cylinderCoefficients.values[3], cylinderCoefficients.values[4], cylinderCoefficients.values[5]);
    cylAxis.normalize();
    float r = cylinderCoefficients.values[6];

    Eigen::Vector4f planeVec(planeCoefficients.values[0], planeCoefficients.values[1], planeCoefficients.values[2],
                             planeCoefficients.values[3]);

    // 构造圆柱局部坐标系（x,y,z）
    Eigen::Vector3f zAxis = cylAxis;
    Eigen::Vector3f xAxis = zAxis.unitOrthogonal();
    Eigen::Vector3f yAxis = zAxis.cross(xAxis);

    // 构造局部 → 全局变换矩阵
    Eigen::Matrix4f localToGlobal = Eigen::Matrix4f::Identity();
    localToGlobal.block<3, 1>(0, 0) = xAxis;
    localToGlobal.block<3, 1>(0, 1) = yAxis;
    localToGlobal.block<3, 1>(0, 2) = zAxis;
    localToGlobal.block<3, 1>(0, 3) = cylPt;

    // 将平面方程变换到圆柱局部坐标系
    Eigen::Vector4f localPlane = localToGlobal.transpose() * planeVec;
    float A = localPlane[0];
    float B = localPlane[1];
    float C = localPlane[2];
    float D = localPlane[3];

    // 在圆柱表面进行角度扫描，解析求解交线点
    const int numPoints = 200;
    for (int i = 0; i < numPoints; ++i) {
        float theta = 2.0f * M_PI * i / numPoints;
        float lx = r * std::cos(theta);
        float ly = r * std::sin(theta);

        // 平面方程代入，解析求 z
        float lz = -(A * lx + B * ly + D) / C;

        // 局部 → 全局
        Eigen::Vector4f pLocal(lx, ly, lz, 1.0f);
        Eigen::Vector4f pGlobal = localToGlobal * pLocal;

        pcl::PointXYZ pt;
        pt.x = pGlobal[0];
        pt.y = pGlobal[1];
        pt.z = pGlobal[2];

        cloudResult->push_back(pt);
    }
}

/**
 * 函数：euclideanClusteringRemoval
 * 作用：对点云进行欧式聚类，仅保留点数最多的一个聚类
 * 输入：
 *   - cloud: 输入点云
 *
 * 输出：
 * - cloudResult: 最大连通区域点云
 * 原理：基于 KD-Tree 的欧式距离聚类，用于去除离散噪声
 */
void AccuratePositioning::euclideanClusteringRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                     pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudResult) {
    cloudResult->clear();

    if (!cloud || cloud->empty()) {
        PLOGE << "输入点云为空";
        return;
    }

    // KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // 欧式聚类
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(2);
    ec.setMinClusterSize(1);
    ec.setMaxClusterSize(50000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    ec.extract(clusterIndices);

    if (clusterIndices.empty()) {
        PLOGE << "聚类失败";
        return;
    }

    // 选取点数最多的聚类
    const pcl::PointIndices* maxCluster = nullptr;
    size_t maxSize = 0;

    for (const auto& cluster : clusterIndices) {
        if (cluster.indices.size() > maxSize) {
            maxSize = cluster.indices.size();
            maxCluster = &cluster;
        }
    }

    if (!maxCluster) {
        PLOGE << "未找到有效聚类";
        return;
    }

    cloudResult->reserve(maxSize);

    for (int idx : maxCluster->indices) {
        cloudResult->push_back((*cloud)[idx]);
    }
}

/**
 * 函数：extractCylinderBoundary
 * 作用： 从圆柱面内点中提取边界曲线
 * 输入：
 *   - cylinderCoefficients: 圆柱模型参数
 *   - cylinderInliers: 圆柱内点
 * 输出：
 *   - cloudResult: 圆柱边界点云
 * 原理：
 *   1）所有点投影回理想圆柱面
 *   2）按角度分桶，统计每个角度下的最小轴向高度
 *   3）利用支持点和 MAD 进行鲁棒滤波
 *   4）得到连续边界曲线
 */
void AccuratePositioning::extractCylinderBoundary(const pcl::ModelCoefficients& cylinderCoefficients,
                                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cylinderInliers,
                                                  pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudResult) {
    if (!cloudResult) {
        PLOGE << "输出变量不存在";
        return;
    }

    if (!cylinderInliers || cylinderInliers->empty()) {
        PLOGE << "圆柱面内点为空";
        return;
    }

    if (cylinderCoefficients.values.size() < 7) {
        PLOGE << "圆柱面参数不足";
        return;
    }

    cloudResult->clear();

    /* ================== 解析圆柱参数 ================== */
    Eigen::Vector3d P0(cylinderCoefficients.values[0], cylinderCoefficients.values[1], cylinderCoefficients.values[2]);

    Eigen::Vector3d axis(cylinderCoefficients.values[3], cylinderCoefficients.values[4], cylinderCoefficients.values[5]);
    axis.normalize();

    double radius = cylinderCoefficients.values[6];

    if (radius <= 0.0) {
        PLOGE << "圆柱半径错误";
        return;
    }

    if (axis.norm() < 1e-9) {
        PLOGE << "圆柱轴线错误";
        return;
    }

    /* ================== 构建圆柱局部正交基 ================== */
    Eigen::Vector3d tmp(1, 0, 0);
    if (std::fabs(axis.dot(tmp)) > 0.9) tmp = Eigen::Vector3d(0, 1, 0);

    Eigen::Vector3d u = axis.cross(tmp).normalized();
    Eigen::Vector3d v = axis.cross(u).normalized();

    /* ================== 角度分桶参数 ================== */
    const double dTheta = 2.0 * M_PI / 180.0;
    std::map<int, std::vector<double>> bucket;

    pcl::PointCloud<pcl::PointXYZ>::Ptr projectedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    projectedCloud->reserve(cylinderInliers->size());  // 预分配空间提高效率

// 并行投影点到圆柱面上
#pragma omp parallel
    {
        pcl::PointCloud<pcl::PointXYZ> local_projectedCloud;
#pragma omp for nowait
        for (int i = 0; i < static_cast<int>(cylinderInliers->size()); ++i) {
            const auto& p = cylinderInliers->points[i];
            Eigen::Vector3d P(p.x, p.y, p.z);
            Eigen::Vector3d d = P - P0;

            double h = d.dot(axis);
            Eigen::Vector3d PProj = P0 + h * axis;

            Eigen::Vector3d radial = P - PProj;
            double distanceToAxis = radial.norm();

            if (std::fabs(distanceToAxis - radius) > 1e-6) {
                if (distanceToAxis > 1e-9) {
                    radial.normalize();
                    Eigen::Vector3d PCylinder = PProj + radius * radial;
                    local_projectedCloud.push_back(pcl::PointXYZ(PCylinder.x(), PCylinder.y(), PCylinder.z()));
                } else {
                    Eigen::Vector3d radialDir = u;
                    Eigen::Vector3d PCylinder = PProj + radius * radialDir;
                    local_projectedCloud.push_back(pcl::PointXYZ(PCylinder.x(), PCylinder.y(), PCylinder.z()));
                }
            } else {
                local_projectedCloud.push_back(p);
            }
        }

#pragma omp critical
        { projectedCloud->insert(projectedCloud->end(), local_projectedCloud.begin(), local_projectedCloud.end()); }
    }

    // 并行计算每个点的角度桶
    std::vector<std::pair<int, double>> angle_height_pairs;
    angle_height_pairs.reserve(projectedCloud->size());

#pragma omp parallel
    {
        std::vector<std::pair<int, double>> local_pairs;
#pragma omp for nowait
        for (int i = 0; i < static_cast<int>(projectedCloud->size()); ++i) {
            const auto& p = projectedCloud->points[i];
            Eigen::Vector3d P(p.x, p.y, p.z);
            Eigen::Vector3d d = P - P0;

            double h = d.dot(axis);
            Eigen::Vector3d radial = d - h * axis;

            double x = radial.dot(u);
            double y = radial.dot(v);
            double theta = std::atan2(y, x);

            int idx = static_cast<int>(std::floor(theta / dTheta));
            local_pairs.emplace_back(idx, h);
        }

#pragma omp critical
        { angle_height_pairs.insert(angle_height_pairs.end(), local_pairs.begin(), local_pairs.end()); }
    }

    for (const auto& pair : angle_height_pairs) {
        bucket[pair.first].push_back(pair.second);
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(projectedCloud);

    const double supportRadius = 3.0;
    const int minSupportPoints = 0;

    std::vector<int> validIdx;
    std::map<int, pcl::PointXYZ> boundaryPt;
    std::map<int, double> boundaryH;

    // 准备用于并行处理的数据结构
    std::vector<std::pair<int, std::vector<double>>> bucket_list(bucket.begin(), bucket.end());

#pragma omp parallel
    {
#pragma omp for nowait schedule(dynamic)
        for (int i = 0; i < static_cast<int>(bucket_list.size()); ++i) {
            const auto& kv = bucket_list[i];
            int idx = kv.first;
            const auto& hlist = kv.second;
            if (hlist.size() < 30) continue;

            double hMin = *std::min_element(hlist.begin(), hlist.end());
            double hMax = *std::max_element(hlist.begin(), hlist.end());
            double theta = (idx + 0.5) * dTheta;

            Eigen::Vector3d P = P0 + hMin * axis + radius * (std::cos(theta) * u + std::sin(theta) * v);

            pcl::PointXYZ query(P.x(), P.y(), P.z());

            std::vector<int> indices;
            std::vector<float> sqr_dist;

            int found = kdtree.radiusSearch(query, supportRadius, indices, sqr_dist);

            if (found >= minSupportPoints) {
#pragma omp critical
                {
                    validIdx.push_back(idx);
                    boundaryPt[idx] = query;
                    boundaryH[idx] = hMin;
                }
            }
        }
    }

    if (validIdx.size() < 2) {
        PLOGE << "有效边缘点不足";
        return;
    }

    std::sort(validIdx.begin(), validIdx.end());

    int totalBins = static_cast<int>(std::round(2.0 * M_PI / dTheta));

    int maxGap = -1;
    int breakPos = 0;

    for (int i = 0; i < static_cast<int>(validIdx.size()); ++i) {
        int curr = validIdx[i];
        int next = validIdx[(i + 1) % validIdx.size()];

        int gap;
        if (i + 1 < static_cast<int>(validIdx.size()))
            gap = next - curr;
        else
            gap = (next + totalBins) - curr;

        if (gap > maxGap) {
            maxGap = gap;
            breakPos = (i + 1) % validIdx.size();
        }
    }

    int N = static_cast<int>(validIdx.size());
    std::vector<double> h(N);

    for (int i = 0; i < N; ++i) {
        int idx = validIdx[(breakPos + i) % N];
        h[i] = boundaryH[idx];
    }

    std::vector<double> k(N, 0.0);
    for (int i = 1; i < N - 1; ++i) k[i] = h[i + 1] - 2.0 * h[i] + h[i - 1];

    std::vector<double> absK(N - 2);
    for (int i = 1; i < N - 1; ++i) absK[i - 1] = std::fabs(k[i]);

    std::nth_element(absK.begin(), absK.begin() + absK.size() / 2, absK.end());
    double medK = absK[absK.size() / 2];

    std::vector<double> dev;
    for (double val : absK) {
        dev.push_back(std::fabs(val - medK));
    }

    std::nth_element(dev.begin(), dev.begin() + dev.size() / 2, dev.end());
    double mad = dev[dev.size() / 2] + 1e-6;

    for (int i = 1; i < N - 1; ++i) {
        if (std::fabs(k[i] - medK) > 3.0 * mad) {
            h[i] = 0.5 * (h[i - 1] + h[i + 1]);
        }
    }

    for (int i = 0; i < N; ++i) {
        int idx = validIdx[(breakPos + i) % N];
        double theta = (idx + 0.5) * dTheta;

        Eigen::Vector3d P = P0 + h[i] * axis + radius * (std::cos(theta) * u + std::sin(theta) * v);

        cloudResult->push_back(pcl::PointXYZ(P.x(), P.y(), P.z()));
    }

    if (cloudResult->empty()) {
        PLOGE << "提取圆柱面边缘点失败，边缘点为空";
    } else {
        PLOGD << "成功提取圆柱面边缘点";
    }
}

/**
 * 函数：projectNURBSCurveRadially
 * 作用：将 NURBS 曲线沿圆柱径向方向整体偏移
 * 输入：
 *   - curve: 原始 NURBS 曲线
 *   - cylinderCoefficients: 圆柱参数
 *   - distance: 径向偏移量
 * 输出：
 *   - result: 偏移后的 NURBS 曲线
 * 原理：
 *   对每个控制点沿径向单位方向移动
 */
NURBSCurve AccuratePositioning::projectNURBSCurveRadially(const NURBSCurve& curve,
                                                          const pcl::ModelCoefficients& cylinderCoefficients, double distance) {
    Eigen::Vector3d axisPoint(cylinderCoefficients.values[0], cylinderCoefficients.values[1], cylinderCoefficients.values[2]);

    Eigen::Vector3d axisDir(cylinderCoefficients.values[3], cylinderCoefficients.values[4], cylinderCoefficients.values[5]);
    axisDir.normalize();

    double radius = cylinderCoefficients.values[6];
    if (distance < 0.0 || distance >= radius) return curve;

    NURBSCurve result = curve;
    result.controlPoints.clear();
    result.controlPoints.reserve(curve.controlPoints.size());

    for (const auto& cp : curve.controlPoints) {
        Eigen::Vector3d P(cp.x, cp.y, cp.z);

        Eigen::Vector3d v = P - axisPoint;
        double t = v.dot(axisDir);
        Eigen::Vector3d P_axis = axisPoint + t * axisDir;

        Eigen::Vector3d radial = P - P_axis;
        double r = radial.norm();

        if (r < 1e-9) {
            result.controlPoints.push_back(cp);
            continue;
        }

        Eigen::Vector3d radialUnit = radial / r;

        Eigen::Vector3d P_new = P - distance * radialUnit;

        result.controlPoints.push_back({P_new.x(), P_new.y(), P_new.z()});
    }

    return result;
}

/**
 * 函数：transformCameraToBaseFrame
 * 作用：将相机坐标系下的点云变换到机器人基座坐标系
 * 输入：
 *   - cameraCloud: 相机坐标系点云
 *   - transformationMatrix: 4×4 齐次变换矩阵
 * 输出：
 *   - baseCloud: 基座坐标系点云
 * 原理：齐次坐标变换
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr AccuratePositioning::transformCameraToBaseFrame(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cameraCloud, Eigen::Matrix4d transformationMatrix) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr baseCloud(new pcl::PointCloud<pcl::PointXYZ>);
    baseCloud->resize(cameraCloud->size());

    double filterZMin = -0.1;
    double filterZMax = 0.1;

    size_t validCount = 0;

    for (size_t i = 0; i < cameraCloud->size(); ++i) {
        const pcl::PointXYZ& pt = cameraCloud->points[i];

        Eigen::Vector4d homogPoint(pt.x, pt.y, pt.z, 1.0);
        Eigen::Vector4d transformedHomog = transformationMatrix * homogPoint;

        pcl::PointXYZ transformedPt;
        transformedPt.x = transformedHomog(0) / transformedHomog(3);
        transformedPt.y = transformedHomog(1) / transformedHomog(3);
        transformedPt.z = transformedHomog(2) / transformedHomog(3);

        if (transformedPt.z < filterZMin || transformedPt.z > filterZMax) {
            baseCloud->points[validCount] = transformedPt;
            validCount++;
        }
    }

    baseCloud->resize(validCount);

    return baseCloud;
}

/**
 * 函数：judgeMainorBranch
 * 作用：判断两个圆柱中哪个是主管、哪个是支管
 * 输入 / 输出：
 *   - cylinderCoefficients: 两个圆柱参数
 *   - cylinderInliers: 对应圆柱内点
 * 原理：通过圆柱轴线与世界Y轴夹角判断：更接近轴的视为主管
 */
void AccuratePositioning::judgeMainorBranch(std::vector<pcl::ModelCoefficients>& cylinderCoefficients,
                                            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cylinderInliers) {
    if (cylinderCoefficients.size() != 2 || cylinderInliers.size() != 2) {
        PLOGE << "圆柱面参数不正确";
        return;
    }

    Eigen::Vector3f worldY(0.f, 1.f, 0.f);

    Eigen::Vector3f axis0(cylinderCoefficients[0].values[3], cylinderCoefficients[0].values[4],
                          cylinderCoefficients[0].values[5]);
    axis0.normalize();

    Eigen::Vector3f axis1(cylinderCoefficients[1].values[3], cylinderCoefficients[1].values[4],
                          cylinderCoefficients[1].values[5]);
    axis1.normalize();

    float cosAngle0 = std::abs(axis0.dot(worldY));
    float cosAngle1 = std::abs(axis1.dot(worldY));

    if (cosAngle0 < cosAngle1) {
        std::swap(cylinderCoefficients[0], cylinderCoefficients[1]);
        std::swap(cylinderInliers[0], cylinderInliers[1]);
    }
}

/**
 * 函数：computeToolPostureInPlatePlate
 * 作用：根据板–板焊缝几何关系计算焊枪姿态
 * 输入 / 输出：
 *   - postures:输入焊缝位置与相邻平面信息，输出焊枪位姿（四元数 + 坐标轴）
 * 原理：
 *   1）X 轴沿焊缝切向
 *   2）Z 轴为相邻平面法向量的角平分方向
 *   3）Y 轴由右手系确定
 */
void AccuratePositioning::computeToolPostureInPlatePlate(std::vector<Posture>& postures) {
    const double zDiffThreshold = 30.0;

    for (auto& posture : postures) {
        if (posture.position.size() < 2 || posture.adjacentSurfaces.empty() || posture.tangentVectors.empty()) {
            PLOGE << "姿态数据不完整，跳过计算";
            continue;
        }

        bool needReverse = false;
        double z0 = posture.position[0].z;
        double z1 = posture.position[1].z;

        if (z0 > z1 && std::abs(z0 - z1) > zDiffThreshold) {
            needReverse = true;
        }

        if (needReverse) {
            std::reverse(posture.position.begin(), posture.position.end());
            std::reverse(posture.adjacentSurfaces.begin(), posture.adjacentSurfaces.end());
            std::reverse(posture.tangentVectors.begin(), posture.tangentVectors.end());

            for (auto& t : posture.tangentVectors) {
                t.x = -t.x;
                t.y = -t.y;
                t.z = -t.z;
            }
        }

        posture.toolPose.clear();
        posture.x.clear();
        posture.y.clear();
        posture.z.clear();

        for (size_t i = 0; i < posture.position.size(); ++i) {
            Eigen::Vector3d XAxis(posture.tangentVectors[i].x, posture.tangentVectors[i].y, posture.tangentVectors[i].z);
            XAxis.normalize();

            std::vector<Eigen::Vector3d> surfaceNormals;
            surfaceNormals.reserve(posture.adjacentSurfaces[i].size());

            Eigen::Vector3d cameraCenter(0.0, 0.0, 0.0);

            for (const auto& surface : posture.adjacentSurfaces[i]) {
                Eigen::Vector3d normal(surface.values[0], surface.values[1], surface.values[2]);
                normal.normalize();

                Eigen::Vector3d pointOnSurface(surface.values[3], surface.values[4], surface.values[5]);

                Eigen::Vector3d viewDir = cameraCenter - pointOnSurface;

                if (normal.dot(viewDir) > 0) {
                    normal = -normal;
                }

                surfaceNormals.push_back(normal);
            }

            if (surfaceNormals.size() < 2) {
                PLOGE << "相邻面数量不足，无法计算二分面";
                continue;
            }

            Eigen::Vector3d bisectorNormal = (surfaceNormals[0] + surfaceNormals[1]).normalized();

            Eigen::Vector3d ZAxis = bisectorNormal.cross(XAxis).normalized();
            if (ZAxis.dot(bisectorNormal) < 0) {
                ZAxis = -ZAxis;
            }

            Eigen::Vector3d YAxis = ZAxis.cross(XAxis).normalized();

            posture.x.emplace_back(XAxis.x(), XAxis.y(), XAxis.z());
            posture.y.emplace_back(YAxis.x(), YAxis.y(), YAxis.z());
            posture.z.emplace_back(ZAxis.x(), ZAxis.y(), ZAxis.z());

            Eigen::Matrix3d R;
            R.col(0) = XAxis;
            R.col(1) = YAxis;
            R.col(2) = ZAxis;

            Eigen::Quaternionf quat(R.cast<float>());
            quat.normalize();

            Eigen::AngleAxisf rotX180(M_PI, Eigen::Vector3f::UnitX());
            quat = quat * Eigen::Quaternionf(rotX180);

            if (needReverse) {
                Eigen::AngleAxisf rotY45(M_PI / 4.0f, Eigen::Vector3f::UnitY());
                quat = quat * Eigen::Quaternionf(rotY45);
            }

            quat.normalize();
            posture.toolPose.push_back(quat);
        }

        if (posture.toolPose.size() != posture.position.size()) {
            PLOGE << "工具姿态数量与位置数量不匹配";
        }
    }
}

/**
 * 函数：NURBSfitting
 * 作用：对三维点云进行 NURBS 曲线拟合，求解控制点
 * 输入：
 *  - cloudResult : 拟合用的点云（有序）
 *  - p           : NURBS 曲线阶数
 *  - n           : 控制点个数
 *  - weights     : 控制点权重
 * 输出：
 *  - NURBSCurve  : 包含控制点、节点向量、权重的 NURBS 曲线
 * 原理：
 *  1）向心参数法生成参数 u
 *  2）构造开区间节点向量
 *  3）计算有理 B 样条基函数
 *  4）建立线性方程 A·P = Q，最小二乘求控制点
 */
NURBSCurve AccuratePositioning::NURBSfitting(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudResult, int p, int n,
                                             std::vector<float> weights) {
    NURBSCurve nurbsCurve;
    if (!cloudResult || cloudResult->points.empty()) {
        PLOGE << "NURBS曲线拟合点为空";
        return nurbsCurve;
    }

    /* ================== 点云转为拟合点 ================== */
    std::vector<Point3D> fitPoints;
    Point3D temp;

    for (int i = 0; i < cloudResult->points.size(); i++) {
        temp.x = cloudResult->points[i].x;
        temp.y = cloudResult->points[i].y;
        temp.z = cloudResult->points[i].z;
        fitPoints.push_back(temp);
    }

    // 向心参数法
    std::vector<double> u(fitPoints.size());
    u[0] = 0.0;
    double total_sqrt_dist = 0.0;

    for (size_t i = 1; i < fitPoints.size(); ++i) {
        double dist = sqrt(pow(fitPoints[i].x - fitPoints[i - 1].x, 2) + pow(fitPoints[i].y - fitPoints[i - 1].y, 2) +
                           pow(fitPoints[i].z - fitPoints[i - 1].z, 2));
        total_sqrt_dist += std::sqrt(dist);
    }

    double current = 0.0;
    for (size_t i = 1; i < fitPoints.size(); ++i) {
        double dist = sqrt(pow(fitPoints[i].x - fitPoints[i - 1].x, 2) + pow(fitPoints[i].y - fitPoints[i - 1].y, 2) +
                           pow(fitPoints[i].z - fitPoints[i - 1].z, 2));
        current += std::sqrt(dist);
        u[i] = current / total_sqrt_dist;
    }

    /* ================== 构造开区间节点向量 ================== */
    std::vector<double> knots(n + p + 2, 0.0);
    int m = u.size();

    int internalCount = n - p;
    if (internalCount <= 0) {
        std::fill(knots.begin() + p + 1, knots.end() - p - 1, 0.5);
    } else {
        std::vector<double> internalKnots(internalCount);
        double d = static_cast<double>(m) / (internalCount + 1);
        for (int i = 1; i <= internalCount; ++i) {
            double pos = i * d;
            int idx = static_cast<int>(pos);
            double alpha = pos - idx;
            if (idx >= m) {
                idx = m - 1;
                alpha = 1.0;
            }
            internalKnots[i - 1] = (1 - alpha) * u[idx - 1] + alpha * u[idx];
        }
        std::fill(knots.begin(), knots.begin() + p + 1, 0.0);
        std::copy(internalKnots.begin(), internalKnots.end(), knots.begin() + p + 1);
        std::fill(knots.end() - p - 1, knots.end(), 1.0);
    }

    m = fitPoints.size();
    /* ================== 构造最小二乘矩阵 ================== */
    Eigen::MatrixXd A(m, n + 1);
    Eigen::VectorXd denom(m);
    for (int k = 0; k < m; ++k) {
        /* 查找节点区间（B-spline span） */
        double uk = u[k];
        int span;
        int n = knots.size() - p - 1;
        if (uk >= knots[n]) {
            span = n - 1;
        } else if (uk <= knots[p]) {
            span = p;
        } else {
            int low = p;
            int high = n;
            int mid = (low + high) / 2;
            while (uk < knots[mid] || uk >= knots[mid + 1]) {
                if (uk < knots[mid])
                    high = mid;
                else
                    low = mid;
                mid = (low + high) / 2;
            }
            span = mid;
        }

        /* Cox–de Boor 递推计算基函数 */
        std::vector<double> N;
        N.resize(p + 1);
        std::vector<double> left(p + 1), right(p + 1);
        N[0] = 1.0;
        for (int j = 1; j <= p; ++j) {
            left[j] = uk - knots[span + 1 - j];
            right[j] = knots[span + j] - uk;
            double saved = 0.0;
            for (int r = 0; r < j; ++r) {
                double temp = N[r] / (right[r + 1] + left[j - r]);
                N[r] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            N[j] = saved;
        }

        /* 有理 B 样条权重归一化 */
        double sum = 0.0;
        for (int i = 0; i <= p; ++i) {
            int ctrlIdx = span - p + i;
            sum += N[i] * weights[ctrlIdx];
        }
        denom[k] = sum;
        A.row(k).setZero();
        for (int i = 0; i <= p; ++i) {
            int ctrlIdx = span - p + i;
            A(k, ctrlIdx) = (N[i] * weights[ctrlIdx]) / sum;
        }
    }

    /* ================== 解线性方程求控制点 ================== */
    Eigen::VectorXd bx(m), by(m), bz(m);
    for (int k = 0; k < m; ++k) {
        bx[k] = fitPoints[k].x;
        by[k] = fitPoints[k].y;
        bz[k] = fitPoints[k].z;
    }
    Eigen::VectorXd px = A.householderQr().solve(bx);
    Eigen::VectorXd py = A.householderQr().solve(by);
    Eigen::VectorXd pz = A.householderQr().solve(bz);
    std::vector<Point3D> controlPoints(n + 1);
    for (int i = 0; i <= n; ++i) {
        controlPoints[i] = Point3D(px[i], py[i], pz[i]);
    }

    nurbsCurve.controlPoints = controlPoints;
    nurbsCurve.degree = p;
    nurbsCurve.knots = knots;
    nurbsCurve.weights = weights;
    return nurbsCurve;
}

/**
 * 函数：distanceToSegment
 * 作用：计算点到线段的最短距离
 * 输入：
 *  - p : 查询点
 *  - a : 线段起点
 *  - b : 线段终点
 * 输出：
 *  - double : 点到线段的欧氏距离
 * 原理：
 *  向量投影法，限制投影参数
 */
double AccuratePositioning::distanceToSegment(const Point3D& p, const Point3D& a, const Point3D& b) {
    const Point3D ab(b.x - a.x, b.y - a.y, b.z - a.z);
    const Point3D ap(p.x - a.x, p.y - a.y, p.z - a.z);

    const double dotABAP = ab.x * ap.x + ab.y * ap.y + ab.z * ap.z;
    const double lengthSqAB = ab.x * ab.x + ab.y * ab.y + ab.z * ab.z;

    if (lengthSqAB < 1e-10) {
        return sqrt(ap.x * ap.x + ap.y * ap.y + ap.z * ap.z);
    }

    const double t = std::max(0.0, std::min(1.0, dotABAP / lengthSqAB));
    const Point3D projection(a.x + t * ab.x, a.y + t * ab.y, a.z + t * ab.z);

    const double dx = p.x - projection.x;
    const double dy = p.y - projection.y;
    const double dz = p.z - projection.z;

    return sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * 函数：calculateNURBSPoint
 * 作用：根据参数 t 计算 NURBS 曲线上的三维点
 * 输入：
 *  - t          : 曲线参数（0~1）
 *  - nurbsCurve : NURBS 曲线数据
 * 输出：
 *  - Point3D    : 曲线上的点
 * 原理：
 *  Cox–de Boor 算法 + 权重归一化
 */
Point3D AccuratePositioning::calculateNURBSPoint(double t, NURBSCurve nurbsCurve) {
    int p = nurbsCurve.degree;
    std::vector<double> knots = nurbsCurve.knots;
    std::vector<float> weights = nurbsCurve.weights;
    std::vector<Point3D> controlPoints = nurbsCurve.controlPoints;

    if (controlPoints.empty()) {
        PLOGE << "控制点为空";
    } else {
        int span;
        int n = knots.size() - p - 1;
        if (t >= knots[n]) {
            span = n - 1;
        } else if (t <= knots[p]) {
            span = p;
        } else {
            int low = p;
            int high = n;
            int mid = (low + high) / 2;
            while (t < knots[mid] || t >= knots[mid + 1]) {
                if (t < knots[mid])
                    high = mid;
                else
                    low = mid;
                mid = (low + high) / 2;
            }
            span = mid;
        }

        std::vector<double> basis;
        basis.resize(p + 1);
        std::vector<double> left(p + 1), right(p + 1);
        basis[0] = 1.0;
        for (int j = 1; j <= p; ++j) {
            left[j] = t - knots[span + 1 - j];
            right[j] = knots[span + j] - t;
            double saved = 0.0;
            for (int r = 0; r < j; ++r) {
                double temp = basis[r] / (right[r + 1] + left[j - r]);
                basis[r] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            basis[j] = saved;
        }

        Point3D point;
        double denominator = 0.0;

        for (int i = 0; i <= p; ++i) {
            double temp = basis[i] * weights[span - p + i];
            point.x += temp * controlPoints[span - p + i].x;
            point.y += temp * controlPoints[span - p + i].y;
            point.z += temp * controlPoints[span - p + i].z;
            denominator += temp;
        }

        point.x /= denominator;
        point.y /= denominator;
        point.z /= denominator;
        return point;
    }
}

/**
 * 函数：discretizeNURBSCurve
 * 作用：按最大弓高误差自适应离散 NURBS 曲线
 * 输入：
 *  - hMax        : 最大允许弓高误差
 *  - tStart/end : 参数区间
 *  - initialStep: 初始步长
 *  - nurbsCurve : 曲线
 * 输出：
 *  - vector<Point3D> : 离散点集
 * 原理：
 *  中点弓高误差控制步长自适应细分
 */
std::vector<Point3D> AccuratePositioning::discretizeNURBSCurve(double hMax, double tStart, double tEnd, double initialStep,
                                                               NURBSCurve nurbsCurve) {
    std::vector<Point3D> points;
    double t0 = tStart;
    points.push_back(calculateNURBSPoint(t0, nurbsCurve));
    double currentStep = initialStep;

    while (t0 < tEnd) {
        double t1 = t0 + currentStep;
        if (t1 > tEnd) t1 = tEnd;

        Point3D p0 = calculateNURBSPoint(t0, nurbsCurve);
        Point3D p1 = calculateNURBSPoint(t1, nurbsCurve);

        if (t1 >= tEnd) {
            points.push_back(p1);
            break;
        }

        double tm = (t0 + t1) * 0.5;
        Point3D pm = calculateNURBSPoint(tm, nurbsCurve);
        double h = distanceToSegment(pm, p0, p1);

        if (h > hMax) {
            currentStep *= 0.5;
        } else {
            points.push_back(p1);
            t0 = t1;

            currentStep *= (h < 0.5 * hMax) ? 2.0 : 1.0;

            if (t0 + currentStep > tEnd) {
                currentStep = tEnd - t0;
            }
        }
    }

    return points;
}

/**
 * 函数：computeCylinderPlaneIntersection
 * 作用：
 *  计算圆柱面与平面的空间交线
 * 输入：
 *  - planeCoefficients    : 平面模型系数
 *  - cylinderCoefficients : 圆柱模型系数
 *  - planeInliers         : 平面内点点云
 * 输出：
 *  - result         : 圆柱-平面交线点云（稀疏采样）
 *  - projectedCloud : 经过投影与 Opening 过滤后的平面点云
 * 原理：
 *  1）建立圆柱局部坐标系
 *  2）将平面点云投影到平面二维坐标
 *  3）二维栅格化 + 形态学 Opening 去除噪声
 *  4）沿圆柱轴向采样，解析计算圆柱-平面交点
 *  5）选择真实存在的交线分支
 */
void AccuratePositioning::AccuratePositioning::computeCylinderPlaneIntersection(
    const pcl::ModelCoefficients& planeCoefficients, const pcl::ModelCoefficients& cylinderCoefficients,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& planeInliers, pcl::PointCloud<pcl::PointXYZ>::Ptr& result,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& projectedCloud) {
    planeInliers->height = 1;
    planeInliers->width = static_cast<uint32_t>(planeInliers->size());
    // pcl::io::savePCDFile("./data/seamDetWithPointCloud/tubeSidePlateFilletSeamsDet/planeInliers.pcd", *planeInliers);
    /* ================== 平面参数 ================== */
    Eigen::Vector3d planeN(planeCoefficients.values[0], planeCoefficients.values[1], planeCoefficients.values[2]);
    double planeD = planeCoefficients.values[3];
    planeN.normalize();
    std::cout << "Plane parameters:" << std::endl;
    std::cout << "Normal: [" << planeN.x() << ", " << planeN.y() << ", " << planeN.z() << "]" << std::endl;
    std::cout << "d: " << planeD << std::endl;

    /* ================== 圆柱参数 ================== */
    Eigen::Vector3d p0(cylinderCoefficients.values[0], cylinderCoefficients.values[1], cylinderCoefficients.values[2]);
    Eigen::Vector3d axis(cylinderCoefficients.values[3], cylinderCoefficients.values[4], cylinderCoefficients.values[5]);
    double R = cylinderCoefficients.values[6];
    axis.normalize();
    std::cout << "Cylinder parameters:" << std::endl;
    std::cout << "Axis origin: [" << p0.x() << ", " << p0.y() << ", " << p0.z() << "]" << std::endl;
    std::cout << "Axis direction: [" << axis.x() << ", " << axis.y() << ", " << axis.z() << "]" << std::endl;
    std::cout << "Radius: " << R << std::endl;

    /* ================== 构建圆柱局部坐标系 ================== */
    Eigen::Vector3d ez = axis;
    Eigen::Vector3d tmp =
        (std::abs(ez.dot(Eigen::Vector3d::UnitX())) < 0.9) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
    Eigen::Vector3d ex = (tmp - tmp.dot(ez) * ez).normalized();
    Eigen::Vector3d ey = ez.cross(ex).normalized();

    /* ================== 平面在圆柱坐标系下的表达 ================== */
    double nx = planeN.dot(ex);
    double ny = planeN.dot(ey);
    double nz = planeN.dot(ez);
    double d0 = planeN.dot(p0) + planeD;

    /* ================== 构建平面二维坐标系 ================== */
    Eigen::Vector3d n = planeN;
    Eigen::Vector3d t = (std::abs(n.dot(Eigen::Vector3d::UnitX())) < 0.9) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
    Eigen::Vector3d u = (t - t.dot(n) * n).normalized();
    Eigen::Vector3d v = n.cross(u).normalized();

    /* ================== 平面点云投影到 2D ================== */
    struct P2 {
        double x, y;
    };
    std::vector<P2> points2d;
    std::vector<Eigen::Vector3d> projected3d;
    points2d.reserve(planeInliers->size());
    projected3d.reserve(planeInliers->size());

    for (const auto& pt : planeInliers->points) {
        Eigen::Vector3d p(pt.x, pt.y, pt.z);
        double dist = n.dot(p) + planeD;
        Eigen::Vector3d pp = p - dist * n;
        points2d.push_back({pp.dot(u), pp.dot(v)});
        projected3d.push_back(pp);
    }

    /* ================== 平面点云二维栅格化 ================== */
    double res = 2.0;
    int OpenRadius = 1;

    double minx = 1e9, miny = 1e9, maxx = -1e9, maxy = -1e9;
    for (auto& p : points2d) {
        minx = std::min(minx, p.x);
        miny = std::min(miny, p.y);
        maxx = std::max(maxx, p.x);
        maxy = std::max(maxy, p.y);
    }

    int W = static_cast<int>((maxx - minx) / res) + 3;
    int H = static_cast<int>((maxy - miny) / res) + 3;

    std::vector<uint8_t> binary(W * H, 0);
    std::vector<Eigen::Vector2i> gridIndex(points2d.size());

    for (size_t i = 0; i < points2d.size(); ++i) {
        int ix = static_cast<int>((points2d[i].x - minx) / res);
        int iy = static_cast<int>((points2d[i].y - miny) / res);
        gridIndex[i] = {ix, iy};
        if (ix >= 0 && iy >= 0 && ix < W && iy < H) binary[iy * W + ix] = 1;
    }

    /* ================== 形态学 Opening（去孤立噪点） ================== */
    auto erode = [&](const std::vector<uint8_t>& src) {
        std::vector<uint8_t> dst(src.size(), 0);
        for (int y = OpenRadius; y < H - OpenRadius; ++y)
            for (int x = OpenRadius; x < W - OpenRadius; ++x) {
                bool ok = true;
                for (int dy = -OpenRadius; dy <= OpenRadius; ++dy)
                    for (int dx = -OpenRadius; dx <= OpenRadius; ++dx) ok &= src[(y + dy) * W + (x + dx)];
                dst[y * W + x] = ok ? 1 : 0;
            }
        return dst;
    };

    auto dilate = [&](const std::vector<uint8_t>& src) {
        std::vector<uint8_t> dst(src.size(), 0);
        for (int y = OpenRadius; y < H - OpenRadius; ++y)
            for (int x = OpenRadius; x < W - OpenRadius; ++x) {
                bool ok = false;
                for (int dy = -OpenRadius; dy <= OpenRadius; ++dy)
                    for (int dx = -OpenRadius; dx <= OpenRadius; ++dx) ok |= src[(y + dy) * W + (x + dx)];
                dst[y * W + x] = ok ? 1 : 0;
            }
        return dst;
    };

    std::vector<uint8_t> opened = dilate(erode(binary));

    /* ================== 恢复过滤后的平面点云 ================== */
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredPlane(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < projected3d.size(); ++i) {
        int ix = gridIndex[i].x();
        int iy = gridIndex[i].y();
        if (ix >= 0 && iy >= 0 && ix < W && iy < H && opened[iy * W + ix]) {
            filteredPlane->push_back(pcl::PointXYZ(projected3d[i].x(), projected3d[i].y(), projected3d[i].z()));
        }
    }

    if (filteredPlane->empty()) return;
    filteredPlane->height = 1;
    filteredPlane->width = static_cast<uint32_t>(filteredPlane->size());
    // pcl::io::savePCDFile("./data/seamDetWithPointCloud/tubeSidePlateFilletSeamsDet/filteredPlane.pcd", *filteredPlane);

    /* ================== KD-Tree：用于选择真实交线分支 ================== */
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(filteredPlane);

    /* ================== 估计圆柱轴向范围 ================== */
    double zmin = std::numeric_limits<double>::max();
    double zmax = -std::numeric_limits<double>::max();
    double threshold = R + 3.0;

    for (const auto& pt : filteredPlane->points) {
        Eigen::Vector3d pw(pt.x, pt.y, pt.z);
        Eigen::Vector3d pl = pw - p0;
        double distAxis = (pl - pl.dot(ez) * ez).norm();
        if (distAxis <= threshold) {
            double z = pl.dot(ez);
            zmin = std::min(zmin, z);
            zmax = std::max(zmax, z);
        }
    }

    if (zmax <= zmin) return;

    /* ================== 解析计算圆柱-平面交线 ================== */
    const int samples = 2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr intersection(new pcl::PointCloud<pcl::PointXYZ>);
    intersection->reserve(samples);

    std::vector<int> nnIdx(1);
    std::vector<float> nnDist(1);

    for (int i = 0; i < samples; ++i) {
        double z = zmin + (zmax - zmin) * i / (samples - 1);
        double rhs = -(nz * z + d0);
        double a = nx, b = ny, c = rhs;
        double denom = a * a + b * b;
        if (denom < 1e-10) continue;

        double d2 = c * c / denom;
        if (d2 > R * R) continue;

        double h = std::sqrt(R * R - d2);
        double x0 = a * c / denom;
        double y0 = b * c / denom;
        double dx = -b * h / std::sqrt(denom);
        double dy = a * h / std::sqrt(denom);

        Eigen::Vector3d p1 = p0 + (x0 + dx) * ex + (y0 + dy) * ey + z * ez;
        Eigen::Vector3d p2 = p0 + (x0 - dx) * ex + (y0 - dy) * ey + z * ez;

        float d1 = std::numeric_limits<float>::max();
        float d2n = std::numeric_limits<float>::max();

        if (kdtree.nearestKSearch(pcl::PointXYZ(p1.x(), p1.y(), p1.z()), 1, nnIdx, nnDist) > 0) d1 = nnDist[0];

        if (kdtree.nearestKSearch(pcl::PointXYZ(p2.x(), p2.y(), p2.z()), 1, nnIdx, nnDist) > 0) d2n = nnDist[0];

        Eigen::Vector3d best = (d1 < d2n) ? p1 : p2;
        intersection->push_back(pcl::PointXYZ(best.x(), best.y(), best.z()));
    }
    intersection->height = 1;
    intersection->width = static_cast<uint32_t>(intersection->size());

    pcl::io::savePCDFile("./data/seamDetWithPointCloud/tubeSidePlateFilletSeamsDet/intersection.pcd", *intersection);

    *result = *intersection;
    *projectedCloud = *filteredPlane;
}

/**
 * 函数：whenAccuratePosition
 * 作用：根据焊缝类型执行不同的精定位流程
 * 输入：
 *   - weldType: 焊缝类型（板板 / 管板 / 管管）
 *   - cloud: 输入点云
 * 输出：- PositioningResult（通过信号发出）
 */
void AccuratePositioning::whenAccuratePosition(WeldType weldType, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (!cloud || cloud->points.empty()) {
        PLOGE << "点云为空，无法进行精定位";
        PositioningResult result;
        result.success = false;
        emit positioningComplete(result);
        return;
    }

    PLOGD << "开始精定位... ...";

    PositioningResult result;
    result.success = true;
    result.weldType = weldType;

    switch (weldType) {
        case WeldType::PlateToPlate: {
            PLOGD << "PlateToPlate类型焊缝检测中";
            pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZ>);
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudResult;
            pointcloudUniformDownsampling(cloud, 1.0f, downsampledCloud);

            std::vector<pcl::ModelCoefficients> planeEquations;
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> planeClouds;
            planeFitting(downsampledCloud, downsampledCloud, planeEquations, planeClouds, 3);

            std::vector<Posture> postures;
            std::vector<pcl::ModelCoefficients> lineEquations;
            computePlaneIntersectionLines(planeEquations, planeClouds, postures, lineEquations, cloudResult);
            computeToolPostureInPlatePlate(postures);

            std::vector<std::vector<pcl::PointXYZ>> allExtremePoints;
            for (const auto& posture : postures) {
                if (!posture.position.empty()) {
                    std::vector<pcl::PointXYZ> points;
                    for (const auto& pos : posture.position) {
                        pcl::PointXYZ point;
                        point.x = pos.x;
                        point.y = pos.y;
                        point.z = pos.z;
                        points.push_back(point);
                    }
                    allExtremePoints.push_back(points);
                }
            }
            result.extremePoints = allExtremePoints;
            std::vector<std::vector<Point3D>> position;
            std::vector<std::vector<Eigen::Quaternionf>> toolPose;
            std::vector<std::vector<Point3D>> x;
            std::vector<std::vector<Point3D>> y;
            std::vector<std::vector<Point3D>> z;

            for (const auto& posture : postures) {
                position.push_back(posture.position);
                toolPose.push_back(posture.toolPose);
                x.push_back(posture.x);
                y.push_back(posture.y);
                z.push_back(posture.z);
            }
            result.position = position;
            result.x = x;
            result.y = y;
            result.z = z;
            result.toolPose = toolPose;
            result.cloudResult = cloudResult[2];

        } break;

        case WeldType::TubeToPlate1: {
            PLOGD << "TubeToPlate1类型焊缝检测中";
            pcl::ModelCoefficients planeCoefficients;
            pcl::PointCloud<pcl::PointXYZ>::Ptr planeInliers(new pcl::PointCloud<pcl::PointXYZ>);
            std::vector<pcl::ModelCoefficients> cylinderCoefficients;
            fitPlaneAnd2Cylinders(cloud, planeCoefficients, planeInliers, cylinderCoefficients);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudResult1(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudResult2(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudResult(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr projected1(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr projected2(new pcl::PointCloud<pcl::PointXYZ>);
            computeCylinderPlaneIntersection(planeCoefficients, cylinderCoefficients[0], planeInliers, cloudResult1, projected1);
            computeCylinderPlaneIntersection(planeCoefficients, cylinderCoefficients[1], planeInliers, cloudResult2, projected2);
            *cloudResult = *cloudResult1 + *cloudResult2;

            std::vector<std::vector<pcl::PointXYZ>> extremePoints;
            if (!cloudResult1->points.empty()) {
                pcl::PointXYZ startPoint1 = cloudResult1->points.front();
                pcl::PointXYZ endPoint1 = cloudResult1->points.back();

                std::vector<pcl::PointXYZ> endpoints1;
                endpoints1.push_back(startPoint1);
                endpoints1.push_back(endPoint1);
                extremePoints.push_back(endpoints1);
            }

            if (!cloudResult2->points.empty()) {
                pcl::PointXYZ startPoint2 = cloudResult2->points.front();
                pcl::PointXYZ endPoint2 = cloudResult2->points.back();

                std::vector<pcl::PointXYZ> endpoints2;
                endpoints2.push_back(startPoint2);
                endpoints2.push_back(endPoint2);
                extremePoints.push_back(endpoints2);
            }

            result.cloudResult = planeInliers;
            result.extremePoints = extremePoints;
            result.cylinderCoefficients2 = cylinderCoefficients[0];
            result.cylinderCoefficients = cylinderCoefficients[1];
        } break;

        case WeldType::TubeToPlate2: {
            PLOGD << "TubeToPlate2类型焊缝检测中";
            pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pointcloudUniformDownsampling(cloud, 1.0f, downsampledCloud);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudResult(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ModelCoefficients cylinderCoefficients;
            pcl::ModelCoefficients planeCoefficients;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cylinderInliers(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr planeInliers(new pcl::PointCloud<pcl::PointXYZ>);
            fitPlaneAndCylinder(downsampledCloud, planeCoefficients, planeInliers, cylinderCoefficients, cylinderInliers);

            computefitPlaneAndCylinderIntersection(planeCoefficients, cylinderCoefficients, cloudResult);

            int p = 3;
            int n = cloudResult->points.size() / 3;
            if (n < p + 1) {
                n = p + 1;
            }
            std::vector<float> weights(n + 1, 1.0);
            NURBSCurve nurbsCurve = NURBSfitting(cloudResult, p, n, weights);

            std::vector<Point3D> generatePoints;
            for (double t = 0.0; t <= 1.0; t += 0.001) {
                Point3D pt = calculateNURBSPoint(t, nurbsCurve);
                generatePoints.push_back(pt);
            }

            result.cloudResult = cloudResult;
            result.generatePoints = generatePoints;

        } break;

        case WeldType::TubeToTube: {
            PLOGD << "TubeToTube类型焊缝检测中";
            pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pointcloudUniformDownsampling(cloud, 1.0f, downsampledCloud);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudResult(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudResultTemp(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr discretePointCloud(new pcl::PointCloud<pcl::PointXYZ>);
            std::vector<pcl::ModelCoefficients> cylinderCoefficients;
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cylinderInliers;

            fit2Cylinders(downsampledCloud, cylinderCoefficients, cylinderInliers);

            //        compute2CylinderIntersection(cylinderCoefficients, cylinderInliers, cloudResult);

            euclideanClusteringRemoval(cylinderInliers[1], cloudResultTemp);
            extractCylinderBoundary(cylinderCoefficients[1], cloudResultTemp, cloudResult);

            int p = 3;
            int n = cloudResult->points.size() / 6;
            if (n < p + 1) {
                n = p + 1;
            }
            std::vector<float> weights(n + 1, 1.0);
            NURBSCurve nurbsCurve = NURBSfitting(cloudResult, p, n, weights);
            std::vector<Point3D> generatePoints;
            for (double t = 0.0; t <= 1.0; t += 0.001) {
                Point3D pt = calculateNURBSPoint(t, nurbsCurve);
                generatePoints.push_back(pt);
            }

            NURBSCurve nurbsCurve2 = projectNURBSCurveRadially(nurbsCurve, cylinderCoefficients[1], 9.8);
            std::vector<Point3D> generatePoints2;
            for (double t = 0.0; t <= 1.0; t += 0.001) {
                Point3D pt = calculateNURBSPoint(t, nurbsCurve2);
                generatePoints2.push_back(pt);
            }
            std::vector<Point3D> discretizePoints = discretizeNURBSCurve(0.01, 0, 1, 0.1, nurbsCurve2);
            for (const auto& pt : discretizePoints) {
                pcl::PointXYZ pclPt;
                pclPt.x = static_cast<float>(pt.x);
                pclPt.y = static_cast<float>(pt.y);
                pclPt.z = static_cast<float>(pt.z);
                discretePointCloud->push_back(pclPt);
            }
            *cloudResult = *cloudResult + *discretePointCloud;

            result.cloudResult = cloudResult;
            result.generatePoints = generatePoints;
            result.generatePoints2 = generatePoints2;
        } break;
    }

    emit positioningComplete(result);
}
