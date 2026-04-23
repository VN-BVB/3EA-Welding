#include "PointCloudFunc.h"

// 直通滤波
void MyToolFunc::passthroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered, double limit_min,
                                   double limit_max) {
    std::vector<int> index;
    for (int i = 0; i < cloud->points.size(); ++i) {
        if (cloud->points[i].z >= limit_min && cloud->points[i].z <= limit_max) index.push_back(i);
    }

    // boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(index);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    inliers->indices = index;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_filtered);
}

// 统计滤波
void MyToolFunc::statisticalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered, int nr_k,
                                   float std_mul) {
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);

    std::vector<float> avg = std::vector<float>(cloud->size());
    ;
#pragma omp parallel for num_threads(12)
    for (int i = 0; i < cloud->points.size(); ++i) {
        std::vector<int> id(nr_k);
        std::vector<float> dist(nr_k);
        tree.nearestKSearch(cloud->points[i], nr_k, id, dist);
        std::for_each(dist.begin(), dist.end(), [](float& d) { d = std::sqrt(d); });
        float mean = std::accumulate(dist.begin(), dist.end(), 0.0) / dist.size();
        avg[i] = mean;
    }
    float u = accumulate(avg.begin(), avg.end(), 0.0) / avg.size();  // 点云密度程度
    float sigma = calcSigma(avg, u);                                 // 点云密度波动程度

    std::vector<int> index;

#pragma omp parallel num_threads(12)
    {
        std::vector<int> index_private;
#pragma omp for nowait
        for (int i = 0; i < cloud->points.size(); ++i) {
            if (avg[i] <= u + std_mul * sigma) {
                index_private.push_back(i);
            }
        }
#pragma omp critical
        { index.insert(index.end(), index_private.begin(), index_private.end()); }
    }

    boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(index);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(index_ptr);
    extract.setNegative(false);
    extract.filter(*cloud_filtered);
}

// 计算向量的标准差, v为输入向量, avg为均值
float MyToolFunc::calcSigma(std::vector<float>& v, float& avg) {
    float sigma = 0.0;
    for (int i = 0; i < v.size(); ++i) {
        sigma += pow(v[i] - avg, 2);
    }
    return sqrt(sigma / v.size());
}

// 对点做矩阵变换
pcl::PointXYZ MyToolFunc::transformSinglePoint(const pcl::PointXYZ& single_point, const Eigen::Matrix4f& Tranfrom_matrix) {
    pcl::PointCloud<pcl::PointXYZ> cloud_in;
    pcl::PointCloud<pcl::PointXYZ> cloud_out;
    cloud_in.push_back(single_point);
    pcl::transformPointCloud(cloud_in, cloud_out, Tranfrom_matrix);
    return cloud_out.points[0];
}
pcl::ModelCoefficients::Ptr MyToolFunc::transformPlane(const pcl::ModelCoefficients::Ptr& plane, const Eigen::Matrix4f& T) {
    // p' = T * p----->πᵀ p = 0----->(T⁻¹)ᵀ π  · p' = 0
    if (!plane || plane->values.size() != 4) return nullptr;
    Eigen::Vector4f p;
    p << plane->values[0], plane->values[1], plane->values[2], plane->values[3];
    Eigen::Matrix4f T_inv_T = T.inverse().transpose();
    Eigen::Vector4f p_new = T_inv_T * p;
    // 归一化
    Eigen::Vector3f n(p_new[0], p_new[1], p_new[2]);
    float norm = n.norm();
    if (norm > 1e-6) p_new /= norm;
    pcl::ModelCoefficients::Ptr result(new pcl::ModelCoefficients);
    result->values.assign(p_new.data(), p_new.data() + 4);
    return result;
}
pcl::ModelCoefficients::Ptr MyToolFunc::transformCylinder(const pcl::ModelCoefficients::Ptr& cyl, const Eigen::Matrix4f& T) {
    if (!cyl || cyl->values.size() != 7) return nullptr;

    Eigen::Vector3f p(cyl->values[0], cyl->values[1], cyl->values[2]);
    Eigen::Vector3f d(cyl->values[3], cyl->values[4], cyl->values[5]);
    float r = cyl->values[6];

    Eigen::Matrix3f R = T.block<3, 3>(0, 0);
    Eigen::Vector3f t = T.block<3, 1>(0, 3);

    Eigen::Vector3f p_new = R * p + t;
    Eigen::Vector3f d_new = R * d;
    d_new.normalize();

    pcl::ModelCoefficients::Ptr result(new pcl::ModelCoefficients);
    result->values.resize(7);

    result->values[0] = p_new.x();
    result->values[1] = p_new.y();
    result->values[2] = p_new.z();

    result->values[3] = d_new.x();
    result->values[4] = d_new.y();
    result->values[5] = d_new.z();

    result->values[6] = r;

    return result;
}
// 对点云做矩阵变换
pcl::PointCloud<pcl::PointXYZ>::Ptr MyToolFunc::transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                                                    const Eigen::Matrix4f& Tranfrom_matrix) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_in, *cloud_out, Tranfrom_matrix);
    return cloud_out;
}

// 点云放缩与平移
void MyToolFunc::scalePointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, double scaleX, double transX, double scaleY, double transY) {
    // std::cout << "X * " << scaleX << " + " << transX << std::endl;
    // std::cout << "Y * " << scaleY << " + " << transY << std::endl;

    if (pointCloud) {
        for (auto& point : *pointCloud) {
            point.x = point.x * scaleX + transX;
            point.y = point.y * scaleY + transY;
        }
    }
}

// 计算两直线夹角
double MyToolFunc::getLineAngle(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2) {
    // 计算两三维向量夹角
    double rad = v1.normalized().dot(v2.normalized());
    if (rad < -1.0) {
        rad = -1.0;
    } else if (rad > 1.0) {
        rad = 1.0;
    }
    double Angle = std::acos(rad) * 180.0 / M_PI;
    if (Angle > 90) Angle = 180 - Angle;
    return Angle;
}

// 点到直线距离
double MyToolFunc::getPoint2LineDis(Eigen::Vector4f Point, pcl::ModelCoefficients::Ptr line_coff) {
    Eigen::Vector4f line_pt(line_coff->values[0], line_coff->values[1], line_coff->values[2], 0);
    Eigen::Vector4f line_dir(line_coff->values[3], line_coff->values[4], line_coff->values[5], 0);

    double distance = sqrt(pcl::sqrPointToLineDistance(Point, line_pt, line_dir));
    return distance;
}

// 点到直线距离
double MyToolFunc::getPoint2LineDis(pcl::PointXYZ point, pcl::ModelCoefficients::Ptr line_coff) {
    Eigen::Vector4f p(point.x, point.y, point.z, 0);

    Eigen::Vector4f line_pt(line_coff->values[0], line_coff->values[1], line_coff->values[2], 0);
    Eigen::Vector4f line_dir(line_coff->values[3], line_coff->values[4], line_coff->values[5], 0);

    double distance = sqrt(pcl::sqrPointToLineDistance(p, line_pt, line_dir));
    return distance;
}

// 点投影到直线
Eigen::Vector4f MyToolFunc::projPoint2Line(Eigen::Vector4f& point, pcl::ModelCoefficients::Ptr& line_coff) {
    Eigen::Vector4f line_pt(line_coff->values[0], line_coff->values[1], line_coff->values[2], 0);
    Eigen::Vector4f line_dir(line_coff->values[3], line_coff->values[4], line_coff->values[5], 0);

    Eigen::Vector4f pt = {point[0], point[1], point[2], 0};
    double k = (pt - line_pt).dot(line_dir) / line_dir.squaredNorm();
    Eigen::Vector4f pp = line_pt + k * line_dir;  // 投影点
    return pp;
}

// 点投影到直线
pcl::PointXYZ MyToolFunc::projPoint2Line(pcl::PointXYZ& p, pcl::ModelCoefficients::Ptr& line_coff) {
    Eigen::Vector4f point(p.x, p.y, p.z, 0);

    Eigen::Vector4f line_pt(line_coff->values[0], line_coff->values[1], line_coff->values[2], 0);
    Eigen::Vector4f line_dir(line_coff->values[3], line_coff->values[4], line_coff->values[5], 0);

    Eigen::Vector4f pt = {point[0], point[1], point[2], 0};
    double k = (pt - line_pt).dot(line_dir) / line_dir.squaredNorm();
    Eigen::Vector4f pp = line_pt + k * line_dir;  // 投影点

    pcl::PointXYZ res(pp[0], pp[1], pp[2]);

    return res;
}

// 点投影到直线
Eigen::Vector4f MyToolFunc::projPoint2Line(Eigen::Vector4f& point, Eigen::Vector4f& line_pt, Eigen::Vector4f& line_dir) {
    Eigen::Vector4f pt = {point[0], point[1], point[2], 0};
    double k = (pt - line_pt).dot(line_dir) / line_dir.squaredNorm();
    Eigen::Vector4f pp = line_pt + k * line_dir;  // 投影点
    return pp;
}

// 计算直线内点端点
void MyToolFunc::lineCloudEndPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::VectorXf& line_coff_vector,
                                    Eigen::Vector4f specified_start_point, std::vector<Eigen::Vector4f>& two_endpoints) {
    if (cloud->points.size() > 0) {
        pcl::PointXYZ A = cloud->points[0];  // 任选一点A
        pcl::PointXYZ B;                     // 离A最远的点B
        double dis = 0;
        for (int i = 0; i < cloud->points.size(); ++i) {  // 遍历直线点集合的点，找到离A最远的点B
            if (sqrt(pow((cloud->points[i].x - A.x), 2) + pow((cloud->points[i].y - A.y), 2) + pow((cloud->points[i].z - A.z), 2)) >= dis) {
                dis = sqrt(pow((cloud->points[i].x - A.x), 2) + pow((cloud->points[i].y - A.y), 2) + pow((cloud->points[i].z - A.z), 2));
                B = cloud->points[i];
            }
        }
        pcl::PointXYZ C;                                  // 离B最远的点C
        for (int i = 0; i < cloud->points.size(); ++i) {  // 遍历直线点集合的点，找到离B最远的点C
            if (sqrt(pow((cloud->points[i].x - B.x), 2) + pow((cloud->points[i].y - B.y), 2) + pow((cloud->points[i].z - B.z), 2)) >= dis) {
                dis = sqrt(pow((cloud->points[i].x - B.x), 2) + pow((cloud->points[i].y - B.y), 2) + pow((cloud->points[i].z - B.z), 2));
                C = cloud->points[i];
            }
        }

        Eigen::Vector4f pt_on_line = {line_coff_vector[0], line_coff_vector[1], line_coff_vector[2], 0};  // 直线上一点
        Eigen::Vector4f dir_line = {line_coff_vector[3], line_coff_vector[4], line_coff_vector[5], 0};    // 直线方向

        Eigen::Vector4f endpoint_1 = {B.x, B.y, B.z, 0};                               // PointXYZ转Vector4f
        double k1 = (endpoint_1 - pt_on_line).dot(dir_line) / dir_line.squaredNorm();  // 延伸值
        Eigen::Vector4f projected_endpoint_1 = pt_on_line + k1 * dir_line;             // B端点的投影点

        Eigen::Vector4f endpoint_2 = {C.x, C.y, C.z, 0};                               // PointXYZ转Vector4f
        double k2 = (endpoint_2 - pt_on_line).dot(dir_line) / dir_line.squaredNorm();  // 延伸值
        Eigen::Vector4f projected_endpoint_2 = pt_on_line + k2 * dir_line;             // C端点的投影点

        double dis_endpoint_1 = (projected_endpoint_1 - specified_start_point).norm();
        double dis_endpoint_2 = (projected_endpoint_2 - specified_start_point).norm();
        if (dis_endpoint_1 > dis_endpoint_2) {  // 将距离指定点最近的投影点，先放入向量
            two_endpoints.push_back(projected_endpoint_2);
            two_endpoints.push_back(projected_endpoint_1);
        } else {
            two_endpoints.push_back(projected_endpoint_1);
            two_endpoints.push_back(projected_endpoint_2);
        }
    }
}
// 计算点到直线垂线
void MyToolFunc::Solve_ProjectVerticalLine(Eigen::Vector4f& point, pcl::ModelCoefficients::Ptr& line_coff, Eigen::Vector4f& VerticalLine_vector) {
    Eigen::Vector4f line_pt(line_coff->values[0], line_coff->values[1], line_coff->values[2], 0);
    Eigen::Vector4f line_dir(line_coff->values[3], line_coff->values[4], line_coff->values[5], 0);

    Eigen::Vector4f pt = {point[0], point[1], point[2], 0};
    double k = (pt - line_pt).dot(line_dir) / line_dir.squaredNorm();
    Eigen::Vector4f pp = line_pt + k * line_dir;  // 投影点
    VerticalLine_vector = pp - pt;                // 投影垂线
}

// 点投影到平面
void MyToolFunc::projPoint2Plane(const pcl::PointXYZ& point, const pcl::ModelCoefficients& coefficients, pcl::PointXYZ& projection) {
    // 从ModelCoefficients中提取平面的参数
    float a = coefficients.values[0];
    float b = coefficients.values[1];
    float c = coefficients.values[2];
    float d = coefficients.values[3];

    // 计算原始点代入平面方程的左侧部分
    float d_value = a * point.x + b * point.y + c * point.z + d;

    // 计算法向量的模的平方
    float normal_squared = a * a + b * b + c * c;

    // 计算投影点的坐标
    projection.x = point.x - a * d_value / normal_squared;
    projection.y = point.y - b * d_value / normal_squared;
    projection.z = point.z - c * d_value / normal_squared;
}

// 计算点云的快速最大聚类点集
// 功能：对输入点云进行欧式聚类分析，提取出包含点数量最多的聚类，并将原输入点云替换为该最大聚类
// 参数：
//   cloud - 输入点云指针，同时作为输出，函数执行后将包含最大聚类的点云
//   cluster_tolerance_ - 聚类容差，决定点云中相邻点被划分为同一聚类的距离阈值
void MyToolFunc::myFastMaxCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double cluster_tolerance_) {
    // 1.---------------欧式聚类.---------------
    std::vector<pcl::PointIndices> clusters;
    clusters.clear();
    // 构建KdTree加速近邻搜索
    // KdTree是一种空间索引数据结构，用于快速查找k维空间中的近邻点，此处用于加速点云的半径搜索
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_(new pcl::search::KdTree<pcl::PointXYZ>);
    tree_->setInputCloud(cloud);

    std::vector<int> labels(cloud->size(), -1);
    std::vector<bool> removed(cloud->size(), false);

    std::vector<int> nn_indices;
    std::vector<float> nn_distances;
    double quality_ = 0;
    auto nn_distance_threshold = std::pow((1.0 - quality_) * cluster_tolerance_, 2.0);

    using Graph = boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS>;
    Graph g;
    std::queue<int> queue;
    {
        int label = 0;

        for (int index = 0; index < cloud->size(); index++) {
            if (removed.at(index)) continue;

            boost::add_edge(label, label, g);

            queue.push(index);
            while (!queue.empty()) {
                auto p = queue.front();
                queue.pop();
                if (removed.at(p)) {
                    continue;
                }
                // 执行半径搜索，查找p点在cluster_tolerance_范围内的所有近邻点
                tree_->radiusSearch(p, cluster_tolerance_, nn_indices, nn_distances);
                // 处理所有近邻点
                for (std::size_t i = 0; i < nn_indices.size(); ++i) {
                    auto q = nn_indices.at(i);    // 近邻点索引
                    auto q_label = labels.at(q);  // 近邻点的当前标签

                    // 如果近邻点已被标记为不同的聚类，则在图中连接两个聚类
                    /* if (q_label != pcl::UNAVAILABLE && q_label != label) {*/
                    if (q_label != -1 && q_label != label) {
                        boost::add_edge(label, q_label, g);  // 建立聚类间的连接
                    }

                    if (removed.at(q)) {
                        continue;  // 跳过已处理的点
                    }

                    labels.at(q) = label;  // 将近邻点标记为当前聚类
                    // 根据距离阈值决定点的处理方式
                    // Must be <= to remove self (p).
                    if (nn_distances.at(i) <= nn_distance_threshold) {
                        removed.at(q) = true;  // 距离较近的点标记为已处理
                    } else {
                        queue.push(q);  // 距离较远但仍在容差内的点继续搜索其近邻
                    }
                }
            }

            label++;  // 进入下一个聚类标签
        }
    }

    // Merge labels.
    // 合并连通的聚类标签
    // 使用boost库的connected_components算法计算图的连通分量
    // 每个连通分量对应一个实际的聚类
    std::vector<int> label_map(boost::num_vertices(g));  // 标签映射表，将原始标签映射到合并后的标签
    //  ✔ 计算图 g 里有多少个“连通分量”
    //  ✔ 并把每个顶点的“新分组编号”写进 label_map
    auto num_components = boost::connected_components(g, label_map.data());
    clusters.resize(num_components);  // 调整聚类容器大小

    for (int index = 0; index < cloud->size(); index++) {
        auto label = labels.at(index);  // 获取点的原始标签
        if (label != -1) {
            auto new_label = label_map.at(label);             // 获取合并后的新标签
            clusters.at(new_label).indices.push_back(index);  // 将点索引加入对应聚类
        }
    }

    // 过滤小聚类和过大聚类
    auto read = clusters.begin();
    auto write = clusters.begin();
    int min_cluster_size_ = 1;
    int max_cluster_size_ = 99999999;
    // 双指针压缩法
    for (; read != clusters.end(); ++read) {
        if (read->indices.size() >= min_cluster_size_ && read->indices.size() <= max_cluster_size_) {
            if (read != write) {
                *write = std::move(*read);  // 交换内部指针，避免拷贝开销
            }
            ++write;
        }
    }
    clusters.resize(std::distance(clusters.begin(), write));

    // 2.---------------最大聚类点集提取.---------------
    std::vector<int> max_cluster;  // 存储最大聚类的点索引
    int max_size = 0;              // 最大聚类的点数量

    // 遍历所有聚类，寻找包含点数量最多的聚类
    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it) {
        if (it->indices.size() > max_size) {
            max_size = it->indices.size();
            max_cluster = it->indices;
        }
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr max_cluster_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = max_cluster.begin(); pit != max_cluster.end(); ++pit)
        max_cluster_pointcloud->points.push_back(cloud->points[*pit]);
    // 使用最大聚类点云替换原输入点云
    // 使用swap函数提高效率，避免大量数据拷贝
    cloud->swap(*max_cluster_pointcloud);
}

// 计算高曲率点
void MyToolFunc::highCurvaturePointsDetect(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_detect, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                           double K_Radius, double sm_ratio, pcl::PointCloud<pcl::PointXYZ>::Ptr high_curvature_scatter_points) {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  // 建立kdtree对象
    kdtree.setInputCloud(cloud);             // 设置需要建立kdtree的点云指针
    pcl::PointXYZ searchPoint;
    // ---------------- All the Points of the cloud -----------------
    for (size_t i = 0; i < cloud_detect->points.size(); ++i) {
        searchPoint.x = cloud_detect->points[i].x;
        searchPoint.y = cloud_detect->points[i].y;
        searchPoint.z = cloud_detect->points[i].z;

        std::vector<int> pointIdxRadiusSearch;          // 保存每个近邻点的索引
        std::vector<float> pointRadiusSquaredDistance;  // 保存每个近邻点与查找点之间的欧式距离平方

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_neighbor(new pcl::PointCloud<pcl::PointXYZ>);  // 半径近邻点

        if (kdtree.radiusSearch(searchPoint, K_Radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 5) {
            for (int j = 0; j < pointIdxRadiusSearch.size(); j++) {
                cloud_neighbor->points.push_back(cloud->points[pointIdxRadiusSearch[j]]);
            }
        } else {
            std::cout << "领域搜索失败" << std::endl;
            continue;
        }

        // 计算点云整体中心和构建协方差矩阵
        Eigen::Vector4f zx;                           // 中心
        pcl::compute3DCentroid(*cloud_neighbor, zx);  // xyz1
        Eigen::Matrix3f xiefancha;
        pcl::computeCovarianceMatrixNormalized(*cloud_neighbor, zx, xiefancha);  // 计算归一化协方差矩阵
        // 计算特征值和特征向量
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solve(xiefancha, Eigen::ComputeEigenvectors);
        Eigen::Vector3f values = solve.eigenvalues();  // 特征值  特征值从小到大
        // cout << "values: " << values[0] << "; " << values[1] << "; " << values[2] <<std::endl;

        if (abs(values[0] / values[1]) > sm_ratio) {  // 排除平面
            high_curvature_scatter_points->push_back(cloud_detect->points[i]);
        }
    }
    if (high_curvature_scatter_points->size() == 0) {
        cerr << "高曲率点提取错误，尝试更改近邻半径和比率阈值的设置" << std::endl;
    }
}

// 计算直线内点端点
std::vector<pcl::PointXYZ> MyToolFunc::lineCloudEndPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr lineCloud, pcl::ModelCoefficients::Ptr coefficients) {
    // 1.排序
    std::vector<std::pair<int, double>> idxSorted(lineCloud->size());
    for (int j = 0; j < lineCloud->size(); j++) {
        idxSorted[j].first = j;
        idxSorted[j].second =
            lineCloud->points[j].getVector3fMap().dot(Eigen::Map<Eigen::Vector3f>(const_cast<float*>(coefficients->values.data() + 3), 3));
    }
    std::sort(idxSorted.begin(), idxSorted.end(),
              [](const std::pair<int, double>& lhs, const std::pair<int, double>& rhs) { return lhs.second < rhs.second; });
    pcl::PointCloud<pcl::PointXYZ>::Ptr sortedCloudLine(new pcl::PointCloud<pcl::PointXYZ>);
    for (int j = 0; j < lineCloud->size(); j++) {
        sortedCloudLine->push_back(lineCloud->points[idxSorted[j].first]);  // 获取排序后的点云
    }
    // 2.投影
    pcl::PointCloud<pcl::PointXYZ>::Ptr projectedCloudLine(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ProjectInliers<pcl::PointXYZ> projector;
    projector.setModelType(pcl::SACMODEL_LINE);
    projector.setInputCloud(sortedCloudLine);
    projector.setModelCoefficients(coefficients);
    projector.filter(*projectedCloudLine);
    // 3.得到端点
    std::vector<pcl::PointXYZ> lineEndpoints;  // 每条直线的两个端点
    lineEndpoints.push_back(projectedCloudLine->points.front());
    lineEndpoints.push_back(projectedCloudLine->points.back());

    return lineEndpoints;
}

// 在球体邻域内保留一个点，避免密度不均；均匀下采样点云
void MyToolFunc::pointcloudUniformDownsampling(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float leafSize,
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

// 体素下采样点云
void MyToolFunc::pointcloudVoxelDownsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leafSize,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudResult) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(leafSize, leafSize, leafSize);
    voxel.filter(*cloudResult);
}
// 点云投影至指定平面
void MyToolFunc::projectCloudToPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud,
                                     pcl::ModelCoefficients::Ptr planeCoeffs) {
    if (!input_cloud || !output_cloud || input_cloud->empty()) {
        PLOGE << "projectCloudToPlane: 输入参数无效";
        return;
    }
    if (!planeCoeffs || planeCoeffs->values.size() < 4) {
        PLOGE << "projectCloudToPlane: 平面系数无效";
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(input_cloud);
    proj.setModelCoefficients(planeCoeffs);
    proj.filter(*tmp);

    output_cloud->swap(*tmp);
}

void MyToolFunc::projectCloudToCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud,
                                        pcl::ModelCoefficients::Ptr cylinder_coeffs) {
    if (!input_cloud || input_cloud->empty() || !output_cloud || !cylinder_coeffs || cylinder_coeffs->values.size() != 7) {
        std::cerr << "projectCloudToCylinder: 参数错误" << std::endl;
        return;
    }

    // 如果输入输出是同一个对象，先拷贝
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src = input_cloud;

    if (input_cloud == output_cloud) {
        cloud_src.reset(new pcl::PointCloud<pcl::PointXYZ>(*input_cloud));
    }
    // ---------- 解析参数 ----------
    Eigen::Vector3f C(cylinder_coeffs->values[0], cylinder_coeffs->values[1], cylinder_coeffs->values[2]);

    Eigen::Vector3f d(cylinder_coeffs->values[3], cylinder_coeffs->values[4], cylinder_coeffs->values[5]);

    float r = cylinder_coeffs->values[6];
    d.normalize();

    output_cloud->clear();
    output_cloud->reserve(cloud_src->size());

    for (const auto& pt : cloud_src->points) {
        Eigen::Vector3f P(pt.x, pt.y, pt.z);

        float t = (P - C).dot(d);
        Eigen::Vector3f P_axis = C + t * d;

        Eigen::Vector3f v = P - P_axis;
        float norm_v = v.norm();

        if (norm_v < 1e-6) continue;

        Eigen::Vector3f P_proj = P_axis + v / norm_v * r;

        output_cloud->points.emplace_back(P_proj.x(), P_proj.y(), P_proj.z());
    }
}
// 构造圆柱点云（理论点云）
pcl::PointCloud<pcl::PointXYZ>::Ptr MyToolFunc::generateCylinderCloud(pcl::ModelCoefficients::Ptr cylinder) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // ---------- 1. 参数检查 ----------
    if (!cylinder || cylinder->values.size() != 7) {
        PLOGE << "generateCylinderCloud: cylinder coeff invalid";
        return cloud;
    }

    Eigen::Vector3f p0(cylinder->values[0], cylinder->values[1], cylinder->values[2]);

    Eigen::Vector3f dir(cylinder->values[3], cylinder->values[4], cylinder->values[5]);

    float radius = cylinder->values[6];

    if (dir.norm() < 1e-6f) {
        PLOGE << "generateCylinderCloud: direction invalid";
        return cloud;
    }

    dir.normalize();

    // ---------- 2. 默认高度 ----------
    float height = 100.0f;  // ⚠️ 可根据你工件尺寸改
    float min_t = -height / 2.0f;
    float max_t = height / 2.0f;

    // ---------- 3. 构造正交基 ----------
    Eigen::Vector3f u = dir.unitOrthogonal();
    Eigen::Vector3f v = dir.cross(u);

    // ---------- 4. 采样密度 ----------
    int height_samples = 100;
    int circle_samples = 100;

    cloud->points.reserve(height_samples * circle_samples);

    // ---------- 5. 生成点云 ----------
    for (int i = 0; i < height_samples; ++i) {
        float t = min_t + (max_t - min_t) * float(i) / float(height_samples - 1);
        Eigen::Vector3f center = p0 + t * dir;

        for (int j = 0; j < circle_samples; ++j) {
            float theta = 2.0f * M_PI * j / circle_samples;

            Eigen::Vector3f pt = center + radius * std::cos(theta) * u + radius * std::sin(theta) * v;

            cloud->points.emplace_back(pt.x(), pt.y(), pt.z());
        }
    }

    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;

    return cloud;
}
