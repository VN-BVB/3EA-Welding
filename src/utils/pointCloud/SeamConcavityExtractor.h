#ifndef SEAMCONCAVITYEXTRACTOR_H
#define SEAMCONCAVITYEXTRACTOR_H
// ========================= 特征结果定义 =========================

struct NeighborhoodData {
    pcl::PointXYZ queryPoint;

    float referenceRadius = 0.0f;           // 最终搜索半径 r
    std::vector<int> neighborIndices;       // 原始邻域索引
    std::vector<int> validNeighborIndices;  // 有效邻域索引
    std::vector<float> rawSqrDistances;     // 原始邻域距离
    std::vector<float> validSqrDistances;   // 有效邻域距离
};

struct SphereProjectionData {
    pcl::PointXYZ queryPoint;

    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();  // 邻域中心 ci
    Eigen::Vector3f normal = Eigen::Vector3f::Zero();    // 法向 ni

    Eigen::Vector3f meanVector = Eigen::Vector3f::Zero();  // 均值向量 u

    std::vector<Eigen::Vector3f> unitVectors;  // 单位投影向量 u_hat_j
    std::vector<float> weights;                // 权重 w_j
};

struct PcaProjectionData {
    pcl::PointXYZ queryPoint;

    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    Eigen::Vector3f eigenValues = Eigen::Vector3f::Zero();

    Eigen::Vector3f v1 = Eigen::Vector3f::Zero();  // 第一主方向
    Eigen::Vector3f v2 = Eigen::Vector3f::Zero();  // 第二主方向

    std::vector<Eigen::Vector3f> centeredVectors;    // 去中心化后的 3D 向量
    std::vector<Eigen::Vector2f> projected2DPoints;  // PCA 投影后的 2D 点
};

struct ReconstructedCurveData {
    pcl::PointXYZ queryPoint;

    std::vector<Eigen::Vector2f> fittedCircle2D;        // 二维标准圆离散点
    std::vector<Eigen::Vector3f> reconstructedCurve3D;  // 重建回 3D 球面的拟合曲线 s(theta)
};

struct FeatureResult {
    pcl::PointXYZ queryPoint;

    bool valid = false;                     // 当前点特征是否成功计算
    bool isConcavityPoint = false;          // 是否判定为凹凸焊缝点
    bool isMeanDeviationCandidate = false;  // 是否通过第一阶段 meanDeviation 筛选
    bool isFinalSeamPoint = false;          // 是否通过第二阶段 concavity 筛选

    float referenceRadius = 0.0f;    // 邻域参考半径 r
    float meanDeviation = 0.0f;      // 平均偏差 e
    float concavityScore = 0.0f;     // 凹凸度分数
    float absConcavityScore = 0.0f;  // |concavityScore|

    float positiveRatio = 0.0f;  // r+
    float negativeRatio = 0.0f;  // r-

    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    Eigen::Vector3f normal = Eigen::Vector3f::Zero();
    Eigen::Vector3f meanVector = Eigen::Vector3f::Zero();

    NeighborhoodData neighborhood;
    SphereProjectionData sphereProjection;
    PcaProjectionData pcaProjection;
    ReconstructedCurveData reconstructedCurve;
};
class SeamConcavityExtractor {
public:
    SeamConcavityExtractor();
    ~SeamConcavityExtractor() = default;

public:
    // ========================= 对外主流程接口 =========================

    /**
     * @brief setInputCloud 设置输入局部点云
     * @param cloud 输入局部区域点云
     */
    void setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    /**
     * @brief setQueryPoints 设置待分析点（通常是一段理论点 / 实际点 / 局部采样点）
     * @param queryPoints 待分析点集
     */
    void setQueryPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    /*设置领域半径*/
    void setNeighborRadius(float radius);

    /**
     * @brief run 总运行函数
     * @return 是否运行成功
     */
    bool run();

    /**
     * @brief getConcavityPoints 获取最终判定为凹凸焊缝点的点集
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getConcavityPoints() const;

    /**
     * @brief getAllFeatureResults 获取所有查询点的特征结果
     */
    const std::vector<FeatureResult>& getAllFeatureResults() const;

    /**
     * @brief setNeighborhoodClusterTolerance
     *        设置邻域内欧式聚类半径
     */
    void setNeighborhoodClusterTolerance(float tol);

    /**
     * @brief setNeighborhoodMinValidPoints
     *        设置有效邻域最少点数
     */
    void setNeighborhoodMinValidPoints(int n);
    void setDotSignEpsilon(float eps);
    void setAreadex(int areanum);

public:
    // ========================= 参数设置接口 =========================

    /**
     * @brief setQuantileRatio 设置有效距离区间比例 γ
     */
    void setQuantileRatio(float gamma);

    /**
     * @brief setWeightAlpha 设置距离权重衰减系数 alpha
     */
    void setWeightAlpha(float alpha);

    /**
     * @brief setCurveSampleCount 设置拟合曲线离散采样数 Kp
     */
    void setCurveSampleCount(int kp);

    /**
     * @brief setThresholds 设置平均偏差与凹凸度阈值
     */
    void setThresholds(float meanDeviationThresh, float concavityScoreThresh);

    /**
     * @brief setDebug 是否打印调试信息
     */
    void setDebug(bool enable);
    void setUseAutoThreshold(bool enable);
    void setAutoThresholdMadScale(float s);

    float getMeanDeviationAutoThreshold() const;
    float getConcavityScoreAutoThreshold() const;

private:
    // ========================= 总流程阶段函数 =========================

    /**
     * @brief buildKdTree 构建 KdTree
     */
    bool buildKdTree();

    /**
     * @brief processSingleQueryPoint 处理单个查询点
     */
    bool processSingleQueryPoint(const pcl::PointXYZ& queryPoint, FeatureResult& result);
    bool processSingleQueryPointMeanDeviationOnly(const pcl::PointXYZ& queryPoint, FeatureResult& result);
    bool processSingleQueryPointConcavityOnly(FeatureResult& result);
    /**
     * @brief extractNeighborhood 提取邻域并确定参考半径
     */
    bool extractNeighborhood(const pcl::PointXYZ& queryPoint, NeighborhoodData& neighborhood);

    /**
     * @brief filterValidNeighborhoodByQuantile 基于分位数筛选有效邻域
     */
    bool filterValidNeighborhoodByQuantile(NeighborhoodData& neighborhood);

    /**
     * @brief clusterValidNeighborhoodKeepMaxCluster
     *        对分位数筛选后的有效邻域做欧式聚类，只保留最大聚类
     */
    bool clusterValidNeighborhoodKeepMaxCluster(NeighborhoodData& neighborhood);
    /**
     * @brief computeSphereProjection 计算球面投影、单位向量和权重
     */
    bool computeSphereProjection(const NeighborhoodData& neighborhood, SphereProjectionData& sphereData);

    /**
     * @brief computeCentroidAndNormal 计算邻域中心和法向
     */
    bool computeCentroidAndNormal(const NeighborhoodData& neighborhood, SphereProjectionData& sphereData);

    /**
     * @brief computePcaProjection PCA降维到二维
     */
    bool computePcaProjection(const SphereProjectionData& sphereData, PcaProjectionData& pcaData);

    /**
     * @brief generateStandardCircle2D 生成二维标准圆离散点
     */
    bool generateStandardCircle2D(ReconstructedCurveData& curveData);

    /**
     * @brief reconstructCurveToSphere 将二维圆重建回三维球面
     */
    bool reconstructCurveToSphere(const SphereProjectionData& sphereData, const PcaProjectionData& pcaData, ReconstructedCurveData& curveData);

    /**
     * @brief computeMeanDeviation 计算平均偏差 e
     */
    bool computeMeanDeviation(const SphereProjectionData& sphereData, const PcaProjectionData& pcaData, const ReconstructedCurveData& curveData,
                              float& meanDeviation);

    /**
     * @brief computeConcavityScore 计算凹凸度分数
     */
    bool computeConcavityScore(const NeighborhoodData& neighborhood, const SphereProjectionData& sphereData, float meanDeviation,
                               float& positiveRatio, float& negativeRatio, float& concavityScore);

    /**
     * @brief judgeConcavityPoint 根据阈值判定是否为凹凸焊缝点
     */
    bool judgeConcavityPoint(float meanDeviation, float concavityScore) const;

private:
    // ========================= 数学辅助函数 =========================

    /**
     * @brief computeTwoQuantiles 计算距离分位数
     */
    bool computeTwoQuantiles(const std::vector<float>& values, float qLow, float qHigh, float& outLow, float& outHigh) const;

    /**
     * @brief computeMedian 计算中位数
     */
    float computeMedian(std::vector<float> values) const;

    /**
     * @brief pointToCurveMinDistance 计算点到离散曲线的最小距离
     */
    float pointToCurveMinDistance(const Eigen::Vector3f& point, const std::vector<Eigen::Vector3f>& curve) const;

    /**
     * @brief toEigen 转 Eigen
     */
    Eigen::Vector3f toEigen(const pcl::PointXYZ& p) const;

    /**
     * @brief toPcl 转 pcl::PointXYZ
     */
    pcl::PointXYZ toPcl(const Eigen::Vector3f& v) const;
    // GMM聚类

    float estimateMeanDeviationThreshold(const std::vector<FeatureResult>& results) const;
    float estimateConcavityThreshold(const std::vector<FeatureResult>& results) const;

    bool saveMeanDeviationCandidateCloud(const std::vector<FeatureResult>& results, const std::string& savePath);
    bool saveFinalSeamPointCloud(const std::vector<FeatureResult>& results, const std::string& savePath);

    /**
     * @brief saveMeanDeviationHeatmap 画偏差图
     */
    bool saveMeanDeviationHeatmap(const std::vector<FeatureResult>& results, const std::string& savePath);
    bool saveConcavityScoreHeatmap(const std::vector<FeatureResult>& results, const std::string& savePath);
    bool saveFeatureCurvesImage(const std::vector<FeatureResult>& results, const std::string& savePath);
    bool saveFeatureCurvesCsv(const std::vector<FeatureResult>& results, const std::string& savePath);

private:
    // ========================= 输入输出 =========================

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr queryPoints_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr concavityPoints_;
    std::vector<FeatureResult> featureResults_;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_;

private:
    // ========================= debug单个点 =========================
    bool debugSaveSingleQueryProcess(const pcl::PointXYZ& targetPoint, const std::string& saveDir);
    bool saveNeighborhoodClouds(const FeatureResult& result, const std::string& saveDir);
    bool savePcaVisualizationClouds(const FeatureResult& result, const std::string& saveDir);
    bool saveReconstructedCurveCloud(const FeatureResult& result, const std::string& saveDir);
    bool saveDebugTextReport(const FeatureResult& result, const std::string& saveDir);

    pcl::PointXYZ findNearestQueryPoint(const pcl::PointXYZ& targetPoint, int& nearestIdx, float& nearestDist) const;

private:
    // ========================= 参数 =========================
    float neighborRadius_ = 3.0f;                // 固定邻域半径
    float quantileGamma_ = 1.0f;                 // 有效距离比例 γ
    float weightAlpha_ = 1.0f;                   // 权重衰减系数 alpha
    int curveSampleCount_ = 72;                  // 拟合圆采样数 Kp
    float neighborhoodClusterTolerance_ = 0.0f;  // 可调，单位与点云一致
    int neighborhoodMinValidPoints_ = 5;
    float meanDeviationThresh_ = 0.1f;   // 手动阈值
    float concavityScoreThresh_ = 0.5f;  // 手动阈值

    float meanDeviationAutoThresh_ = 0.0f;   // 自动估计结果
    float concavityScoreAutoThresh_ = 0.0f;  // 自动估计结果
    bool useAutoThreshold_ = 1;              // 是否启用自动阈值
    float autoThreshMadScale_ = 1.0f;        // median + scale * 1.4826 * MAD
    float dotSignEps_ = 1e-6f;

    bool debug_ = false;
    int areadex = 0;
};

#endif  // SEAMCONCAVITYEXTRACTOR_H
