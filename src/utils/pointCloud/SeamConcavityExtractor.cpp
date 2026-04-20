#include "SeamConcavityExtractor.h"

#include "utils/common/CommonFunc.h"

namespace {
const std::vector<Eigen::Vector2f>& getCachedStandardCircle2D(int sampleCount) {
    static thread_local int cachedSampleCount = 0;
    static thread_local std::vector<Eigen::Vector2f> cachedCircle;

    if (cachedSampleCount != sampleCount) {
        cachedCircle.clear();
        cachedCircle.reserve(sampleCount);

        for (int k = 0; k < sampleCount; ++k) {
            float theta = 2.0f * static_cast<float>(M_PI) * static_cast<float>(k) / static_cast<float>(sampleCount);
            cachedCircle.emplace_back(std::cos(theta), std::sin(theta));
        }

        cachedSampleCount = sampleCount;
    }

    return cachedCircle;
}
}  // namespace

SeamConcavityExtractor::SeamConcavityExtractor() {
    inputCloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    queryPoints_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    concavityPoints_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    kdtree_.reset(new pcl::search::KdTree<pcl::PointXYZ>());
}

void SeamConcavityExtractor::setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (!cloud || cloud->empty()) {
        inputCloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        return;
    }

    inputCloud_ = cloud;
}

void SeamConcavityExtractor::setQueryPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (!cloud || cloud->empty()) {
        queryPoints_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        return;
    }

    queryPoints_ = cloud;
}

void SeamConcavityExtractor::setDebug(bool enable) { debug_ = enable; }
pcl::PointCloud<pcl::PointXYZ>::Ptr SeamConcavityExtractor::getConcavityPoints() const { return concavityPoints_; }
void SeamConcavityExtractor::setNeighborRadius(float radius) { neighborRadius_ = radius; }
const std::vector<FeatureResult>& SeamConcavityExtractor::getAllFeatureResults() const { return featureResults_; }
void SeamConcavityExtractor::setNeighborhoodClusterTolerance(float tol) { neighborhoodClusterTolerance_ = tol; }
void SeamConcavityExtractor::setNeighborhoodMinValidPoints(int n) { neighborhoodMinValidPoints_ = std::max(3, n); }
void SeamConcavityExtractor::setDotSignEpsilon(float eps) { dotSignEps_ = std::max(0.0f, eps); }
void SeamConcavityExtractor::setAreadex(int areanum) { areadex = std::max(0, areanum); }
void SeamConcavityExtractor::setUseAutoThreshold(bool enable) { useAutoThreshold_ = enable; }
void SeamConcavityExtractor::setAutoThresholdMadScale(float s) { autoThreshMadScale_ = std::max(0.5f, s); }

float SeamConcavityExtractor::getMeanDeviationAutoThreshold() const { return meanDeviationAutoThresh_; }
float SeamConcavityExtractor::getConcavityScoreAutoThreshold() const { return concavityScoreAutoThresh_; }
/*-- --zhu逻辑-- --*/
// bool SeamConcavityExtractor::run() {
//     if (!inputCloud_ || inputCloud_->empty()) {
//         return false;
//     }

//     if (!queryPoints_ || queryPoints_->empty()) {
//         return false;
//     }

//     if (neighborRadius_ <= 1e-6f) {
//         return false;
//     }

//     if (!concavityPoints_) {
//         concavityPoints_.reset(new pcl::PointCloud<pcl::PointXYZ>());
//     }
//     concavityPoints_->clear();

//     const int totalPts = static_cast<int>(queryPoints_->size());
//     featureResults_.clear();
//     featureResults_.resize(totalPts);

//     if (!buildKdTree()) {
//         return false;
//     }
//     // ==== debug 指定点 ====
//     // {
//     //     pcl::PointXYZ debugTarget;
//     //     debugTarget.x = 45.0f;
//     //     debugTarget.y = -73.0f;
//     //     debugTarget.z = 437.0f;

//     //     debugSaveSingleQueryProcess(debugTarget, "./data/seamDetWithPointCloud/tubePlateFilletSeamsDet/debug_single_point");
//     // }
//     // 线程数
//     int numThreads = omp_get_max_threads();
//     if (numThreads < 1) numThreads = 1;

//     // 每组大小
//     int chunkSize = (totalPts + numThreads - 1) / numThreads;

// #pragma omp parallel for schedule(static, 1)
//     for (int chunkId = 0; chunkId < numThreads; ++chunkId) {
//         int beginIdx = chunkId * chunkSize;
//         int endIdx = std::min(beginIdx + chunkSize, totalPts);

//         for (int i = beginIdx; i < endIdx; ++i) {
//             const auto& queryPoint = queryPoints_->points[i];

//             FeatureResult result;
//             result.queryPoint = queryPoint;

//             bool ok = processSingleQueryPoint(queryPoint, result);
//             result.valid = ok;

//             featureResults_[i] = result;
//         }
//     }

//     concavityPoints_->width = static_cast<uint32_t>(concavityPoints_->size());
//     concavityPoints_->height = 1;
//     concavityPoints_->is_dense = false;
//     //====debug可视化====
//     std::string path = "./data/seamDetWithPointCloud/tubePlateFilletSeamsDet/debug_meanDeviation_heatmap" + std::to_string(areadex) + ".pcd";
//     saveMeanDeviationHeatmap(featureResults_, path);

//     std::string path2 = "./data/seamDetWithPointCloud/tubePlateFilletSeamsDet/debug_absConcavityScore_heatmap_" + std::to_string(areadex) + ".pcd";
//     saveConcavityScoreHeatmap(featureResults_, path2);
//     std::string csvPath = "./data/seamDetWithPointCloud/tubePlateFilletSeamsDet/debug_feature_curves_" + std::to_string(areadex) + ".csv";
//     saveFeatureCurvesCsv(featureResults_, csvPath);

//     std::string imgPath = "./data/seamDetWithPointCloud/tubePlateFilletSeamsDet/debug_feature_curves_" + std::to_string(areadex) + ".png";
//     saveFeatureCurvesImage(featureResults_, imgPath);
//     return true;
// }
bool SeamConcavityExtractor::run() {
    if (!inputCloud_ || inputCloud_->empty()) {
        return false;
    }

    if (!queryPoints_ || queryPoints_->empty()) {
        return false;
    }

    if (neighborRadius_ <= 1e-6f) {
        return false;
    }

    if (!concavityPoints_) {
        concavityPoints_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    }
    concavityPoints_->clear();

    const int totalPts = static_cast<int>(queryPoints_->size());
    featureResults_.clear();
    featureResults_.resize(totalPts);

    if (!buildKdTree()) {
        return false;
    }
    // // ==== debug 指定点 ====
    // {
    //     pcl::PointXYZ debugTarget;
    //     debugTarget.x = 45.249f;
    //     debugTarget.y = -71.767f;
    //     debugTarget.z = 435.770f;

    //     debugSaveSingleQueryProcess(debugTarget, "./data/seamDetWithPointCloud/tubePlateFilletSeamsDet/debug_single_point");
    // }
    // ================= 第一阶段：只计算 meanDeviation =================
    int numThreads = omp_in_parallel() ? 1 : omp_get_max_threads();
    if (numThreads < 1) numThreads = 1;

#pragma omp parallel for schedule(dynamic, 64) if (numThreads > 1) num_threads(numThreads)
    for (int i = 0; i < totalPts; ++i) {
        auto& result = featureResults_[i];
        result.queryPoint = queryPoints_->points[i];

        bool ok = processSingleQueryPointMeanDeviationOnly(queryPoints_->points[i], result);
        result.valid = ok;
    }

    // ================= 自动估计 meanDeviation 阈值 =================
    if (useAutoThreshold_) {
        meanDeviationAutoThresh_ = estimateMeanDeviationThreshold(featureResults_);
    } else {
        meanDeviationAutoThresh_ = meanDeviationThresh_;
    }

    // ================= 第一阶段划分：平面 / 焊缝候选 =================
    int candidateCount = 0;
    for (auto& r : featureResults_) {
        if (!r.valid) continue;
        r.isMeanDeviationCandidate = (r.meanDeviation >= meanDeviationAutoThresh_);
        if (r.isMeanDeviationCandidate) candidateCount++;
    }

    /*保存第一阶段候选点 */
    if (debug_) {
        std::string path = "./data/seamDetWithPointCloud/tubePlateFilletSeamsDet/meanDeviationCandidates_" + std::to_string(areadex) + ".pcd";
        saveMeanDeviationCandidateCloud(featureResults_, path);
    }
    PLOGD << "meanDeviationAutoThresh = " << meanDeviationAutoThresh_ << ", candidateCount = " << candidateCount << ", totalPts = " << totalPts;

// ================= 第二阶段：只对候选点计算 concavity =================
#pragma omp parallel for schedule(dynamic, 32) if (numThreads > 1) num_threads(numThreads)
    for (int i = 0; i < totalPts; ++i) {
        auto& r = featureResults_[i];
        if (!r.valid) continue;

        if (!r.isMeanDeviationCandidate) {
            r.positiveRatio = 0.0f;
            r.negativeRatio = 0.0f;
            r.concavityScore = 0.0f;
            r.absConcavityScore = 0.0f;
            r.isFinalSeamPoint = false;
            r.isConcavityPoint = false;
            continue;
        }

        bool ok = processSingleQueryPointConcavityOnly(r);
        if (!ok) {
            r.positiveRatio = 0.0f;
            r.negativeRatio = 0.0f;
            r.concavityScore = 0.0f;
            r.absConcavityScore = 0.0f;
            r.isFinalSeamPoint = false;
            r.isConcavityPoint = false;
        }
    }
    // ================= 自动估计 concavity 阈值 =================
    if (useAutoThreshold_) {
        concavityScoreAutoThresh_ = estimateConcavityThreshold(featureResults_);
    } else {
        concavityScoreAutoThresh_ = concavityScoreThresh_;
    }

    // ================= 最终判定 =================
    concavityPoints_->clear();

    for (auto& r : featureResults_) {
        if (!r.valid) continue;

        r.isFinalSeamPoint = r.isMeanDeviationCandidate && (r.absConcavityScore >= concavityScoreAutoThresh_);

        r.isConcavityPoint = r.isFinalSeamPoint;

        if (r.isFinalSeamPoint) {
            concavityPoints_->push_back(r.queryPoint);
        }
    }

    concavityPoints_->width = static_cast<uint32_t>(concavityPoints_->size());
    concavityPoints_->height = 1;
    concavityPoints_->is_dense = false;
    if (concavityPoints_->size() == 0) return false;

    if (debug_) {
        std::string path = "./data/seamDetWithPointCloud/tubePlateFilletSeamsDet/finalSeamPoints_" + std::to_string(areadex) + ".pcd";
        saveFinalSeamPointCloud(featureResults_, path);
    }
    if (debug_) {
        std::string path1 = "./data/seamDetWithPointCloud/tubePlateFilletSeamsDet/debug_meanDeviation_intensity_" + std::to_string(areadex) + ".pcd";
        saveMeanDeviationHeatmap(featureResults_, path1);

        std::string path2 =
            "./data/seamDetWithPointCloud/tubePlateFilletSeamsDet/debug_absConcavityScore_intensity_" + std::to_string(areadex) + ".pcd";
        saveConcavityScoreHeatmap(featureResults_, path2);

        // std::string csvPath = "./data/seamDetWithPointCloud/tubePlateFilletSeamsDet/debug_feature_curves_" + std::to_string(areadex) + ".csv";
        // saveFeatureCurvesCsv(featureResults_, csvPath);

        // std::string imgPath = "./data/seamDetWithPointCloud/tubePlateFilletSeamsDet/debug_feature_curves_" + std::to_string(areadex) + ".png";
        // saveFeatureCurvesImage(featureResults_, imgPath);
    }

    PLOGD << "meanDeviationAutoThresh = " << meanDeviationAutoThresh_ << ", concavityScoreAutoThresh = " << concavityScoreAutoThresh_
          << ", final seam points = " << concavityPoints_->size();

    areadex++;
    return true;
}
bool SeamConcavityExtractor::buildKdTree() {
    if (!inputCloud_ || inputCloud_->empty()) {
        return false;
    }

    if (!kdtree_) {
        kdtree_.reset(new pcl::search::KdTree<pcl::PointXYZ>());
    }

    kdtree_->setInputCloud(inputCloud_);
    return true;
}
bool SeamConcavityExtractor::processSingleQueryPoint(const pcl::PointXYZ& queryPoint, FeatureResult& result) {
    NeighborhoodData neighborhood;
    neighborhood.queryPoint = queryPoint;

    if (!extractNeighborhood(queryPoint, neighborhood)) {
        return false;
    }

    SphereProjectionData sphereData;
    sphereData.queryPoint = queryPoint;

    if (!computeSphereProjection(neighborhood, sphereData)) {
        return false;
    }

    PcaProjectionData pcaData;
    pcaData.queryPoint = queryPoint;

    if (!computePcaProjection(sphereData, pcaData)) {
        return false;
    }

    ReconstructedCurveData curveData;
    curveData.queryPoint = queryPoint;

    if (!generateStandardCircle2D(curveData)) {
        return false;
    }

    if (!reconstructCurveToSphere(sphereData, pcaData, curveData)) {
        return false;
    }

    float meanDeviation = 0.0f;
    if (!computeMeanDeviation(sphereData, pcaData, curveData, meanDeviation)) {
        return false;
    }

    float positiveRatio = 0.0f;
    float negativeRatio = 0.0f;
    float concavityScore = 0.0f;
    // =====  meanDeviation 做门控 =====
    if (meanDeviation >= meanDeviationThresh_) {
        if (!computeConcavityScore(neighborhood, sphereData, meanDeviation, positiveRatio, negativeRatio, concavityScore)) {
            return false;
        }
    } else {
        // 低偏差点：直接归零
        concavityScore = 0.0f;
        positiveRatio = 0.0f;
        negativeRatio = 0.0f;
    }

    result.queryPoint = queryPoint;
    result.referenceRadius = neighborhood.referenceRadius;

    result.meanDeviation = meanDeviation;
    result.positiveRatio = positiveRatio;
    result.negativeRatio = negativeRatio;
    result.concavityScore = concavityScore;
    result.absConcavityScore = std::fabs(concavityScore);

    result.centroid = sphereData.centroid;
    result.normal = sphereData.normal;
    result.meanVector = sphereData.meanVector;

    if (!debug_) {
        neighborhood.neighborIndices.clear();
        neighborhood.rawSqrDistances.clear();
        neighborhood.validSqrDistances.clear();
        sphereData.unitVectors.clear();
        sphereData.weights.clear();
        pcaData.centeredVectors.clear();
        pcaData.projected2DPoints.clear();
        curveData.fittedCircle2D.clear();
        curveData.reconstructedCurve3D.clear();
    }

    result.neighborhood = neighborhood;
    result.sphereProjection = sphereData;
    if (debug_) {
        result.pcaProjection = pcaData;
        result.reconstructedCurve = curveData;
    }

    result.isConcavityPoint = judgeConcavityPoint(result.meanDeviation, result.absConcavityScore);
    result.valid = true;

    return true;
}
bool SeamConcavityExtractor::processSingleQueryPointMeanDeviationOnly(const pcl::PointXYZ& queryPoint, FeatureResult& result) {
    NeighborhoodData neighborhood;
    neighborhood.queryPoint = queryPoint;

    if (!extractNeighborhood(queryPoint, neighborhood)) {
        return false;
    }

    SphereProjectionData sphereData;
    sphereData.queryPoint = queryPoint;
    if (!computeSphereProjection(neighborhood, sphereData)) {
        return false;
    }

    PcaProjectionData pcaData;
    pcaData.queryPoint = queryPoint;
    if (!computePcaProjection(sphereData, pcaData)) {
        return false;
    }

    ReconstructedCurveData curveData;
    curveData.queryPoint = queryPoint;
    if (!generateStandardCircle2D(curveData)) {
        return false;
    }

    if (!reconstructCurveToSphere(sphereData, pcaData, curveData)) {
        return false;
    }

    float meanDeviation = 0.0f;
    if (!computeMeanDeviation(sphereData, pcaData, curveData, meanDeviation)) {
        return false;
    }

    result.queryPoint = queryPoint;
    result.referenceRadius = neighborhood.referenceRadius;
    result.meanDeviation = meanDeviation;

    result.positiveRatio = 0.0f;
    result.negativeRatio = 0.0f;
    result.concavityScore = 0.0f;
    result.absConcavityScore = 0.0f;

    result.centroid = sphereData.centroid;
    result.normal = sphereData.normal;
    result.meanVector = sphereData.meanVector;

    if (!debug_) {
        neighborhood.neighborIndices.clear();
        neighborhood.rawSqrDistances.clear();
        neighborhood.validSqrDistances.clear();
        sphereData.unitVectors.clear();
        sphereData.weights.clear();
        pcaData.centeredVectors.clear();
        pcaData.projected2DPoints.clear();
        curveData.fittedCircle2D.clear();
        curveData.reconstructedCurve3D.clear();
    }

    result.neighborhood = neighborhood;
    result.sphereProjection = sphereData;
    if (debug_) {
        result.pcaProjection = pcaData;
        result.reconstructedCurve = curveData;
    }

    result.isMeanDeviationCandidate = false;
    result.isFinalSeamPoint = false;
    result.isConcavityPoint = false;
    result.valid = true;

    return true;
}
bool SeamConcavityExtractor::processSingleQueryPointConcavityOnly(FeatureResult& result) {
    if (!result.valid) {
        return false;
    }

    float positiveRatio = 0.0f;
    float negativeRatio = 0.0f;
    float concavityScore = 0.0f;

    if (!computeConcavityScore(result.neighborhood, result.sphereProjection, result.meanDeviation, positiveRatio, negativeRatio, concavityScore)) {
        return false;
    }

    result.positiveRatio = positiveRatio;
    result.negativeRatio = negativeRatio;
    result.concavityScore = concavityScore;
    result.absConcavityScore = std::fabs(concavityScore);

    return true;
}
bool SeamConcavityExtractor::extractNeighborhood(const pcl::PointXYZ& queryPoint, NeighborhoodData& neighborhood) {
    if (!kdtree_ || !inputCloud_ || inputCloud_->empty()) {
        return false;
    }

    if (neighborRadius_ <= 1e-6f) {
        return false;
    }

    neighborhood.referenceRadius = neighborRadius_;

    static thread_local std::vector<int> radiusIndices;
    static thread_local std::vector<float> radiusSqrDists;
    radiusIndices.clear();
    radiusSqrDists.clear();

    int radiusFound = kdtree_->radiusSearch(queryPoint, neighborRadius_, radiusIndices, radiusSqrDists);
    if (radiusFound < 5) {
        return false;
    }

    neighborhood.neighborIndices.clear();
    neighborhood.rawSqrDistances.clear();

    neighborhood.neighborIndices.reserve(radiusFound);
    neighborhood.rawSqrDistances.reserve(radiusFound);

    for (int i = 0; i < radiusFound; ++i) {
        int idx = radiusIndices[i];
        const auto& p = inputCloud_->points[idx];

        // 排除自己
        if (std::fabs(p.x - queryPoint.x) < 1e-6f && std::fabs(p.y - queryPoint.y) < 1e-6f && std::fabs(p.z - queryPoint.z) < 1e-6f) {
            continue;
        }

        neighborhood.neighborIndices.push_back(idx);
        neighborhood.rawSqrDistances.push_back(radiusSqrDists[i]);
    }

    if (static_cast<int>(neighborhood.neighborIndices.size()) < neighborhoodMinValidPoints_) {
        return false;
    }

    if (!filterValidNeighborhoodByQuantile(neighborhood)) {
        return false;
    }

    if (!clusterValidNeighborhoodKeepMaxCluster(neighborhood)) {
        return false;
    }

    return true;
}
bool SeamConcavityExtractor::filterValidNeighborhoodByQuantile(NeighborhoodData& neighborhood) {
    if (static_cast<int>(neighborhood.rawSqrDistances.size()) < neighborhoodMinValidPoints_) {
        return false;
    }

    if (quantileGamma_ >= 0.999f) {
        neighborhood.validNeighborIndices = neighborhood.neighborIndices;
        neighborhood.validSqrDistances = neighborhood.rawSqrDistances;
        return static_cast<int>(neighborhood.validNeighborIndices.size()) >= neighborhoodMinValidPoints_;
    }

    float gamma = quantileGamma_;
    gamma = std::max(0.05f, std::min(gamma, 0.95f));

    float qLow = 0.5f - gamma / 2.0f;
    float qHigh = 0.5f + gamma / 2.0f;

    qLow = std::max(0.0f, qLow);
    qHigh = std::min(1.0f, qHigh);

    float dlow2, dhigh2;
    computeTwoQuantiles(neighborhood.rawSqrDistances, qLow, qHigh, dlow2, dhigh2);

    neighborhood.validNeighborIndices.clear();
    neighborhood.validSqrDistances.clear();

    neighborhood.validNeighborIndices.reserve(neighborhood.neighborIndices.size());
    neighborhood.validSqrDistances.reserve(neighborhood.rawSqrDistances.size());

    for (size_t i = 0; i < neighborhood.neighborIndices.size(); ++i) {
        float dist2 = neighborhood.rawSqrDistances[i];
        if (dist2 >= dlow2 && dist2 <= dhigh2) {
            neighborhood.validNeighborIndices.push_back(neighborhood.neighborIndices[i]);
            neighborhood.validSqrDistances.push_back(dist2);
        }
    }

    return static_cast<int>(neighborhood.validNeighborIndices.size()) >= neighborhoodMinValidPoints_;
}
bool SeamConcavityExtractor::clusterValidNeighborhoodKeepMaxCluster(NeighborhoodData& neighborhood) {
    const int n = static_cast<int>(neighborhood.validNeighborIndices.size());
    if (n < neighborhoodMinValidPoints_) {
        return false;
    }

    if (neighborhoodClusterTolerance_ <= 1e-6f) {
        return true;  // 不聚类，直接通过
    }

    // ================= 1. 构建局部点云 =================
    pcl::PointCloud<pcl::PointXYZ>::Ptr localCloud(new pcl::PointCloud<pcl::PointXYZ>());
    localCloud->reserve(n);

    for (int idx : neighborhood.validNeighborIndices) {
        localCloud->push_back(inputCloud_->points[idx]);
    }

    localCloud->width = static_cast<uint32_t>(localCloud->size());
    localCloud->height = 1;
    localCloud->is_dense = true;

    if (localCloud->empty()) {
        return false;
    }

    // ================= 2. 构建 KDTree =================
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(localCloud);

    // ================= 3. 欧式聚类 =================
    std::vector<pcl::PointIndices> clusterIndices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(neighborhoodClusterTolerance_);  // 聚类半径
    ec.setMinClusterSize(neighborhoodMinValidPoints_);      // 最小聚类点数
    ec.setMaxClusterSize(n);                                // 最大聚类点数
    ec.setSearchMethod(tree);
    ec.setInputCloud(localCloud);
    ec.extract(clusterIndices);

    if (clusterIndices.empty()) {
        return false;
    }

    // ================= 4. 找离 queryPoint 最近的 seed 点 =================
    const pcl::PointXYZ& queryPoint = neighborhood.queryPoint;

    int seedIdx = -1;
    float minDist2 = std::numeric_limits<float>::max();

    for (int i = 0; i < n; ++i) {
        const auto& p = localCloud->points[i];

        float dx = p.x - queryPoint.x;
        float dy = p.y - queryPoint.y;
        float dz = p.z - queryPoint.z;
        float dist2 = dx * dx + dy * dy + dz * dz;

        if (dist2 < minDist2) {
            minDist2 = dist2;
            seedIdx = i;
        }
    }

    if (seedIdx < 0) {
        return false;
    }

    // ================= 5. 找 seed 属于哪个聚类 =================
    int targetClusterId = -1;

    for (int cid = 0; cid < static_cast<int>(clusterIndices.size()); ++cid) {
        const auto& inds = clusterIndices[cid].indices;
        if (std::find(inds.begin(), inds.end(), seedIdx) != inds.end()) {
            targetClusterId = cid;
            break;
        }
    }

    if (targetClusterId < 0) {
        return false;
    }

    const auto& component = clusterIndices[targetClusterId].indices;

    if (static_cast<int>(component.size()) < neighborhoodMinValidPoints_) {
        return false;
    }

    // ================= 6. 回写，只保留 seed 所在聚类 =================
    std::vector<int> keptIndices;
    std::vector<float> keptDistances;
    keptIndices.reserve(component.size());
    keptDistances.reserve(component.size());

    for (int localIdx : component) {
        keptIndices.push_back(neighborhood.validNeighborIndices[localIdx]);
        keptDistances.push_back(neighborhood.validSqrDistances[localIdx]);
    }

    neighborhood.validNeighborIndices.swap(keptIndices);
    neighborhood.validSqrDistances.swap(keptDistances);

    return static_cast<int>(neighborhood.validNeighborIndices.size()) >= neighborhoodMinValidPoints_;
}
bool SeamConcavityExtractor::computeSphereProjection(const NeighborhoodData& neighborhood, SphereProjectionData& sphereData) {
    if (!inputCloud_ || neighborhood.validNeighborIndices.size() < 5) {
        return false;
    }

    if (!computeCentroidAndNormal(neighborhood, sphereData)) {
        return false;
    }

    const Eigen::Vector3f query = toEigen(neighborhood.queryPoint);

    sphereData.unitVectors.clear();
    sphereData.weights.clear();
    sphereData.unitVectors.reserve(neighborhood.validNeighborIndices.size());
    sphereData.weights.reserve(neighborhood.validNeighborIndices.size());

    float medianDist = computeMedian(neighborhood.validSqrDistances);

    Eigen::Vector3f meanVec = Eigen::Vector3f::Zero();

    for (size_t i = 0; i < neighborhood.validNeighborIndices.size(); ++i) {
        int idx = neighborhood.validNeighborIndices[i];
        Eigen::Vector3f pj = toEigen(inputCloud_->points[idx]);
        Eigen::Vector3f diff = pj - query;

        float norm = diff.norm();
        if (norm < 1e-6f) continue;

        Eigen::Vector3f uhat = diff / norm;
        float dist = neighborhood.validSqrDistances[i];

        float w = std::exp(-weightAlpha_ * (dist - medianDist) * (dist - medianDist));

        sphereData.unitVectors.push_back(uhat);
        sphereData.weights.push_back(w);
        meanVec += uhat;
    }

    if (sphereData.unitVectors.size() < 5) {
        return false;
    }

    meanVec /= static_cast<float>(sphereData.unitVectors.size());
    sphereData.meanVector = meanVec;

    return true;
}
bool SeamConcavityExtractor::computeCentroidAndNormal(const NeighborhoodData& neighborhood, SphereProjectionData& sphereData) {
    if (!inputCloud_ || neighborhood.validNeighborIndices.size() < 5) {
        return false;
    }

    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();

    for (int idx : neighborhood.validNeighborIndices) {
        centroid += toEigen(inputCloud_->points[idx]);
    }

    centroid /= static_cast<float>(neighborhood.validNeighborIndices.size());
    sphereData.centroid = centroid;

    Eigen::Vector3f query = toEigen(neighborhood.queryPoint);
    Eigen::Vector3f normal = centroid - query;

    if (normal.norm() < 1e-6f) {
        return false;
    }

    sphereData.normal = normal.normalized();
    return true;
}
bool SeamConcavityExtractor::computePcaProjection(const SphereProjectionData& sphereData, PcaProjectionData& pcaData) {
    if (sphereData.unitVectors.size() < 5) {
        return false;
    }

    pcaData.centeredVectors.clear();
    pcaData.projected2DPoints.clear();
    if (debug_) {
        pcaData.centeredVectors.reserve(sphereData.unitVectors.size());
        pcaData.projected2DPoints.reserve(sphereData.unitVectors.size());
    }

    // 先按你原来的方式去中心化
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();

    for (const auto& uhat : sphereData.unitVectors) {
        Eigen::Vector3f centered = uhat - sphereData.meanVector;
        if (debug_) {
            pcaData.centeredVectors.push_back(centered);
        }

        cov += centered * centered.transpose();
    }

    // PCL 通常是降序：col(0)最大，col(1)次大
    cov /= static_cast<float>(sphereData.unitVectors.size() - 1);
    pcaData.covariance = cov;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    if (solver.info() != Eigen::Success) {
        return false;
    }

    pcaData.eigenValues = solver.eigenvalues();
    const Eigen::Matrix3f eigenVectors = solver.eigenvectors();

    // Eigen returns eigenvalues in ascending order.
    pcaData.v1 = eigenVectors.col(2).normalized();
    pcaData.v2 = eigenVectors.col(1).normalized();

    // 协方差矩阵（可选保存）
    // 投影到2D
    if (debug_) {
        for (const auto& centered : pcaData.centeredVectors) {
            float x = centered.dot(pcaData.v1);
            float y = centered.dot(pcaData.v2);
            pcaData.projected2DPoints.emplace_back(x, y);
        }
    }

    return !debug_ || pcaData.projected2DPoints.size() >= 5;
}
bool SeamConcavityExtractor::generateStandardCircle2D(ReconstructedCurveData& curveData) {
    curveData.fittedCircle2D.clear();

    if (curveSampleCount_ < 8) {
        return false;
    }

    if (debug_) {
        curveData.fittedCircle2D = getCachedStandardCircle2D(curveSampleCount_);
    }

    return true;
}
bool SeamConcavityExtractor::reconstructCurveToSphere(const SphereProjectionData& sphereData, const PcaProjectionData& pcaData,
                                                      ReconstructedCurveData& curveData) {
    if (curveSampleCount_ < 8) {
        return false;
    }

    const auto& circle2D = curveData.fittedCircle2D.empty() ? getCachedStandardCircle2D(curveSampleCount_) : curveData.fittedCircle2D;

    curveData.reconstructedCurve3D.clear();
    curveData.reconstructedCurve3D.reserve(circle2D.size());

    for (const auto& c2 : circle2D) {
        Eigen::Vector3f ck = c2.x() * pcaData.v1 + c2.y() * pcaData.v2;
        Eigen::Vector3f back = ck + sphereData.meanVector;

        float norm = back.norm();
        if (norm < 1e-6f) continue;

        curveData.reconstructedCurve3D.push_back(back / norm);
    }

    return curveData.reconstructedCurve3D.size() >= 8;
}
bool SeamConcavityExtractor::computeMeanDeviation(const SphereProjectionData& sphereData, const PcaProjectionData& pcaData,
                                                  const ReconstructedCurveData& curveData, float& meanDeviation) {
    if (sphereData.unitVectors.empty() || sphereData.weights.size() != sphereData.unitVectors.size() || curveData.reconstructedCurve3D.empty()) {
        return false;
    }

    double sum = 0.0;
    const int curveCount = static_cast<int>(curveData.reconstructedCurve3D.size());
    const bool useLocalCurveSearch = curveCount == curveSampleCount_ && curveCount >= 8 && pcaData.v1.squaredNorm() > 1e-6f && pcaData.v2.squaredNorm() > 1e-6f;
    const float invTwoPi = 1.0f / (2.0f * static_cast<float>(M_PI));

    for (size_t i = 0; i < sphereData.unitVectors.size(); ++i) {
        float ej = 0.0f;

        if (useLocalCurveSearch) {
            const Eigen::Vector3f centered = sphereData.unitVectors[i] - sphereData.meanVector;
            float theta = std::atan2(centered.dot(pcaData.v2), centered.dot(pcaData.v1));
            if (theta < 0.0f) {
                theta += 2.0f * static_cast<float>(M_PI);
            }

            const int centerIdx = static_cast<int>(std::round(theta * invTwoPi * static_cast<float>(curveCount))) % curveCount;
            float minDist2 = std::numeric_limits<float>::max();

            for (int offset = -2; offset <= 2; ++offset) {
                int idx = centerIdx + offset;
                if (idx < 0) {
                    idx += curveCount;
                } else if (idx >= curveCount) {
                    idx -= curveCount;
                }

                float dist2 = (sphereData.unitVectors[i] - curveData.reconstructedCurve3D[idx]).squaredNorm();
                if (dist2 < minDist2) {
                    minDist2 = dist2;
                }
            }

            ej = std::sqrt(minDist2);
        } else {
            ej = pointToCurveMinDistance(sphereData.unitVectors[i], curveData.reconstructedCurve3D);
        }

        sum += static_cast<double>(sphereData.weights[i]) * static_cast<double>(ej);
    }

    meanDeviation = static_cast<float>(sum / static_cast<double>(sphereData.unitVectors.size()));
    return std::isfinite(meanDeviation);
}
bool SeamConcavityExtractor::computeTwoQuantiles(const std::vector<float>& values, float qLow, float qHigh, float& outLow, float& outHigh) const {
    if (values.empty()) return false;

    qLow = std::max(0.0f, std::min(1.0f, qLow));
    qHigh = std::max(0.0f, std::min(1.0f, qHigh));
    if (qLow > qHigh) std::swap(qLow, qHigh);

    const int n = static_cast<int>(values.size());

    float posLow = qLow * (n - 1);
    float posHigh = qHigh * (n - 1);

    int idxLow0 = static_cast<int>(std::floor(posLow));
    int idxLow1 = static_cast<int>(std::ceil(posLow));

    int idxHigh0 = static_cast<int>(std::floor(posHigh));
    int idxHigh1 = static_cast<int>(std::ceil(posHigh));

    std::vector<float> tmp(values);

    // 先处理 low
    std::nth_element(tmp.begin(), tmp.begin() + idxLow0, tmp.end());
    float vLow0 = tmp[idxLow0];

    float vLow;
    if (idxLow0 == idxLow1) {
        vLow = vLow0;
    } else {
        std::nth_element(tmp.begin() + idxLow0, tmp.begin() + idxLow1, tmp.end());
        float vLow1 = tmp[idxLow1];
        float t = posLow - idxLow0;
        vLow = vLow0 * (1.0f - t) + vLow1 * t;
    }

    // 再处理 high
    std::nth_element(tmp.begin(), tmp.begin() + idxHigh0, tmp.end());
    float vHigh0 = tmp[idxHigh0];

    float vHigh;
    if (idxHigh0 == idxHigh1) {
        vHigh = vHigh0;
    } else {
        std::nth_element(tmp.begin() + idxHigh0, tmp.begin() + idxHigh1, tmp.end());
        float vHigh1 = tmp[idxHigh1];
        float t = posHigh - idxHigh0;
        vHigh = vHigh0 * (1.0f - t) + vHigh1 * t;
    }

    outLow = vLow;
    outHigh = vHigh;

    return true;
}
float SeamConcavityExtractor::computeMedian(std::vector<float> values) const {
    if (values.empty()) return 0.0f;

    size_t n = values.size();
    if (n % 2 == 1) {
        size_t mid = n / 2;
        std::nth_element(values.begin(), values.begin() + mid, values.end());
        return values[mid];
    }

    size_t hiIdx = n / 2;
    size_t loIdx = hiIdx - 1;
    std::nth_element(values.begin(), values.begin() + hiIdx, values.end());
    float hi = values[hiIdx];
    std::nth_element(values.begin(), values.begin() + loIdx, values.begin() + hiIdx);
    float lo = values[loIdx];
    return 0.5f * (lo + hi);
}
float SeamConcavityExtractor::pointToCurveMinDistance(const Eigen::Vector3f& point, const std::vector<Eigen::Vector3f>& curve) const {
    float minDist2 = std::numeric_limits<float>::max();

    for (const auto& c : curve) {
        float dist2 = (point - c).squaredNorm();
        if (dist2 < minDist2) {
            minDist2 = dist2;
        }
    }

    return std::sqrt(minDist2);
}
Eigen::Vector3f SeamConcavityExtractor::toEigen(const pcl::PointXYZ& p) const { return Eigen::Vector3f(p.x, p.y, p.z); }

pcl::PointXYZ SeamConcavityExtractor::toPcl(const Eigen::Vector3f& v) const {
    pcl::PointXYZ p;
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
    return p;
}

bool SeamConcavityExtractor::saveMeanDeviationHeatmap(const std::vector<FeatureResult>& results, const std::string& savePath) {
    if (results.empty()) {
        PLOGE << "saveMeanDeviationHeatmap: results 为空";
        return false;
    }

    float minDev = std::numeric_limits<float>::max();
    float maxDev = -std::numeric_limits<float>::max();
    int validCount = 0;

    pcl::PointCloud<pcl::PointXYZI>::Ptr intensityCloud(new pcl::PointCloud<pcl::PointXYZI>());
    intensityCloud->reserve(results.size());

    for (const auto& r : results) {
        if (!r.valid) continue;
        if (!std::isfinite(r.meanDeviation)) continue;

        minDev = std::min(minDev, r.meanDeviation);
        maxDev = std::max(maxDev, r.meanDeviation);
        validCount++;

        pcl::PointXYZI p;
        p.x = r.queryPoint.x;
        p.y = r.queryPoint.y;
        p.z = r.queryPoint.z;
        p.intensity = r.meanDeviation;  // 直接存偏差值
        intensityCloud->points.push_back(p);
    }

    if (validCount == 0) {
        PLOGE << "saveMeanDeviationHeatmap: 没有有效 meanDeviation";
        return false;
    }

    if (!std::isfinite(minDev) || !std::isfinite(maxDev)) {
        PLOGE << "saveMeanDeviationHeatmap: min/max 无效";
        return false;
    }

    intensityCloud->width = static_cast<uint32_t>(intensityCloud->size());
    intensityCloud->height = 1;
    intensityCloud->is_dense = false;

    int ret = pcl::io::savePCDFileBinary(savePath, *intensityCloud);
    if (ret != 0) {
        PLOGE << "saveMeanDeviationHeatmap: 保存Intensity点云失败 " << savePath;
        return false;
    }

    PLOGD << "saveMeanDeviationHeatmap: 已保存强度图 " << savePath << ", validCount = " << validCount << ", minDev = " << minDev
          << ", maxDev = " << maxDev << ", intensity = meanDeviation";

    return true;
}
bool SeamConcavityExtractor::saveConcavityScoreHeatmap(const std::vector<FeatureResult>& results, const std::string& savePath) {
    if (results.empty()) {
        PLOGE << "saveConcavityScoreHeatmap: results 为空";
        return false;
    }

    float minScore = std::numeric_limits<float>::max();
    float maxScore = -std::numeric_limits<float>::max();
    int validCount = 0;

    pcl::PointCloud<pcl::PointXYZI>::Ptr intensityCloud(new pcl::PointCloud<pcl::PointXYZI>());
    intensityCloud->reserve(results.size());

    for (const auto& r : results) {
        if (!r.valid) continue;

        float v = r.absConcavityScore;
        if (!std::isfinite(v)) continue;

        minScore = std::min(minScore, v);
        maxScore = std::max(maxScore, v);
        validCount++;

        pcl::PointXYZI p;
        p.x = r.queryPoint.x;
        p.y = r.queryPoint.y;
        p.z = r.queryPoint.z;
        p.intensity = v;  // 直接存 absConcavityScore
        intensityCloud->points.push_back(p);
    }

    if (validCount == 0) {
        PLOGE << "saveConcavityScoreHeatmap: 没有有效 absConcavityScore";
        return false;
    }

    if (!std::isfinite(minScore) || !std::isfinite(maxScore)) {
        PLOGE << "saveConcavityScoreHeatmap: min/max 无效";
        return false;
    }

    intensityCloud->width = static_cast<uint32_t>(intensityCloud->size());
    intensityCloud->height = 1;
    intensityCloud->is_dense = false;

    int ret = pcl::io::savePCDFileBinary(savePath, *intensityCloud);
    if (ret != 0) {
        PLOGE << "saveConcavityScoreHeatmap: 保存Intensity点云失败 " << savePath;
        return false;
    }

    PLOGD << "saveConcavityScoreHeatmap: 已保存强度图 " << savePath << ", validCount = " << validCount << ", minScore = " << minScore
          << ", maxScore = " << maxScore << ", intensity = absConcavityScore";

    return true;
}

bool SeamConcavityExtractor::saveFeatureCurvesCsv(const std::vector<FeatureResult>& results, const std::string& savePath) {
    std::ofstream ofs(savePath);
    if (!ofs.is_open()) {
        PLOGE << "saveFeatureCurvesCsv: 打开失败 " << savePath;
        return false;
    }

    std::vector<const FeatureResult*> sorted;
    sorted.reserve(results.size());

    for (const auto& r : results) {
        if (!r.valid) continue;
        if (!std::isfinite(r.meanDeviation) || !std::isfinite(r.absConcavityScore)) continue;
        sorted.push_back(&r);
    }

    std::sort(sorted.begin(), sorted.end(), [](const FeatureResult* a, const FeatureResult* b) { return a->meanDeviation < b->meanDeviation; });

    ofs << "sorted_index,x,y,z,valid,meanDeviation,concavityScore,absConcavityScore,positiveRatio,negativeRatio\n";

    for (size_t i = 0; i < sorted.size(); ++i) {
        const auto& r = *sorted[i];
        ofs << i << "," << r.queryPoint.x << "," << r.queryPoint.y << "," << r.queryPoint.z << "," << 1 << "," << r.meanDeviation << ","
            << r.concavityScore << "," << r.absConcavityScore << "," << r.positiveRatio << "," << r.negativeRatio << "\n";
    }

    ofs.close();
    PLOGD << "saveFeatureCurvesCsv: 已保存 " << savePath;
    return true;
}
bool SeamConcavityExtractor::saveFeatureCurvesImage(const std::vector<FeatureResult>& results, const std::string& savePath) {
    if (results.empty()) return false;

    std::vector<float> meanVals, scoreVals;

    for (const auto& r : results) {
        if (!r.valid) continue;
        if (!std::isfinite(r.meanDeviation) || !std::isfinite(r.absConcavityScore)) continue;

        meanVals.push_back(r.meanDeviation);
        scoreVals.push_back(r.absConcavityScore);
    }

    if (meanVals.size() < 10) return false;

    // 排序
    std::vector<int> idx(meanVals.size());
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(), [&](int a, int b) { return meanVals[a] < meanVals[b]; });

    std::vector<float> meanSorted, scoreSorted;
    for (int i : idx) {
        meanSorted.push_back(meanVals[i]);
        scoreSorted.push_back(scoreVals[i]);
    }

    float meanMin = meanSorted.front();
    float meanMax = meanSorted.back();
    float scoreMin = *std::min_element(scoreSorted.begin(), scoreSorted.end());
    float scoreMax = *std::max_element(scoreSorted.begin(), scoreSorted.end());

    float meanRange = std::max(meanMax - meanMin, 1e-6f);
    float scoreRange = std::max(scoreMax - scoreMin, 1e-6f);

    // ================= 分桶 =================
    const int BIN = 80;
    std::vector<int> hist(BIN, 0);
    std::vector<float> scoreSum(BIN, 0);

    for (size_t i = 0; i < meanSorted.size(); ++i) {
        int b = (int)((meanSorted[i] - meanMin) / meanRange * (BIN - 1));
        b = std::max(0, std::min(BIN - 1, b));
        hist[b]++;
        scoreSum[b] += scoreSorted[i];
    }

    std::vector<float> scoreAvg(BIN, 0);
    for (int i = 0; i < BIN; ++i)
        if (hist[i] > 0) scoreAvg[i] = scoreSum[i] / hist[i];

    int maxCount = std::max(1, *std::max_element(hist.begin(), hist.end()));

    // ================= 画布 =================
    int W = 1600, H = 900;
    cv::Mat img(H, W, CV_8UC3, cv::Scalar(255, 255, 255));

    cv::Rect plot1(120, 100, 600, 650);
    cv::Rect plot2(880, 100, 600, 650);

    cv::rectangle(img, plot1, cv::Scalar(0, 0, 0), 1);
    cv::rectangle(img, plot2, cv::Scalar(0, 0, 0), 1);

    // ================= 图1：count vs deviation =================
    for (int i = 1; i < BIN; ++i) {
        float t1 = (float)(i - 1) / (BIN - 1);
        float t2 = (float)i / (BIN - 1);

        float dev1 = meanMin + t1 * meanRange;
        float dev2 = meanMin + t2 * meanRange;

        int y1 = plot1.y + plot1.height - (int)(t1 * plot1.height);
        int y2 = plot1.y + plot1.height - (int)(t2 * plot1.height);

        int x1 = plot1.x + (int)((float)hist[i - 1] / maxCount * plot1.width);
        int x2 = plot1.x + (int)((float)hist[i] / maxCount * plot1.width);

        cv::line(img, {x1, y1}, {x2, y2}, cv::Scalar(255, 0, 0), 2);
    }

    // ================= 图2：deviation vs score =================
    for (int i = 1; i < BIN; ++i) {
        float dev1 = meanMin + (float)(i - 1) / (BIN - 1) * meanRange;
        float dev2 = meanMin + (float)i / (BIN - 1) * meanRange;

        int x1 = plot2.x + (int)((dev1 - meanMin) / meanRange * plot2.width);
        int x2 = plot2.x + (int)((dev2 - meanMin) / meanRange * plot2.width);

        int y1 = plot2.y + plot2.height - (int)((scoreAvg[i - 1] - scoreMin) / scoreRange * plot2.height);
        int y2 = plot2.y + plot2.height - (int)((scoreAvg[i] - scoreMin) / scoreRange * plot2.height);

        cv::line(img, {x1, y1}, {x2, y2}, cv::Scalar(0, 0, 255), 2);
    }

    // ================= 坐标轴刻度 =================
    int ticks = 5;

    // 图1 Y轴（Deviation）
    for (int i = 0; i <= ticks; i++) {
        float t = (float)i / ticks;
        float val = meanMax - t * meanRange;
        int y = plot1.y + t * plot1.height;

        cv::putText(img, cv::format("%.4f", val), {plot1.x - 100, y + 5}, 0, 0.5, {0, 0, 0}, 1);
    }

    // 图1 X轴（Count）
    for (int i = 0; i <= ticks; i++) {
        float t = (float)i / ticks;
        int val = t * maxCount;
        int x = plot1.x + t * plot1.width;

        cv::putText(img, std::to_string(val), {x - 10, plot1.y + plot1.height + 25}, 0, 0.5, {0, 0, 0}, 1);
    }

    // 图2 X轴（Deviation）
    for (int i = 0; i <= ticks; i++) {
        float t = (float)i / ticks;
        float val = meanMin + t * meanRange;
        int x = plot2.x + t * plot2.width;

        cv::putText(img, cv::format("%.4f", val), {x - 20, plot2.y + plot2.height + 25}, 0, 0.5, {0, 0, 0}, 1);
    }

    // 图2 Y轴（Score）
    for (int i = 0; i <= ticks; i++) {
        float t = (float)i / ticks;
        float val = scoreMax - t * scoreRange;
        int y = plot2.y + t * plot2.height;

        cv::putText(img, cv::format("%.4f", val), {plot2.x - 100, y + 5}, 0, 0.5, {0, 0, 0}, 1);
    }

    // ================= 标题 =================
    cv::putText(img, "Count vs Deviation", {200, 60}, 0, 0.8, {0, 0, 0}, 2);
    cv::putText(img, "Deviation vs Score", {980, 60}, 0, 0.8, {0, 0, 0}, 2);

    cv::putText(img, "Count", {350, 850}, 0, 0.7, {255, 0, 0}, 2);
    cv::putText(img, "Deviation", {20, 400}, 0, 0.7, {0, 0, 0}, 2);

    cv::putText(img, "Deviation", {1050, 850}, 0, 0.7, {0, 0, 0}, 2);
    cv::putText(img, "Score", {780, 400}, 0, 0.7, {0, 0, 255}, 2);

    cv::imwrite(savePath, img);

    return true;
}

pcl::PointXYZ SeamConcavityExtractor::findNearestQueryPoint(const pcl::PointXYZ& targetPoint, int& nearestIdx, float& nearestDist) const {
    nearestIdx = -1;
    nearestDist = std::numeric_limits<float>::max();

    pcl::PointXYZ nearestPt;

    if (!queryPoints_ || queryPoints_->empty()) {
        return nearestPt;
    }

    for (int i = 0; i < static_cast<int>(queryPoints_->size()); ++i) {
        const auto& p = queryPoints_->points[i];
        float dx = p.x - targetPoint.x;
        float dy = p.y - targetPoint.y;
        float dz = p.z - targetPoint.z;
        float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (dist < nearestDist) {
            nearestDist = dist;
            nearestIdx = i;
            nearestPt = p;
        }
    }

    return nearestPt;
}
bool SeamConcavityExtractor::debugSaveSingleQueryProcess(const pcl::PointXYZ& targetPoint, const std::string& saveDir) {
    int nearestIdx = -1;
    float nearestDist = 0.0f;
    pcl::PointXYZ queryPoint = findNearestQueryPoint(targetPoint, nearestIdx, nearestDist);

    if (nearestIdx < 0) {
        PLOGE << "debugSaveSingleQueryProcess: 未找到最近查询点";
        return false;
    }
    // ================= 新增：距离阈值判断 =================
    const float maxAllowDist = 1.0f;  // 1mm

    if (nearestIdx < 0 || nearestDist > maxAllowDist) {
        PLOGW << "debugSaveSingleQueryProcess: 最近点距离过大，跳过";
        PLOGW << "nearestDist = " << nearestDist;
        return false;
    }

    PLOGD << "===== DEBUG SINGLE QUERY =====";
    PLOGD << "targetPoint = (" << targetPoint.x << ", " << targetPoint.y << ", " << targetPoint.z << ")";
    PLOGD << "nearestIdx   = " << nearestIdx;
    PLOGD << "queryPoint   = (" << queryPoint.x << ", " << queryPoint.y << ", " << queryPoint.z << ")";
    PLOGD << "nearestDist  = " << nearestDist;

    FeatureResult result;
    result.queryPoint = queryPoint;

    bool ok = processSingleQueryPoint(queryPoint, result);
    result.valid = ok;

    if (!ok) {
        PLOGE << "debugSaveSingleQueryProcess: processSingleQueryPoint 失败";
        return false;
    }

    saveNeighborhoodClouds(result, saveDir);
    savePcaVisualizationClouds(result, saveDir);
    saveReconstructedCurveCloud(result, saveDir);
    saveDebugTextReport(result, saveDir);

    return true;
}
bool SeamConcavityExtractor::saveNeighborhoodClouds(const FeatureResult& result, const std::string& saveDir) {
    // 查询点
    pcl::PointCloud<pcl::PointXYZ>::Ptr queryCloud(new pcl::PointCloud<pcl::PointXYZ>());
    queryCloud->push_back(result.queryPoint);
    queryCloud->width = 1;
    queryCloud->height = 1;
    pcl::io::savePCDFile(saveDir + "_queryPoint.pcd", *queryCloud);

    // 原始邻域
    pcl::PointCloud<pcl::PointXYZ>::Ptr rawCloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (int idx : result.neighborhood.neighborIndices) {
        rawCloud->push_back(inputCloud_->points[idx]);
    }
    rawCloud->width = static_cast<uint32_t>(rawCloud->size());
    rawCloud->height = 1;
    pcl::io::savePCDFile(saveDir + "_rawNeighborhood.pcd", *rawCloud);

    // 有效邻域
    pcl::PointCloud<pcl::PointXYZ>::Ptr validCloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (int idx : result.neighborhood.validNeighborIndices) {
        validCloud->push_back(inputCloud_->points[idx]);
    }
    validCloud->width = static_cast<uint32_t>(validCloud->size());
    validCloud->height = 1;
    pcl::io::savePCDFile(saveDir + "_validNeighborhood.pcd", *validCloud);

    return true;
}
bool SeamConcavityExtractor::savePcaVisualizationClouds(const FeatureResult& result, const std::string& saveDir) {
    const Eigen::Vector3f query = toEigen(result.queryPoint);

    // 1) centeredVectors 点云：这是去中心化向量，显示时要平移回 queryPoint
    pcl::PointCloud<pcl::PointXYZ>::Ptr centeredCloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& v : result.pcaProjection.centeredVectors) {
        Eigen::Vector3f p3 = query + v;

        pcl::PointXYZ p;
        p.x = p3.x();
        p.y = p3.y();
        p.z = p3.z();
        centeredCloud->push_back(p);
    }
    centeredCloud->width = static_cast<uint32_t>(centeredCloud->size());
    centeredCloud->height = 1;
    centeredCloud->is_dense = true;
    pcl::io::savePCDFile(saveDir + "_centeredVectors.pcd", *centeredCloud);

    // 2) PCA 2D点投回3D平面
    // projected2DPoints 本质上还是局部坐标，要先恢复成3D向量，再平移到 queryPoint
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcaPlaneCloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& pt2 : result.pcaProjection.projected2DPoints) {
        Eigen::Vector3f localVec = pt2.x() * result.pcaProjection.v1 + pt2.y() * result.pcaProjection.v2;

        // 这里建议把 meanVector 加回去，再平移到 queryPoint
        Eigen::Vector3f p3 = query + result.sphereProjection.meanVector + localVec;

        pcl::PointXYZ p;
        p.x = p3.x();
        p.y = p3.y();
        p.z = p3.z();
        pcaPlaneCloud->push_back(p);
    }
    pcaPlaneCloud->width = static_cast<uint32_t>(pcaPlaneCloud->size());
    pcaPlaneCloud->height = 1;
    pcaPlaneCloud->is_dense = true;
    pcl::io::savePCDFile(saveDir + "_pcaPlanePoints.pcd", *pcaPlaneCloud);

    // 3) 保存 v1 / v2 方向线
    pcl::PointCloud<pcl::PointXYZ>::Ptr axisCloud(new pcl::PointCloud<pcl::PointXYZ>());
    const float lineLen = 1.0f;

    // 轴线锚点也别放原点，放到 query + meanVector 附近更合理
    Eigen::Vector3f axisOrigin = query + result.sphereProjection.meanVector;

    for (int i = -20; i <= 20; ++i) {
        float t = lineLen * static_cast<float>(i) / 20.0f;

        Eigen::Vector3f p1 = axisOrigin + t * result.pcaProjection.v1;
        Eigen::Vector3f p2 = axisOrigin + t * result.pcaProjection.v2;

        pcl::PointXYZ q1, q2;
        q1.x = p1.x();
        q1.y = p1.y();
        q1.z = p1.z();

        q2.x = p2.x();
        q2.y = p2.y();
        q2.z = p2.z();

        axisCloud->push_back(q1);
        axisCloud->push_back(q2);
    }
    axisCloud->width = static_cast<uint32_t>(axisCloud->size());
    axisCloud->height = 1;
    axisCloud->is_dense = true;
    pcl::io::savePCDFile(saveDir + "_pcaAxes.pcd", *axisCloud);

    return true;
}
bool SeamConcavityExtractor::saveReconstructedCurveCloud(const FeatureResult& result, const std::string& saveDir) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr curveCloud(new pcl::PointCloud<pcl::PointXYZ>());

    const Eigen::Vector3f query = toEigen(result.queryPoint);

    // 用 referenceRadius 作为显示尺度更直观
    float showRadius = result.referenceRadius;
    if (showRadius < 1e-6f) {
        showRadius = 1.0f;
    }

    for (const auto& v : result.reconstructedCurve.reconstructedCurve3D) {
        Eigen::Vector3f p3 = query + showRadius * v;

        pcl::PointXYZ p;
        p.x = p3.x();
        p.y = p3.y();
        p.z = p3.z();
        curveCloud->push_back(p);
    }

    curveCloud->width = static_cast<uint32_t>(curveCloud->size());
    curveCloud->height = 1;
    curveCloud->is_dense = true;
    pcl::io::savePCDFile(saveDir + "_reconstructedCurve.pcd", *curveCloud);

    return true;
}

bool SeamConcavityExtractor::saveDebugTextReport(const FeatureResult& result, const std::string& saveDir) {
    std::ofstream ofs(saveDir + "_report.txt");
    if (!ofs.is_open()) {
        return false;
    }

    ofs << "queryPoint: " << result.queryPoint.x << ", " << result.queryPoint.y << ", " << result.queryPoint.z << "\n";

    ofs << "referenceRadius: " << result.referenceRadius << "\n";
    ofs << "meanDeviation: " << result.meanDeviation << "\n";

    ofs << "centroid: " << result.centroid.x() << ", " << result.centroid.y() << ", " << result.centroid.z() << "\n";

    ofs << "normal: " << result.normal.x() << ", " << result.normal.y() << ", " << result.normal.z() << "\n";

    ofs << "meanVector: " << result.meanVector.x() << ", " << result.meanVector.y() << ", " << result.meanVector.z() << "\n";

    ofs << "eigenValues: " << result.pcaProjection.eigenValues[0] << ", " << result.pcaProjection.eigenValues[1] << ", "
        << result.pcaProjection.eigenValues[2] << "\n";

    ofs << "v1: " << result.pcaProjection.v1.x() << ", " << result.pcaProjection.v1.y() << ", " << result.pcaProjection.v1.z() << "\n";

    ofs << "v2: " << result.pcaProjection.v2.x() << ", " << result.pcaProjection.v2.y() << ", " << result.pcaProjection.v2.z() << "\n";

    ofs << "rawNeighborCount: " << result.neighborhood.neighborIndices.size() << "\n";
    ofs << "validNeighborCount: " << result.neighborhood.validNeighborIndices.size() << "\n";

    ofs.close();
    return true;
}
bool SeamConcavityExtractor::computeConcavityScore(const NeighborhoodData& neighborhood, const SphereProjectionData& sphereData, float meanDeviation,
                                                   float& positiveRatio, float& negativeRatio, float& concavityScore) {
    positiveRatio = 0.0f;
    negativeRatio = 0.0f;
    concavityScore = 0.0f;

    if (!inputCloud_) {
        return false;
    }

    const int M = static_cast<int>(neighborhood.validNeighborIndices.size());
    if (M < neighborhoodMinValidPoints_) {
        return false;
    }

    Eigen::Vector3f normal = sphereData.normal;

    float normalNorm = normal.norm();
    if (normalNorm < 1e-6f) {
        return false;
    }
    normal /= normalNorm;

    const float qx = neighborhood.queryPoint.x;
    const float qy = neighborhood.queryPoint.y;
    const float qz = neighborhood.queryPoint.z;
    const float nx = normal.x();
    const float ny = normal.y();
    const float nz = normal.z();

    int positiveCount = 0;
    int negativeCount = 0;

    for (int idx : neighborhood.validNeighborIndices) {
        const auto& p = inputCloud_->points[idx];
        float dotVal = (p.x - qx) * nx + (p.y - qy) * ny + (p.z - qz) * nz;

        if (dotVal > dotSignEps_) {
            positiveCount++;
        } else if (dotVal < -dotSignEps_) {
            negativeCount++;
        }
        // 落在 [-eps, eps] 内的近零点不参与正负计数
    }

    positiveRatio = static_cast<float>(positiveCount) / static_cast<float>(M);
    negativeRatio = static_cast<float>(negativeCount) / static_cast<float>(M);

    // 论文公式：concavity_score = (r- - r+) * (1 + e)
    concavityScore = (negativeRatio - positiveRatio) * (1.0f + meanDeviation);

    return std::isfinite(positiveRatio) && std::isfinite(negativeRatio) && std::isfinite(concavityScore);
}
bool SeamConcavityExtractor::judgeConcavityPoint(float meanDeviation, float concavityScoreAbs) const {
    if (!std::isfinite(meanDeviation) || !std::isfinite(concavityScoreAbs)) {
        return false;
    }

    return (meanDeviation >= meanDeviationThresh_) && (concavityScoreAbs >= concavityScoreThresh_);
}
float SeamConcavityExtractor::estimateMeanDeviationThreshold(const std::vector<FeatureResult>& results) const {
    std::vector<float> vals;
    vals.reserve(results.size());

    for (const auto& r : results) {
        if (!r.valid) continue;
        if (!std::isfinite(r.meanDeviation)) continue;
        vals.push_back(r.meanDeviation);
    }

    if (vals.empty()) {
        return meanDeviationThresh_;
    }

    float med = computeMedian(vals);

    std::vector<float> absDev;
    absDev.reserve(vals.size());
    for (float v : vals) {
        absDev.push_back(std::fabs(v - med));
    }

    float mad = computeMedian(absDev);
    float robustSigma = 1.4826f * mad;

    float thresh = med + autoThreshMadScale_ * robustSigma;

    if (!std::isfinite(thresh) || thresh <= 0.0f) {
        thresh = meanDeviationThresh_;
    }

    return thresh;
}
float SeamConcavityExtractor::estimateConcavityThreshold(const std::vector<FeatureResult>& results) const {
    std::vector<float> vals;
    vals.reserve(results.size());

    for (const auto& r : results) {
        if (!r.valid) continue;
        // if (!r.isMeanDeviationCandidate) continue;
        if (!std::isfinite(r.absConcavityScore)) continue;
        vals.push_back(r.absConcavityScore);
    }

    if (vals.empty()) {
        return concavityScoreThresh_;
    }

    float med = computeMedian(vals);

    std::vector<float> absDev;
    absDev.reserve(vals.size());
    for (float v : vals) {
        absDev.push_back(std::fabs(v - med));
    }

    float mad = computeMedian(absDev);
    float robustSigma = 1.4826f * mad;

    float thresh = med + autoThreshMadScale_ * robustSigma;

    if (!std::isfinite(thresh) || thresh <= 0.0f) {
        thresh = concavityScoreThresh_;
    }

    return thresh;
}
bool SeamConcavityExtractor::saveMeanDeviationCandidateCloud(const std::vector<FeatureResult>& results, const std::string& savePath) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

    float minVal = std::numeric_limits<float>::max();
    float maxVal = -std::numeric_limits<float>::max();
    int validCount = 0;

    for (const auto& r : results) {
        if (!r.valid) continue;
        if (!r.isMeanDeviationCandidate) continue;
        if (!std::isfinite(r.meanDeviation)) continue;

        pcl::PointXYZI p;
        p.x = r.queryPoint.x;
        p.y = r.queryPoint.y;
        p.z = r.queryPoint.z;
        p.intensity = r.meanDeviation;  // 候选点强度 = meanDeviation
        cloud->push_back(p);

        minVal = std::min(minVal, r.meanDeviation);
        maxVal = std::max(maxVal, r.meanDeviation);
        validCount++;
    }

    if (validCount == 0) {
        PLOGE << "saveMeanDeviationCandidateCloud: 没有有效候选点";
        return false;
    }

    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = false;

    int ret = pcl::io::savePCDFileBinary(savePath, *cloud);
    if (ret != 0) {
        PLOGE << "saveMeanDeviationCandidateCloud: 保存失败 " << savePath;
        return false;
    }

    PLOGD << "saveMeanDeviationCandidateCloud: 已保存强度图 " << savePath << ", validCount = " << validCount << ", minMeanDeviation = " << minVal
          << ", maxMeanDeviation = " << maxVal << ", intensity = meanDeviation";

    return true;
}

bool SeamConcavityExtractor::saveFinalSeamPointCloud(const std::vector<FeatureResult>& results, const std::string& savePath) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

    float minVal = std::numeric_limits<float>::max();
    float maxVal = -std::numeric_limits<float>::max();
    int validCount = 0;

    for (const auto& r : results) {
        if (!r.valid) continue;
        if (!r.isFinalSeamPoint) continue;
        if (!std::isfinite(r.absConcavityScore)) continue;

        pcl::PointXYZI p;
        p.x = r.queryPoint.x;
        p.y = r.queryPoint.y;
        p.z = r.queryPoint.z;
        p.intensity = r.absConcavityScore;  // 最终焊缝点强度 = absConcavityScore
        cloud->push_back(p);

        minVal = std::min(minVal, r.absConcavityScore);
        maxVal = std::max(maxVal, r.absConcavityScore);
        validCount++;
    }

    if (validCount == 0) {
        PLOGE << "saveFinalSeamPointCloud: 没有有效最终焊缝点";
        return false;
    }

    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = false;

    int ret = pcl::io::savePCDFileBinary(savePath, *cloud);
    if (ret != 0) {
        PLOGE << "saveFinalSeamPointCloud: 保存失败 " << savePath;
        return false;
    }

    PLOGD << "saveFinalSeamPointCloud: 已保存强度图 " << savePath << ", validCount = " << validCount << ", minAbsConcavityScore = " << minVal
          << ", maxAbsConcavityScore = " << maxVal << ", intensity = absConcavityScore";

    return true;
}
