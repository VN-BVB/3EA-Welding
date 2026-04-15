#include "SeamConcavityExtractor.h"
SeamConcavityExtractor::SeamConcavityExtractor() {
    inputCloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    queryPoints_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    concavityPoints_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    kdtree_.reset(new pcl::search::KdTree<pcl::PointXYZ>());
}

void SeamConcavityExtractor::setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (!inputCloud_) {
        inputCloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    }

    inputCloud_->clear();

    if (!cloud || cloud->empty()) {
        return;
    }

    *inputCloud_ = *cloud;
}

void SeamConcavityExtractor::setQueryPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (!queryPoints_) {
        queryPoints_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    }

    queryPoints_->clear();

    if (!cloud || cloud->empty()) {
        return;
    }

    *queryPoints_ = *cloud;
}
void SeamConcavityExtractor::setDebug(bool enable) { debug_ = enable; }
pcl::PointCloud<pcl::PointXYZ>::Ptr SeamConcavityExtractor::getConcavityPoints() const { return concavityPoints_; }
void SeamConcavityExtractor::setNeighborRadius(float radius) { neighborRadius_ = radius; }
const std::vector<FeatureResult>& SeamConcavityExtractor::getAllFeatureResults() const { return featureResults_; }
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

    featureResults_.clear();
    featureResults_.reserve(queryPoints_->size());

    if (!buildKdTree()) {
        return false;
    }
    for (const auto& queryPoint : queryPoints_->points) {
        FeatureResult result;
        result.queryPoint = queryPoint;

        bool ok = processSingleQueryPoint(queryPoint, result);
        result.valid = ok;

        featureResults_.push_back(result);
    }

    concavityPoints_->width = static_cast<uint32_t>(concavityPoints_->size());
    concavityPoints_->height = 1;
    concavityPoints_->is_dense = false;

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
    if (!computeMeanDeviation(sphereData, curveData, meanDeviation)) {
        return false;
    }

    result.queryPoint = queryPoint;
    result.referenceRadius = neighborhood.referenceRadius;
    result.meanDeviation = meanDeviation;

    result.centroid = sphereData.centroid;
    result.normal = sphereData.normal;
    result.meanVector = sphereData.meanVector;

    result.neighborhood = neighborhood;
    result.sphereProjection = sphereData;
    result.pcaProjection = pcaData;
    result.reconstructedCurve = curveData;

    result.valid = true;
    result.isConcavityPoint = false;  // 这一版先不判定

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

    std::vector<int> radiusIndices;
    std::vector<float> radiusSqrDists;

    int radiusFound = kdtree_->radiusSearch(queryPoint, neighborRadius_, radiusIndices, radiusSqrDists);
    if (radiusFound < 5) {
        return false;
    }

    neighborhood.neighborIndices.clear();
    neighborhood.rawDistances.clear();

    for (int i = 0; i < radiusFound; ++i) {
        int idx = radiusIndices[i];
        const auto& p = inputCloud_->points[idx];

        // 排除自己
        if (std::fabs(p.x - queryPoint.x) < 1e-6f && std::fabs(p.y - queryPoint.y) < 1e-6f && std::fabs(p.z - queryPoint.z) < 1e-6f) {
            continue;
        }

        neighborhood.neighborIndices.push_back(idx);
        neighborhood.rawDistances.push_back(std::sqrt(radiusSqrDists[i]));
    }

    if (neighborhood.neighborIndices.size() < 5) {
        return false;
    }

    return filterValidNeighborhoodByQuantile(neighborhood);
}
bool SeamConcavityExtractor::filterValidNeighborhoodByQuantile(NeighborhoodData& neighborhood) {
    if (neighborhood.rawDistances.size() < 5) {
        return false;
    }

    float gamma = quantileGamma_;

    float qLow = 0.5f - gamma / 2.0f;
    float qHigh = 0.5f + gamma / 2.0f;

    float dlow = computeDistanceQuantile(neighborhood.rawDistances, qLow);
    float dhigh = computeDistanceQuantile(neighborhood.rawDistances, qHigh);

    neighborhood.validNeighborIndices.clear();
    neighborhood.validDistances.clear();

    for (size_t i = 0; i < neighborhood.neighborIndices.size(); ++i) {
        float dist = neighborhood.rawDistances[i];
        if (dist >= dlow && dist <= dhigh) {
            neighborhood.validNeighborIndices.push_back(neighborhood.neighborIndices[i]);
            neighborhood.validDistances.push_back(dist);
        }
    }

    return neighborhood.validNeighborIndices.size() >= 5;
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

    std::vector<float> dvalid = neighborhood.validDistances;
    float medianDist = computeMedian(dvalid);

    Eigen::Vector3f meanVec = Eigen::Vector3f::Zero();

    for (size_t i = 0; i < neighborhood.validNeighborIndices.size(); ++i) {
        int idx = neighborhood.validNeighborIndices[i];
        Eigen::Vector3f pj = toEigen(inputCloud_->points[idx]);
        Eigen::Vector3f diff = pj - query;

        float norm = diff.norm();
        if (norm < 1e-6f) continue;

        Eigen::Vector3f uhat = diff / norm;
        float dist = neighborhood.validDistances[i];

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

    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();

    for (const auto& uhat : sphereData.unitVectors) {
        Eigen::Vector3f centered = uhat - sphereData.meanVector;
        pcaData.centeredVectors.push_back(centered);
        cov += centered * centered.transpose();
    }

    cov /= static_cast<float>(sphereData.unitVectors.size() - 1);
    pcaData.covariance = cov;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    if (solver.info() != Eigen::Success) {
        return false;
    }

    // SelfAdjointEigenSolver 特征值升序
    Eigen::Vector3f evals = solver.eigenvalues();
    Eigen::Matrix3f evecs = solver.eigenvectors();

    pcaData.eigenValues = evals;
    pcaData.v1 = evecs.col(2).normalized();  // 最大
    pcaData.v2 = evecs.col(1).normalized();  // 次大

    for (const auto& centered : pcaData.centeredVectors) {
        float x = centered.dot(pcaData.v1);
        float y = centered.dot(pcaData.v2);
        pcaData.projected2DPoints.emplace_back(x, y);
    }

    return pcaData.projected2DPoints.size() >= 5;
}
bool SeamConcavityExtractor::generateStandardCircle2D(ReconstructedCurveData& curveData) {
    curveData.fittedCircle2D.clear();

    if (curveSampleCount_ < 8) {
        return false;
    }

    curveData.fittedCircle2D.reserve(curveSampleCount_);

    for (int k = 0; k < curveSampleCount_; ++k) {
        float theta = 2.0f * static_cast<float>(M_PI) * static_cast<float>(k) / static_cast<float>(curveSampleCount_);
        curveData.fittedCircle2D.emplace_back(std::cos(theta), std::sin(theta));
    }

    return true;
}
bool SeamConcavityExtractor::reconstructCurveToSphere(const SphereProjectionData& sphereData, const PcaProjectionData& pcaData,
                                                      ReconstructedCurveData& curveData) {
    if (curveData.fittedCircle2D.empty()) {
        return false;
    }

    curveData.reconstructedCurve3D.clear();
    curveData.reconstructedCurve3D.reserve(curveData.fittedCircle2D.size());

    for (const auto& c2 : curveData.fittedCircle2D) {
        Eigen::Vector3f ck = c2.x() * pcaData.v1 + c2.y() * pcaData.v2;
        Eigen::Vector3f back = ck + sphereData.meanVector;

        float norm = back.norm();
        if (norm < 1e-6f) continue;

        curveData.reconstructedCurve3D.push_back(back / norm);
    }

    return curveData.reconstructedCurve3D.size() >= 8;
}
bool SeamConcavityExtractor::computeMeanDeviation(const SphereProjectionData& sphereData, const ReconstructedCurveData& curveData,
                                                  float& meanDeviation) {
    if (sphereData.unitVectors.empty() || sphereData.weights.size() != sphereData.unitVectors.size() || curveData.reconstructedCurve3D.empty()) {
        return false;
    }

    double sum = 0.0;

    for (size_t i = 0; i < sphereData.unitVectors.size(); ++i) {
        float ej = pointToCurveMinDistance(sphereData.unitVectors[i], curveData.reconstructedCurve3D);
        sum += static_cast<double>(sphereData.weights[i]) * static_cast<double>(ej);
    }

    meanDeviation = static_cast<float>(sum / static_cast<double>(sphereData.unitVectors.size()));
    return std::isfinite(meanDeviation);
}
float SeamConcavityExtractor::computeDistanceQuantile(std::vector<float> values, float q) const {
    if (values.empty()) return 0.0f;

    q = std::max(0.0f, std::min(1.0f, q));
    std::sort(values.begin(), values.end());

    float pos = q * static_cast<float>(values.size() - 1);
    int idx0 = static_cast<int>(std::floor(pos));
    int idx1 = static_cast<int>(std::ceil(pos));

    if (idx0 == idx1) return values[idx0];

    float t = pos - static_cast<float>(idx0);
    return values[idx0] * (1.0f - t) + values[idx1] * t;
}
float SeamConcavityExtractor::computeMedian(std::vector<float> values) const {
    if (values.empty()) return 0.0f;
    std::sort(values.begin(), values.end());

    size_t n = values.size();
    if (n % 2 == 1) {
        return values[n / 2];
    }
    return 0.5f * (values[n / 2 - 1] + values[n / 2]);
}
float SeamConcavityExtractor::pointToCurveMinDistance(const Eigen::Vector3f& point, const std::vector<Eigen::Vector3f>& curve) const {
    float minDist = std::numeric_limits<float>::max();

    for (const auto& c : curve) {
        float dist = (point - c).norm();
        if (dist < minDist) {
            minDist = dist;
        }
    }

    return minDist;
}
Eigen::Vector3f SeamConcavityExtractor::toEigen(const pcl::PointXYZ& p) const { return Eigen::Vector3f(p.x, p.y, p.z); }

pcl::PointXYZ SeamConcavityExtractor::toPcl(const Eigen::Vector3f& v) const {
    pcl::PointXYZ p;
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
    return p;
}
