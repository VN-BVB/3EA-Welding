#include "DataClustering.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <queue>
#include <random>
#include <sstream>
#include <utility>

namespace SeamClustering {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kPi = 3.14159265358979323846;
constexpr double kMADToSigma = 1.48260221850560203194;  // 1 / Phi^-1(0.75)

struct PreparedData {
    std::vector<double> x;               // 保留有限值，保持原始顺序
    std::vector<std::size_t> origIndex;  // x -> 原始索引
    std::size_t originalSize = 0;
};

struct SortedData {
    std::vector<double> x;                // 升序值
    std::vector<std::size_t> cleanIndex;  // 排序位置 -> PreparedData::x 索引
};

template <typename T>
T clampValue(T v, T lo, T hi) {
    return std::max(lo, std::min(v, hi));
}

inline bool isFinite(double v) { return std::isfinite(v) != 0; }

PreparedData prepareData(const std::vector<double>& data, bool ignoreNonFinite, ThresholdResult& result) {
    PreparedData out;
    out.originalSize = data.size();
    out.x.reserve(data.size());
    out.origIndex.reserve(data.size());

    if (data.empty()) {
        result.status = Status::EmptyInput;
        result.message = "输入数据为空";
        return out;
    }

    for (std::size_t i = 0; i < data.size(); ++i) {
        if (isFinite(data[i])) {
            out.x.push_back(data[i]);
            out.origIndex.push_back(i);
        } else if (!ignoreNonFinite) {
            result.status = Status::InvalidParameter;
            result.message = "输入包含 NaN/Inf，且 ignoreNonFinite=false";
            out.x.clear();
            out.origIndex.clear();
            return out;
        }
    }

    if (out.x.empty()) {
        result.status = Status::EmptyInput;
        result.message = "没有可用的有限值样本";
    }
    return out;
}

SortedData sortPrepared(const PreparedData& in) {
    SortedData out;
    out.x = in.x;
    out.cleanIndex.resize(in.x.size());
    std::iota(out.cleanIndex.begin(), out.cleanIndex.end(), 0);

    std::sort(out.cleanIndex.begin(), out.cleanIndex.end(), [&](std::size_t a, std::size_t b) {
        if (out.x[a] == out.x[b]) return a < b;
        return out.x[a] < out.x[b];
    });

    std::vector<double> sortedX(out.cleanIndex.size());
    for (std::size_t i = 0; i < out.cleanIndex.size(); ++i) {
        sortedX[i] = out.x[out.cleanIndex[i]];
    }
    out.x.swap(sortedX);
    return out;
}

std::vector<int> makeOriginalAssignments(const PreparedData& data, const std::vector<int>& cleanAssignments) {
    std::vector<int> assignments(data.originalSize, -1);
    if (cleanAssignments.size() != data.x.size()) {
        return assignments;
    }
    for (std::size_t i = 0; i < data.x.size(); ++i) {
        assignments[data.origIndex[i]] = cleanAssignments[i];
    }
    return assignments;
}

std::vector<double> normalizeThresholdList(std::vector<double> thresholds) {
    thresholds.erase(std::remove_if(thresholds.begin(), thresholds.end(), [](double v) { return !isFinite(v); }), thresholds.end());
    std::sort(thresholds.begin(), thresholds.end());
    thresholds.erase(std::unique(thresholds.begin(), thresholds.end()), thresholds.end());
    return thresholds;
}

void populateThresholdBuckets(const std::vector<double>& x, const std::vector<double>& rawThresholds, ThresholdResult& result) {
    result.thresholds = normalizeThresholdList(rawThresholds);
    result.groupedValues.clear();

    if (result.thresholds.empty()) {
        if (!x.empty()) result.groupedValues.push_back(x);
        return;
    }

    result.groupedValues.assign(result.thresholds.size() + 1, std::vector<double>{});
    for (double v : x) {
        const std::size_t bucket =
            static_cast<std::size_t>(std::upper_bound(result.thresholds.begin(), result.thresholds.end(), v) - result.thresholds.begin());
        result.groupedValues[bucket].push_back(v);
    }

    result.threshold = result.thresholds.back();
}

std::vector<std::vector<double>> groupValuesByLabels(const std::vector<double>& x, const std::vector<int>& labels,
                                                     std::size_t expectedGroupCount = 0) {
    if (x.size() != labels.size()) return {};

    int maxLabel = -1;
    for (int lab : labels) {
        if (lab >= 0) maxLabel = std::max(maxLabel, lab);
    }

    const std::size_t groupCount = std::max<std::size_t>(expectedGroupCount, static_cast<std::size_t>(maxLabel + 1));
    std::vector<std::vector<double>> groups(groupCount);
    for (std::size_t i = 0; i < x.size(); ++i) {
        const int lab = labels[i];
        if (lab >= 0 && static_cast<std::size_t>(lab) < groups.size()) {
            groups[static_cast<std::size_t>(lab)].push_back(x[i]);
        }
    }
    return groups;
}

std::vector<double> thresholdsFromCenters(const std::vector<double>& centers) {
    std::vector<double> thresholds;
    if (centers.size() < 2) return thresholds;

    thresholds.reserve(centers.size() - 1);
    for (std::size_t i = 1; i < centers.size(); ++i) {
        thresholds.push_back(0.5 * (centers[i - 1] + centers[i]));
    }
    return thresholds;
}

void renumberLabelsDense(std::vector<int>& labels) {
    int maxLabel = -1;
    for (int lab : labels) {
        if (lab >= 0) maxLabel = std::max(maxLabel, lab);
    }
    if (maxLabel < 0) return;

    std::vector<int> remap(static_cast<std::size_t>(maxLabel + 1), -1);
    int nextLabel = 0;
    for (int& lab : labels) {
        if (lab < 0) continue;
        int& dst = remap[static_cast<std::size_t>(lab)];
        if (dst < 0) dst = nextLabel++;
        lab = dst;
    }
}

double medianOfCopy(std::vector<double> v) {
    if (v.empty()) return kNaN;
    const std::size_t n = v.size();
    const std::size_t mid = n / 2;
    std::nth_element(v.begin(), v.begin() + mid, v.end());
    double m = v[mid];

    if ((n % 2) == 0) {
        // 下中位数是前半段的最大值
        std::nth_element(v.begin(), v.begin() + mid - 1, v.begin() + mid);
        m = 0.5 * (m + v[mid - 1]);
    }
    return m;
}

double quantileOfCopy(std::vector<double> v, double q) {
    if (v.empty()) return kNaN;
    q = clampValue(q, 0.0, 1.0);

    const double pos = q * static_cast<double>(v.size() - 1);
    const std::size_t lo = static_cast<std::size_t>(std::floor(pos));
    const std::size_t hi = static_cast<std::size_t>(std::ceil(pos));

    std::nth_element(v.begin(), v.begin() + lo, v.end());
    const double vLo = v[lo];
    if (hi == lo) return vLo;

    std::nth_element(v.begin(), v.begin() + hi, v.end());
    const double vHi = v[hi];
    const double t = pos - static_cast<double>(lo);
    return vLo + t * (vHi - vLo);
}

double meanOf(const std::vector<double>& x) {
    if (x.empty()) return kNaN;
    return std::accumulate(x.begin(), x.end(), 0.0) / static_cast<double>(x.size());
}

double varianceOf(const std::vector<double>& x, double mean, double floorVar = 0.0) {
    if (x.size() < 2) return std::max(floorVar, 0.0);
    double acc = 0.0;
    for (double v : x) {
        const double d = v - mean;
        acc += d * d;
    }
    acc /= static_cast<double>(x.size() - 1);
    return std::max(acc, floorVar);
}

double madThresholdValue(const std::vector<double>& x, double madScale, bool scaleToNormalSigma) {
    if (x.empty()) return kNaN;

    const double med = medianOfCopy(x);
    std::vector<double> absDev;
    absDev.reserve(x.size());
    for (double v : x) absDev.push_back(std::abs(v - med));

    const double mad = medianOfCopy(absDev);
    double sigma = mad;
    if (scaleToNormalSigma) sigma *= kMADToSigma;
    return med + madScale * sigma;
}

double iqrThresholdValue(const std::vector<double>& x, double fence) {
    if (x.empty()) return kNaN;
    const double q1 = quantileOfCopy(x, 0.25);
    const double q3 = quantileOfCopy(x, 0.75);
    return q3 + fence * (q3 - q1);
}

double quantileThresholdValue(const std::vector<double>& x, double q) { return quantileOfCopy(x, q); }

double logQuantileThresholdValue(const std::vector<double>& x, double q, double logBase, double shift, bool autoShiftToPositive) {
    if (x.empty()) return kNaN;
    if (!(logBase > 0.0) || std::abs(logBase - 1.0) < 1e-15) return kNaN;

    const double minX = *std::min_element(x.begin(), x.end());
    if (autoShiftToPositive && minX + shift <= 0.0) {
        shift = -minX + 1e-12;
    }
    if (minX + shift <= 0.0) return kNaN;

    const double denom = std::log(logBase);
    if (!isFinite(denom) || std::abs(denom) < 1e-15) return kNaN;

    std::vector<double> y;
    y.reserve(x.size());
    for (double v : x) {
        const double z = v + shift;
        if (z <= 0.0) return kNaN;
        y.push_back(std::log(z) / denom);
    }
    const double tq = quantileOfCopy(y, q);
    return std::pow(logBase, tq) - shift;
}

bool maybeApplyFallback(const std::vector<double>& x, const CommonOptions& common, ThresholdResult& result) {
    if (common.fallback == FallbackPolicy::None || x.empty()) {
        return false;
    }

    if (common.fallback == FallbackPolicy::MAD) {
        result.threshold = madThresholdValue(x, 1.0, true);
        result.usedFallback = isFinite(result.threshold);
        if (result.usedFallback) {
            result.status = Status::Ok;
            populateThresholdBuckets(x, {result.threshold}, result);
            if (!result.message.empty()) result.message += "；";
            result.message += "已回退到 MAD";
        }
        return result.usedFallback;
    }

    if (common.fallback == FallbackPolicy::Quantile) {
        result.threshold = quantileThresholdValue(x, common.fallbackQuantile);
        result.usedFallback = isFinite(result.threshold);
        if (result.usedFallback) {
            result.status = Status::Ok;
            if (!result.message.empty()) result.message += "；";
            populateThresholdBuckets(x, {result.threshold}, result);
            std::ostringstream oss;
            oss << "已回退到分位数 q=" << common.fallbackQuantile;
            result.message += oss.str();
        }
        return result.usedFallback;
    }

    return false;
}

/* ------------------------------ KMeans ------------------------------ */

struct KMeansFit {
    std::vector<double> centers;
    std::vector<int> labels;
    std::vector<int> counts;
    double sse = kInf;
    bool converged = false;
};

std::vector<double> initRandomCenters1D(const std::vector<double>& x, int k, std::mt19937& rng) {
    std::vector<double> centers;
    if (x.empty() || k <= 0) return centers;
    centers.reserve(static_cast<std::size_t>(k));
    std::uniform_int_distribution<std::size_t> dist(0, x.size() - 1);
    for (int i = 0; i < k; ++i) {
        centers.push_back(x[dist(rng)]);
    }
    return centers;
}

std::vector<double> initKMeansPP1D(const std::vector<double>& x, int k, std::mt19937& rng) {
    std::vector<double> centers;
    if (x.empty() || k <= 0) return centers;

    centers.reserve(static_cast<std::size_t>(k));
    std::uniform_int_distribution<std::size_t> firstDist(0, x.size() - 1);
    centers.push_back(x[firstDist(rng)]);

    std::vector<double> minDist2(x.size(), kInf);
    for (int c = 1; c < k; ++c) {
        bool allZero = true;
        for (std::size_t i = 0; i < x.size(); ++i) {
            const double d = x[i] - centers.back();
            minDist2[i] = std::min(minDist2[i], d * d);
            if (minDist2[i] > 0.0) allZero = false;
        }

        if (allZero) {
            centers.push_back(x[firstDist(rng)]);
            continue;
        }

        std::discrete_distribution<std::size_t> dist(minDist2.begin(), minDist2.end());
        centers.push_back(x[dist(rng)]);
    }

    return centers;
}

KMeansFit runKMeans1D(const std::vector<double>& x, int k, int maxIter, int restarts, double tol, unsigned int seed, bool usePP) {
    KMeansFit best;
    if (x.empty() || k <= 0) return best;

    restarts = std::max(1, restarts);
    k = std::min<int>(k, static_cast<int>(x.size()));

    for (int r = 0; r < restarts; ++r) {
        std::mt19937 rng(seed + static_cast<unsigned int>(r * 1337u));
        std::vector<double> centers = usePP ? initKMeansPP1D(x, k, rng) : initRandomCenters1D(x, k, rng);
        if (centers.size() != static_cast<std::size_t>(k)) continue;

        std::vector<int> labels(x.size(), -1);
        std::vector<int> counts(static_cast<std::size_t>(k), 0);
        std::vector<double> sums(static_cast<std::size_t>(k), 0.0);
        bool converged = false;
        double sse = kInf;

        for (int iter = 0; iter < maxIter; ++iter) {
            std::fill(labels.begin(), labels.end(), -1);
            std::fill(counts.begin(), counts.end(), 0);
            std::fill(sums.begin(), sums.end(), 0.0);
            sse = 0.0;

            for (std::size_t i = 0; i < x.size(); ++i) {
                double bestDist = kInf;
                int bestCluster = 0;
                for (int c = 0; c < k; ++c) {
                    const double d = x[i] - centers[static_cast<std::size_t>(c)];
                    const double d2 = d * d;
                    if (d2 < bestDist) {
                        bestDist = d2;
                        bestCluster = c;
                    }
                }
                labels[i] = bestCluster;
                counts[static_cast<std::size_t>(bestCluster)] += 1;
                sums[static_cast<std::size_t>(bestCluster)] += x[i];
                sse += bestDist;
            }

            std::vector<double> newCenters = centers;
            for (int c = 0; c < k; ++c) {
                if (counts[static_cast<std::size_t>(c)] > 0) {
                    newCenters[static_cast<std::size_t>(c)] =
                        sums[static_cast<std::size_t>(c)] / static_cast<double>(counts[static_cast<std::size_t>(c)]);
                } else {
                    // 空簇：用当前误差最大的点重置
                    std::size_t farIndex = 0;
                    double farDist = -1.0;
                    for (std::size_t i = 0; i < x.size(); ++i) {
                        const int lab = labels[i];
                        const double d = x[i] - centers[static_cast<std::size_t>(lab)];
                        const double d2 = d * d;
                        if (d2 > farDist) {
                            farDist = d2;
                            farIndex = i;
                        }
                    }
                    newCenters[static_cast<std::size_t>(c)] = x[farIndex];
                }
            }

            double shift = 0.0;
            for (int c = 0; c < k; ++c) {
                shift = std::max(shift, std::abs(newCenters[static_cast<std::size_t>(c)] - centers[static_cast<std::size_t>(c)]));
            }

            centers.swap(newCenters);
            if (shift <= tol) {
                converged = true;
                break;
            }
        }

        if (sse < best.sse) {
            best.centers = centers;
            best.labels = labels;
            best.counts = counts;
            best.sse = sse;
            best.converged = converged;
        }
    }

    return best;
}

void sortClustersByCenter(std::vector<double>& centers, std::vector<int>& labels, std::vector<int>& counts, std::vector<double>* weights = nullptr,
                          std::vector<double>* variances = nullptr) {
    const std::size_t k = centers.size();
    std::vector<std::size_t> order(k);
    std::iota(order.begin(), order.end(), 0);

    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        if (centers[a] == centers[b]) return a < b;
        return centers[a] < centers[b];
    });

    std::vector<int> remap(k, -1);
    std::vector<double> newCenters(k, 0.0);
    std::vector<int> newCounts(k, 0);
    std::vector<double> newWeights;
    std::vector<double> newVariances;
    if (weights) newWeights.resize(k);
    if (variances) newVariances.resize(k);

    for (std::size_t i = 0; i < k; ++i) {
        const std::size_t oldIdx = order[i];
        remap[oldIdx] = static_cast<int>(i);

        newCenters[i] = centers[oldIdx];
        if (oldIdx < counts.size()) newCounts[i] = counts[oldIdx];
        if (weights && oldIdx < weights->size()) newWeights[i] = (*weights)[oldIdx];
        if (variances && oldIdx < variances->size()) newVariances[i] = (*variances)[oldIdx];
    }

    for (int& lab : labels) {
        if (lab >= 0 && static_cast<std::size_t>(lab) < remap.size()) {
            lab = remap[static_cast<std::size_t>(lab)];
        }
    }

    centers.swap(newCenters);
    counts.swap(newCounts);
    if (weights) weights->swap(newWeights);
    if (variances) variances->swap(newVariances);
}

double kmeansThresholdFromCenters(const std::vector<double>& centers) {
    if (centers.size() < 2) return kNaN;
    return 0.5 * (centers[centers.size() - 2] + centers.back());
}

/* ------------------------------- GMM ------------------------------- */

struct GMMFit {
    std::vector<double> means;
    std::vector<double> vars;
    std::vector<double> weights;
    std::vector<int> labels;
    std::vector<int> counts;
    double logLikelihood = -kInf;
    bool converged = false;
};

double logGaussian1D(double x, double mean, double var) {
    var = std::max(var, 1e-300);
    const double d = x - mean;
    return -0.5 * (std::log(2.0 * kPi * var) + (d * d) / var);
}

GMMFit initGMM1D(const std::vector<double>& x, int k, double varFloor, unsigned int seed, bool usePP) {
    GMMFit fit;
    if (x.empty() || k <= 0) return fit;

    KMeansFit km = runKMeans1D(x, k, 20, 1, 1e-6, seed, usePP);
    fit.means = km.centers;
    fit.vars.assign(static_cast<std::size_t>(k), varFloor);
    fit.weights.assign(static_cast<std::size_t>(k), 1.0 / static_cast<double>(k));
    fit.labels = km.labels;
    fit.counts = km.counts;

    const double globalMean = meanOf(x);
    const double globalVar = std::max(varianceOf(x, globalMean, varFloor), varFloor);

    if (fit.means.size() != static_cast<std::size_t>(k)) {
        fit.means.assign(static_cast<std::size_t>(k), globalMean);
    }

    for (int c = 0; c < k; ++c) {
        if (c >= static_cast<int>(km.counts.size()) || km.counts[static_cast<std::size_t>(c)] == 0) {
            fit.vars[static_cast<std::size_t>(c)] = globalVar;
            fit.weights[static_cast<std::size_t>(c)] = 1.0 / static_cast<double>(k);
            continue;
        }

        std::vector<double> subset;
        subset.reserve(static_cast<std::size_t>(km.counts[static_cast<std::size_t>(c)]));
        for (std::size_t i = 0; i < x.size(); ++i) {
            if (km.labels[i] == c) subset.push_back(x[i]);
        }

        const double m = subset.empty() ? globalMean : meanOf(subset);
        fit.means[static_cast<std::size_t>(c)] = m;
        fit.vars[static_cast<std::size_t>(c)] = subset.size() >= 2 ? varianceOf(subset, m, varFloor) : globalVar;
        fit.weights[static_cast<std::size_t>(c)] = static_cast<double>(subset.size()) / static_cast<double>(x.size());
    }

    return fit;
}

GMMFit runGMM1D(const std::vector<double>& x, int k, int maxIter, int restarts, double tol, double varFloor, unsigned int seed, bool usePP) {
    GMMFit best;
    if (x.empty() || k <= 0) return best;

    restarts = std::max(1, restarts);
    k = std::min<int>(k, static_cast<int>(x.size()));

    for (int r = 0; r < restarts; ++r) {
        GMMFit fit = initGMM1D(x, k, varFloor, seed + static_cast<unsigned int>(r * 977u), usePP);
        if (fit.means.size() != static_cast<std::size_t>(k)) continue;

        double prevLL = -kInf;
        bool converged = false;

        for (int iter = 0; iter < maxIter; ++iter) {
            std::vector<double> Nk(static_cast<std::size_t>(k), 0.0);
            std::vector<double> S1(static_cast<std::size_t>(k), 0.0);
            std::vector<double> S2(static_cast<std::size_t>(k), 0.0);
            double ll = 0.0;

            for (double xi : x) {
                std::vector<double> logp(static_cast<std::size_t>(k), -kInf);
                double maxLogP = -kInf;

                for (int c = 0; c < k; ++c) {
                    const double wc = std::max(fit.weights[static_cast<std::size_t>(c)], 1e-300);
                    const double lp = std::log(wc) + logGaussian1D(xi, fit.means[static_cast<std::size_t>(c)], fit.vars[static_cast<std::size_t>(c)]);
                    logp[static_cast<std::size_t>(c)] = lp;
                    if (lp > maxLogP) maxLogP = lp;
                }

                double sumExp = 0.0;
                for (int c = 0; c < k; ++c) {
                    sumExp += std::exp(logp[static_cast<std::size_t>(c)] - maxLogP);
                }
                if (!(sumExp > 0.0) || !isFinite(sumExp)) {
                    ll = -kInf;
                    break;
                }

                ll += maxLogP + std::log(sumExp);

                for (int c = 0; c < k; ++c) {
                    const double resp = std::exp(logp[static_cast<std::size_t>(c)] - maxLogP) / sumExp;
                    Nk[static_cast<std::size_t>(c)] += resp;
                    S1[static_cast<std::size_t>(c)] += resp * xi;
                    S2[static_cast<std::size_t>(c)] += resp * xi * xi;
                }
            }

            if (!isFinite(ll)) {
                break;
            }

            for (int c = 0; c < k; ++c) {
                const double nk = std::max(Nk[static_cast<std::size_t>(c)], 1e-12);
                fit.weights[static_cast<std::size_t>(c)] = nk / static_cast<double>(x.size());
                fit.means[static_cast<std::size_t>(c)] = S1[static_cast<std::size_t>(c)] / nk;

                double var = (S2[static_cast<std::size_t>(c)] / nk) - fit.means[static_cast<std::size_t>(c)] * fit.means[static_cast<std::size_t>(c)];
                fit.vars[static_cast<std::size_t>(c)] = std::max(var, varFloor);
            }

            double sumW = std::accumulate(fit.weights.begin(), fit.weights.end(), 0.0);
            if (sumW <= 0.0 || !isFinite(sumW)) break;
            for (double& w : fit.weights) w /= sumW;

            const double avgGain = (iter == 0) ? kInf : std::abs((ll - prevLL) / static_cast<double>(x.size()));
            prevLL = ll;
            fit.logLikelihood = ll;

            if (avgGain <= tol) {
                converged = true;
                break;
            }
        }

        fit.converged = converged;
        fit.labels.assign(x.size(), -1);
        fit.counts.assign(static_cast<std::size_t>(k), 0);

        if (isFinite(fit.logLikelihood)) {
            for (std::size_t i = 0; i < x.size(); ++i) {
                double bestLogP = -kInf;
                int bestCluster = 0;
                for (int c = 0; c < k; ++c) {
                    const double wc = std::max(fit.weights[static_cast<std::size_t>(c)], 1e-300);
                    const double lp =
                        std::log(wc) + logGaussian1D(x[i], fit.means[static_cast<std::size_t>(c)], fit.vars[static_cast<std::size_t>(c)]);
                    if (lp > bestLogP) {
                        bestLogP = lp;
                        bestCluster = c;
                    }
                }
                fit.labels[i] = bestCluster;
                fit.counts[static_cast<std::size_t>(bestCluster)] += 1;
            }
        }

        if (fit.logLikelihood > best.logLikelihood) {
            best = fit;
        }
    }

    return best;
}

bool solveGaussianIntersection(double w1, double m1, double v1, double w2, double m2, double v2, double& xout) {
    v1 = std::max(v1, 1e-300);
    v2 = std::max(v2, 1e-300);

    if (m1 > m2) {
        std::swap(w1, w2);
        std::swap(m1, m2);
        std::swap(v1, v2);
    }

    const double A = 1.0 / v2 - 1.0 / v1;
    const double B = -2.0 * m2 / v2 + 2.0 * m1 / v1;
    const double C = (m2 * m2) / v2 - (m1 * m1) / v1 + 2.0 * std::log(std::max(w1, 1e-300) / std::max(w2, 1e-300)) - std::log(v1 / v2);

    if (std::abs(A) < 1e-15) {
        if (std::abs(B) < 1e-15) return false;
        xout = -C / B;
        return isFinite(xout);
    }

    const double disc = B * B - 4.0 * A * C;
    if (disc < 0.0 || !isFinite(disc)) return false;

    const double sqrtDisc = std::sqrt(std::max(0.0, disc));
    const double x1 = (-B + sqrtDisc) / (2.0 * A);
    const double x2 = (-B - sqrtDisc) / (2.0 * A);

    const bool in1 = (x1 >= m1 && x1 <= m2);
    const bool in2 = (x2 >= m1 && x2 <= m2);

    if (in1 && in2) {
        xout = 0.5 * (x1 + x2);
        return true;
    }
    if (in1) {
        xout = x1;
        return true;
    }
    if (in2) {
        xout = x2;
        return true;
    }

    // 若交点都不在两个均值之间，则退化为离中点更近的一根
    const double mid = 0.5 * (m1 + m2);
    xout = (std::abs(x1 - mid) <= std::abs(x2 - mid)) ? x1 : x2;
    return isFinite(xout);
}

double gmmThresholdFromFit(const GMMFit& fit) {
    if (fit.means.size() < 2) return kNaN;

    std::vector<double> means = fit.means;
    std::vector<int> dummyLabels;
    std::vector<int> counts = fit.counts;
    std::vector<double> weights = fit.weights;
    std::vector<double> vars = fit.vars;
    sortClustersByCenter(means, dummyLabels, counts, &weights, &vars);

    const std::size_t hi = means.size() - 1;
    const std::size_t lo = hi - 1;

    double crossing = kNaN;
    if (!solveGaussianIntersection(weights[lo], means[lo], vars[lo], weights[hi], means[hi], vars[hi], crossing) || !isFinite(crossing)) {
        crossing = 0.5 * (means[lo] + means[hi]);
    }
    return crossing;
}

std::vector<double> gmmThresholdsFromSortedFit(const GMMFit& fit) {
    std::vector<double> thresholds;
    if (fit.means.size() < 2) return thresholds;

    thresholds.reserve(fit.means.size() - 1);
    for (std::size_t i = 1; i < fit.means.size(); ++i) {
        double crossing = kNaN;
        if (!solveGaussianIntersection(fit.weights[i - 1], fit.means[i - 1], fit.vars[i - 1], fit.weights[i], fit.means[i], fit.vars[i], crossing) ||
            !isFinite(crossing)) {
            crossing = 0.5 * (fit.means[i - 1] + fit.means[i]);
        }
        thresholds.push_back(crossing);
    }
    return thresholds;
}

/* -------------------------- 1D 邻域辅助 -------------------------- */

// 计算 1D 数据中第 k 个近邻距离（k 从 1 开始，不包含自己）
std::vector<double> kthNeighborDistance1D(const std::vector<double>& sortedX, int k) {
    const std::size_t n = sortedX.size();
    std::vector<double> out(n, kInf);
    if (n == 0 || k <= 0) return out;

    for (std::size_t i = 0; i < n; ++i) {
        int left = static_cast<int>(i) - 1;
        int right = static_cast<int>(i) + 1;
        double lastDist = 0.0;

        for (int taken = 0; taken < k; ++taken) {
            const double dl = (left >= 0) ? std::abs(sortedX[i] - sortedX[static_cast<std::size_t>(left)]) : kInf;
            const double dr = (right < static_cast<int>(n)) ? std::abs(sortedX[static_cast<std::size_t>(right)] - sortedX[i]) : kInf;

            if (dl <= dr) {
                lastDist = dl;
                --left;
            } else {
                lastDist = dr;
                ++right;
            }
        }
        out[i] = lastDist;
    }

    return out;
}

double autoEpsFromKDistance(const std::vector<double>& sortedX, int minPts, double quantile) {
    if (sortedX.empty()) return kNaN;
    const int k = std::max(1, minPts - 1);
    std::vector<double> kd = kthNeighborDistance1D(sortedX, k);
    for (double& v : kd) {
        if (!isFinite(v)) v = 0.0;
    }
    return quantileOfCopy(kd, quantile);
}

void neighborhoodBounds1D(const std::vector<double>& sortedX, double eps, std::vector<int>& left, std::vector<int>& right) {
    const int n = static_cast<int>(sortedX.size());
    left.assign(static_cast<std::size_t>(n), 0);
    right.assign(static_cast<std::size_t>(n), n - 1);

    int l = 0;
    for (int i = 0; i < n; ++i) {
        while (l < n && sortedX[i] - sortedX[l] > eps) ++l;
        left[static_cast<std::size_t>(i)] = l;
    }

    int r = 0;
    for (int i = 0; i < n; ++i) {
        if (r < i) r = i;
        while (r + 1 < n && sortedX[r + 1] - sortedX[i] <= eps) ++r;
        right[static_cast<std::size_t>(i)] = r;
    }
}

struct ClusterInterval {
    int label = -1;
    int begin = -1;  // sorted index inclusive
    int end = -1;    // sorted index inclusive
    double mean = kNaN;
};

std::vector<ClusterInterval> extractIntervalsFromLabelsSorted(const std::vector<double>& sortedX, const std::vector<int>& sortedLabels) {
    std::vector<ClusterInterval> intervals;
    const int n = static_cast<int>(sortedX.size());
    if (n == 0 || sortedLabels.size() != sortedX.size()) return intervals;

    int i = 0;
    while (i < n) {
        if (sortedLabels[static_cast<std::size_t>(i)] < 0) {
            ++i;
            continue;
        }

        const int label = sortedLabels[static_cast<std::size_t>(i)];
        int j = i;
        double sum = 0.0;
        int cnt = 0;
        while (j < n && sortedLabels[static_cast<std::size_t>(j)] == label) {
            sum += sortedX[static_cast<std::size_t>(j)];
            ++cnt;
            ++j;
        }

        ClusterInterval ci;
        ci.label = label;
        ci.begin = i;
        ci.end = j - 1;
        ci.mean = (cnt > 0) ? (sum / static_cast<double>(cnt)) : kNaN;
        intervals.push_back(ci);

        i = j;
    }

    return intervals;
}

void populateIntervalClusterResult(const std::vector<double>& sortedX, const std::vector<ClusterInterval>& intervals, ThresholdResult& result) {
    result.groupedValues.clear();
    result.thresholds.clear();
    result.clusterCounts.clear();
    result.centers.clear();

    if (intervals.empty()) return;

    result.groupedValues.reserve(intervals.size());
    result.thresholds.reserve(intervals.size() > 0 ? intervals.size() - 1 : 0);
    result.clusterCounts.reserve(intervals.size());
    result.centers.reserve(intervals.size());

    for (std::size_t i = 0; i < intervals.size(); ++i) {
        const ClusterInterval& ci = intervals[i];
        result.groupedValues.emplace_back(sortedX.begin() + ci.begin, sortedX.begin() + ci.end + 1);
        result.clusterCounts.push_back(ci.end - ci.begin + 1);
        result.centers.push_back(ci.mean);

        if (i > 0) {
            const ClusterInterval& prev = intervals[i - 1];
            result.thresholds.push_back(0.5 * (sortedX[static_cast<std::size_t>(prev.end)] + sortedX[static_cast<std::size_t>(ci.begin)]));
        }
    }
}

bool chooseUpperTailClusterThreshold(const std::vector<double>& sortedX, const std::vector<int>& sortedLabels, double& threshold, ThresholdResult* result = nullptr) {
    threshold = kNaN;

    const auto intervals = extractIntervalsFromLabelsSorted(sortedX, sortedLabels);
    if (result) populateIntervalClusterResult(sortedX, intervals, *result);
    if (intervals.size() < 2) return false;

    const ClusterInterval* best = nullptr;
    for (const auto& ci : intervals) {
        if (!best || ci.mean > best->mean) best = &ci;
    }
    if (!best || best->begin <= 0) return false;

    threshold = 0.5 * (sortedX[static_cast<std::size_t>(best->begin - 1)] + sortedX[static_cast<std::size_t>(best->begin)]);
    return isFinite(threshold);
}

/* ------------------------------ DBSCAN ------------------------------ */

ThresholdResult computeDBSCAN1DRaw(const PreparedData& data, Method method, double eps, int minPts, bool allowFallback, const CommonOptions& common) {
    ThresholdResult result;
    result.method = method;

    if (data.x.size() < 2) {
        result.status = Status::NotEnoughData;
        result.message = "样本量不足";
        if (allowFallback) maybeApplyFallback(data.x, common, result);
        return result;
    }

    if (!(eps > 0.0) || minPts < 2) {
        result.status = Status::InvalidParameter;
        result.message = "eps 必须 > 0 且 minPts >= 2";
        if (allowFallback) maybeApplyFallback(data.x, common, result);
        return result;
    }

    SortedData sorted = sortPrepared(data);
    const int n = static_cast<int>(sorted.x.size());

    std::vector<int> left, right;
    neighborhoodBounds1D(sorted.x, eps, left, right);

    std::vector<bool> core(static_cast<std::size_t>(n), false);
    for (int i = 0; i < n; ++i) {
        const int cnt = right[static_cast<std::size_t>(i)] - left[static_cast<std::size_t>(i)] + 1;
        core[static_cast<std::size_t>(i)] = (cnt >= minPts);
    }

    std::vector<int> sortedLabels(static_cast<std::size_t>(n), -1);
    int clusterId = -1;
    int i = 0;

    while (i < n) {
        while (i < n && !core[static_cast<std::size_t>(i)]) ++i;
        if (i >= n) break;

        ++clusterId;
        int clusterStart = left[static_cast<std::size_t>(i)];
        int clusterEnd = right[static_cast<std::size_t>(i)];

        int j = i + 1;
        while (j < n && j <= clusterEnd) {
            if (core[static_cast<std::size_t>(j)]) {
                clusterStart = std::min(clusterStart, left[static_cast<std::size_t>(j)]);
                clusterEnd = std::max(clusterEnd, right[static_cast<std::size_t>(j)]);
            }
            ++j;
        }

        for (int t = clusterStart; t <= clusterEnd; ++t) {
            sortedLabels[static_cast<std::size_t>(t)] = clusterId;
        }
        i = clusterEnd + 1;
    }

    renumberLabelsDense(sortedLabels);

    std::vector<int> cleanLabels(data.x.size(), -1);
    for (std::size_t s = 0; s < sorted.cleanIndex.size(); ++s) {
        cleanLabels[sorted.cleanIndex[s]] = sortedLabels[s];
    }
    result.assignments = makeOriginalAssignments(data, cleanLabels);

    double threshold = kNaN;
    if (!chooseUpperTailClusterThreshold(sorted.x, sortedLabels, threshold, &result)) {
        result.status = Status::NoSeparationFound;
        result.message = "DBSCAN 未得到可用于阈值化的高值簇";
        if (allowFallback) maybeApplyFallback(data.x, common, result);
        return result;
    }

    result.threshold = threshold;
    result.status = Status::Ok;
    result.message = "成功";
    return result;
}

/* ------------------------------ OPTICS ------------------------------ */

ThresholdResult computeOPTICS1DRaw(const PreparedData& data, Method method, double maxEps, int minPts, double extractEps, int minClusterSize,
                                   bool allowFallback, const CommonOptions& common) {
    ThresholdResult result;
    result.method = method;

    if (data.x.size() < 2) {
        result.status = Status::NotEnoughData;
        result.message = "样本量不足";
        if (allowFallback) maybeApplyFallback(data.x, common, result);
        return result;
    }

    if (!(maxEps > 0.0) || minPts < 2) {
        result.status = Status::InvalidParameter;
        result.message = "maxEps 必须 > 0 且 minPts >= 2";
        if (allowFallback) maybeApplyFallback(data.x, common, result);
        return result;
    }

    SortedData sorted = sortPrepared(data);
    const int n = static_cast<int>(sorted.x.size());

    std::vector<int> left, right;
    neighborhoodBounds1D(sorted.x, maxEps, left, right);

    std::vector<double> coreDist(static_cast<std::size_t>(n), kInf);
    const int k = std::max(1, minPts - 1);
    std::vector<double> kth = kthNeighborDistance1D(sorted.x, k);

    for (int i = 0; i < n; ++i) {
        const int cnt = right[static_cast<std::size_t>(i)] - left[static_cast<std::size_t>(i)] + 1;
        if (cnt >= minPts && kth[static_cast<std::size_t>(i)] <= maxEps) {
            coreDist[static_cast<std::size_t>(i)] = kth[static_cast<std::size_t>(i)];
        }
    }

    std::vector<bool> processed(static_cast<std::size_t>(n), false);
    std::vector<double> reach(static_cast<std::size_t>(n), kInf);
    std::vector<int> predecessor(static_cast<std::size_t>(n), -1);
    std::vector<int> ordering;
    ordering.reserve(static_cast<std::size_t>(n));

    using Node = std::pair<double, int>;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> seeds;

    auto update = [&](int p) {
        if (!isFinite(coreDist[static_cast<std::size_t>(p)])) return;
        for (int q = left[static_cast<std::size_t>(p)]; q <= right[static_cast<std::size_t>(p)]; ++q) {
            if (processed[static_cast<std::size_t>(q)]) continue;

            const double newReach = std::max(coreDist[static_cast<std::size_t>(p)],
                                             std::abs(sorted.x[static_cast<std::size_t>(q)] - sorted.x[static_cast<std::size_t>(p)]));

            if (newReach < reach[static_cast<std::size_t>(q)]) {
                reach[static_cast<std::size_t>(q)] = newReach;
                predecessor[static_cast<std::size_t>(q)] = p;
                seeds.emplace(newReach, q);
            }
        }
    };

    for (int start = 0; start < n; ++start) {
        if (processed[static_cast<std::size_t>(start)]) continue;

        processed[static_cast<std::size_t>(start)] = true;
        ordering.push_back(start);
        update(start);

        while (!seeds.empty()) {
            const int q = seeds.top().second;
            seeds.pop();
            if (processed[static_cast<std::size_t>(q)]) continue;
            processed[static_cast<std::size_t>(q)] = true;
            ordering.push_back(q);
            update(q);
        }
    }

    if (!(extractEps > 0.0)) {
        std::vector<double> finiteReach;
        finiteReach.reserve(reach.size());
        for (double v : reach)
            if (isFinite(v)) finiteReach.push_back(v);

        extractEps = finiteReach.empty() ? maxEps : quantileOfCopy(finiteReach, 0.90);
        if (!(extractEps > 0.0)) extractEps = maxEps;
    }

    // DBSCAN-like 提取
    std::vector<int> sortedLabels(static_cast<std::size_t>(n), -1);
    int clusterId = -1;

    for (int pos = 0; pos < static_cast<int>(ordering.size()); ++pos) {
        const int idx = ordering[static_cast<std::size_t>(pos)];
        const double rd = reach[static_cast<std::size_t>(idx)];
        const double cd = coreDist[static_cast<std::size_t>(idx)];

        if (rd > extractEps) {
            if (cd <= extractEps) {
                ++clusterId;
                sortedLabels[static_cast<std::size_t>(idx)] = clusterId;
            } else {
                sortedLabels[static_cast<std::size_t>(idx)] = -1;
            }
        } else {
            if (clusterId >= 0) sortedLabels[static_cast<std::size_t>(idx)] = clusterId;
        }
    }

    // 小簇过滤
    int maxLabel = -1;
    for (int lab : sortedLabels) maxLabel = std::max(maxLabel, lab);

    std::vector<int> counts(static_cast<std::size_t>(maxLabel + 1), 0);
    for (int lab : sortedLabels) {
        if (lab >= 0) counts[static_cast<std::size_t>(lab)] += 1;
    }

    for (int& lab : sortedLabels) {
        if (lab >= 0 && counts[static_cast<std::size_t>(lab)] < minClusterSize) {
            lab = -1;
        }
    }

    renumberLabelsDense(sortedLabels);

    std::vector<int> cleanLabels(data.x.size(), -1);
    for (std::size_t s = 0; s < sorted.cleanIndex.size(); ++s) {
        cleanLabels[sorted.cleanIndex[s]] = sortedLabels[s];
    }
    result.assignments = makeOriginalAssignments(data, cleanLabels);

    std::vector<int> orderingOrig;
    orderingOrig.reserve(ordering.size());
    for (int sortedIdx : ordering) {
        const std::size_t cleanIdx = sorted.cleanIndex[static_cast<std::size_t>(sortedIdx)];
        orderingOrig.push_back(static_cast<int>(data.origIndex[cleanIdx]));
    }
    result.ordering = orderingOrig;

    result.reachability.assign(data.originalSize, kNaN);
    for (std::size_t s = 0; s < sorted.cleanIndex.size(); ++s) {
        const std::size_t cleanIdx = sorted.cleanIndex[s];
        result.reachability[data.origIndex[cleanIdx]] = reach[s];
    }

    double threshold = kNaN;
    if (!chooseUpperTailClusterThreshold(sorted.x, sortedLabels, threshold, &result)) {
        result.status = Status::NoSeparationFound;
        result.message = "OPTICS 未得到可用于阈值化的高值簇";
        if (allowFallback) maybeApplyFallback(data.x, common, result);
        return result;
    }

    result.threshold = threshold;
    result.status = Status::Ok;
    result.message = "成功";
    return result;
}

/* ------------------------------- KDE ------------------------------- */

double silvermanBandwidthUnivariate(const std::vector<double>& x) {
    if (x.size() < 2) return 0.0;

    const double mean = meanOf(x);
    const double sd = std::sqrt(std::max(varianceOf(x, mean, 0.0), 0.0));
    const double q1 = quantileOfCopy(x, 0.25);
    const double q3 = quantileOfCopy(x, 0.75);
    const double robust = (q3 - q1) / 1.34;
    const double A = std::min(sd, robust > 0.0 ? robust : sd);

    if (!(A > 0.0)) return 0.0;
    return 0.9 * A * std::pow(static_cast<double>(x.size()), -0.2);
}

ThresholdResult computeKDERaw(const PreparedData& data, const KDEOptions& options, bool allowFallback) {
    ThresholdResult result;
    result.method = Method::KDEModeSeparation;

    if (data.x.size() < 4) {
        result.status = Status::NotEnoughData;
        result.message = "KDE 至少需要 4 个样本";
        if (allowFallback) maybeApplyFallback(data.x, options, result);
        return result;
    }

    if (options.gridSize < 64) {
        result.status = Status::InvalidParameter;
        result.message = "gridSize 建议 >= 64";
        if (allowFallback) maybeApplyFallback(data.x, options, result);
        return result;
    }

    const double minX = *std::min_element(data.x.begin(), data.x.end());
    const double maxX = *std::max_element(data.x.begin(), data.x.end());

    double h = options.bandwidth;
    if (!(h > 0.0)) {
        h = silvermanBandwidthUnivariate(data.x) * options.bandwidthScale;
    }
    if (!(h > 0.0) || !isFinite(h)) {
        result.status = Status::NumericalFailure;
        result.message = "KDE 带宽无效";
        if (allowFallback) maybeApplyFallback(data.x, options, result);
        return result;
    }

    const double lo = minX - options.cut * h;
    const double hi = maxX + options.cut * h;
    const std::size_t m = options.gridSize;
    const double dx = (hi - lo) / static_cast<double>(m - 1);
    if (!(dx > 0.0)) {
        result.status = Status::NumericalFailure;
        result.message = "KDE 网格步长无效";
        if (allowFallback) maybeApplyFallback(data.x, options, result);
        return result;
    }

    result.auxCurveX.resize(m);
    result.auxCurveY.assign(m, 0.0);
    for (std::size_t i = 0; i < m; ++i) {
        result.auxCurveX[i] = lo + static_cast<double>(i) * dx;
    }

    // 生产场景默认：分箱 + 离散高斯卷积，避免 O(N*M)
    if (options.useBinnedApproximation) {
        std::vector<double> hist(m, 0.0);
        for (double v : data.x) {
            const double pos = (v - lo) / dx;
            if (pos <= 0.0) {
                hist.front() += 1.0;
            } else if (pos >= static_cast<double>(m - 1)) {
                hist.back() += 1.0;
            } else {
                const std::size_t i0 = static_cast<std::size_t>(std::floor(pos));
                const std::size_t i1 = i0 + 1;
                const double t = pos - static_cast<double>(i0);
                hist[i0] += (1.0 - t);
                if (i1 < m) hist[i1] += t;
            }
        }

        const double sigmaBins = h / dx;
        const int radius = std::max(1, static_cast<int>(std::ceil(4.0 * sigmaBins)));

        std::vector<double> kernel(static_cast<std::size_t>(2 * radius + 1), 0.0);
        double ksum = 0.0;
        for (int i = -radius; i <= radius; ++i) {
            const double z = static_cast<double>(i) / sigmaBins;
            const double kv = std::exp(-0.5 * z * z);
            kernel[static_cast<std::size_t>(i + radius)] = kv;
            ksum += kv;
        }
        for (double& v : kernel) v /= ksum;

        for (std::size_t i = 0; i < m; ++i) {
            double acc = 0.0;
            for (int kidx = -radius; kidx <= radius; ++kidx) {
                const long long j = static_cast<long long>(i) + static_cast<long long>(kidx);
                if (j < 0 || j >= static_cast<long long>(m)) continue;
                acc += hist[static_cast<std::size_t>(j)] * kernel[static_cast<std::size_t>(kidx + radius)];
            }
            result.auxCurveY[i] = acc / (static_cast<double>(data.x.size()) * dx);
        }
    } else {
        const double norm = 1.0 / (std::sqrt(2.0 * kPi) * h * static_cast<double>(data.x.size()));
        for (std::size_t i = 0; i < m; ++i) {
            double acc = 0.0;
            const double xi = result.auxCurveX[i];
            for (double v : data.x) {
                const double z = (xi - v) / h;
                acc += std::exp(-0.5 * z * z);
            }
            result.auxCurveY[i] = acc * norm;
        }
    }

    std::vector<std::size_t> maxima;
    std::vector<std::size_t> minima;
    for (std::size_t i = 1; i + 1 < m; ++i) {
        const double y0 = result.auxCurveY[i - 1];
        const double y1 = result.auxCurveY[i];
        const double y2 = result.auxCurveY[i + 1];
        if (y1 >= y0 && y1 > y2) maxima.push_back(i);
        if (y1 <= y0 && y1 < y2) minima.push_back(i);
    }

    if (maxima.size() < 2) {
        result.status = Status::NoSeparationFound;
        result.message = "KDE 未检测到至少两个模式";
        if (allowFallback) maybeApplyFallback(data.x, options, result);
        return result;
    }

    // 取最右侧模式与其左侧最近模式之间的局部最小值作为阈值
    const std::size_t hiMode = maxima.back();
    const std::size_t leftMode = maxima[maxima.size() - 2];

    double bestMinY = kInf;
    std::size_t bestMin = std::numeric_limits<std::size_t>::max();
    for (std::size_t idx : minima) {
        if (idx > leftMode && idx < hiMode) {
            if (result.auxCurveY[idx] < bestMinY) {
                bestMinY = result.auxCurveY[idx];
                bestMin = idx;
            }
        }
    }

    if (bestMin == std::numeric_limits<std::size_t>::max()) {
        result.status = Status::NoSeparationFound;
        result.message = "KDE 模式之间没有检测到谷值";
        if (allowFallback) maybeApplyFallback(data.x, options, result);
        return result;
    }

    const double prominence = std::min(result.auxCurveY[leftMode], result.auxCurveY[hiMode]) - result.auxCurveY[bestMin];

    if (prominence < options.minProminence) {
        result.status = Status::NoSeparationFound;
        result.message = "KDE 谷值显著性不足";
        if (allowFallback) maybeApplyFallback(data.x, options, result);
        return result;
    }

    std::vector<double> allThresholds;
    allThresholds.reserve(maxima.size() - 1);
    for (std::size_t modeIdx = 1; modeIdx < maxima.size(); ++modeIdx) {
        const std::size_t currentMode = maxima[modeIdx];
        const std::size_t previousMode = maxima[modeIdx - 1];

        double valleyY = kInf;
        std::size_t valleyIndex = std::numeric_limits<std::size_t>::max();
        for (std::size_t idx : minima) {
            if (idx > previousMode && idx < currentMode && result.auxCurveY[idx] < valleyY) {
                valleyY = result.auxCurveY[idx];
                valleyIndex = idx;
            }
        }

        if (valleyIndex == std::numeric_limits<std::size_t>::max()) continue;

        const double pairProminence = std::min(result.auxCurveY[previousMode], result.auxCurveY[currentMode]) - result.auxCurveY[valleyIndex];
        if (pairProminence >= options.minProminence) {
            allThresholds.push_back(result.auxCurveX[valleyIndex]);
        }
    }

    result.threshold = result.auxCurveX[bestMin];
    populateThresholdBuckets(data.x, allThresholds.empty() ? std::vector<double>{result.threshold} : allThresholds, result);
    result.status = Status::Ok;
    result.message = "成功";
    return result;
}

/* ------------------------------- POT ------------------------------- */

ThresholdResult computePOTRaw(const PreparedData& data, const POTOptions& options, bool allowFallback) {
    ThresholdResult result;
    result.method = Method::POT;

    if (data.x.size() < static_cast<std::size_t>(std::max(10, options.minExceedances))) {
        result.status = Status::NotEnoughData;
        result.message = "POT 所需样本不足";
        if (allowFallback) maybeApplyFallback(data.x, options, result);
        return result;
    }

    if (!(options.initialQuantile > 0.0 && options.initialQuantile < 1.0) ||
        !(options.targetTailProbability > 0.0 && options.targetTailProbability < 1.0)) {
        result.status = Status::InvalidParameter;
        result.message = "initialQuantile 和 targetTailProbability 必须位于 (0,1)";
        if (allowFallback) maybeApplyFallback(data.x, options, result);
        return result;
    }

    const double u = quantileOfCopy(data.x, options.initialQuantile);

    std::vector<double> y;
    y.reserve(data.x.size());
    for (double v : data.x) {
        if (v > u) y.push_back(v - u);
    }

    if (y.size() < static_cast<std::size_t>(options.minExceedances)) {
        result.status = Status::NotEnoughData;
        result.message = "超过初始阈值的样本数不足";
        if (allowFallback) maybeApplyFallback(data.x, options, result);
        return result;
    }

    const double pu = static_cast<double>(y.size()) / static_cast<double>(data.x.size());
    if (!(options.targetTailProbability < pu)) {
        result.status = Status::InvalidParameter;
        result.message = "targetTailProbability 必须小于初始超阈值比例";
        if (allowFallback) maybeApplyFallback(data.x, options, result);
        return result;
    }

    const double m = meanOf(y);
    const double v = varianceOf(y, m, 0.0);
    if (!(m > 0.0) || !(v > 0.0)) {
        result.status = Status::NumericalFailure;
        result.message = "GPD 拟合失败：超阈值样本矩无效";
        if (allowFallback) maybeApplyFallback(data.x, options, result);
        return result;
    }

    // 需求未指定估计器，这里采用 1D 矩估计；若需要可替换为 Grimshaw MLE。
    // xi = 0.5 * (1 - m^2 / v), beta = m * (1 - xi)
    double xi = 0.5 * (1.0 - (m * m) / v);
    xi = clampValue(xi, -0.49, 0.49);  // 避免无限方差或不稳定区域
    double beta = m * (1.0 - xi);

    if (!(beta > 0.0) || !isFinite(beta)) {
        xi = 0.0;  // 退化到指数尾
        beta = m;
    }

    double thr = kNaN;
    if (std::abs(xi) < 1e-8) {
        thr = u + beta * std::log(pu / options.targetTailProbability);
    } else {
        thr = u + (beta / xi) * (std::pow(pu / options.targetTailProbability, xi) - 1.0);
    }

    if (!isFinite(thr) || thr < u) {
        result.status = Status::NumericalFailure;
        result.message = "POT/GPD 外推阈值失败";
        if (allowFallback) maybeApplyFallback(data.x, options, result);
        return result;
    }

    result.threshold = thr;
    populateThresholdBuckets(data.x, {result.threshold}, result);
    result.status = Status::Ok;
    result.message = "成功";

    // centers / weights / variances 作为附加诊断：
    // centers[0] = 初始阈值 u
    // weights[0] = pu
    // variances = {xi, beta}
    result.centers = {u};
    result.weights = {pu};
    result.variances = {xi, beta};

    return result;
}

}  // anonymous namespace

/* =============================== public =============================== */

std::string statusToString(Status status) {
    switch (status) {
        case Status::Ok:
            return "Ok";
        case Status::EmptyInput:
            return "EmptyInput";
        case Status::InvalidParameter:
            return "InvalidParameter";
        case Status::NotEnoughData:
            return "NotEnoughData";
        case Status::NumericalFailure:
            return "NumericalFailure";
        case Status::NoSeparationFound:
            return "NoSeparationFound";
    }
    return "Unknown";
}

std::string methodToString(Method method) {
    switch (method) {
        case Method::MAD:
            return "MAD";
        case Method::IQR:
            return "IQR";
        case Method::Quantile:
            return "Quantile";
        case Method::LogQuantile:
            return "LogQuantile";
        case Method::KMeans:
            return "KMeans";
        case Method::KMeansPP:
            return "KMeansPP";
        case Method::GMM:
            return "GMM";
        case Method::DBSCAN1D:
            return "DBSCAN1D";
        case Method::OPTICS1D:
            return "OPTICS1D";
        case Method::KDEModeSeparation:
            return "KDEModeSeparation";
        case Method::POT:
            return "POT";
    }
    return "Unknown";
}

ThresholdResult computeThreshold(Method method, const std::vector<double>& data, const GenericOptions& options) {
    switch (method) {
        case Method::MAD: {
            MADOptions o;
            static_cast<CommonOptions&>(o) = static_cast<const CommonOptions&>(options);
            o.madScale = options.madScale;
            return computeMADThreshold(data, o);
        }
        case Method::IQR: {
            IQROptions o;
            static_cast<CommonOptions&>(o) = static_cast<const CommonOptions&>(options);
            o.fence = options.fence;
            return computeIQRThreshold(data, o);
        }
        case Method::Quantile: {
            QuantileOptions o;
            static_cast<CommonOptions&>(o) = static_cast<const CommonOptions&>(options);
            o.q = options.q;
            return computeQuantileThreshold(data, o);
        }
        case Method::LogQuantile: {
            LogQuantileOptions o;
            static_cast<CommonOptions&>(o) = static_cast<const CommonOptions&>(options);
            o.q = options.q;
            o.logBase = options.logBase;
            o.shift = options.shift;
            o.autoShiftToPositive = options.autoShiftToPositive;
            return computeLogQuantileThreshold(data, o);
        }
        case Method::KMeans: {
            KMeansOptions o;
            static_cast<CommonOptions&>(o) = static_cast<const CommonOptions&>(options);
            o.k = options.k;
            o.maxIter = options.maxIter;
            o.restarts = options.restarts;
            o.tol = options.tol;
            return computeKMeansThreshold(data, o);
        }
        case Method::KMeansPP: {
            KMeansPPOptions o;
            static_cast<CommonOptions&>(o) = static_cast<const CommonOptions&>(options);
            o.k = options.k;
            o.maxIter = options.maxIter;
            o.restarts = options.restarts;
            o.tol = options.tol;
            return computeKMeansPPThreshold(data, o);
        }
        case Method::GMM: {
            GMMOptions o;
            static_cast<CommonOptions&>(o) = static_cast<const CommonOptions&>(options);
            o.components = options.k;
            o.maxIter = options.maxIter;
            o.restarts = options.restarts;
            o.tol = options.tol;
            o.varianceFloor = options.varianceFloor;
            o.initWithKMeansPP = options.initWithKMeansPP;
            return computeGMMThreshold(data, o);
        }
        case Method::DBSCAN1D: {
            DBSCAN1DOptions o;
            static_cast<CommonOptions&>(o) = static_cast<const CommonOptions&>(options);
            o.eps = options.eps;
            o.autoEps = options.autoEps;
            o.kDistanceQuantile = options.kDistanceQuantile;
            o.minPts = options.minPts;
            return computeDBSCAN1DThreshold(data, o);
        }
        case Method::OPTICS1D: {
            OPTICS1DOptions o;
            static_cast<CommonOptions&>(o) = static_cast<const CommonOptions&>(options);
            o.maxEps = options.maxEps;
            o.autoMaxEps = options.autoMaxEps;
            o.extractEps = options.extractEps;
            o.extractReachabilityQuantile = options.extractReachabilityQuantile;
            o.minPts = options.minPts;
            o.minClusterSize = options.minClusterSize;
            return computeOPTICS1DThreshold(data, o);
        }
        case Method::KDEModeSeparation: {
            KDEOptions o;
            static_cast<CommonOptions&>(o) = static_cast<const CommonOptions&>(options);
            o.gridSize = options.gridSize;
            o.bandwidth = options.bandwidth;
            o.bandwidthScale = options.bandwidthScale;
            o.cut = options.cut;
            o.minProminence = options.minProminence;
            o.useBinnedApproximation = options.useBinnedApproximation;
            return computeKDEModeThreshold(data, o);
        }
        case Method::POT: {
            POTOptions o;
            static_cast<CommonOptions&>(o) = static_cast<const CommonOptions&>(options);
            o.initialQuantile = options.initialQuantile;
            o.targetTailProbability = options.targetTailProbability;
            o.minExceedances = options.minExceedances;
            return computePOTThreshold(data, o);
        }
    }

    ThresholdResult r;
    r.method = method;
    r.status = Status::InvalidParameter;
    r.message = "未知方法";
    return r;
}

ThresholdResult computeMADThreshold(const std::vector<double>& data, const MADOptions& options) {
    ThresholdResult result;
    result.method = Method::MAD;

    PreparedData prep = prepareData(data, options.ignoreNonFinite, result);
    if (prep.x.empty()) return result;

    if (options.madScale < 0.0) {
        result.status = Status::InvalidParameter;
        result.message = "madScale 不能为负";
        return result;
    }

    result.threshold = madThresholdValue(prep.x, options.madScale, options.scaleToNormalSigma);
    result.status = isFinite(result.threshold) ? Status::Ok : Status::NumericalFailure;
    if (result.status == Status::Ok) populateThresholdBuckets(prep.x, {result.threshold}, result);
    result.message = (result.status == Status::Ok) ? "成功" : "MAD 计算失败";
    return result;
}

ThresholdResult computeIQRThreshold(const std::vector<double>& data, const IQROptions& options) {
    ThresholdResult result;
    result.method = Method::IQR;

    PreparedData prep = prepareData(data, options.ignoreNonFinite, result);
    if (prep.x.empty()) return result;

    if (options.fence < 0.0) {
        result.status = Status::InvalidParameter;
        result.message = "fence 不能为负";
        return result;
    }

    result.threshold = iqrThresholdValue(prep.x, options.fence);
    result.status = isFinite(result.threshold) ? Status::Ok : Status::NumericalFailure;
    if (result.status == Status::Ok) populateThresholdBuckets(prep.x, {result.threshold}, result);
    result.message = (result.status == Status::Ok) ? "成功" : "IQR 计算失败";
    return result;
}

ThresholdResult computeQuantileThreshold(const std::vector<double>& data, const QuantileOptions& options) {
    ThresholdResult result;
    result.method = Method::Quantile;

    PreparedData prep = prepareData(data, options.ignoreNonFinite, result);
    if (prep.x.empty()) return result;

    if (!(options.q >= 0.0 && options.q <= 1.0)) {
        result.status = Status::InvalidParameter;
        result.message = "q 必须位于 [0,1]";
        return result;
    }

    result.threshold = quantileThresholdValue(prep.x, options.q);
    result.status = isFinite(result.threshold) ? Status::Ok : Status::NumericalFailure;
    if (result.status == Status::Ok) populateThresholdBuckets(prep.x, {result.threshold}, result);
    result.message = (result.status == Status::Ok) ? "成功" : "Quantile 计算失败";
    return result;
}

ThresholdResult computeLogQuantileThreshold(const std::vector<double>& data, const LogQuantileOptions& options) {
    ThresholdResult result;
    result.method = Method::LogQuantile;

    PreparedData prep = prepareData(data, options.ignoreNonFinite, result);
    if (prep.x.empty()) return result;

    if (!(options.q >= 0.0 && options.q <= 1.0)) {
        result.status = Status::InvalidParameter;
        result.message = "q 必须位于 [0,1]";
        return result;
    }

    result.threshold = logQuantileThresholdValue(prep.x, options.q, options.logBase, options.shift, options.autoShiftToPositive);

    if (!isFinite(result.threshold)) {
        result.status = Status::NumericalFailure;
        result.message = "LogQuantile 计算失败";
        maybeApplyFallback(prep.x, options, result);
        return result;
    }

    result.status = Status::Ok;
    result.message = "成功";
    return result;
}

ThresholdResult computeKMeansThreshold(const std::vector<double>& data, const KMeansOptions& options) {
    ThresholdResult result;
    result.method = Method::KMeans;

    PreparedData prep = prepareData(data, options.ignoreNonFinite, result);
    if (prep.x.empty()) return result;

    if (options.k < 2 || options.maxIter <= 0 || options.restarts <= 0 || options.tol < 0.0) {
        result.status = Status::InvalidParameter;
        result.message = "KMeans 参数无效";
        if (!prep.x.empty()) maybeApplyFallback(prep.x, options, result);
        return result;
    }

    KMeansFit fit = runKMeans1D(prep.x, options.k, options.maxIter, options.restarts, options.tol, options.seed, false);

    if (fit.centers.size() < 2 || fit.labels.empty()) {
        result.status = Status::NumericalFailure;
        result.message = "KMeans 拟合失败";
        maybeApplyFallback(prep.x, options, result);
        return result;
    }

    sortClustersByCenter(fit.centers, fit.labels, fit.counts);
    result.assignments = makeOriginalAssignments(prep, fit.labels);
    result.groupedValues = groupValuesByLabels(prep.x, fit.labels, fit.centers.size());
    result.clusterCounts = fit.counts;
    result.centers = fit.centers;
    result.thresholds = thresholdsFromCenters(fit.centers);
    result.threshold = result.thresholds.empty() ? kNaN : result.thresholds.back();
    result.objective = fit.sse;

    if (!isFinite(result.threshold)) {
        result.status = Status::NoSeparationFound;
        result.message = "KMeans 未得到可用阈值";
        maybeApplyFallback(prep.x, options, result);
        return result;
    }

    result.status = Status::Ok;
    result.message = fit.converged ? "成功" : "成功（达到最大迭代次数）";
    return result;
}

ThresholdResult computeKMeansPPThreshold(const std::vector<double>& data, const KMeansPPOptions& options) {
    ThresholdResult result;
    result.method = Method::KMeansPP;

    PreparedData prep = prepareData(data, options.ignoreNonFinite, result);
    if (prep.x.empty()) return result;

    if (options.k < 2 || options.maxIter <= 0 || options.restarts <= 0 || options.tol < 0.0) {
        result.status = Status::InvalidParameter;
        result.message = "KMeans++ 参数无效";
        if (!prep.x.empty()) maybeApplyFallback(prep.x, options, result);
        return result;
    }

    KMeansFit fit = runKMeans1D(prep.x, options.k, options.maxIter, options.restarts, options.tol, options.seed, true);

    if (fit.centers.size() < 2 || fit.labels.empty()) {
        result.status = Status::NumericalFailure;
        result.message = "KMeans++ 拟合失败";
        maybeApplyFallback(prep.x, options, result);
        return result;
    }

    sortClustersByCenter(fit.centers, fit.labels, fit.counts);
    result.assignments = makeOriginalAssignments(prep, fit.labels);
    result.groupedValues = groupValuesByLabels(prep.x, fit.labels, fit.centers.size());
    result.clusterCounts = fit.counts;
    result.centers = fit.centers;
    result.thresholds = thresholdsFromCenters(fit.centers);
    result.threshold = result.thresholds.empty() ? kNaN : result.thresholds.back();
    result.objective = fit.sse;

    if (!isFinite(result.threshold)) {
        result.status = Status::NoSeparationFound;
        result.message = "KMeans++ 未得到可用阈值";
        maybeApplyFallback(prep.x, options, result);
        return result;
    }

    result.status = Status::Ok;
    result.message = fit.converged ? "成功" : "成功（达到最大迭代次数）";
    return result;
}

ThresholdResult computeGMMThreshold(const std::vector<double>& data, const GMMOptions& options) {
    ThresholdResult result;
    result.method = Method::GMM;

    PreparedData prep = prepareData(data, options.ignoreNonFinite, result);
    if (prep.x.empty()) return result;

    if (options.components < 2 || options.maxIter <= 0 || options.restarts <= 0 || options.tol < 0.0 || options.varianceFloor <= 0.0) {
        result.status = Status::InvalidParameter;
        result.message = "GMM 参数无效";
        maybeApplyFallback(prep.x, options, result);
        return result;
    }

    GMMFit fit = runGMM1D(prep.x, options.components, options.maxIter, options.restarts, options.tol, options.varianceFloor, options.seed,
                          options.initWithKMeansPP);

    if (fit.means.size() < 2 || !isFinite(fit.logLikelihood)) {
        result.status = Status::NumericalFailure;
        result.message = "GMM 拟合失败";
        maybeApplyFallback(prep.x, options, result);
        return result;
    }

    sortClustersByCenter(fit.means, fit.labels, fit.counts, &fit.weights, &fit.vars);
    result.assignments = makeOriginalAssignments(prep, fit.labels);
    result.groupedValues = groupValuesByLabels(prep.x, fit.labels, fit.means.size());
    result.clusterCounts = fit.counts;
    result.centers = fit.means;
    result.weights = fit.weights;
    result.variances = fit.vars;
    result.thresholds = gmmThresholdsFromSortedFit(fit);
    result.threshold = result.thresholds.empty() ? gmmThresholdFromFit(fit) : result.thresholds.back();
    result.objective = fit.logLikelihood;

    if (!isFinite(result.threshold)) {
        result.status = Status::NoSeparationFound;
        result.message = "GMM 未得到可用交点";
        maybeApplyFallback(prep.x, options, result);
        return result;
    }

    result.status = Status::Ok;
    result.message = fit.converged ? "成功" : "成功（达到最大迭代次数）";
    return result;
}

ThresholdResult computeDBSCAN1DThreshold(const std::vector<double>& data, const DBSCAN1DOptions& options) {
    ThresholdResult result;
    result.method = Method::DBSCAN1D;

    PreparedData prep = prepareData(data, options.ignoreNonFinite, result);
    if (prep.x.empty()) return result;

    double eps = options.eps;
    if (!(eps > 0.0) && options.autoEps) {
        SortedData sorted = sortPrepared(prep);
        eps = autoEpsFromKDistance(sorted.x, options.minPts, options.kDistanceQuantile);
    }

    return computeDBSCAN1DRaw(prep, Method::DBSCAN1D, eps, options.minPts, true, options);
}

ThresholdResult computeOPTICS1DThreshold(const std::vector<double>& data, const OPTICS1DOptions& options) {
    ThresholdResult result;
    result.method = Method::OPTICS1D;

    PreparedData prep = prepareData(data, options.ignoreNonFinite, result);
    if (prep.x.empty()) return result;

    double maxEps = options.maxEps;
    if (!(maxEps > 0.0) && options.autoMaxEps) {
        SortedData sorted = sortPrepared(prep);
        maxEps = autoEpsFromKDistance(sorted.x, options.minPts, options.extractReachabilityQuantile);
    }

    return computeOPTICS1DRaw(prep, Method::OPTICS1D, maxEps, options.minPts, options.extractEps, options.minClusterSize, true, options);
}

ThresholdResult computeKDEModeThreshold(const std::vector<double>& data, const KDEOptions& options) {
    ThresholdResult result;
    result.method = Method::KDEModeSeparation;

    PreparedData prep = prepareData(data, options.ignoreNonFinite, result);
    if (prep.x.empty()) return result;

    return computeKDERaw(prep, options, true);
}

ThresholdResult computePOTThreshold(const std::vector<double>& data, const POTOptions& options) {
    ThresholdResult result;
    result.method = Method::POT;

    PreparedData prep = prepareData(data, options.ignoreNonFinite, result);
    if (prep.x.empty()) return result;

    return computePOTRaw(prep, options, true);
}

}  // namespace SeamClustering
