#ifndef SEAM_CLUSTERING_H
#define SEAM_CLUSTERING_H

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace SeamClustering {

enum class Method { MAD, IQR, Quantile, LogQuantile, KMeans, KMeansPP, GMM, DBSCAN1D, OPTICS1D, KDEModeSeparation, POT };

enum class Status { Ok, EmptyInput, InvalidParameter, NotEnoughData, NumericalFailure, NoSeparationFound };

// =========================== 通用基础配置 ===========================

// 回退策略：当当前方法失败、数据不足、数值不稳定、未分出有效簇时，
// 可以自动回退到更稳妥的阈值法，例如 MAD 或 Quantile。
enum class FallbackPolicy {
    None,     // 不回退，失败就直接返回失败状态
    MAD,      // 回退到 median + scale * MAD
    Quantile  // 回退到分位数阈值
};

struct CommonOptions {
    // 是否忽略非有限值（NaN / +Inf / -Inf）
    // true：预处理时自动过滤这些非法值，推荐默认开启
    // false：如果数据中存在非法值，很多方法会直接失败
    bool ignoreNonFinite = true;

    // 当当前算法失败时的兜底策略
    // 例如：
    // - GMM 没收敛
    // - DBSCAN 没找到有效高值簇
    // - KDE 没找到两个明显峰
    // 默认回退到 MAD，因为它通常最稳
    FallbackPolicy fallback = FallbackPolicy::MAD;

    // 当 fallback = Quantile 时，使用的分位数
    // 例如 0.90 表示取 90% 分位作为阈值
    // 适合“只想保留最靠右的一部分高值”
    double fallbackQuantile = 0.90;

    // 随机种子
    // 用于 KMeans / KMeans++ / GMM 等带随机初始化的方法
    // 固定后结果可复现，调试和对比实验时非常重要
    unsigned int seed = 42u;
};

// =========================== MAD 阈值法 ===========================
// 原理：threshold = median(x) + madScale * MAD
// 如果 scaleToNormalSigma = true，则 MAD 会先乘 1.4826，
// 使其在正态分布下近似于标准差 sigma。
struct MADOptions : public CommonOptions {
    // MAD 的放大倍数
    // 越大：阈值越高，筛选越严格，保留点越少
    // 越小：阈值越低，筛选越宽松，保留点越多
    // 常用范围：
    // 1.0 ~ 3.0
    double madScale = 1.0;

    // 是否把 MAD 按正态分布等效为 sigma
    // true：MAD * 1.4826，再参与阈值计算，更接近“均值+sigma”的尺度理解
    // false：直接使用原始 MAD
    bool scaleToNormalSigma = true;
};

// =========================== IQR 箱线图阈值法 ===========================
// 原理：threshold = Q3 + fence * (Q3 - Q1)
// 对长尾和异常值较稳健，通常比均值方差更适合工业数据。
struct IQROptions : public CommonOptions {
    // 箱线图 fence 系数
    // 1.5：标准异常值界限
    // 3.0：更严格，只保留更极端的高值
    // 越大：阈值越高
    double fence = 1.5;
};

// =========================== 分位数阈值法 ===========================
// 原理：threshold = quantile(x, q)
// 最直接、最快，适合“我只想保留右侧 top p% 的值”。
struct QuantileOptions : public CommonOptions {
    // 目标分位数，范围 [0,1]
    // 0.90 表示 90% 分位
    // 0.95 表示更严格，只保留更高的一小部分
    double q = 0.90;
};

// =========================== 对数分位数阈值法 ===========================
// 原理：先对数据做 log 变换，再取分位数，再反变换。
// 适合明显长尾、右尾很长、跨度很大的数据。
struct LogQuantileOptions : public CommonOptions {
    // 对数域中使用的分位数
    // 含义与 QuantileOptions::q 相同
    double q = 0.90;

    // 对数底
    // 自然对数 e：最常用
    // 10：如果你更习惯十进制尺度
    // 2：如果你更喜欢“翻倍”尺度
    double logBase = 2.71828182845904523536;

    // 对数前的平移量
    // 实际变换近似为 log(x + shift)
    // 用于避免 x<=0 导致无法取对数
    double shift = 1.0;

    // 是否自动把数据整体平移到正数区间
    // true：如果最小值太小，会自动增加 shift，保证 log 有定义
    // false：严格使用你提供的 shift
    bool autoShiftToPositive = true;
};

// =========================== KMeans 聚类 ===========================
// 原理：把 1D 数据分成 k 类，然后根据簇中心或高值簇边界推阈值。
// 适合“数据大致能分成低值群和高值群”的情况。
struct KMeansOptions : public CommonOptions {
    // 聚类簇数
    // 对阈值问题通常设为 2：低值簇 + 高值簇
    // 如果你想做更细分，可设为 3 或更多
    int k = 2;

    // 单次 KMeans 最大迭代次数
    // 越大越容易收敛，但耗时会增加
    int maxIter = 100;

    // 重启次数
    // KMeans 会受初始中心影响，restarts 越多，越不容易陷入差的局部最优
    // 工程上很有用
    int restarts = 8;

    // 收敛阈值
    // 两次迭代中心变化小于 tol 时停止
    // 越小：越精细，但迭代可能更多
    double tol = 1e-6;
};

// =========================== KMeans++ 聚类 ===========================
// 与 KMeansOptions 相同，只是初始化方式更好。
// KMeans++ 会优先把初始中心分散开，通常更稳、更少陷入坏局部最优。
struct KMeansPPOptions : public KMeansOptions {};

// =========================== GMM 高斯混合模型 ===========================
// 原理：把数据拟合为多个高斯分布的加权和，通常用于“低值一团 + 高值一团”。
// 比 KMeans 更柔性，因为它允许簇重叠、方差不同。
struct GMMOptions : public CommonOptions {
    // 高斯成分个数
    // 阈值问题常用 2：低值高斯 + 高值高斯
    int components = 2;

    // EM 最大迭代次数
    int maxIter = 200;

    // 多次随机重启次数
    // GMM 也会受初始化影响，restarts 越多越稳，但更慢
    int restarts = 5;

    // 收敛阈值
    // 通常比较的是对数似然提升量
    double tol = 1e-6;

    // 方差下界
    // 防止某个成分方差塌缩到 0，导致数值爆炸
    // 是 GMM 很关键的稳定参数
    double varianceFloor = 1e-9;

    // 是否用 KMeans++ 初始化 GMM
    // true：通常更稳
    // false：可改为随机初始化
    bool initWithKMeansPP = true;
};

// =========================== DBSCAN 1D ===========================
// 原理：基于密度，把一维数据里“足够密集的区间”识别为簇。
// 适合找连续段 / 右侧高密度尾部簇。
struct DBSCAN1DOptions : public CommonOptions {
    // 邻域半径 eps
    // 两个点差值 <= eps 就认为相邻
    // 如果 <=0，则由 autoEps 自动估计
    double eps = 0.0;

    // 一个核心点至少需要多少邻域点
    // 越大：簇更稳定，但更容易漏掉小簇
    int minPts = 5;

    // 是否自动估计 eps
    // true：通常根据 k-distance 分布自动取一个分位数
    bool autoEps = true;

    // 自动估计 eps 时，使用的 k-distance 分位数
    // 越大：估计出的 eps 往往越大，簇更容易连起来
    // 越小：eps 更小，簇更碎、更严格
    double kDistanceQuantile = 0.90;
};

// =========================== OPTICS 1D ===========================
// 原理：与 DBSCAN 类似，但不要求固定一个 eps，能描述多密度簇。
// 更适合数据密度变化明显的情况。
struct OPTICS1DOptions : public CommonOptions {
    // 最大搜索半径 maxEps
    // 如果 <=0，则自动估计
    double maxEps = 0.0;

    // 形成核心点的最小邻域点数
    int minPts = 5;

    // 最终从 reachability 曲线抽取簇时的 eps
    // 如果 <=0，则由 reachability 曲线自动估计
    double extractEps = 0.0;

    // 自动估计 extractEps 时使用的 reachability 分位数
    // 越大：抽簇更宽松
    // 越小：抽簇更严格
    double extractReachabilityQuantile = 0.90;

    // 最小簇大小
    // 小于这个大小的簇直接忽略
    int minClusterSize = 5;

    // 是否自动估计 maxEps
    bool autoMaxEps = true;
};

// =========================== KDE 核密度估计 ===========================
// 原理：对一维数据估计连续密度曲线，然后找峰、谷、模态分离点。
// 适合双峰/多峰分析，比硬聚类更平滑。 只适合双峰
struct KDEOptions : public CommonOptions {
    // 密度曲线采样网格数
    // 越大：曲线更细致，但更慢
    // 常见 256 / 512 / 1024
    std::size_t gridSize = 512;

    // 核带宽
    // <=0 时自动用 Silverman 法则估计
    // 是 KDE 最关键参数：
    // - 小：曲线锯齿多，容易虚假峰
    // - 大：曲线过平滑，容易把双峰抹成单峰
    double bandwidth = 0.0;

    // 带宽缩放因子
    // 最终实际带宽 = 自动估计带宽 * bandwidthScale
    // >1：更平滑
    // <1：更保留细节
    double bandwidthScale = 1.0;

    // 在数据范围外额外扩展多少个带宽做采样
    // 避免边界效应
    double cut = 3.0;

    // 最小峰显著性
    // 如果两个峰之间的“谷”不够明显，则不认为是真分离
    // 越大：越保守，假峰更少
    double minProminence = 0.0;

    // 是否使用分箱近似
    // true：更快，适合大数据
    // false：逐点核叠加，更精确但更慢
    bool useBinnedApproximation = true;
};

// =========================== POT / 极值理论 ===========================
// POT = Peaks Over Threshold
// 原理：先取一个较高基线阈值 u，只对超过 u 的尾部数据建模，
// 再用 GPD（广义帕累托分布）外推更高阈值。
// 适合“只关心最右侧极端值”的情况。
struct POTOptions : public CommonOptions {
    // 初始 POT 基线阈值 u 的分位数
    // 例如 0.90 表示先取 90% 分位作为基线
    // 太低：尾部不纯
    // 太高：尾部样本太少
    double initialQuantile = 0.90;

    // 目标尾概率 P(X > T)
    // 例如 0.01 表示想求“只有 1% 数据会超过”的极端阈值 T
    // 越小：阈值越高
    double targetTailProbability = 0.01;

    // 拟合 GPD 至少需要的超阈值样本数
    // 太少会导致极值拟合不稳定
    int minExceedances = 30;
};

// =========================== 通用 dispatcher 参数 ===========================
// 这是“总控参数”，用于统一调用不同算法。
// 比如：computeThreshold(method, data, GenericOptions)
// 内部再根据 method 拆给各个算法。
struct GenericOptions : public CommonOptions {
    // ===== 通用统计阈值参数 =====

    // Quantile / LogQuantile / fallback 中可能会用到的分位数
    double q = 0.90;

    // MAD 使用的放大倍数
    double madScale = 1.0;

    // IQR 使用的 fence
    double fence = 1.5;

    // LogQuantile 的对数底
    double logBase = 2.71828182845904523536;

    // LogQuantile 的平移项
    double shift = 1.0;

    // 是否自动平移到正区间
    bool autoShiftToPositive = true;

    // ===== KMeans / GMM 通用聚类参数 =====

    // 聚类数 / 高斯成分数
    int k = 2;

    // 最大迭代次数
    int maxIter = 100;

    // 随机重启次数
    int restarts = 8;

    // 收敛精度
    double tol = 1e-6;

    // GMM 方差下界
    double varianceFloor = 1e-9;

    // 是否用 KMeans++ 初始化
    bool initWithKMeansPP = true;

    // ===== DBSCAN 相关参数 =====

    // 固定 eps；<=0 时用自动估计
    double eps = 0.0;

    // 是否自动估计 eps
    bool autoEps = true;

    // 自动估计 eps 时所用分位数
    double kDistanceQuantile = 0.90;

    // DBSCAN / OPTICS 共用最小邻域点数
    int minPts = 5;

    // ===== OPTICS 相关参数 =====

    // 最大搜索半径；<=0 时自动估计
    double maxEps = 0.0;

    // 是否自动估计 maxEps
    bool autoMaxEps = true;

    // 抽簇使用的 eps；<=0 时自动估计
    double extractEps = 0.0;

    // 自动估计 extractEps 时的分位数
    double extractReachabilityQuantile = 0.90;

    // 最小簇大小
    int minClusterSize = 5;

    // ===== KDE 相关参数 =====

    // KDE 网格采样数
    std::size_t gridSize = 512;

    // KDE 带宽；<=0 时自动估计
    double bandwidth = 0.0;

    // KDE 带宽缩放
    double bandwidthScale = 1.0;

    // KDE 额外扩展范围（以带宽为单位）
    double cut = 3.0;

    // 最小峰显著性
    double minProminence = 0.0;

    // 是否使用分箱近似
    bool useBinnedApproximation = true;

    // ===== POT 相关参数 =====

    // POT 的初始基线分位数
    double initialQuantile = 0.90;

    // POT 目标尾概率
    double targetTailProbability = 0.01;

    // POT 最少超阈值样本数
    int minExceedances = 30;
};

struct ThresholdResult {
    double threshold = std::numeric_limits<double>::quiet_NaN();
    Status status = Status::InvalidParameter;
    Method method = Method::MAD;
    bool usedFallback = false;
    std::string message;
    std::vector<std::vector<double>> groupedValues;
    std::vector<double> thresholds;

    // 若 ignoreNonFinite = true，则 assignments 与原始输入等长；
    // 被忽略的非法值位置为 -1。
    std::vector<int> assignments;
    std::vector<int> clusterCounts;
    std::vector<double> centers;
    std::vector<double> weights;
    std::vector<double> variances;
    std::vector<double> reachability;
    std::vector<int> ordering;
    std::vector<double> auxCurveX;
    std::vector<double> auxCurveY;

    // KMeans: SSE；GMM: 对数似然等
    double objective = std::numeric_limits<double>::quiet_NaN();
};

std::string statusToString(Status status);
std::string methodToString(Method method);

// 统一 dispatcher
ThresholdResult computeThreshold(Method method, const std::vector<double>& data, const GenericOptions& options = GenericOptions{});

// 基础稳健阈值
ThresholdResult computeMADThreshold(const std::vector<double>& data, const MADOptions& options = MADOptions{});
ThresholdResult computeIQRThreshold(const std::vector<double>& data, const IQROptions& options = IQROptions{});
ThresholdResult computeQuantileThreshold(const std::vector<double>& data, const QuantileOptions& options = QuantileOptions{});
ThresholdResult computeLogQuantileThreshold(const std::vector<double>& data, const LogQuantileOptions& options = LogQuantileOptions{});

// 1D 聚类/混合模型
ThresholdResult computeKMeansThreshold(const std::vector<double>& data, const KMeansOptions& options = KMeansOptions{});
ThresholdResult computeKMeansPPThreshold(const std::vector<double>& data, const KMeansPPOptions& options = KMeansPPOptions{});
ThresholdResult computeGMMThreshold(const std::vector<double>& data, const GMMOptions& options = GMMOptions{});

// 1D 密度 / 尾部 / 区间簇方法
ThresholdResult computeDBSCAN1DThreshold(const std::vector<double>& data, const DBSCAN1DOptions& options = DBSCAN1DOptions{});
ThresholdResult computeOPTICS1DThreshold(const std::vector<double>& data, const OPTICS1DOptions& options = OPTICS1DOptions{});
ThresholdResult computeKDEModeThreshold(const std::vector<double>& data, const KDEOptions& options = KDEOptions{});
ThresholdResult computePOTThreshold(const std::vector<double>& data, const POTOptions& options = POTOptions{});

}  // namespace SeamClustering
/* // 运行demo
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#ifdef _WIN32
#include <direct.h>
#endif

#include "src/utils/common/SeamClustering.h"

using CloudT = pcl::PointCloud<pcl::PointXYZI>;
typedef std::chrono::high_resolution_clock Clock;

// ===========================
// C++11 版路径工具
// ===========================

// 统一把路径里的 '\' 转成 '/'
// 这样后面拆目录更稳定
static std::string normalizePath(const std::string& path) {
    std::string s = path;
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '\\') s[i] = '/';
    }
    return s;
}

// 取父目录
static std::string getParentDir(const std::string& path) {
    std::string s = normalizePath(path);
    std::string::size_type pos = s.find_last_of('/');
    if (pos == std::string::npos) return ".";
    if (pos == 0) return "/";
    return s.substr(0, pos);
}

// 判断目录是否存在
static bool directoryExists(const std::string& path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0) return false;
    return (info.st_mode & S_IFDIR) != 0;
}

// 创建单层目录
static bool createSingleDir(const std::string& path) {
    if (path.empty() || directoryExists(path)) return true;

#ifdef _WIN32
    int ret = _mkdir(path.c_str());
#else
    int ret = mkdir(path.c_str(), 0755);
#endif
    return ret == 0 || directoryExists(path);
}

// 递归创建多层目录，替代 std::filesystem::create_directories
static bool createDirectories(const std::string& path) {
    if (path.empty()) return false;

    std::string s = normalizePath(path);
    if (directoryExists(s)) return true;

    std::string current;
    size_t start = 0;

    // Windows 盘符处理，如 D:/...
    if (s.size() >= 2 && s[1] == ':') {
        current = s.substr(0, 2);  // "D:"
        start = 2;
    }

    // 根目录开头
    if (start < s.size() && s[start] == '/') {
        current += "/";
        ++start;
    }

    std::stringstream ss(s.substr(start));
    std::string item;

    while (std::getline(ss, item, '/')) {
        if (item.empty()) continue;

        if (!current.empty() && current[current.size() - 1] != '/') current += "/";
        current += item;

        if (!directoryExists(current)) {
            if (!createSingleDir(current)) {
                return false;
            }
        }
    }

    return true;
}

// 拼接路径
static std::string joinPath(const std::string& dir, const std::string& fileName) {
    if (dir.empty()) return fileName;
    char last = dir[dir.size() - 1];
    if (last == '/' || last == '\\') return dir + fileName;
    return dir + "/" + fileName;
}

// ===========================
// 点云阈值过滤
// ===========================
static std::vector<double> getEffectiveThresholds(const SeamClustering::ThresholdResult& result) {
    if (!result.thresholds.empty()) return result.thresholds;
    if (std::isfinite(result.threshold)) return std::vector<double>{result.threshold};
    return {};
}

static std::vector<size_t> getEffectiveGroupSizes(const SeamClustering::ThresholdResult& result, const std::vector<double>& values) {
    if (!result.groupedValues.empty()) {
        std::vector<size_t> sizes;
        sizes.reserve(result.groupedValues.size());
        for (size_t i = 0; i < result.groupedValues.size(); ++i) {
            sizes.push_back(result.groupedValues[i].size());
        }
        return sizes;
    }

    const std::vector<double> thresholds = getEffectiveThresholds(result);
    if (thresholds.empty()) return {};

    std::vector<size_t> sizes(thresholds.size() + 1, 0);
    for (size_t i = 0; i < values.size(); ++i) {
        const size_t bucket = static_cast<size_t>(std::upper_bound(thresholds.begin(), thresholds.end(), values[i]) - thresholds.begin());
        sizes[bucket] += 1;
    }
    return sizes;
}

static std::string formatDoubleList(const std::vector<double>& values) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) oss << ", ";
        oss << std::setprecision(6) << values[i];
    }
    oss << "]";
    return oss.str();
}

static std::string formatSizeList(const std::vector<size_t>& values) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) oss << ", ";
        oss << values[i];
    }
    oss << "]";
    return oss.str();
}

static CloudT::Ptr makeEmptyCloudLike(const CloudT::ConstPtr& in) {
    CloudT::Ptr out(new CloudT);
    out->header = in->header;
    out->width = 0;
    out->height = 1;
    out->is_dense = false;
    return out;
}

static std::vector<CloudT::Ptr> splitCloudByThresholds(const CloudT::ConstPtr& in, const std::vector<double>& thresholds) {
    std::vector<CloudT::Ptr> segments(thresholds.size() + 1);
    for (size_t i = 0; i < segments.size(); ++i) {
        segments[i] = makeEmptyCloudLike(in);
        segments[i]->reserve(in->size() / std::max<size_t>(segments.size(), 1));
    }

    for (size_t i = 0; i < in->points.size(); ++i) {
        const pcl::PointXYZI& p = in->points[i];
        if (!std::isfinite(p.intensity)) continue;

        const size_t bucket =
            static_cast<size_t>(std::upper_bound(thresholds.begin(), thresholds.end(), static_cast<double>(p.intensity)) - thresholds.begin());
        segments[bucket]->points.push_back(p);
    }

    for (size_t i = 0; i < segments.size(); ++i) {
        segments[i]->width = static_cast<uint32_t>(segments[i]->points.size());
    }

    return segments;
}

static void saveSegmentedPCDs(const CloudT::ConstPtr& in, const std::vector<double>& thresholds, const std::string& outDir, const std::string& stem) {
    const std::vector<CloudT::Ptr> segments = splitCloudByThresholds(in, thresholds);
    for (size_t i = 0; i < segments.size(); ++i) {
        std::ostringstream fileName;
        fileName << stem << "_segment_" << std::setw(2) << std::setfill('0') << i << ".pcd";
        pcl::io::savePCDFileBinary(joinPath(outDir, fileName.str()), *segments[i]);
    }
}

int main() {
    const std::string filePath =
        "D:/Code/3EA-Welding/3EA-Welding/data/"
        "seamDetWithPointCloud/tubePlateFilletSeamsDet/"
        "debug_meanDeviation_intensity_0.pcd";

    CloudT::Ptr cloud(new CloudT);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(filePath, *cloud) != 0) {
        std::cerr << "读取 PCD 失败: " << filePath << std::endl;
        return 1;
    }

    std::vector<double> feat;
    feat.reserve(cloud->size());
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const pcl::PointXYZI& p = cloud->points[i];
        if (std::isfinite(p.intensity)) {
            feat.push_back(static_cast<double>(p.intensity));
        }
    }

    if (feat.empty()) {
        std::cerr << "点云里没有有效 intensity" << std::endl;
        return 1;
    }

    // C++11 替代 std::filesystem
    const std::string inDir = getParentDir(filePath);
    const std::string outDir = joinPath(inDir, "threshold_compare_outputs");
    if (!createDirectories(outDir)) {
        std::cerr << "创建输出目录失败: " << outDir << std::endl;
        return 1;
    }

    std::function<void(const std::string&, const std::function<SeamClustering::ThresholdResult()>&)> runAndReport =
        [&](const std::string& name, const std::function<SeamClustering::ThresholdResult()>& fn) {
            const Clock::time_point t1 = Clock::now();
            SeamClustering::ThresholdResult r = fn();
            const Clock::time_point t2 = Clock::now();

            const double us = std::chrono::duration<double, std::micro>(t2 - t1).count();
            const std::vector<double> thresholds = getEffectiveThresholds(r);
            const std::vector<size_t> groupSizes = getEffectiveGroupSizes(r, feat);

            std::cout << std::left << std::setw(18) << name << " status=" << std::setw(18) << SeamClustering::statusToString(r.status)
                      << " thr=" << std::setw(14) << r.threshold << " thrCount=" << std::setw(4) << thresholds.size()
                      << " groupCount=" << std::setw(4) << groupSizes.size() << " time(us)=" << std::setw(12) << us
                      << " fallback=" << (r.usedFallback ? "yes" : "no") << " msg=" << r.message << std::endl;

            if (!thresholds.empty()) {
                std::cout << "  thresholds=" << formatDoubleList(thresholds) << std::endl;
            }
            if (!groupSizes.empty()) {
                std::cout << "  groupSizes=" << formatSizeList(groupSizes) << std::endl;
            }

            if (r.status == SeamClustering::Status::Ok) {
                const std::string methodDir = joinPath(outDir, name);
                if (!createDirectories(methodDir)) {
                    // std::cerr << "疲惫å想放假ºæ没有个头¹æ³找不到工作è¾赚不到¥: " << methodDir << std::endl;
                    return;
                }
                saveSegmentedPCDs(cloud, thresholds, methodDir, name);
            }
        };

    // MAD
    SeamClustering::MADOptions madOpt;
    madOpt.madScale = 1.0;
    madOpt.scaleToNormalSigma = true;
    madOpt.fallback = SeamClustering::FallbackPolicy::None;

    // IQR
    SeamClustering::IQROptions iqrOpt;
    iqrOpt.fence = 1.5;
    iqrOpt.fallback = SeamClustering::FallbackPolicy::None;

    // Quantile
    SeamClustering::QuantileOptions qOpt;
    qOpt.q = 0.90;
    qOpt.fallback = SeamClustering::FallbackPolicy::None;

    // LogQuantile
    SeamClustering::LogQuantileOptions lqOpt;
    lqOpt.q = 0.90;
    lqOpt.logBase = std::exp(1.0);
    lqOpt.shift = 1.0;
    lqOpt.autoShiftToPositive = true;
    lqOpt.fallback = SeamClustering::FallbackPolicy::None;

    // KMeans
    SeamClustering::KMeansOptions kmOpt;
    kmOpt.k = 2;
    kmOpt.maxIter = 100;
    kmOpt.restarts = 1;
    kmOpt.tol = 1e-6;
    kmOpt.seed = 42;  // 随机种子，用于可复现
    kmOpt.fallback = SeamClustering::FallbackPolicy::MAD;

    // KMeans++
    SeamClustering::KMeansPPOptions kmppOpt;
    kmppOpt.k = 2;
    kmppOpt.maxIter = 100;
    kmppOpt.restarts = 1;
    kmppOpt.tol = 1e-6;
    kmppOpt.seed = 42;
    kmppOpt.fallback = SeamClustering::FallbackPolicy::MAD;

    // GMM
    SeamClustering::GMMOptions gmmOpt;
    gmmOpt.components = 2;
    gmmOpt.maxIter = 200;
    gmmOpt.restarts = 1;
    gmmOpt.tol = 1e-6;
    gmmOpt.varianceFloor = 1e-9;
    gmmOpt.initWithKMeansPP = true;
    gmmOpt.seed = 42;
    gmmOpt.fallback = SeamClustering::FallbackPolicy::MAD;

    // DBSCAN 1D
    SeamClustering::DBSCAN1DOptions dbOpt;
    dbOpt.eps = 0.0;
    dbOpt.minPts = 5;
    dbOpt.autoEps = true;
    dbOpt.kDistanceQuantile = 0.90;
    dbOpt.fallback = SeamClustering::FallbackPolicy::MAD;

    // OPTICS 1D
    SeamClustering::OPTICS1DOptions optOpt;
    optOpt.maxEps = 0.0;
    optOpt.minPts = 5;
    optOpt.extractEps = 0.0;
    optOpt.extractReachabilityQuantile = 0.90;
    optOpt.minClusterSize = 5;
    optOpt.autoMaxEps = true;
    optOpt.fallback = SeamClustering::FallbackPolicy::MAD;

    // KDE
    SeamClustering::KDEOptions kdeOpt;
    kdeOpt.gridSize = 64;
    kdeOpt.bandwidth = 0.0;
    kdeOpt.bandwidthScale = 1.0;
    kdeOpt.cut = 3.0;
    kdeOpt.minProminence = 0.0;
    kdeOpt.useBinnedApproximation = true;
    kdeOpt.fallback = SeamClustering::FallbackPolicy::MAD;

    // POT
    SeamClustering::POTOptions potOpt;
    potOpt.initialQuantile = 0.90;
    potOpt.targetTailProbability = 0.01;
    potOpt.minExceedances = 30;
    potOpt.fallback = SeamClustering::FallbackPolicy::MAD;

    runAndReport("MAD", [&]() { return SeamClustering::computeMADThreshold(feat, madOpt); });
    runAndReport("IQR", [&]() { return SeamClustering::computeIQRThreshold(feat, iqrOpt); });
    runAndReport("Quantile90", [&]() { return SeamClustering::computeQuantileThreshold(feat, qOpt); });
    runAndReport("LogQuantile", [&]() { return SeamClustering::computeLogQuantileThreshold(feat, lqOpt); });
    runAndReport("KMeans", [&]() { return SeamClustering::computeKMeansThreshold(feat, kmOpt); });
    runAndReport("KMeansPP", [&]() { return SeamClustering::computeKMeansPPThreshold(feat, kmppOpt); });
    runAndReport("GMM", [&]() { return SeamClustering::computeGMMThreshold(feat, gmmOpt); });
    runAndReport("DBSCAN1D", [&]() { return SeamClustering::computeDBSCAN1DThreshold(feat, dbOpt); });
    runAndReport("OPTICS1D", [&]() { return SeamClustering::computeOPTICS1DThreshold(feat, optOpt); });
    runAndReport("KDE", [&]() { return SeamClustering::computeKDEModeThreshold(feat, kdeOpt); });
    runAndReport("POT", [&]() { return SeamClustering::computePOTThreshold(feat, potOpt); });

    return 0;
}
*/
#endif  // SEAM_CLUSTERING_H
