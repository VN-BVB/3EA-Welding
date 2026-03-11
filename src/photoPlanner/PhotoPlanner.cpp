#include "PhotoPlanner.h"

#include <algorithm>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

PhotoPlanner::PhotoPlanner(double photoWidth, double photoHeight) : photoWidth_(photoWidth), photoHeight_(photoHeight) {}

// 根据粗定位框规划拍照位置
std::vector<cv::Point2d> PhotoPlanner::plan(const std::vector<cv::Rect_<double>>& roughRegions) {
    std::vector<cv::Point2d> photoCenters;
    if (roughRegions.empty()) return photoCenters;

    // 当前还未覆盖的粗定位框索引
    std::vector<size_t> uncoveredIndices(roughRegions.size());
    for (size_t i = 0; i < roughRegions.size(); ++i) uncoveredIndices[i] = i;

    // 循环计算拍照中心点, 直到每个粗定位框都被覆盖
    while (!uncoveredIndices.empty()) {
        cv::Point2d center = getBestPhotoCenter(roughRegions, uncoveredIndices);  // 计算出一个拍照中心
        photoCenters.push_back(center);

        cv::Rect_<double> photoArea(center.x - photoWidth_ / 2.0, center.y - photoHeight_ / 2.0, photoWidth_, photoHeight_);
        std::vector<size_t> covered = getCoveredIndices(photoArea, roughRegions, uncoveredIndices);  // 获取已覆盖区域索引

        // 更新未覆盖粗定位框索引容器
        std::set<size_t> coveredSet(covered.begin(), covered.end());
        std::vector<size_t> newUncoveredIndices;
        for (size_t idx : uncoveredIndices) {
            if (coveredSet.find(idx) == coveredSet.end()) {
                newUncoveredIndices.push_back(idx);
            }
        }
        uncoveredIndices = std::move(newUncoveredIndices);
    }

    // 合并拍照位置结果, 将『能用一个拍照位置同时拍到的多个位置』合并为一个拍照位置
    return mergePhotoCenters(photoCenters, roughRegions);
}

// 判断一个矩形框是否完全在另一个框的内部
bool PhotoPlanner::isFullyCovered(const cv::Rect_<double>& outer, const cv::Rect_<double>& inner) const {
    return outer.x <= inner.x && outer.y <= inner.y && outer.x + outer.width >= inner.x + inner.width &&
           outer.y + outer.height >= inner.y + inner.height;
}

/**
 * @brief PhotoPlanner::getCoveredIndices 获取 roughRegions 中被 photoArea 完全覆盖区域的索引
 * @param photoArea 当前拍照区域
 * @param roughRegions 所有粗定位区域
 * @param uncoveredIndices 还未被覆盖区域索引
 * @return 被完全覆盖的粗定位区域索引
 */
std::vector<size_t> PhotoPlanner::getCoveredIndices(const cv::Rect_<double>& photoArea, const std::vector<cv::Rect_<double>>& roughRegions,
                                                    const std::vector<size_t>& uncoveredIndices) const {
    std::vector<size_t> covered;

    for (size_t idx : uncoveredIndices) {
        if (isFullyCovered(photoArea, roughRegions[idx])) {
            covered.push_back(idx);
        }
    }

    return covered;
}

/**
 * @brief PhotoPlanner::getBestPhotoCenter 根据 roughRegions 和 uncoveredIndices, 确定下一个拍照中心点
 * @param roughRegions 所有粗定位区域框
 * @param uncoveredIndices 还未覆盖的区域框序号
 * @return 当前找到的拍照位置中心点
 */
cv::Point2d PhotoPlanner::getBestPhotoCenter(const std::vector<cv::Rect_<double>>& roughRegions, const std::vector<size_t>& uncoveredIndices) const {
    size_t bestCount = 0;
    cv::Point2d bestCenter;
    std::vector<size_t> bestCovered;

    // 遍历所有未覆盖的粗定位区域, 尝试在每个区域的中心点和四个角落寻找最佳拍照位置
    for (size_t idx : uncoveredIndices) {
        const cv::Rect_<double>& rect = roughRegions[idx];

        // 候选拍照中心点
        std::vector<cv::Point2d> candidates = {
            {rect.x + rect.width / 2.0, rect.y + rect.height / 2.0},
            {rect.x,                    rect.y                    },
            {rect.x + rect.width,       rect.y                    },
            {rect.x,                    rect.y + rect.height      },
            {rect.x + rect.width,       rect.y + rect.height      }
        };

        // 对于每个候选中心点, 检查其覆盖的粗定位区域数量
        for (const cv::Point2d& center : candidates) {
            cv::Rect_<double> photoArea(center.x - photoWidth_ / 2.0, center.y - photoHeight_ / 2.0, photoWidth_, photoHeight_);
            std::vector<size_t> covered = getCoveredIndices(photoArea, roughRegions, uncoveredIndices);  // 获取能被覆盖的区域索引

            // 当前覆盖的区域数量大于之前的最佳数量, 则更新最佳拍照位置
            if (covered.size() > bestCount) {
                bestCount = covered.size();
                bestCenter = center;
                bestCovered = std::move(covered);
            }
        }
    }

    if (bestCount == 0) {
        PLOGE << "无法找到能覆盖剩余粗定位区域的相机视野，请检查参数或算法！";
        throw std::runtime_error("无法找到能覆盖剩余粗定位区域的相机视野，请检查参数或算法！");
    }

    // 将所有 bestCovered 中粗定位框的几何中心的平均值作为新的 center
    double sumX = 0.0, sumY = 0.0;
    for (size_t idx : bestCovered) {
        const cv::Rect_<double>& rect = roughRegions[idx];
        sumX += rect.x + rect.width / 2.0;
        sumY += rect.y + rect.height / 2.0;
    }

    return cv::Point2d(sumX / bestCovered.size(), sumY / bestCovered.size());
}

// 合并拍照位置结果, 将『能用一个拍照位置同时拍到的多个位置』合并为一个拍照位置
std::vector<cv::Point2d> PhotoPlanner::mergePhotoCenters(const std::vector<cv::Point2d>& centers,
                                                         const std::vector<cv::Rect_<double>>& roughRegions) const {
    if (centers.size() <= 1) return centers;  // 小于等于1个拍照中心, 无需合并

    std::vector<cv::Point2d> merged;                // 合并后的结果
    std::vector<bool> used(centers.size(), false);  // 记录哪些拍照中心已经被使用

    // 遍历每个中心点, 寻找可以合并的其它中心点
    for (size_t i = 0; i < centers.size(); ++i) {
        if (used[i]) continue;  // 区域已经被用过, 就跳过

        // 当前拍照区域
        cv::Rect_<double> areaI(centers[i].x - photoWidth_ / 2.0, centers[i].y - photoHeight_ / 2.0, photoWidth_, photoHeight_);
        std::set<size_t> coveredByI;

        // 获取areaI覆盖的粗定位区域索引
        for (size_t k = 0; k < roughRegions.size(); ++k) {
            if (isFullyCovered(areaI, roughRegions[k])) coveredByI.insert(k);
        }

        cv::Point2d mergedCenter = centers[i];
        used[i] = true;

        for (size_t j = i + 1; j < centers.size(); ++j) {
            if (used[j]) continue;  // 区域已经被用过, 就跳过

            // 欲合并的拍照区域
            cv::Rect_<double> areaJ(centers[j].x - photoWidth_ / 2.0, centers[j].y - photoHeight_ / 2.0, photoWidth_, photoHeight_);
            std::set<size_t> coveredByJ;

            // 获取areaJ覆盖的粗定位区域索引
            for (size_t k = 0; k < roughRegions.size(); ++k) {
                if (isFullyCovered(areaJ, roughRegions[k])) coveredByJ.insert(k);
            }

            // 如果areaI和areaJ合并后能覆盖coveredByI∪coveredByJ, 那么合并
            cv::Point2d tentativeCenter((mergedCenter.x + centers[j].x) / 2.0, (mergedCenter.y + centers[j].y) / 2.0);
            cv::Rect_<double> combinedArea(tentativeCenter.x - photoWidth_ / 2.0, tentativeCenter.y - photoHeight_ / 2.0, photoWidth_, photoHeight_);

            bool canCoverAll = true;
            for (size_t idx : coveredByI) {
                if (!isFullyCovered(combinedArea, roughRegions[idx])) {
                    canCoverAll = false;
                    break;
                }
            }
            if (canCoverAll) {
                for (size_t idx : coveredByJ) {
                    if (!isFullyCovered(combinedArea, roughRegions[idx])) {
                        canCoverAll = false;
                        break;
                    }
                }
            }
            if (canCoverAll) {
                mergedCenter = tentativeCenter;
                used[j] = true;
                // 更新coveredByI
                coveredByI.insert(coveredByJ.begin(), coveredByJ.end());
            }
        }

        merged.push_back(mergedCenter);
    }

    return merged;
}
