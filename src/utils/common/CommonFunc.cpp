#include "CommonFunc.h"

// 对std::vector中的值进行从大到小排序，返回索引
std::vector<int> MyToolFunc::sortVetorIndexMax2Min(std::vector<double>& v) {
    std::vector<int> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(), [&v](int i1, int i2) { return v[i1] > v[i2]; });
    return idx;
}

// 对std::vector中的值进行从小到大排序，返回索引
std::vector<int> MyToolFunc::sortVetorIndexMin2Max(std::vector<double>& v) {
    std::vector<int> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(), [&v](int i1, int i2) { return v[i1] < v[i2]; });
    return idx;
}

// 获取机器人类型字符串
std::string MyToolFunc::getRobotTypeString(ROBOT_TYPE robotType) {
    switch (robotType) {
        case ROBOT_TYPE::AN_CHUAN:
            return "an chuan";
        case ROBOT_TYPE::BAO_YUAN:
            return "bao yuan";
        default:
            return "unknown robot";
    }
}

// 获取手眼关系字符串
std::string MyToolFunc::getHandTypeTypeString(HAND_EYE_TYPE handTypeType) {
    switch (handTypeType) {
        case HAND_EYE_TYPE::EYE_IN_HAND:
            return "eye in hand";
        case HAND_EYE_TYPE::EYE_TO_HAND:
            return "eye to hand";
        default:
            return "unknown type";
    }
}
