#ifndef COMMONFUNC_H
#define COMMONFUNC_H

#include <plog/Log.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <iterator>
#include <limits>
#include <numeric>
#include <queue>
#include <random>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

enum ROBOT_TYPE {  // 机器人类型
    AN_CHUAN,      // 安川
    BAO_YUAN       // 宝元
};

enum HAND_EYE_TYPE {  // 手眼关系类型
    EYE_IN_HAND,      // 眼在手上
    EYE_TO_HAND       // 眼在手外
};

namespace MyToolFunc {

// 对std::vector中的值进行从大到小排序，返回索引
std::vector<int> sortVetorIndexMax2Min(std::vector<double> &v);

// 对std::vector中的值进行从小到大排序，返回索引
std::vector<int> sortVetorIndexMin2Max(std::vector<double> &v);

// 获取机器人类型字符串
std::string getRobotTypeString(ROBOT_TYPE robotType);

// 获取手眼关系字符串
std::string getHandTypeTypeString(HAND_EYE_TYPE handTypeType);

}  // namespace MyToolFunc

#endif  // COMMONFUNC_H
