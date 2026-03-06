#include "ErrorSave.h"

#include "utils/common/WeldSeamInfo.h"

ErrorSave::ErrorSave(QObject *parent) : QObject{parent} {}

// 生成当前时间字符串
void ErrorSave::creatorTimeStr() {
    auto now = std::chrono::system_clock::now();                         // 获取当前时间点
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);  // 转换为time_t类型
    struct tm now_tm;                                                    // 转换为tm结构体
    localtime_s(&now_tm, &now_time_t);                                   // 使用localtime_s（Windows）
    std::stringstream ss;                                                // 进行格式化
    ss << std::put_time(&now_tm, "%Y%m%d-%H%M%S");
    time_str = ss.str();  // 获取格式化的时间字符串
}
