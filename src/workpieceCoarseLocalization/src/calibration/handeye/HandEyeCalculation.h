#ifndef HANDEYECALCULATION_H
#define HANDEYECALCULATION_H

#include <string>

#include "../CalibrationData.h"

namespace HandEyeCalculation {
bool calibrate(const std::string& imagePath, const std::string& posePath, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
               const BoardConfig& boardConfig, Result& result);
}  // HandEyeCalculation 命名空间

#endif  // HANDEYECALCULATION_H
