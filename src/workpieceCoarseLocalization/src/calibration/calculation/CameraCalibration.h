#ifndef CAMERACALIBRATION_H
#define CAMERACALIBRATION_H

#include <string>
#include <vector>

#include "CalibrationData.h"

namespace CameraCalibration {
bool calibrate(std::vector<std::string>& imagePaths, Result& result);
}

#endif  // CAMERACALIBRATION_H
