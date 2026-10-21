#ifndef PLANECALIBRATION_H
#define PLANECALIBRATION_H

#include <string>
#include <vector>

#include "CalibrationData.h"

namespace PlaneCalibration {
bool calibrate(std::vector<std::string>& imagePaths, const CameraCalibration::Result& cameraResult, Result& result);
}

#endif  // PLANECALIBRATION_H
