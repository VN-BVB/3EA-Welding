#ifndef AXISCALIBRATION_H
#define AXISCALIBRATION_H

#include <string>

#include "CalibrationData.h"

namespace AxisCalibration {
bool calibrate(const Input& input, Type axisType, Result& result, std::string& errorMessage);
}

#endif  // AXISCALIBRATION_H
