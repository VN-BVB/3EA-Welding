#ifndef CALIBRATIONSTATISTICS_H
#define CALIBRATIONSTATISTICS_H

#include <vector>

namespace CalibrationStatistics {
void calculateSampleStdDev(std::vector<double>& values, double& stdDev, double& mean);
}

#endif  // CALIBRATIONSTATISTICS_H
