#include "CalibrationStatistics.h"

#include <cmath>
#include <numeric>

namespace CalibrationStatistics {

void calculateSampleStdDev(std::vector<double>& values, double& stdDev, double& mean) {
    const double sum = std::accumulate(values.begin(), values.end(), 0.0);
    mean = sum / values.size();
    double accum = 0.0;
    for (double value : values) {
        accum += (value - mean) * (value - mean);
    }
    stdDev = std::sqrt(accum / (values.size() - 1));
}

}  // CalibrationStatistics 命名空间
