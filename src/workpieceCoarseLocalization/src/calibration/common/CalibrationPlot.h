#ifndef CALIBRATIONPLOT_H
#define CALIBRATIONPLOT_H

#include <string>
#include <vector>

namespace pcl {
namespace visualization {
class PCLPlotter;
}
}

namespace CalibrationPlot {
void drawLineChart(std::vector<double>& x, std::vector<double>& y, const std::string& title);
void drawLineChartWithPlotter(pcl::visualization::PCLPlotter* plotter, std::vector<double>& x, std::vector<double>& y, const std::string& title);
}

#endif  // CALIBRATIONPLOT_H
