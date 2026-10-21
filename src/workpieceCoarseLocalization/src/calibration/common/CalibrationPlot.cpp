#include "CalibrationPlot.h"

#include <pcl/visualization/pcl_plotter.h>

namespace CalibrationPlot {

void drawLineChart(std::vector<double>& x, std::vector<double>& y, const std::string& title) {
    pcl::visualization::PCLPlotter* plotter = new pcl::visualization::PCLPlotter(title.c_str());
    plotter->setBackgroundColor(1, 1, 1);
    plotter->setTitle(title.c_str());
    plotter->setXTitle("X");
    plotter->setYTitle("Y");
    plotter->addPlotData(x, y, "display", vtkChart::LINE);
    plotter->plot();
}

void drawLineChartWithPlotter(pcl::visualization::PCLPlotter* plotter, std::vector<double>& x, std::vector<double>& y, const std::string& title) {
    plotter->setBackgroundColor(1, 1, 1);
    plotter->setTitle(title.c_str());
    plotter->setXTitle("X");
    plotter->setYTitle("Y");
    plotter->addPlotData(x, y, "display", vtkChart::LINE);
    plotter->plot();
    while (!plotter->wasStopped()) {
        plotter->spinOnce(100);
    }
}

}  // CalibrationPlot 命名空间
