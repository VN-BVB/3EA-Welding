#ifndef COORDINATECALCULATION_H
#define COORDINATECALCULATION_H

#include <string>

#include "CalibrationData.h"

namespace CoordinateCalculation {
double projectPointToAxis(const cv::Mat& direction, const cv::Point3d& point, int componentIndex = 1);
double requiredMove(const cv::Mat& direction, const cv::Point3d& targetPoint, const cv::Point3d& currentToolPoint, int componentIndex = 1);

bool locateAxis(const AxisRequest& request, AxisResult& result, std::string& errorMessage);
bool locateThreeAxes(const AxisRequest& xRequest, const AxisRequest& yRequest, const AxisRequest& zRequest, ThreeAxisResult& result,
                     std::string& errorMessage);
bool moveToPlane(const MoveRequest& request, MoveResult& result, std::string& errorMessage);
bool moveThreeAxesToPlane(const MoveRequest& xRequest, const MoveRequest& yRequest, const MoveRequest& zRequest, ThreeAxisMoveResult& result,
                          std::string& errorMessage);
}

#endif  // COORDINATECALCULATION_H
