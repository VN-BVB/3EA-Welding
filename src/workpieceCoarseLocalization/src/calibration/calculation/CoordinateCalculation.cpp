#include "CoordinateCalculation.h"

#include <cmath>
#include <iostream>
#include <vector>

#include "common/CameraPlaneAlgorithms.h"

namespace {

bool pointOnPlaneInCamera(const cv::Point2d& pixelPoint, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
                          const std::vector<double>& globalPlane, cv::Point3d& pointInCamera, std::string& errorMessage);
bool cameraPointToBase(const cv::Point3d& pointInCamera, const cv::Mat& cameraToBaseMatrix, cv::Point3d& pointInBase, std::string& errorMessage);
bool validDirection(const cv::Mat& direction, std::string& errorMessage);

}  // 匿名命名空间

namespace CoordinateCalculation {

double projectPointToAxis(const cv::Mat& direction, const cv::Point3d& point, int componentIndex) {
    if (direction.empty() || direction.rows != 3 || direction.cols != 1) {
        std::cerr << "轨道方向向量无效！" << std::endl;
        return 0.0;
    }

    componentIndex = componentIndex < 0 || componentIndex > 2 ? 1 : componentIndex;
    const double directionValues[3] = {direction.at<double>(0, 0), direction.at<double>(1, 0), direction.at<double>(2, 0)};
    const double pointValues[3] = {point.x, point.y, point.z};

    if (std::abs(directionValues[componentIndex]) <= 1e-9) {
        std::cerr << "轨道方向向量选定分量为零，无法进行除法计算。" << std::endl;
        return 0.0;
    }

    const double parameter = pointValues[componentIndex] / directionValues[componentIndex];
    const cv::Point3d projected(parameter * directionValues[0], parameter * directionValues[1], parameter * directionValues[2]);
    const double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

    std::cout << "原始点：" << point << "\n";
    std::cout << "投影到轨道方向上的点：" << projected << "\n";
    std::cout << "到原点的距离：" << distance << "\n";
    std::cout << "-----------------------------\n";
    return parameter;
}

double requiredMove(const cv::Mat& direction, const cv::Point3d& targetPoint, const cv::Point3d& currentToolPoint, int componentIndex) {
    return projectPointToAxis(direction, targetPoint, componentIndex) - projectPointToAxis(direction, currentToolPoint, componentIndex);
}

bool locateAxis(const AxisRequest& request, AxisResult& result, std::string& errorMessage) {
    cv::Point3d pointInCamera;
    cv::Point3d pointInBase;
    errorMessage.clear();

    if (!pointOnPlaneInCamera(request.pixelPoint, request.cameraMatrix, request.distCoeffs, request.globalPlane, pointInCamera, errorMessage) ||
        !cameraPointToBase(pointInCamera, request.cameraToBaseMatrix, pointInBase, errorMessage) ||
        !validDirection(request.unitDirectionMat, errorMessage)) {
        return false;
    }

    cv::Mat projectionDirection = -request.unitDirectionMat.clone();
    result.axisType = request.axisType;
    result.pointInCamera = pointInCamera;
    result.pointInBase = pointInBase;
    result.projectedValue = projectPointToAxis(projectionDirection, pointInBase, request.projectionComponentIndex);
    result.requiredMoveValue = result.projectedValue;
    result.targetEncoderValue = result.projectedValue;

    if (request.axisType == AxisCalibration::Type::XAxis) {
        result.targetEncoderValue = request.currentEncoderValue + result.requiredMoveValue;
    }
    return true;
}

bool moveToPlane(const MoveRequest& request, MoveResult& result, std::string& errorMessage) {
    cv::Point3d pointInCamera;
    cv::Point3d pointInBase;
    errorMessage.clear();

    if (!pointOnPlaneInCamera(request.pixelPoint, request.cameraMatrix, request.distCoeffs, request.globalPlane, pointInCamera, errorMessage) ||
        !cameraPointToBase(pointInCamera, request.cameraToBaseMatrix, pointInBase, errorMessage) ||
        !validDirection(request.unitDirectionMat, errorMessage)) {
        return false;
    }

    cv::Mat projectionDirection = -request.unitDirectionMat.clone();
    result.axisType = request.axisType;
    result.pointInCamera = pointInCamera;
    result.pointInBase = pointInBase;
    result.targetAxisValue = projectPointToAxis(projectionDirection, pointInBase, request.projectionComponentIndex);
    result.currentToolAxisValue = projectPointToAxis(projectionDirection, request.currentToolPointInBase, request.projectionComponentIndex);
    result.requiredMoveValue = result.targetAxisValue - result.currentToolAxisValue;
    result.targetEncoderValue = request.currentEncoderValue + result.requiredMoveValue;
    return true;
}

bool locateThreeAxes(const AxisRequest& xRequest, const AxisRequest& yRequest, const AxisRequest& zRequest, ThreeAxisResult& result,
                     std::string& errorMessage) {
    return locateAxis(xRequest, result.xAxisResult, errorMessage) && locateAxis(yRequest, result.yAxisResult, errorMessage) &&
           locateAxis(zRequest, result.zAxisResult, errorMessage);
}

bool moveThreeAxesToPlane(const MoveRequest& xRequest, const MoveRequest& yRequest, const MoveRequest& zRequest, ThreeAxisMoveResult& result,
                          std::string& errorMessage) {
    return moveToPlane(xRequest, result.xAxisResult, errorMessage) && moveToPlane(yRequest, result.yAxisResult, errorMessage) &&
           moveToPlane(zRequest, result.zAxisResult, errorMessage);
}

}  // CoordinateCalculation 命名空间

namespace {

bool pointOnPlaneInCamera(const cv::Point2d& pixelPoint, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
                          const std::vector<double>& globalPlane, cv::Point3d& pointInCamera, std::string& errorMessage) {
    if (cameraMatrix.empty() || distCoeffs.empty()) {
        errorMessage = "相机内参为空。";
        return false;
    }
    if (globalPlane.size() != 4) {
        errorMessage = "全局平面数据无效。";
        return false;
    }

    std::vector<cv::Point2d> pixelPoints(1, pixelPoint);
    std::vector<cv::Point3d> cameraPoints;
    CameraPlaneAlgorithms::pixelsToPointsOnPlane(globalPlane, cameraMatrix, distCoeffs, pixelPoints, cameraPoints);
    if (cameraPoints.empty()) {
        errorMessage = "无法将像素点转换为平面上的相机坐标点。";
        return false;
    }
    pointInCamera = cameraPoints.front();
    return true;
}

bool cameraPointToBase(const cv::Point3d& pointInCamera, const cv::Mat& cameraToBaseMatrix, cv::Point3d& pointInBase, std::string& errorMessage) {
    if (cameraToBaseMatrix.empty() || cameraToBaseMatrix.rows != 4 || cameraToBaseMatrix.cols != 4) {
        errorMessage = "相机到基坐标系矩阵为空或不是 4×4 矩阵。";
        return false;
    }

    cv::Mat homogeneousPoint = (cv::Mat_<double>(4, 1) << pointInCamera.x, pointInCamera.y, pointInCamera.z, 1.0);
    cv::Mat basePoint = cameraToBaseMatrix * homogeneousPoint;
    pointInBase = cv::Point3d(basePoint.at<double>(0, 0), basePoint.at<double>(1, 0), basePoint.at<double>(2, 0));
    return true;
}

bool validDirection(const cv::Mat& direction, std::string& errorMessage) {
    if (direction.empty() || direction.rows != 3 || direction.cols != 1) {
        errorMessage = "轴方向向量为空或不是 3×1 矩阵。";
        return false;
    }
    return true;
}

}  // 匿名命名空间
