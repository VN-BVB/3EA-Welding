#ifndef CALIBRATIONDATA_H
#define CALIBRATIONDATA_H

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

struct BoardConfig {
    int boardWidth = 6;
    int boardHeight = 9;
    double circleDistance = 0.060;  // 圆心间距，单位：米。内参/平面旧算法使用毫米，调用时保持既有换算。
};

namespace CameraCalibration {
struct Result {
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<cv::Mat> tvecsMat;
    std::vector<cv::Mat> rvecsMat;
    int imageCount = 0;
    double totalErr = 0.0;
};
}

namespace PlaneCalibration {
struct Result {
    std::vector<double> globalPlane;
    double meanErrorX = 0.0;
    double meanErrorY = 0.0;
    double meanErrorZ = 0.0;
    double meanTotalError = 0.0;
};
}

namespace HandEyeCalculation {
struct Result {
    cv::Mat cameraToBaseMatrix;
    double stdDevX = 0.0;
    double stdDevY = 0.0;
    double stdDevZ = 0.0;
};
}  // HandEyeCalculation 命名空间

namespace AxisCalibration {
enum class Type { XAxis = 0, YAxis = 1, ZAxis = 2 };

struct Input {
    BoardConfig boardConfig;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Mat cameraToBaseMatrix;          // 输入手眼矩阵的平移单位为米。
    std::string imagePath;               // 当前轴的标定图片目录。
    double referenceEncoderValue = 0.0;  // 当前轴的标定参考编码器读数，单位：毫米。
};

struct Result {
    Type axisType = Type::XAxis;
    cv::Mat cameraToBaseMatrix;
    cv::Mat translationMat;
    cv::Mat unitDirectionMat;
    double referenceEncoderValue = 0.0;  // 标定参考编码器读数，单位：毫米。
};
}

namespace CoordinateCalculation {
struct AxisRequest {
    AxisCalibration::Type axisType = AxisCalibration::Type::XAxis;
    cv::Point2d pixelPoint;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<double> globalPlane;
    cv::Mat cameraToBaseMatrix;
    cv::Mat unitDirectionMat;
    double currentEncoderValue = 0.0;
    int projectionComponentIndex = 1;
};

struct AxisResult {
    AxisCalibration::Type axisType = AxisCalibration::Type::XAxis;
    cv::Point3d pointInCamera;
    cv::Point3d pointInBase;
    double projectedValue = 0.0;
    double requiredMoveValue = 0.0;
    double targetEncoderValue = 0.0;
};

struct ThreeAxisResult {
    AxisResult xAxisResult;
    AxisResult yAxisResult;
    AxisResult zAxisResult;
};

struct MoveRequest {
    AxisCalibration::Type axisType = AxisCalibration::Type::XAxis;
    cv::Point2d pixelPoint;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<double> globalPlane;
    cv::Mat cameraToBaseMatrix;
    cv::Mat unitDirectionMat;
    cv::Point3d currentToolPointInBase;
    double currentEncoderValue = 0.0;
    int projectionComponentIndex = 1;
};

struct MoveResult {
    AxisCalibration::Type axisType = AxisCalibration::Type::XAxis;
    cv::Point3d pointInCamera;
    cv::Point3d pointInBase;
    double targetAxisValue = 0.0;
    double currentToolAxisValue = 0.0;
    double requiredMoveValue = 0.0;
    double targetEncoderValue = 0.0;
};

struct ThreeAxisMoveResult {
    MoveResult xAxisResult;
    MoveResult yAxisResult;
    MoveResult zAxisResult;
};
}

#endif  // CALIBRATIONDATA_H
