#ifndef CORETYPES_H
#define CORETYPES_H

#include <array>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

struct BoardConfig {
    int board_width = 6;
    int board_height = 9;
    double circle_distance = 0.060;
};

struct CalibrationImagePaths {
    std::vector<std::string> imgPaths;
    std::vector<std::string> planePaths;
};

struct RobotPoseRecord {
    std::array<double, 6> pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

struct RobotPoseDataset {
    std::vector<RobotPoseRecord> poses;
};

struct CameraCalibrationResult {
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<cv::Mat> tvecsMat;
    std::vector<cv::Mat> rvecsMat;
    int imageCount = 0;
    double totalErr = 0.0;
};

struct PlaneCalibrationResult {
    std::vector<double> globalPlane;
    double meanErrorX = 0.0;
    double meanErrorY = 0.0;
    double meanErrorZ = 0.0;
    double meanTotalError = 0.0;
};

struct HandEyeCalibrationResult {
    cv::Mat cameraToBaseMatrix;
    double stdDevX = 0.0;
    double stdDevY = 0.0;
    double stdDevZ = 0.0;
};

enum class ExternalAxisType {
    XAxis = 0,
    YAxis = 1,
    ZAxis = 2
};

struct ExternalAxisCalibrationInput {
    std::string imagePath;
    double currentTrackPosToCalib = 0.0;
};

struct ExternalAxisCalibrationRequest {
    BoardConfig boardConfig;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Mat cameraToBaseMatrix;
    ExternalAxisCalibrationInput xAxisInput;
    ExternalAxisCalibrationInput yAxisInput;
    ExternalAxisCalibrationInput zAxisInput;
};

struct ExternalAxisCalibrationResult {
    ExternalAxisType axisType = ExternalAxisType::XAxis;
    cv::Mat updatedCameraToBaseMatrix;
    cv::Mat translationMat;
    cv::Mat unitDirectionMat;
    double currentTrackPosToCalib = 0.0;
};

struct ExternalAxisLocalizationRequest {
    ExternalAxisType axisType = ExternalAxisType::XAxis;
    cv::Point2d pixelPoint;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<double> globalPlane;
    cv::Mat cameraToBaseMatrix;
    cv::Mat unitDirectionMat;
    double currentEncoderValue = 0.0;
    int projectionComponentIndex = 1;
};

struct ExternalAxisLocalizationResult {
    ExternalAxisType axisType = ExternalAxisType::XAxis;
    cv::Point3d pointInCamera;
    cv::Point3d pointInBase;
    double projectedValue = 0.0;
    double requiredMoveValue = 0.0;
    double targetEncoderValue = 0.0;
};

struct ThreeAxisLocalizationResult {
    ExternalAxisLocalizationResult xAxisResult;
    ExternalAxisLocalizationResult yAxisResult;
    ExternalAxisLocalizationResult zAxisResult;
};

struct ExternalAxisMoveToPlaneRequest {
    ExternalAxisType axisType = ExternalAxisType::XAxis;
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

struct ExternalAxisMoveToPlaneResult {
    ExternalAxisType axisType = ExternalAxisType::XAxis;
    cv::Point3d pointInCamera;
    cv::Point3d pointInBase;
    double targetAxisValue = 0.0;
    double currentToolAxisValue = 0.0;
    double requiredMoveValue = 0.0;
    double targetEncoderValue = 0.0;
};

struct ThreeAxisMoveToPlaneResult {
    ExternalAxisMoveToPlaneResult xAxisResult;
    ExternalAxisMoveToPlaneResult yAxisResult;
    ExternalAxisMoveToPlaneResult zAxisResult;
};

#endif  // CORETYPES_H
