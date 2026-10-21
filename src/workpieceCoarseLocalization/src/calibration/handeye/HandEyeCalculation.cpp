#include "HandEyeCalculation.h"

#include <vector>

#include <opencv2/calib3d.hpp>

#include "HandEyeCalibration.h"
#include "../common/BoardDetection.h"
#include "../common/CalibrationStatistics.h"
#include "../common/PoseCalculation.h"

namespace HandEyeCalculation {

bool calibrate(const std::string& imagePath, const std::string& posePath, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
               const BoardConfig& boardConfig, Result& result) {
    cv::Mat cameraMatrixCopy = cameraMatrix.clone();
    cv::Mat distCoeffsCopy = distCoeffs.clone();
    const cv::Size boardSize(boardConfig.boardWidth, boardConfig.boardHeight);
    std::vector<cv::Point3f> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<cv::Mat> boardToCameraTransforms;
    std::vector<cv::Mat> gripperToBaseTransforms;
    std::vector<cv::Mat> gripperToBaseRotations;
    std::vector<cv::Mat> gripperToBaseTranslations;
    std::vector<cv::Mat> targetToCameraRotations;
    std::vector<cv::Mat> targetToCameraTranslations;
    std::vector<double> worldX;
    std::vector<double> worldY;
    std::vector<double> worldZ;
    cv::Mat cameraToGripperRotation(3, 3, CV_64F);
    cv::Mat cameraToGripperTranslation(3, 1, CV_64F);
    cv::Mat rotation;
    cv::Mat translation;

    if (cameraMatrixCopy.empty() || distCoeffsCopy.empty()) {
        return false;
    }

    BoardDetection::createObjectPoints(boardConfig.boardWidth, boardConfig.boardHeight, boardConfig.circleDistance, objectPoints);
    if (!BoardDetection::findCirclesInFolder(imagePath, boardSize, imagePoints)) {
        return false;
    }

    PoseCalculation::solveBoardPoses(cameraMatrixCopy, distCoeffsCopy, objectPoints, imagePoints, boardToCameraTransforms);
    if (boardToCameraTransforms.empty()) {
        return false;
    }

    if (!HandEyeCalibration::readRobotDatasFromFile(posePath, gripperToBaseTransforms, false)) {
        return false;
    }
    if (gripperToBaseTransforms.empty() || gripperToBaseTransforms.size() != boardToCameraTransforms.size()) {
        return false;
    }

    for (cv::Mat& transform : gripperToBaseTransforms) {
        PoseCalculation::splitTransform(transform, rotation, translation);
        gripperToBaseRotations.push_back(rotation);
        gripperToBaseTranslations.push_back(translation);
    }
    for (cv::Mat& transform : boardToCameraTransforms) {
        PoseCalculation::splitTransform(transform, rotation, translation);
        targetToCameraRotations.push_back(rotation);
        targetToCameraTranslations.push_back(translation);
    }

    cv::calibrateHandEye(gripperToBaseRotations, gripperToBaseTranslations, targetToCameraRotations, targetToCameraTranslations,
                         cameraToGripperRotation, cameraToGripperTranslation, cv::CALIB_HAND_EYE_PARK);

    result.cameraToBaseMatrix = PoseCalculation::composeTransform(cameraToGripperRotation, cameraToGripperTranslation);
    if (result.cameraToBaseMatrix.empty()) {
        return false;
    }

    for (const cv::Mat& boardToCamera : boardToCameraTransforms) {
        cv::Mat boardOrigin = (cv::Mat_<double>(4, 1) << 0.0, 0.0, 0.0, 1.0);
        cv::Mat worldPosition = result.cameraToBaseMatrix * boardToCamera * boardOrigin;
        worldX.push_back(worldPosition.at<double>(0, 0));
        worldY.push_back(worldPosition.at<double>(1, 0));
        worldZ.push_back(worldPosition.at<double>(2, 0));
    }

    if (!worldX.empty()) {
        double meanX = 0.0;
        double meanY = 0.0;
        double meanZ = 0.0;
        CalibrationStatistics::calculateSampleStdDev(worldX, result.stdDevX, meanX);
        CalibrationStatistics::calculateSampleStdDev(worldY, result.stdDevY, meanY);
        CalibrationStatistics::calculateSampleStdDev(worldZ, result.stdDevZ, meanZ);
    }
    return true;
}

}  // HandEyeCalculation 命名空间
