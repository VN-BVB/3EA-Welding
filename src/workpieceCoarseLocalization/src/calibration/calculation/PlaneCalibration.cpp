#include "PlaneCalibration.h"

#include "common/CameraPlaneAlgorithms.h"

namespace PlaneCalibration {

bool calibrate(std::vector<std::string>& imagePaths, const CameraCalibration::Result& cameraResult, Result& result) {
    if (imagePaths.empty() || cameraResult.cameraMatrix.empty() || cameraResult.distCoeffs.empty()) {
        return false;
    }

    cv::Mat cameraMatrix = cameraResult.cameraMatrix.clone();
    cv::Mat distCoeffs = cameraResult.distCoeffs.clone();
    CameraPlaneAlgorithms::ErrorMetrics metrics = {0.0, 0.0, 0.0, 0.0};
    CameraPlaneAlgorithms::calibratePlane(imagePaths, cameraMatrix, distCoeffs, result.globalPlane, metrics);
    result.meanErrorX = metrics.meanErrorX;
    result.meanErrorY = metrics.meanErrorY;
    result.meanErrorZ = metrics.meanErrorZ;
    result.meanTotalError = metrics.meanTotalError;
    return !result.globalPlane.empty();
}

}  // PlaneCalibration 命名空间
