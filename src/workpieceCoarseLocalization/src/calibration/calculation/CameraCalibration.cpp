#include "CameraCalibration.h"

#include "common/CameraPlaneAlgorithms.h"

namespace CameraCalibration {

bool calibrate(std::vector<std::string>& imagePaths, Result& result) {
    if (imagePaths.empty()) {
        return false;
    }

    CameraPlaneAlgorithms::calibrateCamera(imagePaths, result.cameraMatrix, result.distCoeffs, result.tvecsMat, result.rvecsMat, result.imageCount,
                                           result.totalErr);
    return !result.cameraMatrix.empty() && !result.distCoeffs.empty();
}

}  // CameraCalibration 命名空间
