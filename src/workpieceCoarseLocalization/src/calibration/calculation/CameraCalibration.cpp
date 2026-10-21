#include "CameraCalibration.h"

#include "common/CameraPlaneAlgorithms.h"

namespace CameraCalibration {


/**
 * @brief 中转接口，处理数据格式
 * @param imagePaths 图片文件名容器
 * @param result 标定结果（相机内参，畸变系数，外参）
 * @return
 */
bool calibrate(std::vector<std::string>& imagePaths, Result& result) {
    if (imagePaths.empty()) {
        return false;
    }

    CameraPlaneAlgorithms::calibrateCamera(imagePaths, result.cameraMatrix, result.distCoeffs, result.tvecsMat, result.rvecsMat, result.imageCount,
                                           result.totalErr);
    return !result.cameraMatrix.empty() && !result.distCoeffs.empty();
}

}  // CameraCalibration 命名空间
