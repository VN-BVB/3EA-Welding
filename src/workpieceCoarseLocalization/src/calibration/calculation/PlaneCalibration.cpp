#include "PlaneCalibration.h"

#include "common/CameraPlaneAlgorithms.h"

namespace PlaneCalibration {

bool calibrate(std::vector<std::string>& imagePaths, const CameraCalibration::Result& cameraResult, Result& result) {
    if (imagePaths.empty() || cameraResult.cameraMatrix.empty() || cameraResult.distCoeffs.empty()) {
        return false;
    }

    //深拷贝，防止对原数据修改
    cv::Mat cameraMatrix = cameraResult.cameraMatrix.clone();
    cv::Mat distCoeffs = cameraResult.distCoeffs.clone();
    //ErrorMetrics结构体用于保存平面标定时的X,Y,Z以及平均误差
    CameraPlaneAlgorithms::ErrorMetrics metrics = {0.0, 0.0, 0.0, 0.0};
    //执行平面标定（误差计算为从像素点经过射线与平面相加算出来的世界点比较得出）
    CameraPlaneAlgorithms::calibratePlane(imagePaths, cameraMatrix, distCoeffs, result.globalPlane, metrics);
    result.meanErrorX = metrics.meanErrorX;
    result.meanErrorY = metrics.meanErrorY;
    result.meanErrorZ = metrics.meanErrorZ;
    result.meanTotalError = metrics.meanTotalError;
    return !result.globalPlane.empty();
}

}  // PlaneCalibration 命名空间
