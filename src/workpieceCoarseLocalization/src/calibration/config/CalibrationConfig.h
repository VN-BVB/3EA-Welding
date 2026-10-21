#ifndef CALIBRATIONCONFIG_H
#define CALIBRATIONCONFIG_H

#include <mutex>
#include <string>
#include <vector>

#include "CalibrationData.h"
#include "workpieceCoarseLocalization/include/CoarseLocalizationMatrix.h"

// 标定结果的单例资源管理类：保存当前结果，统一负责文件读写和数据转换。
class CalibrationConfig {
public:
    static CalibrationConfig& getInstance();

    CalibrationConfig(const CalibrationConfig&) = delete;
    CalibrationConfig& operator=(const CalibrationConfig&) = delete;

    // 每次都从指定文件读取；读取失败时恢复为空结果，避免沿用另一个文件的数据。
    bool readConfig(const std::string& path);
    bool writeConfig(const std::string& path) const;

    // 以下方法直接使用当前结果，不读写文件；更新后调用 writeConfig 才会保存到本地。
    bool getCameraIntrinsics(CameraCalibration::Result& result) const;
    void updateCameraResult(const CameraCalibration::Result& result);
    std::vector<double> getGlobalPlane() const;
    void updatePlaneResult(const PlaneCalibration::Result& result);
    void updateHandEyeResult(const HandEyeCalculation::Result& result);
    void updateAxisResult(const AxisCalibration::Result& result);

    bool getHandEyeMatrixInMeters(cv::Mat& cameraToBaseMatrix) const;
    bool hasThreeAxisMatrices() const;
    bool getAxisMatrix(AxisCalibration::Type axisType, cv::Mat& cameraToBaseMatrix) const;
    bool getAxisDirection(AxisCalibration::Type axisType, cv::Mat& unitDirectionMat) const;
    double referenceEncoderValue(AxisCalibration::Type axisType) const;

private:
    CalibrationConfig() = default;
    ~CalibrationConfig() = default;

    static std::string parentPath(const std::string& filename);
    static bool directoryExists(const std::string& path);
    static void createDirectory(const std::string& path);
    static std::vector<double> matToVector(const cv::Mat& mat);

    CoarseLocalizationMatrix calibrationResult;
    mutable std::mutex configMutex;
};

#endif  // CALIBRATIONCONFIG_H
