#include "CalibrationConfig.h"

#include <cereal/archives/json.hpp>

#include <direct.h>
#include <sys/stat.h>

#include <algorithm>
#include <fstream>
#include <mutex>
#include <vector>

CalibrationConfig& CalibrationConfig::getInstance() {
    // 函数内静态对象只初始化一次，由程序管理生命周期。
    static CalibrationConfig instance;
    return instance;
}

bool CalibrationConfig::readConfig(const std::string& path) {
    std::lock_guard<std::mutex> lock(configMutex);
    calibrationResult = CoarseLocalizationMatrix();

    std::ifstream file(path);
    if (!file.is_open()) {
        return false;
    }

    CoarseLocalizationMatrix result;
    try {
        cereal::JSONInputArchive archive(file);
        archive(result);
    } catch (...) {
        file.close();
        std::ifstream legacyFile(path);
        if (!legacyFile.is_open()) {
            return false;
        }
        try {
            cereal::JSONInputArchive archive(legacyFile);
            archive(cereal::make_nvp("CalibrationResult", result));
        } catch (...) {
            return false;
        }
    }

    calibrationResult = result;
    return true;
}

bool CalibrationConfig::writeConfig(const std::string& path) const {
    std::lock_guard<std::mutex> lock(configMutex);
    createDirectory(parentPath(path));
    std::ofstream file(path);
    if (!file.is_open()) {
        return false;
    }

    {
        cereal::JSONOutputArchive archive(file);
        archive(cereal::make_nvp("CalibrationResult", calibrationResult));
    }
    file.flush();
    return file.good();
}

bool CalibrationConfig::getCameraIntrinsics(CameraCalibration::Result& result) const {
    std::lock_guard<std::mutex> lock(configMutex);
    if (calibrationResult.cameraMatrixData.size() != 9) {
        return false;
    }
    if (calibrationResult.distCoeffsData.size() != 4 && calibrationResult.distCoeffsData.size() != 5) {
        return false;
    }

    result.cameraMatrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(calibrationResult.cameraMatrixData.data())).clone();
    result.distCoeffs = cv::Mat(1, 5, CV_64F, const_cast<double*>(calibrationResult.distCoeffsData.data())).clone();
    return true;
}

void CalibrationConfig::updateCameraResult(const CameraCalibration::Result& result) {
    std::lock_guard<std::mutex> lock(configMutex);
    calibrationResult.cameraMatrixData = std::vector<double>(result.cameraMatrix.begin<double>(), result.cameraMatrix.end<double>());
    calibrationResult.distCoeffsData = std::vector<double>(result.distCoeffs.begin<double>(), result.distCoeffs.end<double>());
}

std::vector<double> CalibrationConfig::getGlobalPlane() const {
    std::lock_guard<std::mutex> lock(configMutex);
    return calibrationResult.globalPlaneData;
}

void CalibrationConfig::updatePlaneResult(const PlaneCalibration::Result& result) {
    std::lock_guard<std::mutex> lock(configMutex);
    calibrationResult.globalPlaneData = result.globalPlane;
}

void CalibrationConfig::updateHandEyeResult(const HandEyeCalculation::Result& result) {
    std::lock_guard<std::mutex> lock(configMutex);
    cv::Mat matrix = result.cameraToBaseMatrix.clone();
    if (matrix.empty() || matrix.rows != 4 || matrix.cols != 4) {
        return;
    }
    for (int i = 0; i < 3; ++i) {
        matrix.at<double>(i, 3) *= 1000.0;
    }
    calibrationResult.cameraToBaseMatrixData = std::vector<double>(matrix.begin<double>(), matrix.end<double>());
}

void CalibrationConfig::updateAxisResult(const AxisCalibration::Result& result) {
    std::lock_guard<std::mutex> lock(configMutex);
    const std::vector<double> matrix = matToVector(result.cameraToBaseMatrix);
    const std::vector<double> direction = matToVector(result.unitDirectionMat);
    switch (result.axisType) {
        case AxisCalibration::Type::XAxis:
            calibrationResult.xAxisCameraToBaseMatrixData = matrix;
            calibrationResult.xAxisTrackDirectionData = direction;
            calibrationResult.xAxisReferenceEncoderValue = result.referenceEncoderValue;
            break;
        case AxisCalibration::Type::YAxis:
            calibrationResult.yAxisCameraToBaseMatrixData = matrix;
            calibrationResult.yAxisTrackDirectionData = direction;
            calibrationResult.yAxisReferenceEncoderValue = result.referenceEncoderValue;
            break;
        case AxisCalibration::Type::ZAxis:
            calibrationResult.zAxisCameraToBaseMatrixData = matrix;
            calibrationResult.zAxisTrackDirectionData = direction;
            calibrationResult.zAxisReferenceEncoderValue = result.referenceEncoderValue;
            break;
    }
}

bool CalibrationConfig::getHandEyeMatrixInMeters(cv::Mat& matrix) const {
    std::lock_guard<std::mutex> lock(configMutex);
    if (calibrationResult.cameraToBaseMatrixData.size() != 16) {
        return false;
    }
    matrix = cv::Mat(4, 4, CV_64F, const_cast<double*>(calibrationResult.cameraToBaseMatrixData.data())).clone();
    for (int i = 0; i < 3; ++i) {
        matrix.at<double>(i, 3) /= 1000.0;
    }
    return true;
}

bool CalibrationConfig::hasThreeAxisMatrices() const {
    std::lock_guard<std::mutex> lock(configMutex);
    return calibrationResult.xAxisCameraToBaseMatrixData.size() == 16 && calibrationResult.yAxisCameraToBaseMatrixData.size() == 16 &&
           calibrationResult.zAxisCameraToBaseMatrixData.size() == 16;
}

bool CalibrationConfig::getAxisMatrix(AxisCalibration::Type type, cv::Mat& matrix) const {
    std::lock_guard<std::mutex> lock(configMutex);
    const std::vector<double>* data = nullptr;
    switch (type) {
        case AxisCalibration::Type::XAxis:
            data = &calibrationResult.xAxisCameraToBaseMatrixData;
            break;
        case AxisCalibration::Type::YAxis:
            data = &calibrationResult.yAxisCameraToBaseMatrixData;
            break;
        case AxisCalibration::Type::ZAxis:
            data = &calibrationResult.zAxisCameraToBaseMatrixData;
            break;
    }
    if (data == nullptr || data->size() != 16) {
        return false;
    }
    matrix = cv::Mat(4, 4, CV_64F, const_cast<double*>(data->data())).clone();
    return true;
}

bool CalibrationConfig::getAxisDirection(AxisCalibration::Type type, cv::Mat& direction) const {
    std::lock_guard<std::mutex> lock(configMutex);
    const std::vector<double>* data = nullptr;
    switch (type) {
        case AxisCalibration::Type::XAxis:
            data = &calibrationResult.xAxisTrackDirectionData;
            break;
        case AxisCalibration::Type::YAxis:
            data = &calibrationResult.yAxisTrackDirectionData;
            break;
        case AxisCalibration::Type::ZAxis:
            data = &calibrationResult.zAxisTrackDirectionData;
            break;
    }
    if (data == nullptr || data->size() != 3) {
        return false;
    }
    direction = cv::Mat(3, 1, CV_64F, const_cast<double*>(data->data())).clone();
    return true;
}

double CalibrationConfig::referenceEncoderValue(AxisCalibration::Type type) const {
    std::lock_guard<std::mutex> lock(configMutex);
    switch (type) {
        case AxisCalibration::Type::XAxis:
            return calibrationResult.xAxisReferenceEncoderValue;
        case AxisCalibration::Type::YAxis:
            return calibrationResult.yAxisReferenceEncoderValue;
        case AxisCalibration::Type::ZAxis:
            return calibrationResult.zAxisReferenceEncoderValue;
    }
    return 0.0;
}

std::string CalibrationConfig::parentPath(const std::string& filename) {
    std::string path = filename;
    std::replace(path.begin(), path.end(), '/', '\\');
    const size_t slash = path.find_last_of('\\');
    return slash == std::string::npos ? std::string() : path.substr(0, slash);
}

bool CalibrationConfig::directoryExists(const std::string& path) {
    struct _stat info;
    return _stat(path.c_str(), &info) == 0 && (info.st_mode & _S_IFDIR) != 0;
}

void CalibrationConfig::createDirectory(const std::string& path) {
    if (path.empty() || directoryExists(path)) {
        return;
    }
    const std::string parent = parentPath(path);
    if (!parent.empty() && parent != path) {
        createDirectory(parent);
    }
    _mkdir(path.c_str());
}

std::vector<double> CalibrationConfig::matToVector(const cv::Mat& mat) {
    return mat.empty() ? std::vector<double>() : std::vector<double>(mat.begin<double>(), mat.end<double>());
}

