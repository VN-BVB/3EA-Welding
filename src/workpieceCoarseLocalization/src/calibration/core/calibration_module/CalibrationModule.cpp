#include "CalibrationModule.h"

#include "calibration/CameraAndLaserPlaneCalibration.h"
#include "handeye/include/handEyeCalibration.h"
#include "handeye/include/others.h"

#include <cereal/archives/json.hpp>

#include <direct.h>
#include <sys/stat.h>
#include <windows.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace {

CoarseLocalizationMatrix g_currentCalibrationResult;
bool g_hasCurrentCalibrationResult = false;
std::mutex g_currentCalibrationResultMutex;

std::string normalizedDirectoryPath(std::string path) {
    std::replace(path.begin(), path.end(), '/', '\\');
    return path;
}

std::string parentDirectoryPath(const std::string& filename) {
    std::string normalizedPath = normalizedDirectoryPath(filename);
    const size_t slashPos = normalizedPath.find_last_of('\\');
    if (slashPos == std::string::npos) {
        return std::string();
    }
    return normalizedPath.substr(0, slashPos);
}

bool directoryExists(const std::string& path) {
    struct _stat info;
    if (_stat(path.c_str(), &info) != 0) {
        return false;
    }
    return (info.st_mode & _S_IFDIR) != 0;
}

void createDirectoryRecursive(const std::string& path) {
    if (path.empty() || directoryExists(path)) {
        return;
    }

    const std::string parentPath = parentDirectoryPath(path);
    if (!parentPath.empty() && parentPath != path) {
        createDirectoryRecursive(parentPath);
    }

    _mkdir(path.c_str());
}

template <typename CalibrationDataType>
bool saveJsonFile(const std::string& filename, const CalibrationDataType& calibrationResult) {
    createDirectoryRecursive(parentDirectoryPath(filename));

    std::ofstream outFile(filename);
    if (!outFile.is_open()) {
        return false;
    }

    cereal::JSONOutputArchive outputArchive(outFile);
    outputArchive(cereal::make_nvp("CalibrationResult", calibrationResult));
    outFile.flush();
    return true;
}

std::vector<double> matToVector(const cv::Mat& mat) {
    if (mat.empty()) {
        return std::vector<double>();
    }

    return std::vector<double>(mat.begin<double>(), mat.end<double>());
}

cv::Mat buildCameraToBaseMatrixInMillimeter(const cv::Mat& cameraToBaseMatrixInMeter) {
    cv::Mat cameraToBaseMatrixInMillimeter = cameraToBaseMatrixInMeter.clone();
    if (!cameraToBaseMatrixInMillimeter.empty()
        && cameraToBaseMatrixInMillimeter.rows == 4
        && cameraToBaseMatrixInMillimeter.cols == 4) {
        for (int i = 0; i < 3; ++i) {
            cameraToBaseMatrixInMillimeter.at<double>(i, 3) = cameraToBaseMatrixInMillimeter.at<double>(i, 3) * 1000.0;
        }
    }

    return cameraToBaseMatrixInMillimeter;
}

bool solveDirectionInBase(const ExternalAxisCalibrationRequest& calibrationRequest,
                          const std::string& imagePath,
                          cv::Mat& translationMat,
                          cv::Mat& unitDirectionMat,
                          std::string& errorMessage) {
    std::vector<cv::Mat> cvImages;
    std::vector<cv::String> imagePathList;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<cv::Point3f> objPoints;
    std::vector<cv::Mat> vecHc;
    std::vector<std::vector<cv::Point3d>> basePointsVec;
    cv::Size boardSize = {calibrationRequest.boardConfig.board_width, calibrationRequest.boardConfig.board_height};
    cv::Point3d totalTranslation(0.0, 0.0, 0.0);
    int totalPairs = 0;

    translationMat.release();
    unitDirectionMat.release();

    if (calibrationRequest.cameraMatrix.empty() || calibrationRequest.distCoeffs.empty()) {
        errorMessage = "Camera intrinsics are empty.";
        return false;
    }

    if (calibrationRequest.cameraToBaseMatrix.empty()
        || calibrationRequest.cameraToBaseMatrix.rows != 4
        || calibrationRequest.cameraToBaseMatrix.cols != 4) {
        errorMessage = "cameraToBaseMatrix is empty or not 4x4.";
        return false;
    }

    cv::glob(imagePath + "/*.bmp", imagePathList);
    if (imagePathList.empty()) {
        errorMessage = "No bmp images were found in: " + imagePath;
        return false;
    }

    for (const cv::String& currentImagePath : imagePathList) {
        cv::Mat image = cv::imread(currentImagePath);
        if (!image.empty()) {
            cvImages.push_back(image);
        }
    }

    if (cvImages.size() < 2) {
        errorMessage = "At least two valid images are required in: " + imagePath;
        return false;
    }

    calculate_Object_Points(calibrationRequest.boardConfig.board_width,
                            calibrationRequest.boardConfig.board_height,
                            calibrationRequest.boardConfig.circle_distance,
                            objPoints);

    if (!calculate_Image_Points_V(cvImages, boardSize, imagePoints) || imagePoints.size() < 2) {
        errorMessage = "Failed to extract circle centers from: " + imagePath;
        return false;
    }

    cv::Mat cameraMatrix = calibrationRequest.cameraMatrix.clone();
    cv::Mat distCoeffs = calibrationRequest.distCoeffs.clone();
    Calibration_Solve_Extrinsics(cameraMatrix, distCoeffs, objPoints, imagePoints, vecHc);

    if (vecHc.size() < 2) {
        errorMessage = "Extrinsic solving failed for: " + imagePath;
        return false;
    }

    for (int i = 0; i < static_cast<int>(vecHc.size()); ++i) {
        std::vector<cv::Point3d> basePoints;

        for (int j = 0; j < static_cast<int>(objPoints.size()); ++j) {
            cv::Mat RT_Mat = vecHc[i];
            cv::Mat point3d = (cv::Mat_<double>(4, 1) << objPoints[j].x,
                                                        objPoints[j].y,
                                                        objPoints[j].z,
                                                        1.0);
            cv::Mat robotBasePoint = calibrationRequest.cameraToBaseMatrix * RT_Mat * point3d;

            basePoints.push_back(cv::Point3d(robotBasePoint.at<double>(0, 0),
                                             robotBasePoint.at<double>(1, 0),
                                             robotBasePoint.at<double>(2, 0)));
        }

        basePointsVec.push_back(basePoints);
    }

    if (basePointsVec.size() < 2) {
        errorMessage = "Not enough valid image pairs were generated in: " + imagePath;
        return false;
    }

    for (int i = 0; i < static_cast<int>(basePointsVec.size()) - 1; ++i) {
        const std::vector<cv::Point3d>& basePoints1 = basePointsVec[i];
        const std::vector<cv::Point3d>& basePoints2 = basePointsVec[i + 1];

        if (basePoints1.size() != basePoints2.size()) {
            continue;
        }

        for (int j = 0; j < static_cast<int>(basePoints1.size()); ++j) {
            cv::Point3d diff = basePoints2[j] - basePoints1[j];
            totalTranslation += diff;
            totalPairs++;
        }
    }

    if (totalPairs <= 0) {
        errorMessage = "No valid point pairs were generated in: " + imagePath;
        return false;
    }

    totalTranslation = totalTranslation / static_cast<double>(totalPairs);
    translationMat = (cv::Mat_<double>(3, 1) << totalTranslation.x,
                                               totalTranslation.y,
                                               totalTranslation.z);

    const double magnitude = std::sqrt(totalTranslation.x * totalTranslation.x
                                       + totalTranslation.y * totalTranslation.y
                                       + totalTranslation.z * totalTranslation.z);
    if (magnitude < 1e-9) {
        errorMessage = "The translation magnitude is too small in: " + imagePath;
        return false;
    }

    unitDirectionMat = (cv::Mat_<double>(3, 1) << totalTranslation.x / magnitude,
                                                 totalTranslation.y / magnitude,
                                                 totalTranslation.z / magnitude);
    return true;
}

class ExternalAxisCalibrator {
public:
    virtual ~ExternalAxisCalibrator() = default;

    virtual ExternalAxisType axisType() const = 0;

    //末尾的const表示这是一个const成员函数，表示这个函数只能读取这个类
    bool solve(const ExternalAxisCalibrationRequest& calibrationRequest,
               ExternalAxisCalibrationResult& calibrationResult,
               std::string& errorMessage) const {
        const ExternalAxisCalibrationInput& currentAxisInput = axisInput(calibrationRequest);
        if (currentAxisInput.imagePath.empty()) {
            errorMessage = "Axis image path is empty.";
            return false;
        }

        cv::Mat translationMat;
        cv::Mat unitDirectionMat;
        if (!solveDirectionInBase(calibrationRequest,
                                  currentAxisInput.imagePath,
                                  translationMat,
                                  unitDirectionMat,
                                  errorMessage)) {
            return false;
        }

        cv::Mat updatedCameraToBaseMatrix;
        if (!buildUpdatedCameraToBaseMatrix(calibrationRequest,
                                            currentAxisInput,
                                            unitDirectionMat,
                                            updatedCameraToBaseMatrix,
                                            errorMessage)) {
            return false;
        }

        calibrationResult.axisType = axisType();
        calibrationResult.updatedCameraToBaseMatrix = updatedCameraToBaseMatrix;
        calibrationResult.translationMat = translationMat;
        calibrationResult.unitDirectionMat = unitDirectionMat;
        calibrationResult.currentTrackPosToCalib = currentAxisInput.currentTrackPosToCalib;
        return true;
    }

protected:
    virtual const ExternalAxisCalibrationInput& axisInput(const ExternalAxisCalibrationRequest& calibrationRequest) const = 0;

    static bool buildUpdatedMatrixFromTrackPosition(const ExternalAxisCalibrationRequest& calibrationRequest,
                                                    const ExternalAxisCalibrationInput& axisInput,
                                                    const cv::Mat& unitDirectionMat,
                                                    cv::Mat& updatedCameraToBaseMatrix,
                                                    std::string& errorMessage) {
        if (calibrationRequest.cameraToBaseMatrix.empty()
            || calibrationRequest.cameraToBaseMatrix.rows != 4
            || calibrationRequest.cameraToBaseMatrix.cols != 4) {
            errorMessage = "cameraToBaseMatrix is empty or not 4x4.";
            return false;
        }

        cv::Mat rotationMatrix = calibrationRequest.cameraToBaseMatrix(cv::Rect(0, 0, 3, 3)).clone();
        cv::Mat translationVector = calibrationRequest.cameraToBaseMatrix(cv::Rect(3, 0, 1, 3)).clone();
        cv::Mat updatedTranslation = translationVector * 1000.0 - unitDirectionMat * axisInput.currentTrackPosToCalib;

        updatedCameraToBaseMatrix = cv::Mat::eye(4, 4, CV_64F);
        rotationMatrix.copyTo(updatedCameraToBaseMatrix(cv::Rect(0, 0, 3, 3)));
        updatedTranslation.copyTo(updatedCameraToBaseMatrix(cv::Rect(3, 0, 1, 3)));
        return true;
    }

    static bool buildCurrentCameraToBaseMatrix(const ExternalAxisCalibrationRequest& calibrationRequest,
                                               cv::Mat& updatedCameraToBaseMatrix,
                                               std::string& errorMessage) {
        if (calibrationRequest.cameraToBaseMatrix.empty()
            || calibrationRequest.cameraToBaseMatrix.rows != 4
            || calibrationRequest.cameraToBaseMatrix.cols != 4) {
            errorMessage = "cameraToBaseMatrix is empty or not 4x4.";
            return false;
        }

        updatedCameraToBaseMatrix = buildCameraToBaseMatrixInMillimeter(calibrationRequest.cameraToBaseMatrix);
        return true;
    }

private:
    //纯虚函数，子类必须实现这个函数
    virtual bool buildUpdatedCameraToBaseMatrix(const ExternalAxisCalibrationRequest& calibrationRequest,
                                                const ExternalAxisCalibrationInput& axisInput,
                                                const cv::Mat& unitDirectionMat,
                                                cv::Mat& updatedCameraToBaseMatrix,
                                                std::string& errorMessage) const = 0;
};

class XAxisCalibrator : public ExternalAxisCalibrator {
public:
    ExternalAxisType axisType() const override {
        return ExternalAxisType::XAxis;
    }

protected:
    const ExternalAxisCalibrationInput& axisInput(const ExternalAxisCalibrationRequest& calibrationRequest) const override {
        return calibrationRequest.xAxisInput;
    }

private:
    bool buildUpdatedCameraToBaseMatrix(const ExternalAxisCalibrationRequest& calibrationRequest,
                                        const ExternalAxisCalibrationInput& axisInput,
                                        const cv::Mat& unitDirectionMat,
                                        cv::Mat& updatedCameraToBaseMatrix,
                                        std::string& errorMessage) const override {
        (void)axisInput;
        (void)unitDirectionMat;
        return buildCurrentCameraToBaseMatrix(calibrationRequest,
                                              updatedCameraToBaseMatrix,
                                              errorMessage);
    }
};

class YAxisCalibrator : public ExternalAxisCalibrator {
public:
    ExternalAxisType axisType() const override {
        return ExternalAxisType::YAxis;
    }

protected:
    const ExternalAxisCalibrationInput& axisInput(const ExternalAxisCalibrationRequest& calibrationRequest) const override {
        return calibrationRequest.yAxisInput;
    }

private:
    bool buildUpdatedCameraToBaseMatrix(const ExternalAxisCalibrationRequest& calibrationRequest,
                                        const ExternalAxisCalibrationInput& axisInput,
                                        const cv::Mat& unitDirectionMat,
                                        cv::Mat& updatedCameraToBaseMatrix,
                                        std::string& errorMessage) const override {
        return buildUpdatedMatrixFromTrackPosition(calibrationRequest,
                                                   axisInput,
                                                   unitDirectionMat,
                                                   updatedCameraToBaseMatrix,
                                                   errorMessage);
    }
};

class ZAxisCalibrator : public ExternalAxisCalibrator {
public:
    ExternalAxisType axisType() const override {
        return ExternalAxisType::ZAxis;
    }

protected:
    const ExternalAxisCalibrationInput& axisInput(const ExternalAxisCalibrationRequest& calibrationRequest) const override {
        return calibrationRequest.zAxisInput;
    }

private:
    bool buildUpdatedCameraToBaseMatrix(const ExternalAxisCalibrationRequest& calibrationRequest,
                                        const ExternalAxisCalibrationInput& axisInput,
                                        const cv::Mat& unitDirectionMat,
                                        cv::Mat& updatedCameraToBaseMatrix,
                                        std::string& errorMessage) const override {
        return buildUpdatedMatrixFromTrackPosition(calibrationRequest,
                                                   axisInput,
                                                   unitDirectionMat,
                                                   updatedCameraToBaseMatrix,
                                                   errorMessage);
    }
};

std::shared_ptr<ExternalAxisCalibrator> createCalibrator(ExternalAxisType axisType) {
    switch (axisType) {
    case ExternalAxisType::XAxis:
        return std::make_shared<XAxisCalibrator>();
    case ExternalAxisType::YAxis:
        return std::make_shared<YAxisCalibrator>();
    case ExternalAxisType::ZAxis:
        return std::make_shared<ZAxisCalibrator>();
    default:
        break;
    }

    return std::shared_ptr<ExternalAxisCalibrator>();
}

bool buildPointOnPlaneInCamera(const cv::Point2d& pixelPoint,
                               const cv::Mat& cameraMatrix,
                               const cv::Mat& distCoeffs,
                               const std::vector<double>& globalPlane,
                               cv::Point3d& pointInCamera,
                               std::string& errorMessage) {
    if (cameraMatrix.empty() || distCoeffs.empty()) {
        errorMessage = "Camera intrinsics are empty.";
        return false;
    }

    if (globalPlane.size() != 4) {
        errorMessage = "Global plane data is invalid.";
        return false;
    }

    std::vector<cv::Point2d> pixelPoints;
    std::vector<cv::Point3d> cameraPoints;
    pixelPoints.push_back(pixelPoint);
    CameraAndLaserPlaneCalibration::Point2dto3d(globalPlane,
                                                cameraMatrix,
                                                distCoeffs,
                                                pixelPoints,
                                                cameraPoints);

    if (cameraPoints.empty()) {
        errorMessage = "Failed to convert pixel point to camera point on plane.";
        return false;
    }

    pointInCamera = cameraPoints.front();
    return true;
}

bool transformPointCameraToBase(const cv::Point3d& pointInCamera,
                                const cv::Mat& cameraToBaseMatrix,
                                cv::Point3d& pointInBase,
                                std::string& errorMessage) {
    if (cameraToBaseMatrix.empty()
        || cameraToBaseMatrix.rows != 4
        || cameraToBaseMatrix.cols != 4) {
        errorMessage = "cameraToBaseMatrix is empty or not 4x4.";
        return false;
    }

    cv::Mat pointInCameraH = (cv::Mat_<double>(4, 1) << pointInCamera.x,
                                                       pointInCamera.y,
                                                       pointInCamera.z,
                                                       1.0);
    cv::Mat pointInBaseH = cameraToBaseMatrix * pointInCameraH;
    pointInBase = cv::Point3d(pointInBaseH.at<double>(0, 0),
                              pointInBaseH.at<double>(1, 0),
                              pointInBaseH.at<double>(2, 0));
    return true;
}

int normalizeProjectionComponentIndex(int projectionComponentIndex) {
    if (projectionComponentIndex < 0 || projectionComponentIndex > 2) {
        return 1;
    }

    return projectionComponentIndex;
}

}  // namespace

namespace CalibrationModule {

bool hasCurrentCalibrationResult()
{
    std::lock_guard<std::mutex> lock(g_currentCalibrationResultMutex);
    return g_hasCurrentCalibrationResult;
}

const CoarseLocalizationMatrix& currentCalibrationResult()
{
    return g_currentCalibrationResult;
}

bool getCurrentCalibrationResult(CoarseLocalizationMatrix& calibrationResult)
{
    std::lock_guard<std::mutex> lock(g_currentCalibrationResultMutex);
    if (!g_hasCurrentCalibrationResult) {
        return false;
    }

    calibrationResult = g_currentCalibrationResult;
    return true;
}

void setCurrentCalibrationResult(const CoarseLocalizationMatrix& calibrationResult)
{
    std::lock_guard<std::mutex> lock(g_currentCalibrationResultMutex);
    g_currentCalibrationResult = calibrationResult;
    g_hasCurrentCalibrationResult = true;
}

void clearCurrentCalibrationResult()
{
    std::lock_guard<std::mutex> lock(g_currentCalibrationResultMutex);
    g_currentCalibrationResult = CoarseLocalizationMatrix();
    g_hasCurrentCalibrationResult = false;
}

bool loadCalibrationResultFile(const std::string& filename, CoarseLocalizationMatrix& calibrationResult)
{
    std::ifstream inFile(filename);
    if (!inFile.is_open()) {
        return false;
    }

    try {
        cereal::JSONInputArchive inputArchive(inFile);
        inputArchive(calibrationResult);
    } catch (...) {
        inFile.close();

        std::ifstream legacyFile(filename);
        if (!legacyFile.is_open()) {
            return false;
        }

        try {
            cereal::JSONInputArchive inputArchive(legacyFile);
            inputArchive(cereal::make_nvp("CalibrationResult", calibrationResult));
        } catch (...) {
            return false;
        }
    }

    setCurrentCalibrationResult(calibrationResult);
    return true;
}

bool saveCalibrationResultFile(const std::string& filename, const CoarseLocalizationMatrix& calibrationResult)
{
    const bool ok = saveJsonFile(filename, calibrationResult);
    if (ok) {
        setCurrentCalibrationResult(calibrationResult);
    }
    return ok;
}

bool loadCurrentCalibrationResultFile(const std::string& filename)
{
    CoarseLocalizationMatrix calibrationResult;
    return loadCalibrationResultFile(filename, calibrationResult);
}

bool saveCurrentCalibrationResultFile(const std::string& filename)
{
    CoarseLocalizationMatrix calibrationResult;
    if (!getCurrentCalibrationResult(calibrationResult)) {
        return false;
    }
    return saveCalibrationResultFile(filename, calibrationResult);
}

void updateCameraCalibrationResult(const CameraCalibrationResult& cameraCalibrationResult,
                                   const PlaneCalibrationResult& planeCalibrationResult,
                                   CoarseLocalizationMatrix& calibrationResult)
{
    calibrationResult.cameraMatrixData =
        std::vector<double>(cameraCalibrationResult.cameraMatrix.begin<double>(), cameraCalibrationResult.cameraMatrix.end<double>());
    calibrationResult.distCoeffsData =
        std::vector<double>(cameraCalibrationResult.distCoeffs.begin<double>(), cameraCalibrationResult.distCoeffs.end<double>());
    calibrationResult.globalPlaneData = planeCalibrationResult.globalPlane;
}

void updateHandEyeCalibrationResult(const HandEyeCalibrationResult& handEyeCalibrationResult,
                                    CoarseLocalizationMatrix& calibrationResult)
{
    cv::Mat cameraToBaseMatrix = handEyeCalibrationResult.cameraToBaseMatrix.clone();
    if (!cameraToBaseMatrix.empty() && cameraToBaseMatrix.rows == 4 && cameraToBaseMatrix.cols == 4) {
        for (int i = 0; i < 3; ++i) {
            cameraToBaseMatrix.at<double>(i, 3) = cameraToBaseMatrix.at<double>(i, 3) * 1000.0;
        }
        calibrationResult.cameraToBaseMatrixData =
            std::vector<double>(cameraToBaseMatrix.begin<double>(), cameraToBaseMatrix.end<double>());
    }
}

void updateExternalAxisCalibrationResult(const ExternalAxisCalibrationResult& externalAxisCalibrationResult,
                                         CoarseLocalizationMatrix& calibrationResult)
{
    const std::vector<double> updatedMatrixData = matToVector(externalAxisCalibrationResult.updatedCameraToBaseMatrix);
    const std::vector<double> unitDirectionData = matToVector(externalAxisCalibrationResult.unitDirectionMat);

    switch (externalAxisCalibrationResult.axisType) {
    case ExternalAxisType::XAxis:
        calibrationResult.xAxisCameraToBaseMatrixData = updatedMatrixData;
        calibrationResult.xAxisTrackDirectionData = unitDirectionData;
        calibrationResult.xAxisReferenceEncoderValue = externalAxisCalibrationResult.currentTrackPosToCalib;
        break;
    case ExternalAxisType::YAxis:
        calibrationResult.yAxisCameraToBaseMatrixData = updatedMatrixData;
        calibrationResult.yAxisTrackDirectionData = unitDirectionData;
        calibrationResult.yAxisReferenceEncoderValue = externalAxisCalibrationResult.currentTrackPosToCalib;
        break;
    case ExternalAxisType::ZAxis:
        calibrationResult.zAxisCameraToBaseMatrixData = updatedMatrixData;
        calibrationResult.zAxisTrackDirectionData = unitDirectionData;
        calibrationResult.zAxisReferenceEncoderValue = externalAxisCalibrationResult.currentTrackPosToCalib;
        break;
    default:
        break;
    }
}

bool hasSuffix(const std::string& str, const std::string& suffix) {
    if (str.length() < suffix.length()) {
        return false;
    }
    return str.substr(str.length() - suffix.length()) == suffix;
}

void createDirectoryIfNotExists(const std::string& path) {
    if (!directoryExists(path)) {
        _mkdir(path.c_str());
    }
}

void listImagesInDirectory(const std::string& basePath, std::vector<std::string>& imagePaths) {
    std::string searchPath = basePath + "\\*";
    WIN32_FIND_DATAA findData;
    HANDLE hFind = FindFirstFileA(searchPath.c_str(), &findData);

    imagePaths.clear();
    if (hFind == INVALID_HANDLE_VALUE) {
        std::cerr << "Warning: Cannot open directory " << basePath << std::endl;
        return;
    }

    do {
        if (!(findData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
            std::string filename = findData.cFileName;
            std::string fullPath = basePath + "\\" + filename;
            if (hasSuffix(filename, ".bmp")) {
                imagePaths.push_back(fullPath);
            }
        }
    } while (FindNextFileA(hFind, &findData) != 0);

    FindClose(hFind);
}

void loadCalibrationImagePaths(const std::string& calibPath, CalibrationImagePaths& calibrationImagePaths)
{
    calibrationImagePaths.imgPaths.clear();
    calibrationImagePaths.planePaths.clear();

    listImagesInDirectory(calibPath + "\\img", calibrationImagePaths.imgPaths);
    listImagesInDirectory(calibPath + "\\plane", calibrationImagePaths.planePaths);
}

bool runCameraCalibration(std::vector<std::string>& imagePaths, CameraCalibrationResult& cameraCalibrationResult)
{
    if (imagePaths.empty()) {
        return false;
    }

    CameraAndLaserPlaneCalibration::cameraCalibration(imagePaths,
                                                      cameraCalibrationResult.cameraMatrix,
                                                      cameraCalibrationResult.distCoeffs,
                                                      cameraCalibrationResult.tvecsMat,
                                                      cameraCalibrationResult.rvecsMat,
                                                      cameraCalibrationResult.imageCount,
                                                      cameraCalibrationResult.totalErr);
    return !cameraCalibrationResult.cameraMatrix.empty() && !cameraCalibrationResult.distCoeffs.empty();
}

bool runPlaneCalibration(std::vector<std::string>& planeImagePaths,
                         const CameraCalibrationResult& cameraCalibrationResult,
                         PlaneCalibrationResult& planeCalibrationResult)
{
    if (planeImagePaths.empty() || cameraCalibrationResult.cameraMatrix.empty() || cameraCalibrationResult.distCoeffs.empty()) {
        return false;
    }

    CameraAndLaserPlaneCalibration::ErrorMetrics errorMetrics = {0.0, 0.0, 0.0, 0.0};
    cv::Mat cameraMatrix = cameraCalibrationResult.cameraMatrix.clone();
    cv::Mat distCoeffs = cameraCalibrationResult.distCoeffs.clone();

    CameraAndLaserPlaneCalibration::planeCalibration(planeImagePaths,
                                                     cameraMatrix,
                                                     distCoeffs,
                                                     planeCalibrationResult.globalPlane,
                                                     errorMetrics);

    planeCalibrationResult.meanErrorX = errorMetrics.meanErrorX;
    planeCalibrationResult.meanErrorY = errorMetrics.meanErrorY;
    planeCalibrationResult.meanErrorZ = errorMetrics.meanErrorZ;
    planeCalibrationResult.meanTotalError = errorMetrics.meanTotalError;
    return !planeCalibrationResult.globalPlane.empty();
}

bool runHandEyeCalibration(std::string imagePathEyeToHand,
                           const std::string& posePath,
                           const cv::Mat& cameraMatrix,
                           const cv::Mat& distCoeffs,
                           const BoardConfig& boardConfig,
                           HandEyeCalibrationResult& handEyeCalibrationResult)
{
    cv::Mat Kc = cameraMatrix.clone();
    cv::Mat camera_distortion = distCoeffs.clone();
    cv::Size boardSize = {boardConfig.board_width, boardConfig.board_height};
    std::vector<cv::Point3f> objPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<cv::Mat> vecHc;
    std::vector<cv::Mat> vecHg;
    std::vector<cv::Mat> R_gripper2base;
    std::vector<cv::Mat> t_gripper2base;
    std::vector<cv::Mat> R_target2cam;
    std::vector<cv::Mat> t_target2cam;
    std::vector<double> vecWorldX;
    std::vector<double> vecWorldY;
    std::vector<double> vecWorldZ;
    cv::Mat R_cam2gripper = (cv::Mat_<double>(3, 3));
    cv::Mat t_cam2gripper = (cv::Mat_<double>(3, 1));
    cv::Mat tempR;
    cv::Mat tempT;

    if (Kc.empty() || camera_distortion.empty()) {
        return false;
    }

    calculate_Object_Points(boardConfig.board_width, boardConfig.board_height, boardConfig.circle_distance, objPoints);
    if (!calculate_Image_Points(imagePathEyeToHand, boardSize, imagePoints)) {
        return false;
    }

    Calibration_Solve_Extrinsics(Kc, camera_distortion, objPoints, imagePoints, vecHc);
    if (vecHc.empty()) {
        return false;
    }

    if (!HandEyeCalibration::readRobotDatasFromFile(posePath, vecHg, false)) {
        return false;
    }

    if (vecHg.empty() || vecHg.size() != vecHc.size()) {
        return false;
    }

    for (size_t i = 0; i < vecHg.size(); i++) {
        RT2R_T(vecHg[i], tempR, tempT);
        R_gripper2base.push_back(tempR);
        t_gripper2base.push_back(tempT);
    }

    for (size_t i = 0; i < vecHc.size(); i++) {
        RT2R_T(vecHc[i], tempR, tempT);
        R_target2cam.push_back(tempR);
        t_target2cam.push_back(tempT);
    }

    cv::calibrateHandEye(R_gripper2base,
                         t_gripper2base,
                         R_target2cam,
                         t_target2cam,
                         R_cam2gripper,
                         t_cam2gripper,
                         cv::CALIB_HAND_EYE_PARK);

    handEyeCalibrationResult.cameraToBaseMatrix = R_T2RT(R_cam2gripper, t_cam2gripper);
    if (handEyeCalibrationResult.cameraToBaseMatrix.empty()) {
        return false;
    }

    for (size_t i = 0; i < vecHc.size(); ++i) {
        cv::Mat cheesePos = (cv::Mat_<double>(4, 1) << 0.0, 0.0, 0.0, 1.0);
        cv::Mat worldPos = handEyeCalibrationResult.cameraToBaseMatrix * vecHc[i] * cheesePos;
        vecWorldX.push_back(worldPos.at<double>(0, 0));
        vecWorldY.push_back(worldPos.at<double>(1, 0));
        vecWorldZ.push_back(worldPos.at<double>(2, 0));
    }

    if (!vecWorldX.empty()) {
        double meanX = 0.0;
        double meanY = 0.0;
        double meanZ = 0.0;
        calc_stdev(vecWorldX, handEyeCalibrationResult.stdDevX, meanX);
        calc_stdev(vecWorldY, handEyeCalibrationResult.stdDevY, meanY);
        calc_stdev(vecWorldZ, handEyeCalibrationResult.stdDevZ, meanZ);
    }

    return true;
}

bool runExternalAxisCalibration(const ExternalAxisCalibrationRequest& calibrationRequest,
                                const std::vector<ExternalAxisType>& axisTypes,
                                std::vector<ExternalAxisCalibrationResult>& calibrationResults,
                                std::string& errorMessage)
{
    std::vector<std::shared_ptr<ExternalAxisCalibrator>> calibrators;

    calibrationResults.clear();
    errorMessage.clear();

    if (axisTypes.empty()) {
        errorMessage = "No external axis type was selected.";
        return false;
    }

    calibrators.reserve(axisTypes.size());
    for (ExternalAxisType axisType : axisTypes) {
        std::shared_ptr<ExternalAxisCalibrator> calibrator = createCalibrator(axisType);
        if (!calibrator) {
            errorMessage = "Failed to create external axis calibrator.";
            return false;
        }
        calibrators.push_back(calibrator);
    }

    std::vector<ExternalAxisCalibrationResult> localResults(calibrators.size());
    std::vector<std::string> localErrors(calibrators.size());
    std::vector<int> successFlags(calibrators.size(), 0);

#pragma omp parallel for
    for (int i = 0; i < static_cast<int>(calibrators.size()); ++i) {
        if (calibrators[i]->solve(calibrationRequest, localResults[i], localErrors[i])) {
            successFlags[i] = 1;
        }
    }

    for (int i = 0; i < static_cast<int>(calibrators.size()); ++i) {
        if (successFlags[i] == 0) {
            errorMessage = localErrors[i];
            return false;
        }
    }

    calibrationResults = localResults;
    return true;
}

bool runExternalAxisCalibration(const ExternalAxisCalibrationRequest& calibrationRequest,
                                ExternalAxisType axisType,
                                ExternalAxisCalibrationResult& calibrationResult,
                                std::string& errorMessage)
{
    std::vector<ExternalAxisCalibrationResult> calibrationResults;
    if (!runExternalAxisCalibration(calibrationRequest, std::vector<ExternalAxisType>{axisType}, calibrationResults, errorMessage)) {
        return false;
    }

    if (calibrationResults.empty()) {
        errorMessage = "No external axis calibration result was generated.";
        return false;
    }

    calibrationResult = calibrationResults.front();
    return true;
}

std::string axisTypeToName(ExternalAxisType axisType)
{
    switch (axisType) {
    case ExternalAxisType::XAxis:
        return "x_axis";
    case ExternalAxisType::YAxis:
        return "y_axis";
    case ExternalAxisType::ZAxis:
        return "z_axis";
    default:
        break;
    }

    return "unknown_axis";
}

double projectPointToAxisComponent(const cv::Mat& trackDirection,
                                   const cv::Point3d& pt,
                                   int projectionComponentIndex)
{
    if (trackDirection.empty() || trackDirection.rows != 3 || trackDirection.cols != 1) {
        std::cerr << "Invalid trackDirection vector!" << std::endl;
        return 0.0;
    }

    projectionComponentIndex = normalizeProjectionComponentIndex(projectionComponentIndex);

    const double dx = trackDirection.at<double>(0, 0);
    const double dy = trackDirection.at<double>(1, 0);
    const double dz = trackDirection.at<double>(2, 0);
    const double directionValues[3] = {dx, dy, dz};
    const double pointValues[3] = {pt.x, pt.y, pt.z};

    if (std::abs(directionValues[projectionComponentIndex]) <= 1e-9) {
        std::cerr << "trackDirection selected component is zero, cannot divide by zero." << std::endl;
        return 0.0;
    }

    const double t = pointValues[projectionComponentIndex] / directionValues[projectionComponentIndex];

    const double projected_x = t * dx;
    const double projected_y = t * dy;
    const double projected_z = t * dz;
    const double distance = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);

    std::cout << "Original Point: " << pt << "\n";
    std::cout << "Projected Point on trackDirection: " << cv::Point3d(projected_x, projected_y, projected_z) << "\n";
    std::cout << "Distance to origin: " << distance << "\n";
    std::cout << "-----------------------------\n";
    return t;
}

double calculateRequiredMoveValue(const cv::Mat& trackDirection,
                                  const cv::Point3d& targetPointInBase,
                                  const cv::Point3d& currentToolPointInBase,
                                  int projectionComponentIndex)
{
    const double targetAxisValue = projectPointToAxisComponent(trackDirection,
                                                               targetPointInBase,
                                                               projectionComponentIndex);
    const double currentToolAxisValue = projectPointToAxisComponent(trackDirection,
                                                                    currentToolPointInBase,
                                                                    projectionComponentIndex);
    return targetAxisValue - currentToolAxisValue;
}

bool localizeExternalAxisCoordinate(const ExternalAxisLocalizationRequest& localizationRequest,
                                    ExternalAxisLocalizationResult& localizationResult,
                                    std::string& errorMessage)
{
    cv::Point3d pointInCamera;
    cv::Point3d pointInBase;
    cv::Mat projectionDirection;

    errorMessage.clear();

    if (!buildPointOnPlaneInCamera(localizationRequest.pixelPoint,
                                   localizationRequest.cameraMatrix,
                                   localizationRequest.distCoeffs,
                                   localizationRequest.globalPlane,
                                   pointInCamera,
                                   errorMessage)) {
        return false;
    }

    if (!transformPointCameraToBase(pointInCamera,
                                    localizationRequest.cameraToBaseMatrix,
                                    pointInBase,
                                    errorMessage)) {
        return false;
    }

    if (localizationRequest.unitDirectionMat.empty()
        || localizationRequest.unitDirectionMat.rows != 3
        || localizationRequest.unitDirectionMat.cols != 1) {
        errorMessage = "Axis direction vector is empty or not 3x1.";
        return false;
    }

    projectionDirection = -localizationRequest.unitDirectionMat.clone();

    localizationResult.axisType = localizationRequest.axisType;
    localizationResult.pointInCamera = pointInCamera;
    localizationResult.pointInBase = pointInBase;
    localizationResult.projectedValue = projectPointToAxisComponent(projectionDirection,
                                                                    pointInBase,
                                                                    localizationRequest.projectionComponentIndex);
    localizationResult.requiredMoveValue = localizationResult.projectedValue;
    localizationResult.targetEncoderValue = localizationResult.projectedValue;

    if (localizationRequest.axisType == ExternalAxisType::XAxis) {
        localizationResult.targetEncoderValue =
            localizationRequest.currentEncoderValue + localizationResult.requiredMoveValue;
    }

    return true;
}

bool calculateExternalAxisMoveToPlane(const ExternalAxisMoveToPlaneRequest& moveRequest,
                                      ExternalAxisMoveToPlaneResult& moveResult,
                                      std::string& errorMessage)
{
    cv::Point3d pointInCamera;
    cv::Point3d pointInBase;
    cv::Mat projectionDirection;

    errorMessage.clear();

    if (!buildPointOnPlaneInCamera(moveRequest.pixelPoint,
                                   moveRequest.cameraMatrix,
                                   moveRequest.distCoeffs,
                                   moveRequest.globalPlane,
                                   pointInCamera,
                                   errorMessage)) {
        return false;
    }

    if (!transformPointCameraToBase(pointInCamera,
                                    moveRequest.cameraToBaseMatrix,
                                    pointInBase,
                                    errorMessage)) {
        return false;
    }

    if (moveRequest.unitDirectionMat.empty()
        || moveRequest.unitDirectionMat.rows != 3
        || moveRequest.unitDirectionMat.cols != 1) {
        errorMessage = "Axis direction vector is empty or not 3x1.";
        return false;
    }

    projectionDirection = -moveRequest.unitDirectionMat.clone();

    moveResult.axisType = moveRequest.axisType;
    moveResult.pointInCamera = pointInCamera;
    moveResult.pointInBase = pointInBase;
    moveResult.targetAxisValue = projectPointToAxisComponent(projectionDirection,
                                                             pointInBase,
                                                             moveRequest.projectionComponentIndex);
    moveResult.currentToolAxisValue = projectPointToAxisComponent(projectionDirection,
                                                                  moveRequest.currentToolPointInBase,
                                                                  moveRequest.projectionComponentIndex);
    moveResult.requiredMoveValue = moveResult.targetAxisValue - moveResult.currentToolAxisValue;
    moveResult.targetEncoderValue = moveRequest.currentEncoderValue + moveResult.requiredMoveValue;
    return true;
}

bool localizeThreeAxisCoordinate(const ExternalAxisLocalizationRequest& xAxisRequest,
                                 const ExternalAxisLocalizationRequest& yAxisRequest,
                                 const ExternalAxisLocalizationRequest& zAxisRequest,
                                 ThreeAxisLocalizationResult& localizationResult,
                                 std::string& errorMessage)
{
    if (!localizeExternalAxisCoordinate(xAxisRequest, localizationResult.xAxisResult, errorMessage)) {
        return false;
    }

    if (!localizeExternalAxisCoordinate(yAxisRequest, localizationResult.yAxisResult, errorMessage)) {
        return false;
    }

    if (!localizeExternalAxisCoordinate(zAxisRequest, localizationResult.zAxisResult, errorMessage)) {
        return false;
    }

    return true;
}

bool calculateThreeAxisMoveToPlane(const ExternalAxisMoveToPlaneRequest& xAxisRequest,
                                   const ExternalAxisMoveToPlaneRequest& yAxisRequest,
                                   const ExternalAxisMoveToPlaneRequest& zAxisRequest,
                                   ThreeAxisMoveToPlaneResult& moveResult,
                                   std::string& errorMessage)
{
    if (!calculateExternalAxisMoveToPlane(xAxisRequest, moveResult.xAxisResult, errorMessage)) {
        return false;
    }

    if (!calculateExternalAxisMoveToPlane(yAxisRequest, moveResult.yAxisResult, errorMessage)) {
        return false;
    }

    if (!calculateExternalAxisMoveToPlane(zAxisRequest, moveResult.zAxisResult, errorMessage)) {
        return false;
    }

    return true;
}

}  // namespace CalibrationModule
