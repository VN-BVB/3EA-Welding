#include "CoarseCalibration.h"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <QDir>

#include <windows.h>

#include "calculation/AxisCalibration.h"
#include "config/CalibrationConfig.h"
#include "calculation/CameraCalibration.h"
#include "calculation/CoordinateCalculation.h"
#include "calculation/PlaneCalibration.h"
#include "handeye/HandEyeCalculation.h"

CoarseCalibration::CoarseCalibration(QObject* parent)
    : QObject(parent), calibrationConfig(CalibrationConfig::getInstance()), boardConfig{6, 9, 0.060} {}

CoarseCalibration::~CoarseCalibration() = default;

void CoarseCalibration::whenUpdateCurrentXAxisEncoderValue(double encoderValue) { currentXAxisEncoderValue = encoderValue; }


/**
 * @brief 执行相机内参、畸变系数标定并保存相机内参、畸变系数
 * @param calibrationRootPath 标定数据根目录，本项目为 ./data/workpieceCoaLoc/Calib
 * @param resultFilePath 标定结果 JSON 文件路径（含文件名），本项目为 ./data/config/workpiece_localization_calib.json
 */
void CoarseCalibration::whenCalibrateCamera(const QString& calibrationRootPath, const QString& resultFilePath) {
    const std::string imageDirectory = calibrationRootPath.toStdString() + "\\img";
    std::vector<std::string> imagePaths;
    listBmpImages(imageDirectory, imagePaths);
    if (imagePaths.empty()) {
        emit sendCalibrationResult(QStringLiteral("camera"), false);
        return;
    }

    CameraCalibration::Result cameraResult;
    try {
        if (!CameraCalibration::calibrate(imagePaths, cameraResult)) {
            emit sendCalibrationResult(QStringLiteral("camera"), false);
            return;
        }
    } catch (...) {
        emit sendCalibrationResult(QStringLiteral("camera"), false);
        return;
    }

    // 首次标定允许结果文件不存在，此时从空结果开始保存相机内参
    calibrationConfig.readConfig(resultFilePath.toStdString());
    //将计算出的内参及畸变系数结果写入calibrationConfig中的calibrationResult，先保存在内存中
    calibrationConfig.updateCameraResult(cameraResult);
    //将计算结果写入本地JSON文件
    if (!calibrationConfig.writeConfig(resultFilePath.toStdString())) {
        emit sendCalibrationResult(QStringLiteral("camera"), false);
        return;
    }

    emit sendMessage(QStringLiteral("相机内参标定误差（opencvRms）：%1 像素").arg(cameraResult.totalErr, 0, 'f', 6));
    emit sendCalibrationResult(QStringLiteral("camera"), true);
}

/**
 * @brief 执行平面标定并保存平面结果
 * @param calibrationRootPath 标定数据根目录，本项目为 ./data/workpieceCoaLoc/Calib/plane
 * @param resultFilePath 标定结果 JSON 文件路径（含文件名），本项目为 ./data/config/workpiece_localization_calib.json
 */
void CoarseCalibration::whenCalibratePlane(const QString& calibrationRootPath, const QString& resultFilePath) {
    //从本地JSON中将标定好的数据读到内存中
    if (!calibrationConfig.readConfig(resultFilePath.toStdString())) {
        emit sendCalibrationResult(QStringLiteral("plane"), false);
        return;
    }

    //从calibrationConfig中提取内参
    CameraCalibration::Result cameraResult;
    if (!calibrationConfig.getCameraIntrinsics(cameraResult)) {
        emit sendCalibrationResult(QStringLiteral("plane"), false);
        return;
    }

    const std::string imageDirectory = calibrationRootPath.toStdString() + "\\plane";
    std::vector<std::string> imagePaths;
    listBmpImages(imageDirectory, imagePaths);
    if (imagePaths.empty()) {
        emit sendCalibrationResult(QStringLiteral("plane"), false);
        return;
    }

    PlaneCalibration::Result planeResult;
    try {
        if (!PlaneCalibration::calibrate(imagePaths, cameraResult, planeResult)) {
            emit sendCalibrationResult(QStringLiteral("plane"), false);
            return;
        }
    } catch (...) {
        emit sendCalibrationResult(QStringLiteral("plane"), false);
        return;
    }

    calibrationConfig.updatePlaneResult(planeResult);
    if (!calibrationConfig.writeConfig(resultFilePath.toStdString())) {
        emit sendCalibrationResult(QStringLiteral("plane"), false);
        return;
    }

    emit sendMessage(QStringLiteral("平面拟合误差：\nX轴：%1 mm\nY轴：%2 mm\nZ轴：%3 mm")
                         .arg(planeResult.meanErrorX, 0, 'f', 6)
                         .arg(planeResult.meanErrorY, 0, 'f', 6)
                         .arg(planeResult.meanErrorZ, 0, 'f', 6));
    emit sendCalibrationResult(QStringLiteral("plane"), true);
}

/**
 * @brief 执行手眼标定并保存手眼矩阵
 * @param calibrationRootPath 标定数据根目录，本项目为 ./data/workpieceCoaLoc/Calib
 * @param resultFilePath 标定结果 JSON 文件路径（含文件名），本项目为 ./data/config/workpiece_localization_calib.json
 */
void CoarseCalibration::whenCalibrateHandEye(const QString& calibrationRootPath, const QString& resultFilePath) {
    //从本地JSON中将标定好的数据读到内存中
    if (!calibrationConfig.readConfig(resultFilePath.toStdString())) {
        emit sendCalibrationResult(QStringLiteral("handeye"), false);
        return;
    }

    //从calibrationConfig中提取内参
    CameraCalibration::Result cameraResult;
    if (!calibrationConfig.getCameraIntrinsics(cameraResult)) {
        emit sendCalibrationResult(QStringLiteral("handeye"), false);
        return;
    }

    const QString imageDirectory = QDir::cleanPath(calibrationRootPath + "/img");
    const QString poseDirectory = QDir::cleanPath(calibrationRootPath + "/pos");

    //手眼标定结果保存结构体
    HandEyeCalculation::Result handEyeResult;
    if (!HandEyeCalculation::calibrate(imageDirectory.toStdString(), poseDirectory.toStdString(), cameraResult.cameraMatrix, cameraResult.distCoeffs,
                                       boardConfig, handEyeResult)) {
        emit sendCalibrationResult(QStringLiteral("handeye"), false);
        return;
    }

    calibrationConfig.updateHandEyeResult(handEyeResult);
    if (!calibrationConfig.writeConfig(resultFilePath.toStdString())) {
        emit sendCalibrationResult(QStringLiteral("handeye"), false);
        return;
    }

    emit sendMessage(QStringLiteral("X方向标准差：%1 mm").arg(handEyeResult.stdDevX * 1000.0));
    emit sendMessage(QStringLiteral("Y方向标准差：%1 mm").arg(handEyeResult.stdDevY * 1000.0));
    emit sendMessage(QStringLiteral("Z方向标准差：%1 mm").arg(handEyeResult.stdDevZ * 1000.0));
    emit sendCalibrationResult(QStringLiteral("handeye"), true);
}

/**
 * @brief 执行外部三轴方向向量标定，每次标定一个轴
 * @param calibrationRootPath 标定数据根目录，本项目为 ./data/workpieceCoaLoc/Calib/x_axis、y_axis 、 z_axis
 * @param resultFilePath 标定结果 JSON 文件路径（含文件名），本项目为 ./data/config/workpiece_localization_calib.json
 * @param axisTypeValue 标定轴类型：0 为 X 轴，1 为 Y 轴，2 为 Z 轴。
 */
void CoarseCalibration::whenCalibrateAxis(const QString& calibrationRootPath, const QString& resultFilePath, int axisTypeValue) {
    AxisCalibration::Type axisType = AxisCalibration::Type::XAxis;
    QString axisName;
    QString imageFolder;
    switch (axisTypeValue) {
        case 0:
            axisType = AxisCalibration::Type::XAxis;
            axisName = QStringLiteral("X轴");
            imageFolder = "x_axis";
            break;
        case 1:
            axisType = AxisCalibration::Type::YAxis;
            axisName = QStringLiteral("Y轴");
            imageFolder = "y_axis";
            break;
        case 2:
            axisType = AxisCalibration::Type::ZAxis;
            axisName = QStringLiteral("Z轴");
            imageFolder = "z_axis";
            break;
        default:
            emit sendCalibrationResult(QStringLiteral("external_axis"), false);
            return;
    }


    //从本地JSON读取标定数据
    if (!calibrationConfig.readConfig(resultFilePath.toStdString())) {
        emit sendCalibrationResult(QStringLiteral("external_axis"), false);
        return;
    }

    CameraCalibration::Result cameraResult;
    if (!calibrationConfig.getCameraIntrinsics(cameraResult)) {
        emit sendCalibrationResult(QStringLiteral("external_axis"), false);
        return;
    }

    cv::Mat cameraToBaseMatrix;
    if (!calibrationConfig.getHandEyeMatrixInMeters(cameraToBaseMatrix)) {
        emit sendCalibrationResult(QStringLiteral("external_axis"), false);
        return;
    }

    const QString imageDirectory = QDir::cleanPath(calibrationRootPath + "/" + imageFolder);
    AxisCalibration::Input input;
    input.boardConfig = boardConfig;
    input.cameraMatrix = cameraResult.cameraMatrix.clone();
    input.distCoeffs = cameraResult.distCoeffs.clone();
    input.cameraToBaseMatrix = cameraToBaseMatrix;
    input.imagePath = imageDirectory.toStdString();
    input.referenceEncoderValue = calibrationConfig.referenceEncoderValue(axisType);

    AxisCalibration::Result axisResult;
    std::string errorMessage;
    if (!AxisCalibration::calibrate(input, axisType, axisResult, errorMessage)) {
        emit sendCalibrationResult(QStringLiteral("external_axis"), false);
        return;
    }

    calibrationConfig.updateAxisResult(axisResult);
    if (!calibrationConfig.writeConfig(resultFilePath.toStdString())) {
        emit sendCalibrationResult(QStringLiteral("external_axis"), false);
        return;
    }

    const cv::Mat unitDirectionMat = axisResult.unitDirectionMat;
    emit sendMessage(QStringLiteral("%1轨道方向向量：[%2, %3, %4]")
                         .arg(axisName)
                         .arg(unitDirectionMat.at<double>(0, 0), 0, 'f', 9)
                         .arg(unitDirectionMat.at<double>(1, 0), 0, 'f', 9)
                         .arg(unitDirectionMat.at<double>(2, 0), 0, 'f', 9));
    emit sendMessage(QStringLiteral("%1手眼参考编码器值：%2 mm").arg(axisName).arg(axisResult.referenceEncoderValue, 0, 'f', 3));
    emit sendMessage(QStringLiteral("外轴矩阵已保存至：%1").arg(QDir::toNativeSeparators(resultFilePath)));
    emit sendCalibrationResult(QStringLiteral("external_axis"), true);
}

void CoarseCalibration::whenQueryCoordinate(const QString& calibrationRootPath, const QString& resultFilePath, double pixelU, double pixelV) {
    Q_UNUSED(calibrationRootPath);

    if (!calibrationConfig.readConfig(resultFilePath.toStdString())) {
        emit sendCoordinateResult(QString(), false);
        return;
    }

    if (!calibrationConfig.hasThreeAxisMatrices()) {
        emit sendCoordinateResult(QString(), false);
        return;
    }

    const cv::Point2d pixelPoint(pixelU, pixelV);
    const double xAxisEncoderValue = std::isfinite(currentXAxisEncoderValue)
                                         ? currentXAxisEncoderValue
                                         : calibrationConfig.referenceEncoderValue(AxisCalibration::Type::XAxis);

    if (std::isfinite(currentXAxisEncoderValue)) {
        emit sendMessage(QStringLiteral("使用当前 X 轴编码器值：%1 mm").arg(currentXAxisEncoderValue, 0, 'f', 3));
    } else {
        emit sendMessage(QStringLiteral("当前 X 轴编码器值不可用，使用手眼参考编码器值：%1 mm")
                             .arg(calibrationConfig.referenceEncoderValue(AxisCalibration::Type::XAxis), 0, 'f', 3));
    }

    CoordinateCalculation::AxisRequest xRequest;
    CoordinateCalculation::AxisRequest yRequest;
    CoordinateCalculation::AxisRequest zRequest;
    if (!buildCoordinateRequest(AxisCalibration::Type::XAxis, pixelPoint, xAxisEncoderValue, xRequest)) {
        emit sendCoordinateResult(QString(), false);
        return;
    }
    if (!buildCoordinateRequest(AxisCalibration::Type::YAxis, pixelPoint, 0.0, yRequest)) {
        emit sendCoordinateResult(QString(), false);
        return;
    }
    if (!buildCoordinateRequest(AxisCalibration::Type::ZAxis, pixelPoint, 0.0, zRequest)) {
        emit sendCoordinateResult(QString(), false);
        return;
    }

    CoordinateCalculation::ThreeAxisResult coordinateResult;
    std::string errorMessage;
    if (!CoordinateCalculation::locateThreeAxes(xRequest, yRequest, zRequest, coordinateResult, errorMessage)) {
        emit sendCoordinateResult(QString(), false);
        return;
    }

    const QString resultText = QStringLiteral("X轴：%1  Y轴：%2  Z轴：%3")
                                   .arg(coordinateResult.xAxisResult.targetEncoderValue, 0, 'f', 3)
                                   .arg(coordinateResult.yAxisResult.targetEncoderValue, 0, 'f', 3)
                                   .arg(coordinateResult.zAxisResult.targetEncoderValue, 0, 'f', 3);

    emit sendMessage(QStringLiteral("像素点 (%1, %2) → %3").arg(pixelU, 0, 'f', 3).arg(pixelV, 0, 'f', 3).arg(resultText));
    emit sendCoordinateResult(resultText, true);
}

void CoarseCalibration::listBmpImages(const std::string& directory, std::vector<std::string>& imagePaths) {
    const std::string searchPath = directory + "\\*";
    //windows提供的文件查找接口，用findData存放找到的文件信息
    WIN32_FIND_DATAA findData;
    //在searchPath.c_str()查找文件，结果保存在findData中，后续可用FindNextFileA查找下一个
    HANDLE handle = FindFirstFileA(searchPath.c_str(), &findData);

    imagePaths.clear();
    if (handle == INVALID_HANDLE_VALUE) {
        std::cerr << "警告：无法打开目录 " << directory << std::endl;
        return;
    }

    do {
        if (!(findData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
            const std::string filename = findData.cFileName;
            if (filename.length() >= 4 && filename.substr(filename.length() - 4) == ".bmp") {
                imagePaths.push_back(directory + "\\" + filename);
            }
        }
    } while (FindNextFileA(handle, &findData) != 0);

    FindClose(handle);
}

bool CoarseCalibration::buildCoordinateRequest(AxisCalibration::Type axisType, const cv::Point2d& pixelPoint, double currentEncoderValue,
                                               CoordinateCalculation::AxisRequest& request) const {
    CameraCalibration::Result cameraResult;
    const std::vector<double> globalPlane = calibrationConfig.getGlobalPlane();
    if (!calibrationConfig.getCameraIntrinsics(cameraResult) || globalPlane.empty()) {
        return false;
    }

    cv::Mat cameraToBaseMatrix;
    if (!calibrationConfig.getAxisMatrix(axisType, cameraToBaseMatrix)) {
        return false;
    }

    cv::Mat direction;
    if (!calibrationConfig.getAxisDirection(axisType, direction)) {
        // 保留既有回退：缺少标定方向时，使用对应坐标轴的单位向量。
        switch (axisType) {
            case AxisCalibration::Type::XAxis:
                direction = (cv::Mat_<double>(3, 1) << 1.0, 0.0, 0.0);
                break;
            case AxisCalibration::Type::YAxis:
                direction = (cv::Mat_<double>(3, 1) << 0.0, 1.0, 0.0);
                break;
            case AxisCalibration::Type::ZAxis:
                direction = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 1.0);
                break;
            default:
                break;
        }
    }
    if (direction.empty()) {
        return false;
    }

    request.axisType = axisType;
    request.pixelPoint = pixelPoint;
    request.cameraMatrix = cameraResult.cameraMatrix;
    request.distCoeffs = cameraResult.distCoeffs;
    request.globalPlane = globalPlane;
    request.cameraToBaseMatrix = cameraToBaseMatrix;
    request.unitDirectionMat = direction;
    request.currentEncoderValue = currentEncoderValue;
    // Type 的 X/Y/Z 取值依次为 0/1/2；无效值继续回退到 Y 分量。
    const int axisIndex = static_cast<int>(axisType);
    request.projectionComponentIndex = axisIndex >= 0 && axisIndex <= 2 ? axisIndex : 1;
    return true;
}
