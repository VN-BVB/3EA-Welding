#include "CalibrationWorker.h"

#include <cmath>
#include <exception>

#include <QDir>

#include "src/calibration/core/calibration_module/CalibrationModule.h"

namespace {

QString calibrationTitle() { return QStringLiteral("Calibration"); }

BoardConfig boardConfig() {
    BoardConfig config;
    config.board_width = 6;
    config.board_height = 9;
    config.circle_distance = 0.060;
    return config;
}

int projectionComponentIndexForAxis(ExternalAxisType axisType) {
    switch (axisType) {
    case ExternalAxisType::XAxis:
        return 0;
    case ExternalAxisType::YAxis:
        return 1;
    case ExternalAxisType::ZAxis:
        return 2;
    default:
        break;
    }

    return 1;
}

cv::Mat fallbackAxisDirectionInBase(ExternalAxisType axisType) {
    switch (axisType) {
    case ExternalAxisType::XAxis:
        return (cv::Mat_<double>(3, 1) << 1.0, 0.0, 0.0);
    case ExternalAxisType::YAxis:
        return (cv::Mat_<double>(3, 1) << 0.0, 1.0, 0.0);
    case ExternalAxisType::ZAxis:
        return (cv::Mat_<double>(3, 1) << 0.0, 0.0, 1.0);
    default:
        break;
    }

    return cv::Mat();
}

struct CalibrationResultIO {
    static bool loadFromCacheOrFile(const QString& resultFilePath, CoarseLocalizationMatrix& calibrationResult) {
        if (CalibrationModule::getCurrentCalibrationResult(calibrationResult)) {
            return true;
        }

        return CalibrationModule::loadCalibrationResultFile(resultFilePath.toStdString(), calibrationResult);
    }

    static bool saveToFile(const QString& resultFilePath, const CoarseLocalizationMatrix& calibrationResult) {
        return CalibrationModule::saveCalibrationResultFile(resultFilePath.toStdString(), calibrationResult);
    }

    static bool loadCameraIntrinsics(const CoarseLocalizationMatrix& calibrationResult, CameraCalibrationResult& cameraCalibrationResult) {
        if (calibrationResult.cameraMatrixData.size() != 9) {
            return false;
        }
        if (calibrationResult.distCoeffsData.size() != 4 && calibrationResult.distCoeffsData.size() != 5) {
            return false;
        }

        cameraCalibrationResult.cameraMatrix =
            cv::Mat(3, 3, CV_64F, const_cast<double*>(calibrationResult.cameraMatrixData.data())).clone();
        cameraCalibrationResult.distCoeffs =
            cv::Mat(1, 5, CV_64F, const_cast<double*>(calibrationResult.distCoeffsData.data())).clone();
        return true;
    }

    static void updateCameraIntrinsics(const CameraCalibrationResult& cameraCalibrationResult, CoarseLocalizationMatrix& calibrationResult) {
        calibrationResult.cameraMatrixData =
            std::vector<double>(cameraCalibrationResult.cameraMatrix.begin<double>(), cameraCalibrationResult.cameraMatrix.end<double>());
        calibrationResult.distCoeffsData =
            std::vector<double>(cameraCalibrationResult.distCoeffs.begin<double>(), cameraCalibrationResult.distCoeffs.end<double>());
    }

    static void updatePlaneResult(const PlaneCalibrationResult& planeCalibrationResult, CoarseLocalizationMatrix& calibrationResult) {
        calibrationResult.globalPlaneData = planeCalibrationResult.globalPlane;
    }

    static void updateHandEyeResult(const HandEyeCalibrationResult& handEyeCalibrationResult, CoarseLocalizationMatrix& calibrationResult) {
        cv::Mat cameraToBaseMatrix = handEyeCalibrationResult.cameraToBaseMatrix.clone();
        if (cameraToBaseMatrix.empty() || cameraToBaseMatrix.rows != 4 || cameraToBaseMatrix.cols != 4) {
            return;
        }

        for (int i = 0; i < 3; ++i) {
            cameraToBaseMatrix.at<double>(i, 3) *= 1000.0;
        }

        calibrationResult.cameraToBaseMatrixData =
            std::vector<double>(cameraToBaseMatrix.begin<double>(), cameraToBaseMatrix.end<double>());
    }

    static bool loadHandEyeCameraToBaseMatrixInMeters(const CoarseLocalizationMatrix& calibrationResult, cv::Mat& cameraToBaseMatrix) {
        if (calibrationResult.cameraToBaseMatrixData.size() != 16) {
            return false;
        }

        cameraToBaseMatrix =
            cv::Mat(4, 4, CV_64F, const_cast<double*>(calibrationResult.cameraToBaseMatrixData.data())).clone();
        for (int i = 0; i < 3; ++i) {
            cameraToBaseMatrix.at<double>(i, 3) /= 1000.0;
        }
        return true;
    }

    static bool axisTypeFromValue(int axisTypeValue, ExternalAxisType& axisType) {
        switch (axisTypeValue) {
        case 0:
            axisType = ExternalAxisType::XAxis;
            return true;
        case 1:
            axisType = ExternalAxisType::YAxis;
            return true;
        case 2:
            axisType = ExternalAxisType::ZAxis;
            return true;
        default:
            break;
        }

        return false;
    }

    static QString axisDisplayName(ExternalAxisType axisType) {
        switch (axisType) {
        case ExternalAxisType::XAxis:
            return QStringLiteral("X axis");
        case ExternalAxisType::YAxis:
            return QStringLiteral("Y axis");
        case ExternalAxisType::ZAxis:
            return QStringLiteral("Z axis");
        default:
            break;
        }

        return QStringLiteral("External axis");
    }

    static QString axisImagePath(const QString& calibRootPath, ExternalAxisType axisType) {
        switch (axisType) {
        case ExternalAxisType::XAxis:
            return QDir::cleanPath(calibRootPath + "/x_axis");
        case ExternalAxisType::YAxis:
            return QDir::cleanPath(calibRootPath + "/y_axis");
        case ExternalAxisType::ZAxis:
            return QDir::cleanPath(calibRootPath + "/z_axis");
        default:
            break;
        }

        return QString();
    }

    static double axisReferenceEncoderValue(const CoarseLocalizationMatrix& calibrationResult, ExternalAxisType axisType) {
        switch (axisType) {
        case ExternalAxisType::XAxis:
            return calibrationResult.xAxisReferenceEncoderValue;
        case ExternalAxisType::YAxis:
            return calibrationResult.yAxisReferenceEncoderValue;
        case ExternalAxisType::ZAxis:
            return calibrationResult.zAxisReferenceEncoderValue;
        default:
            break;
        }

        return 0.0;
    }

    static bool buildExternalAxisLocalizationRequest(const CoarseLocalizationMatrix& calibrationResult,
                                                     ExternalAxisType axisType,
                                                     const cv::Point2d& pixelPoint,
                                                     double currentEncoderValue,
                                                     ExternalAxisLocalizationRequest& localizationRequest) {
        CameraCalibrationResult cameraCalibrationResult;
        if (!loadCameraIntrinsics(calibrationResult, cameraCalibrationResult) || calibrationResult.globalPlaneData.empty()) {
            return false;
        }

        cv::Mat cameraToBaseMatrix;
        if (!loadAxisCameraToBaseMatrix(calibrationResult, axisType, cameraToBaseMatrix)) {
            return false;
        }

        cv::Mat unitDirectionMat;
        if (!loadAxisTrackDirection(calibrationResult, axisType, unitDirectionMat)) {
            unitDirectionMat = fallbackAxisDirectionInBase(axisType);
        }
        if (unitDirectionMat.empty()) {
            return false;
        }

        localizationRequest.axisType = axisType;
        localizationRequest.pixelPoint = pixelPoint;
        localizationRequest.cameraMatrix = cameraCalibrationResult.cameraMatrix;
        localizationRequest.distCoeffs = cameraCalibrationResult.distCoeffs;
        localizationRequest.globalPlane = calibrationResult.globalPlaneData;
        localizationRequest.cameraToBaseMatrix = cameraToBaseMatrix;
        localizationRequest.unitDirectionMat = unitDirectionMat;
        localizationRequest.currentEncoderValue = currentEncoderValue;
        localizationRequest.projectionComponentIndex = projectionComponentIndexForAxis(axisType);
        return true;
    }

private:
    static const std::vector<double>* axisMatrixData(const CoarseLocalizationMatrix& calibrationResult, ExternalAxisType axisType) {
        switch (axisType) {
        case ExternalAxisType::XAxis:
            return &calibrationResult.xAxisCameraToBaseMatrixData;
        case ExternalAxisType::YAxis:
            return &calibrationResult.yAxisCameraToBaseMatrixData;
        case ExternalAxisType::ZAxis:
            return &calibrationResult.zAxisCameraToBaseMatrixData;
        default:
            break;
        }

        return nullptr;
    }

    static const std::vector<double>* axisDirectionData(const CoarseLocalizationMatrix& calibrationResult, ExternalAxisType axisType) {
        switch (axisType) {
        case ExternalAxisType::XAxis:
            return &calibrationResult.xAxisTrackDirectionData;
        case ExternalAxisType::YAxis:
            return &calibrationResult.yAxisTrackDirectionData;
        case ExternalAxisType::ZAxis:
            return &calibrationResult.zAxisTrackDirectionData;
        default:
            break;
        }

        return nullptr;
    }

    static bool loadAxisCameraToBaseMatrix(const CoarseLocalizationMatrix& calibrationResult,
                                           ExternalAxisType axisType,
                                           cv::Mat& cameraToBaseMatrix) {
        const std::vector<double>* matrixData = axisMatrixData(calibrationResult, axisType);
        if (matrixData == nullptr || matrixData->size() != 16) {
            return false;
        }

        cameraToBaseMatrix = cv::Mat(4, 4, CV_64F, const_cast<double*>(matrixData->data())).clone();
        return true;
    }

    static bool loadAxisTrackDirection(const CoarseLocalizationMatrix& calibrationResult,
                                       ExternalAxisType axisType,
                                       cv::Mat& unitDirectionMat) {
        const std::vector<double>* directionData = axisDirectionData(calibrationResult, axisType);
        if (directionData == nullptr || directionData->size() != 3) {
            return false;
        }

        unitDirectionMat = cv::Mat(3, 1, CV_64F, const_cast<double*>(directionData->data())).clone();
        return true;
    }
};

QString imgPath(const QString& calibRootPath) { return QDir::cleanPath(calibRootPath + "/img"); }
QString posePath(const QString& calibRootPath) { return QDir::cleanPath(calibRootPath + "/pos"); }

}  // namespace

CalibrationWorker::CalibrationWorker(QObject* parent)
    : QObject(parent) {}

CalibrationWorker::~CalibrationWorker() = default;

void CalibrationWorker::updateCurrentXAxisEncoderValue(double currentEncoderValue) { currentXAxisEncoderValue = currentEncoderValue; }

void CalibrationWorker::runCameraCalibration(const QString& calibRootPath, const QString& resultFilePath) {
    auto fail = [this](const QString& message) {
        emit warningRaised(calibrationTitle(), message);
        emit calibrationFinished(QStringLiteral("camera"), false);
    };

    CalibrationImagePaths calibrationImagePaths;
    CalibrationModule::loadCalibrationImagePaths(calibRootPath.toStdString(), calibrationImagePaths);
    if (calibrationImagePaths.imgPaths.empty()) {
        fail(QStringLiteral("No valid image files found for camera calibration."));
        return;
    }

    CameraCalibrationResult cameraCalibrationResult;
    try {
        if (!CalibrationModule::runCameraCalibration(calibrationImagePaths.imgPaths, cameraCalibrationResult)) {
            fail(QStringLiteral("Camera calibration failed."));
            return;
        }
    } catch (const cv::Exception& e) {
        fail(QStringLiteral("Camera calibration failed: %1").arg(QString::fromLocal8Bit(e.what())));
        return;
    } catch (const std::exception& e) {
        fail(QStringLiteral("Camera calibration failed: %1").arg(QString::fromLocal8Bit(e.what())));
        return;
    } catch (...) {
        fail(QStringLiteral("Camera calibration failed: unknown error."));
        return;
    }

    CoarseLocalizationMatrix calibrationResult;
    CalibrationResultIO::loadFromCacheOrFile(resultFilePath, calibrationResult);
    CalibrationResultIO::updateCameraIntrinsics(cameraCalibrationResult, calibrationResult);

    if (!CalibrationResultIO::saveToFile(resultFilePath, calibrationResult)) {
        fail(QStringLiteral("Failed to save calibration result."));
        return;
    }

    emit messageRaised(QStringLiteral("Camera calibration error: %1 mm").arg(cameraCalibrationResult.totalErr, 0, 'f', 6));
    emit calibrationFinished(QStringLiteral("camera"), true);
}

void CalibrationWorker::runPlaneCalibration(const QString& calibRootPath, const QString& resultFilePath) {
    auto fail = [this](const QString& message) {
        emit warningRaised(calibrationTitle(), message);
        emit calibrationFinished(QStringLiteral("plane"), false);
    };

    CoarseLocalizationMatrix calibrationResult;
    if (!CalibrationResultIO::loadFromCacheOrFile(resultFilePath, calibrationResult)) {
        fail(QStringLiteral("Calibration result file was not found. Run camera calibration first."));
        return;
    }

    CameraCalibrationResult cameraCalibrationResult;
    if (!CalibrationResultIO::loadCameraIntrinsics(calibrationResult, cameraCalibrationResult)) {
        fail(QStringLiteral("Camera intrinsics are missing. Run camera calibration first."));
        return;
    }

    CalibrationImagePaths calibrationImagePaths;
    CalibrationModule::loadCalibrationImagePaths(calibRootPath.toStdString(), calibrationImagePaths);
    if (calibrationImagePaths.planePaths.empty()) {
        fail(QStringLiteral("Plane calibration images were not found."));
        return;
    }

    PlaneCalibrationResult planeCalibrationResult;
    try {
        if (!CalibrationModule::runPlaneCalibration(calibrationImagePaths.planePaths, cameraCalibrationResult, planeCalibrationResult)) {
            fail(QStringLiteral("Plane calibration failed."));
            return;
        }
    } catch (const cv::Exception& e) {
        fail(QStringLiteral("Plane calibration failed: %1").arg(QString::fromLocal8Bit(e.what())));
        return;
    } catch (const std::exception& e) {
        fail(QStringLiteral("Plane calibration failed: %1").arg(QString::fromLocal8Bit(e.what())));
        return;
    } catch (...) {
        fail(QStringLiteral("Plane calibration failed: unknown error."));
        return;
    }

    CalibrationResultIO::updatePlaneResult(planeCalibrationResult, calibrationResult);
    if (!CalibrationResultIO::saveToFile(resultFilePath, calibrationResult)) {
        fail(QStringLiteral("Failed to save calibration result."));
        return;
    }

    emit messageRaised(QStringLiteral("Plane fitting error:\nX: %1 mm\nY: %2 mm\nZ: %3 mm")
                           .arg(planeCalibrationResult.meanErrorX, 0, 'f', 6)
                           .arg(planeCalibrationResult.meanErrorY, 0, 'f', 6)
                           .arg(planeCalibrationResult.meanErrorZ, 0, 'f', 6));
    emit calibrationFinished(QStringLiteral("plane"), true);
}

void CalibrationWorker::runHandEyeCalibration(const QString& calibRootPath, const QString& resultFilePath) {
    auto fail = [this](const QString& message) {
        emit warningRaised(calibrationTitle(), message);
        emit calibrationFinished(QStringLiteral("handeye"), false);
    };

    CoarseLocalizationMatrix calibrationResult;
    if (!CalibrationResultIO::loadFromCacheOrFile(resultFilePath, calibrationResult)) {
        fail(QStringLiteral("Calibration result file was not found. Run camera calibration first."));
        return;
    }

    CameraCalibrationResult cameraCalibrationResult;
    if (!CalibrationResultIO::loadCameraIntrinsics(calibrationResult, cameraCalibrationResult)) {
        fail(QStringLiteral("Camera intrinsics are missing. Run camera calibration first."));
        return;
    }

    HandEyeCalibrationResult handEyeCalibrationResult;
    if (!CalibrationModule::runHandEyeCalibration(imgPath(calibRootPath).toStdString(),
                                                  posePath(calibRootPath).toStdString(),
                                                  cameraCalibrationResult.cameraMatrix,
                                                  cameraCalibrationResult.distCoeffs,
                                                  boardConfig(),
                                                  handEyeCalibrationResult)) {
        fail(QStringLiteral("Hand-eye calibration failed. Check whether img and pos data match one by one."));
        return;
    }

    CalibrationResultIO::updateHandEyeResult(handEyeCalibrationResult, calibrationResult);
    if (!CalibrationResultIO::saveToFile(resultFilePath, calibrationResult)) {
        fail(QStringLiteral("Failed to save calibration result."));
        return;
    }

    emit messageRaised(QStringLiteral("X direction std dev: %1 mm").arg(handEyeCalibrationResult.stdDevX * 1000.0));
    emit messageRaised(QStringLiteral("Y direction std dev: %1 mm").arg(handEyeCalibrationResult.stdDevY * 1000.0));
    emit messageRaised(QStringLiteral("Z direction std dev: %1 mm").arg(handEyeCalibrationResult.stdDevZ * 1000.0));
    emit calibrationFinished(QStringLiteral("handeye"), true);
}

void CalibrationWorker::runExternalAxisCalibration(const QString& calibRootPath,
                                                   const QString& resultFilePath,
                                                   int axisTypeValue) {
    auto fail = [this](const QString& message) {
        emit warningRaised(calibrationTitle(), message);
        emit calibrationFinished(QStringLiteral("external_axis"), false);
    };

    ExternalAxisType axisType = ExternalAxisType::XAxis;
    if (!CalibrationResultIO::axisTypeFromValue(axisTypeValue, axisType)) {
        fail(QStringLiteral("Invalid external axis type."));
        return;
    }

    CoarseLocalizationMatrix calibrationResult;
    if (!CalibrationResultIO::loadFromCacheOrFile(resultFilePath, calibrationResult)) {
        fail(QStringLiteral("Calibration result file was not found. Run camera and hand-eye calibration first."));
        return;
    }

    CameraCalibrationResult cameraCalibrationResult;
    if (!CalibrationResultIO::loadCameraIntrinsics(calibrationResult, cameraCalibrationResult)) {
        fail(QStringLiteral("Camera intrinsics are missing. Run camera calibration first."));
        return;
    }

    cv::Mat cameraToBaseMatrix;
    if (!CalibrationResultIO::loadHandEyeCameraToBaseMatrixInMeters(calibrationResult, cameraToBaseMatrix)) {
        fail(QStringLiteral("Hand-eye matrix is missing. Run hand-eye calibration first."));
        return;
    }

    ExternalAxisCalibrationRequest calibrationRequest;
    calibrationRequest.boardConfig = boardConfig();
    calibrationRequest.cameraMatrix = cameraCalibrationResult.cameraMatrix.clone();
    calibrationRequest.distCoeffs = cameraCalibrationResult.distCoeffs.clone();
    calibrationRequest.cameraToBaseMatrix = cameraToBaseMatrix;
    calibrationRequest.xAxisInput.imagePath = CalibrationResultIO::axisImagePath(calibRootPath, ExternalAxisType::XAxis).toStdString();
    calibrationRequest.xAxisInput.currentTrackPosToCalib =
        CalibrationResultIO::axisReferenceEncoderValue(calibrationResult, ExternalAxisType::XAxis);
    calibrationRequest.yAxisInput.imagePath = CalibrationResultIO::axisImagePath(calibRootPath, ExternalAxisType::YAxis).toStdString();
    calibrationRequest.yAxisInput.currentTrackPosToCalib =
        CalibrationResultIO::axisReferenceEncoderValue(calibrationResult, ExternalAxisType::YAxis);
    calibrationRequest.zAxisInput.imagePath = CalibrationResultIO::axisImagePath(calibRootPath, ExternalAxisType::ZAxis).toStdString();
    calibrationRequest.zAxisInput.currentTrackPosToCalib =
        CalibrationResultIO::axisReferenceEncoderValue(calibrationResult, ExternalAxisType::ZAxis);

    ExternalAxisCalibrationResult externalAxisCalibrationResult;
    std::string errorMessage;
    if (!CalibrationModule::runExternalAxisCalibration(calibrationRequest, axisType, externalAxisCalibrationResult, errorMessage)) {
        fail(QStringLiteral("%1 calibration failed: %2")
                 .arg(CalibrationResultIO::axisDisplayName(axisType))
                 .arg(QString::fromLocal8Bit(errorMessage.c_str())));
        return;
    }

    CalibrationModule::updateExternalAxisCalibrationResult(externalAxisCalibrationResult, calibrationResult);
    if (!CalibrationResultIO::saveToFile(resultFilePath, calibrationResult)) {
        fail(QStringLiteral("Failed to save external axis calibration result."));
        return;
    }

    const cv::Mat unitDirectionMat = externalAxisCalibrationResult.unitDirectionMat;
    emit messageRaised(QStringLiteral("%1 track direction: [%2, %3, %4]")
                           .arg(CalibrationResultIO::axisDisplayName(axisType))
                           .arg(unitDirectionMat.at<double>(0, 0), 0, 'f', 9)
                           .arg(unitDirectionMat.at<double>(1, 0), 0, 'f', 9)
                           .arg(unitDirectionMat.at<double>(2, 0), 0, 'f', 9));
    emit messageRaised(QStringLiteral("%1 hand-eye reference encoder: %2 mm")
                           .arg(CalibrationResultIO::axisDisplayName(axisType))
                           .arg(externalAxisCalibrationResult.currentTrackPosToCalib, 0, 'f', 3));
    emit messageRaised(QStringLiteral("Saved axis matrix to: %1").arg(QDir::toNativeSeparators(resultFilePath)));
    emit calibrationFinished(QStringLiteral("external_axis"), true);
}

void CalibrationWorker::runQueryCoordinate(const QString& calibRootPath,
                                           const QString& resultFilePath,
                                           double pixelU,
                                           double pixelV) {
    Q_UNUSED(calibRootPath);

    auto fail = [this](const QString& message) {
        emit warningRaised(calibrationTitle(), message);
        emit coordinateQueryFinished(QString(), false);
    };

    CoarseLocalizationMatrix calibrationResult;
    if (!CalibrationResultIO::loadFromCacheOrFile(resultFilePath, calibrationResult)) {
        fail(QStringLiteral("Failed to load workpiece_localization_calib.json. Run calibration first."));
        return;
    }

    if (calibrationResult.xAxisCameraToBaseMatrixData.size() != 16) {
        fail(QStringLiteral("X axis matrix is missing in workpiece_localization_calib.json. Run X axis calibration first."));
        return;
    }
    if (calibrationResult.yAxisCameraToBaseMatrixData.size() != 16) {
        fail(QStringLiteral("Y axis matrix is missing in workpiece_localization_calib.json. Run Y axis calibration first."));
        return;
    }
    if (calibrationResult.zAxisCameraToBaseMatrixData.size() != 16) {
        fail(QStringLiteral("Z axis matrix is missing in workpiece_localization_calib.json. Run Z axis calibration first."));
        return;
    }

    const cv::Point2d pixelPoint(pixelU, pixelV);
    const double xAxisEncoderValue = std::isfinite(currentXAxisEncoderValue)
                                         ? currentXAxisEncoderValue
                                         : CalibrationResultIO::axisReferenceEncoderValue(calibrationResult, ExternalAxisType::XAxis);

    if (std::isfinite(currentXAxisEncoderValue)) {
        emit messageRaised(QStringLiteral("Use current X encoder value: %1 mm").arg(currentXAxisEncoderValue, 0, 'f', 3));
    } else {
        emit messageRaised(QStringLiteral("Current X encoder value is unavailable. Use hand-eye reference encoder value: %1 mm")
                               .arg(CalibrationResultIO::axisReferenceEncoderValue(calibrationResult, ExternalAxisType::XAxis), 0, 'f', 3));
    }

    ExternalAxisLocalizationRequest xAxisRequest;
    ExternalAxisLocalizationRequest yAxisRequest;
    ExternalAxisLocalizationRequest zAxisRequest;
    if (!CalibrationResultIO::buildExternalAxisLocalizationRequest(
            calibrationResult, ExternalAxisType::XAxis, pixelPoint, xAxisEncoderValue, xAxisRequest)) {
        fail(QStringLiteral("Failed to build X axis localization request."));
        return;
    }
    if (!CalibrationResultIO::buildExternalAxisLocalizationRequest(
            calibrationResult, ExternalAxisType::YAxis, pixelPoint, 0.0, yAxisRequest)) {
        fail(QStringLiteral("Failed to build Y axis localization request."));
        return;
    }
    if (!CalibrationResultIO::buildExternalAxisLocalizationRequest(
            calibrationResult, ExternalAxisType::ZAxis, pixelPoint, 0.0, zAxisRequest)) {
        fail(QStringLiteral("Failed to build Z axis localization request."));
        return;
    }

    ThreeAxisLocalizationResult localizationResult;
    std::string errorMessage;
    if (!CalibrationModule::localizeThreeAxisCoordinate(xAxisRequest, yAxisRequest, zAxisRequest, localizationResult, errorMessage)) {
        fail(QStringLiteral("Coordinate query failed: %1").arg(QString::fromLocal8Bit(errorMessage.c_str())));
        return;
    }

    const QString resultText = QStringLiteral("X:%1  Y:%2  Z:%3")
                                   .arg(localizationResult.xAxisResult.targetEncoderValue, 0, 'f', 3)
                                   .arg(localizationResult.yAxisResult.targetEncoderValue, 0, 'f', 3)
                                   .arg(localizationResult.zAxisResult.targetEncoderValue, 0, 'f', 3);

    emit messageRaised(QStringLiteral("Pixel (%1, %2) -> %3")
                           .arg(pixelU, 0, 'f', 3)
                           .arg(pixelV, 0, 'f', 3)
                           .arg(resultText));
    emit coordinateQueryFinished(resultText, true);
}
