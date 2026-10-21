#include "AxisCalibration.h"

#include <cmath>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "common/BoardDetection.h"
#include "common/PoseCalculation.h"

namespace {

bool solveDirectionInBase(const AxisCalibration::Input& input, cv::Mat& translationMat, cv::Mat& unitDirectionMat, std::string& errorMessage);

}  // 匿名命名空间

namespace AxisCalibration {

bool calibrate(const Input& input, Type axisType, Result& result, std::string& errorMessage) {
    errorMessage.clear();
    switch (axisType) {
        case Type::XAxis:
        case Type::YAxis:
        case Type::ZAxis:
            break;
        default:
            errorMessage = "外轴类型无效。";
            return false;
    }
    if (input.imagePath.empty()) {
        errorMessage = "外轴图片路径为空。";
        return false;
    }

    cv::Mat translationMat;
    cv::Mat unitDirectionMat;
    if (!solveDirectionInBase(input, translationMat, unitDirectionMat, errorMessage)) {
        return false;
    }

    cv::Mat cameraToBaseMatrix;
    if (axisType == Type::XAxis) {
        // 保留既有规则：X 轴沿用参考手眼矩阵，仅将平移从米换算为毫米。
        cameraToBaseMatrix = input.cameraToBaseMatrix.clone();
        for (int i = 0; i < 3; ++i) {
            cameraToBaseMatrix.at<double>(i, 3) *= 1000.0;
        }
    } else {
        cv::Mat rotationMatrix = input.cameraToBaseMatrix(cv::Rect(0, 0, 3, 3)).clone();
        cv::Mat translationVector = input.cameraToBaseMatrix(cv::Rect(3, 0, 1, 3)).clone();
        // Y/Z 轴仍按参考编码器值修正平移；手眼为米，编码器和输出矩阵为毫米。
        cv::Mat updatedTranslation = translationVector * 1000.0 - unitDirectionMat * input.referenceEncoderValue;

        cameraToBaseMatrix = cv::Mat::eye(4, 4, CV_64F);
        rotationMatrix.copyTo(cameraToBaseMatrix(cv::Rect(0, 0, 3, 3)));
        updatedTranslation.copyTo(cameraToBaseMatrix(cv::Rect(3, 0, 1, 3)));
    }

    result.axisType = axisType;
    result.cameraToBaseMatrix = cameraToBaseMatrix;
    result.translationMat = translationMat;
    result.unitDirectionMat = unitDirectionMat;
    result.referenceEncoderValue = input.referenceEncoderValue;
    return true;
}

}  // AxisCalibration 命名空间

namespace {

bool solveDirectionInBase(const AxisCalibration::Input& input, cv::Mat& translationMat, cv::Mat& unitDirectionMat,
                          std::string& errorMessage) {
    const std::string& imagePath = input.imagePath;
    std::vector<cv::Mat> images;
    std::vector<cv::String> imagePaths;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Mat> boardToCameraTransforms;
    std::vector<std::vector<cv::Point3d>> basePointsByImage;
    const cv::Size boardSize(input.boardConfig.boardWidth, input.boardConfig.boardHeight);
    cv::Point3d totalTranslation(0.0, 0.0, 0.0);
    int totalPairs = 0;

    translationMat.release();
    unitDirectionMat.release();

    if (input.cameraMatrix.empty() || input.distCoeffs.empty()) {
        errorMessage = "相机内参为空。";
        return false;
    }
    if (input.cameraToBaseMatrix.empty() || input.cameraToBaseMatrix.rows != 4 || input.cameraToBaseMatrix.cols != 4) {
        errorMessage = "相机到基坐标系矩阵为空或不是 4×4 矩阵。";
        return false;
    }

    cv::glob(imagePath + "/*.bmp", imagePaths);
    if (imagePaths.empty()) {
        errorMessage = "未在以下目录中找到 bmp 图片：" + imagePath;
        return false;
    }

    for (const cv::String& currentImagePath : imagePaths) {
        cv::Mat image = cv::imread(currentImagePath);
        if (!image.empty()) {
            images.push_back(image);
        }
    }
    if (images.size() < 2) {
        errorMessage = "以下目录至少需要两张有效图片：" + imagePath;
        return false;
    }

    BoardDetection::createObjectPoints(input.boardConfig.boardWidth, input.boardConfig.boardHeight, input.boardConfig.circleDistance,
                                       objectPoints);
    if (!BoardDetection::findCirclesInImages(images, boardSize, imagePoints) || imagePoints.size() < 2) {
        errorMessage = "无法从以下目录提取圆心：" + imagePath;
        return false;
    }

    cv::Mat cameraMatrix = input.cameraMatrix.clone();
    cv::Mat distCoeffs = input.distCoeffs.clone();
    PoseCalculation::solveBoardPoses(cameraMatrix, distCoeffs, objectPoints, imagePoints, boardToCameraTransforms);
    if (boardToCameraTransforms.size() < 2) {
        errorMessage = "以下目录的外参求解失败：" + imagePath;
        return false;
    }

    for (const cv::Mat& boardToCamera : boardToCameraTransforms) {
        std::vector<cv::Point3d> basePoints;
        for (const cv::Point3f& objectPoint : objectPoints) {
            cv::Mat homogeneousPoint = (cv::Mat_<double>(4, 1) << objectPoint.x, objectPoint.y, objectPoint.z, 1.0);
            cv::Mat basePoint = input.cameraToBaseMatrix * boardToCamera * homogeneousPoint;
            basePoints.push_back(cv::Point3d(basePoint.at<double>(0, 0), basePoint.at<double>(1, 0), basePoint.at<double>(2, 0)));
        }
        basePointsByImage.push_back(basePoints);
    }

    if (basePointsByImage.size() < 2) {
        errorMessage = "以下目录未生成足够的有效图片对：" + imagePath;
        return false;
    }

    for (int i = 0; i < static_cast<int>(basePointsByImage.size()) - 1; ++i) {
        const std::vector<cv::Point3d>& firstPoints = basePointsByImage[i];
        const std::vector<cv::Point3d>& secondPoints = basePointsByImage[i + 1];
        if (firstPoints.size() != secondPoints.size()) {
            continue;
        }
        for (int j = 0; j < static_cast<int>(firstPoints.size()); ++j) {
            totalTranslation += secondPoints[j] - firstPoints[j];
            ++totalPairs;
        }
    }

    if (totalPairs <= 0) {
        errorMessage = "以下目录未生成有效点对：" + imagePath;
        return false;
    }

    totalTranslation /= static_cast<double>(totalPairs);
    translationMat = (cv::Mat_<double>(3, 1) << totalTranslation.x, totalTranslation.y, totalTranslation.z);

    const double magnitude =
        std::sqrt(totalTranslation.x * totalTranslation.x + totalTranslation.y * totalTranslation.y + totalTranslation.z * totalTranslation.z);
    if (magnitude < 1e-9) {
        errorMessage = "以下目录中的位移量过小：" + imagePath;
        return false;
    }

    unitDirectionMat = (cv::Mat_<double>(3, 1) << totalTranslation.x / magnitude, totalTranslation.y / magnitude, totalTranslation.z / magnitude);
    return true;
}

}  // 匿名命名空间
