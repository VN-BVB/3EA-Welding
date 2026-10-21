

#include "Calibration.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <utility>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

Calibration::Calibration(const std::string& imageDirectory, const std::string& outputFilename, cv::Size boardSize, double squareSize, Pattern type) {
    this->imageDirectory = imageDirectory;
    this->outputFilename = outputFilename;
    this->boardSize = boardSize;
    this->squareSize = static_cast<float>(squareSize);
    this->pattern = type;
}

double Calibration::computeReprojectionErrors(const std::vector<std::vector<cv::Point3f>>& objectPoints,
                                              const std::vector<std::vector<cv::Point2f>>& imagePoints, const std::vector<cv::Mat>& rvecs,
                                              const std::vector<cv::Mat>& tvecs, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
                                              std::vector<float>& perViewErrors) {
    std::vector<cv::Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for (i = 0; i < (int)objectPoints.size(); i++) {
        // 根据标定后的内参、外参以及三维坐标重新计算出像素坐标
        cv::projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
        // 计算像素坐标残差的 L2 范数。
        err = cv::norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), cv::NORM_L2);
        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float)std::sqrt(err * err / n);
        totalErr += err * err;
        totalPoints += n;
    }

    return std::sqrt(totalErr / totalPoints);
}

void Calibration::calculateChessboardCorners(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners, Pattern patternType) {
    corners.resize(0);

    switch (patternType) {
        case CHESSBOARD:
        case CIRCLES_GRID:
            for (int i = 0; i < boardSize.height; i++)
                for (int j = 0; j < boardSize.width; j++) corners.push_back(cv::Point3f(float(j * squareSize), float(i * squareSize), 0));
            break;

        case ASYMMETRIC_CIRCLES_GRID:
            for (int i = 0; i < boardSize.height; i++)
                for (int j = 0; j < boardSize.width; j++)
                    corners.push_back(cv::Point3f(float((2 * j + i % 2) * squareSize), float(i * squareSize), 0));
            break;

        default:
            CV_Error(cv::Error::StsBadArg, "Unknown pattern type\n");
    }
}

bool Calibration::runCalibration(std::vector<std::vector<cv::Point2f>> imagePoints, cv::Size imageSize, cv::Size boardSize, Pattern patternType,
                                 float squareSize, float aspectRatio, int flags, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                                 std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs, std::vector<float>& reprojErrs, double& totalAvgErr) {
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    if (flags & cv::CALIB_FIX_ASPECT_RATIO) {
        cameraMatrix.at<double>(0, 0) = aspectRatio;
    }

    distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

    std::vector<std::vector<cv::Point3f>> objectPoints(1);
    // 获取棋盘格坐标系下的棋盘坐标
    calculateChessboardCorners(boardSize, squareSize, objectPoints[0], patternType);
    // 每张图像使用相同的标定板三维坐标。
    objectPoints.resize(imagePoints.size(), objectPoints[0]);

    double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs,
                                     flags | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5);
    ///*|CALIB_FIX_K3*/|CALIB_FIX_K4|CALIB_FIX_K5);
    std::printf("calibrateCamera 返回的 RMS 误差：%g\n", rms);

    bool ok = cv::checkRange(cameraMatrix) && cv::checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;
}

// 保存相机参数
void Calibration::saveCameraParams(const std::string& filename, cv::Size imageSize, cv::Size boardSize, float squareSize, float aspectRatio,
                                   int flags, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, const std::vector<cv::Mat>& rvecs,
                                   const std::vector<cv::Mat>& tvecs, const std::vector<float>& reprojErrs,
                                   const std::vector<std::vector<cv::Point2f>>& imagePoints, double totalAvgErr) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cout << "无法创建 \"" << filename << "\"." << std::endl;
        return;
    }

    if (!cv::utils::fs::exists(filename)) cv::error(cv::Error::StsBadArg, "文件创建失败。", __FUNCTION__, __FILE__, __LINE__);

    std::time_t tt;
    std::time(&tt);
    std::tm* t2 = new std::tm();
    localtime_s(t2, &tt);
    char buf[1024];
    std::strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;

    if (!rvecs.empty() || !reprojErrs.empty()) fs << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << boardSize.width;
    fs << "board_height" << boardSize.height;
    fs << "square_size" << squareSize;

    if (flags & cv::CALIB_FIX_ASPECT_RATIO) fs << "aspectRatio" << aspectRatio;

    if (flags != 0) {
        sprintf_s(buf, "flags: %s%s%s%s", flags & cv::CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
                  flags & cv::CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "", flags & cv::CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
                  flags & cv::CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
        // cvWriteComment( *fs, buf, 0 );
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;
    if (!reprojErrs.empty()) fs << "per_view_reprojection_errors" << cv::Mat(reprojErrs);

    // 分别保存旋转向量和平移向量。
    fs << "rvecs" << rvecs;
    fs << "tvecs" << tvecs;

    if (!rvecs.empty() && !tvecs.empty()) {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        cv::Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for (int i = 0; i < (int)rvecs.size(); i++) {
            // {r, t}
            cv::Mat r = bigmat(cv::Range(i, i + 1), cv::Range(0, 3));
            cv::Mat t = bigmat(cv::Range(i, i + 1), cv::Range(3, 6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            // *.t() 返回 MatExpr（不是 Mat），因此可以使用赋值运算符
            r = rvecs[i].t();  // 旋转向量，单位为弧度。
            t = tvecs[i].t();  // 平移向量，长度单位与 squareSize 相同。

            // Mat r_matrix;
            // cv::Rodrigues(r, r_matrix);
        }
        // cvWriteComment( *fs, "每个视图的一组六元数据（旋转向量与平移向量）", 0 );
        fs << "extrinsic_parameters" << bigmat;

        std::swap(bigmat, extrinsicsBigMat);
    }

    // 保存每张图像的检测标记，读取机器人位姿时据此选择对应数据；文件字段保持原名。
    cv::Mat foundChessboardMat(1, static_cast<int>(foundChessboardFlags.size()), CV_32S, foundChessboardFlags.data());
    fs << "found_cheese_board" << foundChessboardMat;

    // 每行保存一张图像中的全部角点像素坐标。
    if (!imagePoints.empty()) {
        cv::Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for (int i = 0; i < (int)imagePoints.size(); i++) {
            cv::Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            cv::Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }
    fs.release();

    std::cout << "标定完成，数据已保存至文件 " << filename << std::endl << std::endl;
}

bool Calibration::runAndSave(const std::string& outputFilename, const std::vector<std::vector<cv::Point2f>>& imagePoints, cv::Size imageSize,
                             cv::Size boardSize, Pattern patternType, float squareSize, float aspectRatio, int flags, cv::Mat& cameraMatrix,
                             cv::Mat& distCoeffs, bool writeExtrinsics, bool writePoints) {
    // 保存各图像的旋转向量和平移向量。
    std::vector<cv::Mat> rvecs, tvecs;
    // 存放标定后结果的误差值
    std::vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(imagePoints, imageSize, boardSize, patternType, squareSize, aspectRatio, flags, cameraMatrix, distCoeffs, rvecs, tvecs,
                             reprojErrs, totalAvgErr);

    std::printf("%s，平均重投影误差 = %.2f\n", ok ? "标定成功" : "标定失败", totalAvgErr);

    if (ok) {
        saveCameraParams(outputFilename, imageSize, boardSize, squareSize, aspectRatio, flags, cameraMatrix, distCoeffs,
                         writeExtrinsics ? rvecs : std::vector<cv::Mat>(), writeExtrinsics ? tvecs : std::vector<cv::Mat>(),
                         writeExtrinsics ? reprojErrs : std::vector<float>(), writePoints ? imagePoints : std::vector<std::vector<cv::Point2f>>(),
                         totalAvgErr);
    }
    return ok;
}

// 相机内参、外参一起标定
bool Calibration::doCalibration() {
    std::vector<cv::String> imageList;
    std::string path = this->imageDirectory;

    // cv::glob 使用字典序；编号 1、2、…、10 的文件会按 1、10、…、2 的顺序读取。
    // cv::glob(path + "/*.jpg", imageList);  //若为jpg，则切换此
    cv::glob(path + "/*.bmp", imageList);  // 当前标定图像使用 BMP 格式。

    if (imageList.size() == 0) {
        std::cout << "没有图片。" << std::endl;
        return false;
    }
    int nframes = (int)imageList.size();
    foundChessboardFlags.resize(nframes);

    for (int i = 0;; i++) {
        cv::Mat view, viewGray;

        std::cout << "\n";

        if (i < (int)imageList.size()) {
            std::cout << "  " << imageList[i];
            view = cv::imread(imageList[i], cv::IMREAD_COLOR);
        }
        if (view.empty())  // 如果遍历完图像
        {
            if (imagePoints.size() > 0) {
                runAndSave(outputFilename, imagePoints, imageSize, boardSize, pattern, squareSize, aspectRatio, flags, cameraMatrix, distCoeffs,
                           writeExtrinsics, writePoints);
            }
            break;
        }
        imageSize = view.size();
        std::vector<cv::Point2f> pointbuf;
        cv::cvtColor(view, viewGray, cv::COLOR_BGR2GRAY);
        bool found = false;

        // 检测标定板角点，并保存与原始图像顺序一致的成功标记。
        switch (pattern) {
            case CHESSBOARD:
                found = cv::findChessboardCorners(view, boardSize, pointbuf,
                                                  cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
                if (found) {
                    // 获取亚像素角点
                    cv::cornerSubPix(viewGray, pointbuf, cv::Size(11, 11), cv::Size(-1, -1),
                                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
                    imagePoints.push_back(pointbuf);
                    foundChessboardFlags[i] = 1;
                } else {
                    foundChessboardFlags[i] = 0;
                }
                break;

            case CIRCLES_GRID:
                found = cv::findCirclesGrid(view, boardSize, pointbuf, cv::CALIB_CB_SYMMETRIC_GRID);
                if (found) {
                    imagePoints.push_back(pointbuf);
                    foundChessboardFlags[i] = 1;
                } else {
                    foundChessboardFlags[i] = 0;
                }
                break;
                // 非对称的圆形棋盘
            case ASYMMETRIC_CIRCLES_GRID:
                found = cv::findCirclesGrid(view, boardSize, pointbuf, cv::CALIB_CB_ASYMMETRIC_GRID);
                if (found) {
                    imagePoints.push_back(pointbuf);
                    foundChessboardFlags[i] = 1;
                } else {
                    foundChessboardFlags[i] = 0;
                }
                break;
        }

        if (showCorners) {
            if (found) {
                cv::drawChessboardCorners(view, boardSize, cv::Mat(pointbuf), found);
            }
        }
    }
    return true;
}

// 获取结果的方法
cv::Mat Calibration::getExtrinsicsBigMat() const { return this->extrinsicsBigMat; }

cv::Mat Calibration::getCameraMatrix() const { return this->cameraMatrix; }

cv::Mat Calibration::getDistCoeffsMatrix() const { return this->distCoeffs; }

std::vector<int> Calibration::getFoundChessboardFlags() const { return this->foundChessboardFlags; }
