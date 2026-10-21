#include "BoardDetection.h"

#include <algorithm>
#include <cstdio>
#include <iostream>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {

cv::Ptr<cv::FeatureDetector> circleDetector();
void prepareCircleImage(const cv::Mat& view, cv::Mat& gray);
void normalizePointOrder(std::vector<cv::Point2f>& points);

}  // 匿名命名空间

namespace BoardDetection {

void createObjectPoints(int boardWidth, int boardHeight, double spacing, std::vector<cv::Point3f>& points) {
    for (int row = 0; row < boardHeight; ++row) {
        for (int col = 0; col < boardWidth; ++col) {
            points.push_back(cv::Point3f(col * spacing, row * spacing, 0));
        }
    }
}

bool findChessboardCornersInFolder(const std::string& path, cv::Size boardSize, std::vector<std::vector<cv::Point2f>>& imagePoints) {
    std::vector<cv::String> imageList;
    cv::glob(path + "/*.bmp", imageList);
    if (imageList.empty()) {
        std::cerr << "目录中未找到图片！" << std::endl;
        return false;
    }

    int imageCount = 0;
    for (const cv::String& imagePath : imageList) {
        std::cout << "正在处理图片：" << imagePath << std::endl;
        cv::Mat view = cv::imread(imagePath, cv::IMREAD_COLOR);
        if (view.empty()) {
            std::cout << "无法打开或未找到图片：" << imagePath << std::endl;
            continue;
        }

        cv::Mat gray;
        if (view.channels() == 3) {
            cv::cvtColor(view, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = view;
        }

        std::vector<cv::Point2f> points;
        if (cv::findChessboardCornersSB(gray, boardSize, points)) {
            cv::cornerSubPix(gray, points, cv::Size(5, 5), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.1));
            normalizePointOrder(points);
            int pointIndex = 0;
            for (const cv::Point2f& point : points) {
                cv::circle(view, point, 1, cv::Scalar(0, 0, 255), -1);
                cv::putText(view, std::to_string(pointIndex++), point, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
            }
            imagePoints.push_back(points);
        } else {
            std::cout << "第 " << imageCount << " 张图片未找到棋盘格角点！\n";
        }

        ++imageCount;
        if (imageCount == 1) {
            std::cout << "图片尺寸：" << view.cols << "x" << view.rows << std::endl;
        }
    }
    return true;
}

bool findCirclesInFolder(const std::string& path, cv::Size boardSize, std::vector<std::vector<cv::Point2f>>& imagePoints) {
    std::vector<cv::String> imageList;
    cv::glob(path + "/*.bmp", imageList);
    if (imageList.empty()) {
        std::cout << "没有图片。" << std::endl;
        return false;
    }

    for (const cv::String& imagePath : imageList) {
        std::cout << "图片文件：" << imagePath << "\n";
        cv::Mat view = cv::imread(imagePath, cv::IMREAD_COLOR);
        cv::Mat gray;
        prepareCircleImage(view, gray);

        std::vector<cv::Point2f> points;
        const bool found = cv::findCirclesGrid(gray, boardSize, points, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, circleDetector());
        if (found) {
            normalizePointOrder(points);
            imagePoints.push_back(points);
        } else {
            std::cout << "当前图片未检测到圆点阵列。" << std::endl;
            std::string renamedPath = imagePath;
            const size_t dot = renamedPath.find_last_of('.');
            if (dot != std::string::npos) {
                renamedPath.insert(dot, "_unfind");
            }
            if (std::rename(imagePath.c_str(), renamedPath.c_str()) != 0) {
                std::cerr << "无法重命名文件：" << imagePath << std::endl;
            }
        }
        cv::drawChessboardCorners(view, boardSize, cv::Mat(points), found);
    }
    return true;
}

bool findCirclesInImages(std::vector<cv::Mat>& images, cv::Size boardSize, std::vector<std::vector<cv::Point2f>>& imagePoints) {
    if (images.empty()) {
        std::cout << "没有图片。" << std::endl;
        return false;
    }

    for (size_t i = 0; i < images.size(); ++i) {
        cv::Mat view = images[i];
        cv::Mat gray;
        std::cout << "正在处理第 " << i + 1 << " / " << images.size() << std::endl;
        prepareCircleImage(view, gray);

        std::vector<cv::Point2f> points;
        const bool found = cv::findCirclesGrid(gray, boardSize, points, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, circleDetector());
        if (found) {
            normalizePointOrder(points);
            imagePoints.push_back(points);
        } else {
            std::cout << "当前图片未检测到圆点阵列。" << std::endl;
        }
        cv::drawChessboardCorners(view, boardSize, cv::Mat(points), found);
    }
    return true;
}

}  // BoardDetection 命名空间

namespace {

cv::Ptr<cv::FeatureDetector> circleDetector() {
    cv::SimpleBlobDetector::Params params;
    params.maxArea = 10e4;
    params.minArea = 30;
    params.minDistBetweenBlobs = 10;
    params.filterByInertia = true;
    params.minInertiaRatio = 0.5f;
    return cv::SimpleBlobDetector::create(params);
}

void prepareCircleImage(const cv::Mat& view, cv::Mat& gray) {
    cv::cvtColor(view, gray, cv::COLOR_BGR2GRAY);
    cv::bitwise_not(gray, gray);
    for (int row = 0; row < gray.rows; ++row) {
        for (int col = 0; col < gray.cols; ++col) {
            gray.at<uchar>(row, col) = 255 - gray.at<uchar>(row, col);
        }
    }
}

void normalizePointOrder(std::vector<cv::Point2f>& points) {
    if (!points.empty() && points.front().x > points.back().x) {
        std::reverse(points.begin(), points.end());
    }
}

}  // 匿名命名空间
