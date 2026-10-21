#include "PoseCalculation.h"

#include <opencv2/calib3d.hpp>

namespace PoseCalculation {

void splitTransform(cv::Mat& transform, cv::Mat& rotation, cv::Mat& translation) {
    rotation = transform(cv::Rect(0, 0, 3, 3));
    translation = transform(cv::Rect(3, 0, 1, 3));
}

cv::Mat composeTransform(cv::Mat& rotation, cv::Mat& translation) {
    cv::Mat result;
    cv::Mat_<double> rotation4x3 = (cv::Mat_<double>(4, 3) << rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2),
                                    rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2), rotation.at<double>(2, 0),
                                    rotation.at<double>(2, 1), rotation.at<double>(2, 2), 0.0, 0.0, 0.0);
    cv::Mat_<double> translation4x1 =
        (cv::Mat_<double>(4, 1) << translation.at<double>(0, 0), translation.at<double>(1, 0), translation.at<double>(2, 0), 1.0);
    cv::hconcat(rotation4x3, translation4x1, result);
    return result;
}

void solveBoardPoses(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<cv::Point3f>& objectPoints,
                     std::vector<std::vector<cv::Point2f>>& imagePoints, std::vector<cv::Mat>& transforms) {
    std::vector<double> distortion(distCoeffs.begin<double>(), distCoeffs.end<double>());
    for (size_t i = 0; i < imagePoints.size(); ++i) {
        cv::Mat rotationVector = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::Mat translation = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::solvePnP(objectPoints, imagePoints[i], cameraMatrix, distortion, rotationVector, translation);
        cv::Mat rotation = cv::Mat::eye(3, 3, CV_64F);
        cv::Rodrigues(rotationVector, rotation);
        cv::Mat top;
        cv::hconcat(rotation, translation, top);
        cv::Mat bottom = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
        cv::Mat transform;
        cv::vconcat(top, bottom, transform);
        transforms.push_back(transform);
    }
}

}  // PoseCalculation 命名空间
