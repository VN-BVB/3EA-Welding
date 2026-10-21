#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <string>
#include <vector>

#include <opencv2/core.hpp>

class Calibration {
public:
    enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

    explicit Calibration(const std::string& imageDirectory, const std::string& outputFilename, cv::Size boardSize, double squareSize, Pattern type);

    // 读取图像并执行相机内外参标定。
    bool doCalibration();

    // 标定成功后保存相机内参、外参和重投影误差。
    bool runAndSave(const std::string& outputFilename, const std::vector<std::vector<cv::Point2f> >& imagePoints, cv::Size imageSize,
                    cv::Size boardSize, Pattern patternType, float squareSize, float aspectRatio, int flags, cv::Mat& cameraMatrix,
                    cv::Mat& distCoeffs, bool writeExtrinsics, bool writePoints);

    // 相机内参标定；squareSize 与输出平移向量使用相同长度单位。
    bool runCalibration(std::vector<std::vector<cv::Point2f> > imagePoints, cv::Size imageSize, cv::Size boardSize, Pattern patternType,
                        float squareSize, float aspectRatio, int flags, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<cv::Mat>& rvecs,
                        std::vector<cv::Mat>& tvecs, std::vector<float>& reprojErrs, double& totalAvgErr);

    // 保存相机参数，保留现有文件字段。
    void saveCameraParams(const std::string& filename, cv::Size imageSize, cv::Size boardSize, float squareSize, float aspectRatio, int flags,
                          const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, const std::vector<cv::Mat>& rvecs,
                          const std::vector<cv::Mat>& tvecs, const std::vector<float>& reprojErrs,
                          const std::vector<std::vector<cv::Point2f> >& imagePoints, double totalAvgErr);

    // 生成标定板坐标系中的角点，长度单位与 squareSize 相同。
    void calculateChessboardCorners(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners, Pattern patternType);

    // 计算标定后的均方根重投影误差，单位为像素。
    static double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f> >& objectPoints,
                                            const std::vector<std::vector<cv::Point2f> >& imagePoints, const std::vector<cv::Mat>& rvecs,
                                            const std::vector<cv::Mat>& tvecs, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
                                            std::vector<float>& perViewErrors);

    // 获取相机标定结果。
    cv::Mat getCameraMatrix() const;
    cv::Mat getDistCoeffsMatrix() const;
    cv::Mat getExtrinsicsBigMat() const;
    std::vector<int> getFoundChessboardFlags() const;

private:
    std::string imageDirectory;
    std::string outputFilename;
    cv::Size boardSize;
    float squareSize = 30;  // 由构造参数覆盖，长度单位由调用者确定。

    std::vector<int> foundChessboardFlags;  // 每张图像是否找到角点：1 为成功，0 为失败。
    cv::Size imageSize;                     // 图像尺寸，单位为像素。

    Pattern pattern = CHESSBOARD;
    std::vector<std::vector<cv::Point2f> > imagePoints;  // 角点像素坐标。
    bool showCorners = true;                             // 在图像上绘制检测到的角点。

    cv::Mat cameraMatrix, distCoeffs;  // 相机内参和畸变系数。
    cv::Mat extrinsicsBigMat;          // n×6 矩阵，每行依次存放旋转向量和平移向量。

    // 标定函数使用的参数。
    float aspectRatio = 1.0;
    int flags = 0;

    bool writeExtrinsics = true;  // 是否存储外参矩阵。
    bool writePoints = true;      // 是否存储像素坐标点。
};

#endif  // CALIBRATION_H
