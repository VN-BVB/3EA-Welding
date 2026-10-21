#ifndef HANDEYECALIBRATION_H
#define HANDEYECALIBRATION_H

#include "Calibration.h"

class HandEyeCalibration : public Calibration {
public:
    enum HandEyeCaliMethod {
        HAND_EYE_NAVY = 0,
        HAND_EYE_TSAI = 1,
    };

    HandEyeCalibration(const std::string& imageDirectory, const std::string& outputFilename = "camera_data.yml", cv::Size boardSize = cv::Size(11, 8),
                       double squareSize = 10.,
                       Pattern type = CHESSBOARD);  // squareSize 与机器人平移使用相同长度单位。

    ~HandEyeCalibration();

    // 从文件读取相机内参和畸变系数矩阵
    // 例如：out_camera_data.yml
    static bool readCameraParameters(const std::string& outputFilename, cv::Mat& camMatrix, cv::Mat& distCoefs);

    // 读取外参与位姿数据
    [[deprecated("请使用另一个重载方法，必须明确指定文件数据格式。")]]
    static bool readDataFromFile(const std::string& file, std::vector<cv::Mat>& vecHg, std::vector<cv::Mat>& vecHc);

    // 从 yml 文件读取外参数据，例如 out_camera_data.yml
    // 保持一一对应关系
    static bool readDataFromFile(const std::string& file, const std::string& folder, std::vector<cv::Mat>& vecHg, std::vector<cv::Mat>& vecHc,
                                 bool useQuaternion = true);

    // 读取 robotpos##.xml 的 Position0～Position5；非四元数分支按 xyz 欧拉角（度）转换，随后将位姿矩阵取逆。
    static bool readRobotDatasFromFile(const std::string& folder, std::vector<cv::Mat>& vecHg, bool useQuaternion);

    // 计算相邻位姿之间的相对变换，构造 AX = XB 的数据对。
    static void convertRelativePoses(std::vector<cv::Mat>& vecHg, std::vector<cv::Mat>& vecHc, std::vector<cv::Mat>& vecHgij,
                                     std::vector<cv::Mat>& vecHcij);

    // 求解 AX = XB
    static void computeHandEyeMatrix(cv::Mat& Hcg, std::vector<cv::Mat>& Hgij, std::vector<cv::Mat>& Hcij, HandEyeCaliMethod method,
                                     const std::string& file);

    static void solveTsaiHandEye(cv::Mat& Hcg, std::vector<cv::Mat>& Hgij, std::vector<cv::Mat>& Hcij);
    static void solveNavyHandEye(cv::Mat& Hcg, std::vector<cv::Mat>& Hgij, std::vector<cv::Mat>& Hcij);

    // 位姿输入为 {x,y,z,rx,ry,rz} 或 {x,y,z,qw,qx,qy,qz,rx,ry,rz}，输出 4×4 变换矩阵。
    // 欧拉角使用角度制；seq 为空时旋转向量使用弧度制。平移长度单位保持输入约定。
    static cv::Mat attitudeVectorToMatrix(cv::Mat m, bool useQuaternion = true, const std::string& seq = "");

    // 检查输入矩阵左上角 3×3 旋转块的正交性。
    static bool isRotationMatrix(const cv::Mat& R);

    // 将角度制欧拉角按 seq 指定顺序转换为旋转矩阵。
    static cv::Mat eulerAngleToRotatedMatrix(const cv::Mat& eulerAngle, const std::string& seq = "xyz");

    // 四元数向量转换为矩阵
    static cv::Mat quaternionToRotatedMatrix(const cv::Vec4d& q);

    // 求解位置
    static cv::Mat solveLocation(cv::Vec3d& pos, cv::Mat& Hcg, cv::Mat& Hg, cv::Mat& extParamMatrix, cv::Mat& camMatrix) = delete;

    static cv::Point3d getWorldPos(cv::Point2d& imgPt, double z, cv::Mat& Hcg, cv::Mat& Hg, cv::Mat& camMatrix);

    // 求解眼在手上场景的相机到夹具 Rt
    static void calibrateEyeInHand(cv::Mat& Hcg, std::vector<cv::Mat>& vecHg, std::vector<cv::Mat>& vecHc, HandEyeCaliMethod method = HAND_EYE_NAVY,
                                   const std::string& file = "EyeInHandMatrix.yml");
};

#endif  // HANDEYECALIBRATION_H
