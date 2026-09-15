#pragma once

#include "calibration.h"

class HandEyeCalibration : public Calibration {
public:
    enum HandEyeCaliMethod {
        HAND_EYE_NAVY = 0,
        HAND_EYE_TSAI = 1,
    };

    HandEyeCalibration(const std::string& imgsDirector,
                       const std::string& outputFilename = "camera_data.yml",
                       cv::Size boardSize = cv::Size(11, 8),
                       double squareSize = 10.,
                       Pattern type = CHESSBOARD);  // mm or m

    ~HandEyeCalibration();

public:
    // load camera extrinsic and disort coefficients matrix
    // form file, eg. out_camera_data.yml
    static bool readCameraParameters(const std::string& outputFilename, cv::Mat& camMatrix, cv::Mat& distCoefs);

    // load extrinsic and attitude datas
    [[deprecated("use another override method instead of this one. ( Files data format must be specified.)")]]
    static bool readDatasFromFile(const std::string& file, std::vector<cv::Mat>& vecHg, std::vector<cv::Mat>& vecHc);

    // load extrinsic datas from yml, eg. out_camera_data.yml, load camera extrinsic datas
    // one-to-one correspondence
    static bool readDatasFromFile(const std::string& file,
                                  const std::string& folder,
                                  std::vector<cv::Mat>& vecHg,
                                  std::vector<cv::Mat>& vecHc,
                                  bool useQuaternion = true);

    static bool readRobotDatasFromFile(const std::string& folder, std::vector<cv::Mat>& vecHg, bool useQuaternion);

    // calculate A/B pairs
    static void convertVectors2Hij(std::vector<cv::Mat>& vecHg,
                                   std::vector<cv::Mat>& vecHc,
                                   std::vector<cv::Mat>& vecHgij,
                                   std::vector<cv::Mat>& vecHcij);

    // solve Ax = xB
    static void computerHandEyeMatrix(cv::Mat& Hcg,
                                      std::vector<cv::Mat>& Hgij,
                                      std::vector<cv::Mat>& Hcij,
                                      HandEyeCaliMethod method,
                                      const std::string& file);

    static void Tsai_HandEye(cv::Mat& Hcg, std::vector<cv::Mat>& Hgij, std::vector<cv::Mat>& Hcij);
    static void Navy_HandEye(cv::Mat& Hcg, std::vector<cv::Mat>& Hgij, std::vector<cv::Mat>& Hcij);

    // attitude vector => matrix, 10 elems {x,y,z,...}
    static cv::Mat attitudeVectorToMatrix(cv::Mat m, bool useQuaternion = true, const std::string& seq = "");

    // check rotation matrix 3x3 or 4x4 {r,t}
    static bool isRotationMatrix(const cv::Mat& R);

    // eulerAngle => rotated matrix
    static cv::Mat eulerAngleToRotatedMatrix(const cv::Mat& eulerAngle, const std::string& seq = "xyz");

    // quaternion vector to matrix
    static cv::Mat quaternionToRotatedMatrix(const cv::Vec4d& q);

    // solve location
    static cv::Mat solveLocation(cv::Vec3d& pos, cv::Mat& Hcg, cv::Mat& Hg, cv::Mat& extParamMatrix, cv::Mat& camMatrix) = delete;

    static cv::Point3d getWorldPos(cv::Point2d& imgPt, double z, cv::Mat& Hcg, cv::Mat& Hg, cv::Mat& camMatrix);

    // solve eye-in-hand camera-to-gripper Rt
    static void calibrateEyeInHand(cv::Mat& Hcg,
                                   std::vector<cv::Mat>& vecHg,
                                   std::vector<cv::Mat>& vecHc,
                                   HandEyeCaliMethod method = HAND_EYE_NAVY,
                                   const std::string& file = "EyeInHandMatrix.yml");
};
