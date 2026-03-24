#ifndef TRAJECTORYPLANNINGCONFIG_H
#define TRAJECTORYPLANNINGCONFIG_H

#include <plog/Log.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cereal/archives/json.hpp>
#include <cereal/types/string.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>

#include "robotFactory/AbstractRobot.h"
#include "utils/common/CommonFunc.h"
#include "utils/myMatrixVector/MyMatrixVector.h"
#include "utils/pointCloud/PointCloudFunc.h"
class TrajectoryPlanningConfig {
public:
    // 获取单例的方法
    static TrajectoryPlanningConfig& getInstance();

    // 禁止拷贝构造和赋值操作
    TrajectoryPlanningConfig(const TrajectoryPlanningConfig&) = delete;
    TrajectoryPlanningConfig& operator=(const TrajectoryPlanningConfig&) = delete;

    void myDataStructure2LibDataStructure();  // 自定义数据类型转换为库数据类型

private:
    TrajectoryPlanningConfig();
    ~TrajectoryPlanningConfig();

    static TrajectoryPlanningConfig* instance;                                                // 静态实例指针
    static std::mutex instanceMutex;                                                          // 保护静态实例的互斥锁
    std::string robotType = MyToolFunc::getRobotTypeString(ROBOT_TYPE::AN_CHUAN);             // 机器人类型
    std::string handEyeType = MyToolFunc::getHandTypeTypeString(HAND_EYE_TYPE::EYE_IN_HAND);  // 手眼类型

    // ############################ 存入JSON的数据类型 ############################
    MyMatrix HandEyeMatrix;  // 手眼

    MyMatrix leftErrorCompensation;  // 误差补偿
    MyMatrix leftMiddleErrorCompensation;
    MyMatrix middleErrorCompensation;
    MyMatrix rightMiddleErrorCompensation;
    MyMatrix rightErrorCompensation;

    MyMatrix beamButtLeftErrorCompensation;
    MyMatrix beamButtLeftMiddleErrorCompensation;
    MyMatrix beamButtMiddleErrorCompensation;
    MyMatrix beamButtRightMiddleErrorCompensation;
    MyMatrix beamButtRightErrorCompensation;

    MyMatrix beamLeftErrorCompensationF;
    MyMatrix beamLeftMiddleErrorCompensationF;
    MyMatrix beamMiddleErrorCompensationF;
    MyMatrix beamRightMiddleErrorCompensationF;
    MyMatrix beamRightErrorCompensationF;

    MyVector leftWeldingPose;         // 左焊缝姿态
    MyVector rightWeldingPose;        // 右焊缝姿态
    MyVector beamButtWeldingPose;     // 背面横梁焊缝姿态
    MyVector startPointPositionPose;  // 起始点 / 过渡点位姿
    MyVector takePhotoPositionPose;   // 眼在手上时的拍照位姿

    // 机器人左侧
    MyVector leftWeldingPose_LEFT;         // 左焊缝姿态
    MyVector rightWeldingPose_LEFT;        // 右焊缝姿态
    MyVector beamButtWeldingPose_LEFT;     // 背面横梁焊缝姿态
    MyVector startPointPositionPose_LEFT;  // 起始点 / 过渡点位姿
    MyVector takePhotoPositionPose_LEFT;   // 眼在手上时的拍照位姿

    // 机器人右侧
    MyVector leftWeldingPose_RIGHT;         // 左焊缝姿态
    MyVector rightWeldingPose_RIGHT;        // 右焊缝姿态
    MyVector beamButtWeldingPose_RIGHT;     // 背面横梁焊缝姿态
    MyVector startPointPositionPose_RIGHT;  // 起始点 / 过渡点位姿
    MyVector takePhotoPositionPose_RIGHT;   // 眼在手上时的拍照位姿

    // ############################ 程序使用的的数据类型 ############################
    Eigen::Matrix4f matrixEyeHand;  // 眼在手外，相机相对于机器人基坐标系的位姿

    Eigen::Matrix4f leftErrorCompensationMatrix;  // 误差补偿矩阵 1 2 3 4 5
    Eigen::Matrix4f leftMiddleErrorCompensationMatrix;
    Eigen::Matrix4f middleErrorCompensationMatrix;
    Eigen::Matrix4f rightMiddleErrorCompensationMatrix;
    Eigen::Matrix4f rightErrorCompensationMatrix;

    Eigen::Matrix4f beamButtLeftErrorCompensationMatrix;  // 横梁对接误差补偿矩阵 1 2 3 4 5
    Eigen::Matrix4f beamButtLeftMiddleErrorCompensationMatrix;
    Eigen::Matrix4f beamButtMiddleErrorCompensationMatrix;
    Eigen::Matrix4f beamButtRightMiddleErrorCompensationMatrix;
    Eigen::Matrix4f beamButtRightErrorCompensationMatrix;

    Eigen::Matrix4f beamLeftErrorCompensationMatrixFront;  // 正面横梁误差补偿矩阵 1 2 3 4 5
    Eigen::Matrix4f beamLeftMiddleErrorCompensationMatrixFront;
    Eigen::Matrix4f beamMiddleErrorCompensationMatrixFront;
    Eigen::Matrix4f beamRightMiddleErrorCompensationMatrixFront;
    Eigen::Matrix4f beamRightErrorCompensationMatrixFront;

    Eigen::Matrix4f matrixEnd2Base;  // 机器人末端坐标系下的点转基坐标系下的点转换矩阵(先标完工具，这里就默认是工具到基座坐标系了)

    float zeroPointX, zeroPointY, zeroPointZ, zeroPointA, zeroPointB, zeroPointC;  // 焊枪过渡点位姿
    float takePhotoX, takePhotoY, takePhotoZ, takePhotoA, takePhotoB, takePhotoC;  // 眼在手上时的拍照位姿
    float leftPoseA, leftPoseB, leftPoseC;                                         // 左侧焊缝姿态
    float rightPoseA, rightPoseB, rightPoseC;                                      // 右侧焊缝姿态
    float beamButtPoseA, beamButtPoseB, beamButtPoseC;                             // 背面横梁对接焊枪姿态

    // 机器人左侧
    float zeroPointX_LEFT, zeroPointY_LEFT, zeroPointZ_LEFT, zeroPointA_LEFT, zeroPointB_LEFT, zeroPointC_LEFT;  // 焊枪过渡点位姿
    float takePhotoX_LEFT, takePhotoY_LEFT, takePhotoZ_LEFT, takePhotoA_LEFT, takePhotoB_LEFT,
        takePhotoC_LEFT;                                               // 眼在手上时的拍照位姿
    float leftPoseA_LEFT, leftPoseB_LEFT, leftPoseC_LEFT;              // 左侧焊缝姿态
    float rightPoseA_LEFT, rightPoseB_LEFT, rightPoseC_LEFT;           // 右侧焊缝姿态
    float beamButtPoseA_LEFT, beamButtPoseB_LEFT, beamButtPoseC_LEFT;  // 背面横梁对接焊枪姿态

    // 机器人右侧
    float zeroPointX_RIGHT, zeroPointY_RIGHT, zeroPointZ_RIGHT, zeroPointA_RIGHT, zeroPointB_RIGHT,
        zeroPointC_RIGHT;  // 焊枪过渡点位姿
    float takePhotoX_RIGHT, takePhotoY_RIGHT, takePhotoZ_RIGHT, takePhotoA_RIGHT, takePhotoB_RIGHT,
        takePhotoC_RIGHT;                                                 // 眼在手上时的拍照位姿
    float leftPoseA_RIGHT, leftPoseB_RIGHT, leftPoseC_RIGHT;              // 左侧焊缝姿态
    float rightPoseA_RIGHT, rightPoseB_RIGHT, rightPoseC_RIGHT;           // 右侧焊缝姿态
    float beamButtPoseA_RIGHT, beamButtPoseB_RIGHT, beamButtPoseC_RIGHT;  // 背面横梁对接焊枪姿态
    // 机器人当前姿态
    robotPose currentRobotPose;

    friend class RailWeldingSystem;
    friend class WeldingMainWindow;
    friend class SteelAngleTrajectoryPlanning;
    friend class GantrayFrameTrajectoryPlanning;
    friend class cereal::access;
    template <class Archive>
    void serialize(Archive& ar) {
        ar(CEREAL_NVP(robotType), CEREAL_NVP(handEyeType), CEREAL_NVP(HandEyeMatrix), CEREAL_NVP(leftErrorCompensation),
           CEREAL_NVP(leftMiddleErrorCompensation), CEREAL_NVP(middleErrorCompensation), CEREAL_NVP(rightMiddleErrorCompensation),
           CEREAL_NVP(rightErrorCompensation), CEREAL_NVP(beamButtLeftErrorCompensation),
           CEREAL_NVP(beamButtLeftMiddleErrorCompensation), CEREAL_NVP(beamButtMiddleErrorCompensation),
           CEREAL_NVP(beamButtRightMiddleErrorCompensation), CEREAL_NVP(beamButtRightErrorCompensation),
           CEREAL_NVP(beamLeftErrorCompensationF), CEREAL_NVP(beamLeftMiddleErrorCompensationF),
           CEREAL_NVP(beamMiddleErrorCompensationF), CEREAL_NVP(beamRightMiddleErrorCompensationF),
           CEREAL_NVP(beamRightErrorCompensationF), CEREAL_NVP(leftWeldingPose), CEREAL_NVP(rightWeldingPose),
           CEREAL_NVP(beamButtWeldingPose), CEREAL_NVP(startPointPositionPose), CEREAL_NVP(takePhotoPositionPose),
           CEREAL_NVP(leftWeldingPose_LEFT), CEREAL_NVP(rightWeldingPose_LEFT), CEREAL_NVP(beamButtWeldingPose_LEFT),
           CEREAL_NVP(startPointPositionPose_LEFT), CEREAL_NVP(takePhotoPositionPose_LEFT), CEREAL_NVP(leftWeldingPose_RIGHT),
           CEREAL_NVP(rightWeldingPose_RIGHT), CEREAL_NVP(beamButtWeldingPose_RIGHT), CEREAL_NVP(startPointPositionPose_RIGHT),
           CEREAL_NVP(takePhotoPositionPose_RIGHT));
    }
};

#endif  // TRAJECTORYPLANNINGCONFIG_H
