#include "TrajectoryPlanningConfig.h"

// 初始化静态成员
TrajectoryPlanningConfig *TrajectoryPlanningConfig::instance = nullptr;
std::mutex TrajectoryPlanningConfig::instanceMutex;

TrajectoryPlanningConfig &TrajectoryPlanningConfig::getInstance() {
    if (instance == nullptr) {  // 双重检查锁定
        std::lock_guard<std::mutex> lock(instanceMutex);
        if (instance == nullptr) {
            instance = new TrajectoryPlanningConfig();
        }
        (void)lock;
    }

    return *instance;
}

TrajectoryPlanningConfig::TrajectoryPlanningConfig()
    : HandEyeMatrix(4, 4),
      leftErrorCompensation(4, 4),
      leftMiddleErrorCompensation(4, 4),
      middleErrorCompensation(4, 4),
      rightMiddleErrorCompensation(4, 4),
      rightErrorCompensation(4, 4),
      beamButtLeftErrorCompensation(4, 4),
      beamButtLeftMiddleErrorCompensation(4, 4),
      beamButtMiddleErrorCompensation(4, 4),
      beamButtRightMiddleErrorCompensation(4, 4),
      beamButtRightErrorCompensation(4, 4),
      beamLeftErrorCompensationF(4, 4),
      beamLeftMiddleErrorCompensationF(4, 4),
      beamMiddleErrorCompensationF(4, 4),
      beamRightMiddleErrorCompensationF(4, 4),
      beamRightErrorCompensationF(4, 4),
      leftWeldingPose(3),
      rightWeldingPose(3),
      beamButtWeldingPose(3),
      startPointPositionPose(6),
      takePhotoPositionPose(6),
      leftWeldingPose_LEFT(3),
      rightWeldingPose_LEFT(3),
      beamButtWeldingPose_LEFT(3),
      startPointPositionPose_LEFT(6),
      takePhotoPositionPose_LEFT(6),
      leftWeldingPose_RIGHT(3),
      rightWeldingPose_RIGHT(3),
      beamButtWeldingPose_RIGHT(3),
      startPointPositionPose_RIGHT(6),
      takePhotoPositionPose_RIGHT(6) {}

// 自定义数据类型转换为库数据类型
void TrajectoryPlanningConfig::myDataStructure2LibDataStructure() {
    MyToolFunc::write2Eigen4x4f(HandEyeMatrix, matrixEyeHand);

    MyToolFunc::write2Eigen4x4f(leftErrorCompensation, leftErrorCompensationMatrix);
    MyToolFunc::write2Eigen4x4f(leftMiddleErrorCompensation, leftMiddleErrorCompensationMatrix);
    MyToolFunc::write2Eigen4x4f(middleErrorCompensation, middleErrorCompensationMatrix);
    MyToolFunc::write2Eigen4x4f(rightMiddleErrorCompensation, rightMiddleErrorCompensationMatrix);
    MyToolFunc::write2Eigen4x4f(rightErrorCompensation, rightErrorCompensationMatrix);

    MyToolFunc::write2Eigen4x4f(beamButtLeftErrorCompensation, beamButtLeftErrorCompensationMatrix);
    MyToolFunc::write2Eigen4x4f(beamButtLeftMiddleErrorCompensation, beamButtLeftMiddleErrorCompensationMatrix);
    MyToolFunc::write2Eigen4x4f(beamButtMiddleErrorCompensation, beamButtMiddleErrorCompensationMatrix);
    MyToolFunc::write2Eigen4x4f(beamButtRightMiddleErrorCompensation, beamButtRightMiddleErrorCompensationMatrix);
    MyToolFunc::write2Eigen4x4f(beamButtRightErrorCompensation, beamButtRightErrorCompensationMatrix);

    MyToolFunc::write2Eigen4x4f(beamLeftErrorCompensationF, beamLeftErrorCompensationMatrixFront);
    MyToolFunc::write2Eigen4x4f(beamLeftMiddleErrorCompensationF, beamLeftMiddleErrorCompensationMatrixFront);
    MyToolFunc::write2Eigen4x4f(beamMiddleErrorCompensationF, beamMiddleErrorCompensationMatrixFront);
    MyToolFunc::write2Eigen4x4f(beamRightMiddleErrorCompensationF, beamRightMiddleErrorCompensationMatrixFront);
    MyToolFunc::write2Eigen4x4f(beamRightErrorCompensationF, beamRightErrorCompensationMatrixFront);

    MyToolFunc::write2PositionPose(startPointPositionPose, zeroPointX, zeroPointY, zeroPointZ, zeroPointA, zeroPointB,
                                   zeroPointC);
    MyToolFunc::write2PositionPose(takePhotoPositionPose, takePhotoX, takePhotoY, takePhotoZ, takePhotoA, takePhotoB, takePhotoC);
    MyToolFunc::write2Pose(leftWeldingPose, leftPoseA, leftPoseB, leftPoseC);
    MyToolFunc::write2Pose(rightWeldingPose, rightPoseA, rightPoseB, rightPoseC);
    MyToolFunc::write2Pose(beamButtWeldingPose, beamButtPoseA, beamButtPoseB, beamButtPoseC);

    MyToolFunc::write2PositionPose(startPointPositionPose_LEFT, zeroPointX_LEFT, zeroPointY_LEFT, zeroPointZ_LEFT,
                                   zeroPointA_LEFT, zeroPointB_LEFT, zeroPointC_LEFT);
    MyToolFunc::write2PositionPose(takePhotoPositionPose_LEFT, takePhotoX_LEFT, takePhotoY_LEFT, takePhotoZ_LEFT, takePhotoA_LEFT,
                                   takePhotoB_LEFT, takePhotoC_LEFT);
    MyToolFunc::write2Pose(leftWeldingPose_LEFT, leftPoseA_LEFT, leftPoseB_LEFT, leftPoseC_LEFT);
    MyToolFunc::write2Pose(rightWeldingPose_LEFT, rightPoseA_LEFT, rightPoseB_LEFT, rightPoseC_LEFT);
    MyToolFunc::write2Pose(beamButtWeldingPose_LEFT, beamButtPoseA_LEFT, beamButtPoseB_LEFT, beamButtPoseC_LEFT);

    MyToolFunc::write2PositionPose(startPointPositionPose_RIGHT, zeroPointX_RIGHT, zeroPointY_RIGHT, zeroPointZ_RIGHT,
                                   zeroPointA_RIGHT, zeroPointB_RIGHT, zeroPointC_RIGHT);
    MyToolFunc::write2PositionPose(takePhotoPositionPose_RIGHT, takePhotoX_RIGHT, takePhotoY_RIGHT, takePhotoZ_RIGHT,
                                   takePhotoA_RIGHT, takePhotoB_RIGHT, takePhotoC_RIGHT);
    MyToolFunc::write2Pose(leftWeldingPose_RIGHT, leftPoseA_RIGHT, leftPoseB_RIGHT, leftPoseC_RIGHT);
    MyToolFunc::write2Pose(rightWeldingPose_RIGHT, rightPoseA_RIGHT, rightPoseB_RIGHT, rightPoseC_RIGHT);
    MyToolFunc::write2Pose(beamButtWeldingPose_RIGHT, beamButtPoseA_RIGHT, beamButtPoseB_RIGHT, beamButtPoseC_RIGHT);

    if (robotType == MyToolFunc::getRobotTypeString(ROBOT_TYPE::AN_CHUAN)) {
        matrixEnd2Base = MyToolFunc::createTransformationMatrixZYX(takePhotoPositionPose);
    } else if (robotType == MyToolFunc::getRobotTypeString(ROBOT_TYPE::BAO_YUAN)) {
        matrixEnd2Base = MyToolFunc::createTransformationMatrixZYZ(takePhotoPositionPose);
    }
}
