#include "SteelAngleTrajectoryPlanning.h"

#include "robotTrajectoryPlanning/config/TrajectoryPlanningConfig.h"
#include "settingPara/SettingPara.h"
#include "utils/common/CommonFunc.h"
#include "utils/common/WeldSeamInfo.h"
#include "utils/pointCloud/PointCloudFunc.h"

SteelAngleTrajectoryPlanning::SteelAngleTrajectoryPlanning(QObject* parent) : AbstractTrajectoryPlanning(parent) {
    // this->initConfig();
}

// 初始化参数
void SteelAngleTrajectoryPlanning::initPara() {
    if (trajectoryConfig.robotType == MyToolFunc::getRobotTypeString(ROBOT_TYPE::BAO_YUAN)) {
        moveSpeed = settingPara.Value_MoveSpeed * 60;                            // 过渡运动速度
        weldingSpeedDefault = settingPara.Value_WeldingSpeed * 60;               // 焊接速度 (默认速度，宽度检测失败时用这个速度)
        weldingSpeed0To1 = settingPara.Value_WeldingSpeed0To1 * 60;              // 焊接速度 (焊缝宽度1mm以下用这个速度)
        weldingSpeed1To3 = settingPara.Value_WeldingSpeed1To3 * 60;              // 焊接速度 (焊缝宽度1mm到3mm用这个速度)
        weldingSpeed3To5 = settingPara.Value_WeldingSpeed3To5 * 60;              // 焊接速度 (焊缝宽度1mm到3mm用这个速度)
        weldingSpeedHorizontal = settingPara.Value_WeldingSpeedHorizontal * 60;  // 焊接速度 (水平焊缝的焊接速度)
        weldingSpeedVertical = settingPara.Value_WeldingSpeedVertical * 60;      // 焊接速度 (水平焊缝的焊接速度)
    }
}

// 规划焊缝轨迹
void SteelAngleTrajectoryPlanning::whenPlanningTrajectory(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo) {
    PLOGD << "角钢轨迹规划类 收到焊缝数量: " << weldSeamInfo.size();
    // ########################### 按照焊缝的左右对焊缝进行排序 ###########################
    this->sortSeamsWithX(weldSeamInfo);
    PLOGD << "左右排序后的焊缝: ";  // 打印操作后的焊缝
    for (auto& info : weldSeamInfo) {
        if (info->detectSuccFlag == true && info->weldEndPointsInCamera != nullptr && info->weldEndPointsInCamera->size() == 2) {
            PLOGD << info->weldEndPointsInCamera->at(0) << " " << info->weldEndPointsInCamera->at(1);
        }
    }

    // ########################### 区分左侧焊缝和右侧焊缝 ###########################
    int endOfLeftSeamSerial = this->findEndOfLeftSeams(weldSeamInfo);
    this->endOfLeftSeamSerial = endOfLeftSeamSerial;
    PLOGD << "左侧最后一条焊缝的索引: " << endOfLeftSeamSerial;

    // ########################### 转换坐标点到机器人基坐标系下 ###########################
    this->transSeams2Base(weldSeamInfo);
    PLOGD << "转到机器人基坐标系下后的焊缝点: ";  // 打印操作后的焊缝
    for (auto& info : weldSeamInfo) {
        if (info->detectSuccFlag == true && info->weldEndPointsInRobot != nullptr && info->weldEndPointsInRobot->size() == 2) {
            PLOGD << info->weldEndPointsInRobot->at(0) << " " << info->weldEndPointsInRobot->at(1);
        }
    }

    // ########################### 对工件摆放进行方位判别 ###########################
    this->determineWorkpieceOri(weldSeamInfo);
    if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::FRONT)
        PLOGD << "工件位于机器人前面";
    else if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::LEFT)
        PLOGD << "工件位于机器人左面";
    else if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::RIGHT)
        PLOGD << "工件位于机器人右面";
    else
        PLOGD << "工件判面失败";

    // ########################### 对焊缝点进行误差补偿 ###########################
    this->seamsErrorCompensate(weldSeamInfo);
    PLOGD << "误差补偿后的焊缝点: ";  // 打印操作后的焊缝
    for (auto& info : weldSeamInfo) {
        if (info->detectSuccFlag == true && info->weldEndPointsInRobot != nullptr && info->weldEndPointsInRobot->size() == 2) {
            PLOGD << info->weldEndPointsInRobot->at(0) << " " << info->weldEndPointsInRobot->at(1);
        }
    }

    // ########################### 真实坐标系转虚拟坐标系 ###########################
    this->real2Virtual(weldSeamInfo);
    PLOGD << "真实坐标系转虚拟坐标系后的焊缝点: ";  // 打印操作后的焊缝
    for (auto& info : weldSeamInfo) {
        if (info->detectSuccFlag == true && info->weldEndPointsInRobot != nullptr && info->weldEndPointsInRobot->size() == 2) {
            PLOGD << info->weldEndPointsInRobot->at(0) << " " << info->weldEndPointsInRobot->at(1);
        }
    }

    // ########################### 修改焊缝方向 ###########################
    this->transSeamsOri(weldSeamInfo, endOfLeftSeamSerial);
    PLOGD << "修改焊缝方向后的焊缝点: ";  // 打印操作后的焊缝
    for (auto& info : weldSeamInfo) {
        if (info->detectSuccFlag == true && info->weldEndPointsInRobot != nullptr && info->weldEndPointsInRobot->size() == 2) {
            PLOGD << info->weldEndPointsInRobot->at(0) << " " << info->weldEndPointsInRobot->at(1);
        }
    }

    // ########################### 延长焊缝 ###########################
    this->extendSeams(weldSeamInfo, endOfLeftSeamSerial);
    PLOGD << "延长后的焊缝点: ";  // 打印操作后的焊缝
    for (auto& info : weldSeamInfo) {
        if (info->detectSuccFlag == true && info->weldEndPointsInRobot != nullptr && info->weldEndPointsInRobot->size() == 2) {
            PLOGD << info->weldEndPointsInRobot->at(0) << " " << info->weldEndPointsInRobot->at(1);
        }
    }

    // ########################### 虚拟坐标系转真实坐标系 ###########################
    this->virtual2Real(weldSeamInfo);
    PLOGD << "虚拟坐标系转真实坐标系后的焊缝点: ";  // 打印操作后的焊缝
    for (auto& info : weldSeamInfo) {
        if (info->detectSuccFlag == true && info->weldEndPointsInRobot != nullptr && info->weldEndPointsInRobot->size() == 2) {
            PLOGD << info->weldEndPointsInRobot->at(0) << " " << info->weldEndPointsInRobot->at(1);
        }
    }
    for (auto& info : weldSeamInfo) {
        if (!info) continue;

        if (info->cloudFuture.isRunning()) {
            info->cloudFuture.waitForFinished();
        }
        if (info->cloudFuture.isFinished()) {
            info->weldAreaPointCloudInRobot = info->cloudFuture.result();
        }
    }

    // 发出规划完成的焊缝
    emit sendPlannedSeams(weldSeamInfo);
}

// 焊缝写入文件
void SteelAngleTrajectoryPlanning::write2File(const std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo, int endOfLeftSeams) {
    outfile.open(outfile_name, std::ios::out);
    if (!outfile.is_open()) {
        PLOGE << "无法打开输出文件: " << outfile_name;
        return;
    }

    if (weldSeamInfo.size() == 0) {
        PLOGE << "没有焊缝信息";
        outfile.close();
        emit sendTrajectoryPlanOver();  // 但也发送轨迹规划完成
        return;
    } else {
        PLOGD << "开始写入焊缝数据数量: " << weldSeamInfo.size();
    }

    // 更新速度
    moveSpeed = settingPara.Value_MoveSpeed * 60;
    weldingSpeedDefault = settingPara.Value_WeldingSpeed * 60;
    weldingSpeed0To1 = settingPara.Value_WeldingSpeed0To1 * 60;
    weldingSpeed1To3 = settingPara.Value_WeldingSpeed1To3 * 60;
    weldingSpeed3To5 = settingPara.Value_WeldingSpeed3To5 * 60;
    weldingSpeedHorizontal = settingPara.Value_WeldingSpeedHorizontal * 60;
    weldingSpeedVertical = settingPara.Value_WeldingSpeedVertical * 60;
    weldingCurrent = settingPara.Value_WeldingCurrent;
    weldingCurrent_Vertical = settingPara.Value_WeldingCurrent_Vertical;
    weldingVoltage = settingPara.Value_WeldingVoltage;
    weldingVoltage_Vertical = settingPara.Value_WeldingVoltage_Vertical;

    // 预读零点、拍照点、焊接时姿态
    float X0 = trajectoryConfig.zeroPointX, Y0 = trajectoryConfig.zeroPointY, Z0 = trajectoryConfig.zeroPointZ;
    float A0 = trajectoryConfig.zeroPointA, B0 = trajectoryConfig.zeroPointB, C0 = trajectoryConfig.zeroPointC;

    float takePhotoX0 = trajectoryConfig.takePhotoX, takePhotoY0 = trajectoryConfig.takePhotoY, takePhotoZ0 = trajectoryConfig.takePhotoZ;
    float takePhotoA0 = trajectoryConfig.takePhotoA, takePhotoB0 = trajectoryConfig.takePhotoB, takePhotoC0 = trajectoryConfig.takePhotoC;

    float leftA = trajectoryConfig.leftPoseA, leftB = trajectoryConfig.leftPoseB, leftC = trajectoryConfig.leftPoseC;
    float rightA = trajectoryConfig.rightPoseA, rightB = trajectoryConfig.rightPoseB, rightC = trajectoryConfig.rightPoseC;
    float backBeamA = trajectoryConfig.beamButtPoseA, backBeamB = trajectoryConfig.beamButtPoseB, backBeamC = trajectoryConfig.beamButtPoseC;

    // clang-format off
    // 机器人焊接其左侧或右侧工件时, 分别单独配置零点、拍照点、焊接时姿态
    if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::LEFT) {
        X0 = trajectoryConfig.zeroPointX_LEFT; Y0 = trajectoryConfig.zeroPointY_LEFT; Z0 = trajectoryConfig.zeroPointZ_LEFT;
        A0 = trajectoryConfig.zeroPointA_LEFT; B0 = trajectoryConfig.zeroPointB_LEFT; C0 = trajectoryConfig.zeroPointC_LEFT;

        takePhotoX0 = trajectoryConfig.takePhotoX_LEFT; takePhotoY0 = trajectoryConfig.takePhotoY_LEFT; takePhotoZ0 = trajectoryConfig.takePhotoZ_LEFT;
        takePhotoA0 = trajectoryConfig.takePhotoA_LEFT; takePhotoB0 = trajectoryConfig.takePhotoB_LEFT; takePhotoC0 = trajectoryConfig.takePhotoC_LEFT;

        leftA = trajectoryConfig.leftPoseA_LEFT; leftB = trajectoryConfig.leftPoseB_LEFT; leftC = trajectoryConfig.leftPoseC_LEFT;
        rightA = trajectoryConfig.rightPoseA_LEFT; rightB = trajectoryConfig.rightPoseB_LEFT; rightC = trajectoryConfig.rightPoseC_LEFT;
        backBeamA = trajectoryConfig.beamButtPoseA_LEFT; backBeamB = trajectoryConfig.beamButtPoseB_LEFT; backBeamC = trajectoryConfig.beamButtPoseC_LEFT;
    } else if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::RIGHT) {
        X0 = trajectoryConfig.zeroPointX_RIGHT; Y0 = trajectoryConfig.zeroPointY_RIGHT; Z0 = trajectoryConfig.zeroPointZ_RIGHT;
        A0 = trajectoryConfig.zeroPointA_RIGHT; B0 = trajectoryConfig.zeroPointB_RIGHT; C0 = trajectoryConfig.zeroPointC_RIGHT;

        takePhotoX0 = trajectoryConfig.takePhotoX_RIGHT; takePhotoY0 = trajectoryConfig.takePhotoY_RIGHT; takePhotoZ0 = trajectoryConfig.takePhotoZ_RIGHT;
        takePhotoA0 = trajectoryConfig.takePhotoA_RIGHT; takePhotoB0 = trajectoryConfig.takePhotoB_RIGHT; takePhotoC0 = trajectoryConfig.takePhotoC_RIGHT;

        leftA = trajectoryConfig.leftPoseA_RIGHT; leftB = trajectoryConfig.leftPoseB_RIGHT; leftC = trajectoryConfig.leftPoseC_RIGHT;
        rightA = trajectoryConfig.rightPoseA_RIGHT; rightB = trajectoryConfig.rightPoseB_RIGHT; rightC = trajectoryConfig.rightPoseC_RIGHT;
        backBeamA = trajectoryConfig.beamButtPoseA_RIGHT; backBeamB = trajectoryConfig.beamButtPoseB_RIGHT; backBeamC = trajectoryConfig.beamButtPoseC_RIGHT;
    }
    // clang-format on

    // ########################### 眼在手上写入拍照点, 眼在手外写入零过渡点 ###########################
    if (trajectoryConfig.handEyeType == MyToolFunc::getHandTypeTypeString(HAND_EYE_TYPE::EYE_IN_HAND)) {
        if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::FRONT) {
            writeWeldPoint(outfile, takePhotoX0, takePhotoY0, takePhotoZ0, takePhotoA0, takePhotoB0, takePhotoC0, moveSpeed, ARC_STOP, LINE_WELD,
                           weldingCurrent, weldingVoltage);
        }
        writeWeldPoint(outfile, X0, Y0, Z0, A0, B0, C0, moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
    } else if (trajectoryConfig.handEyeType == MyToolFunc::getHandTypeTypeString(HAND_EYE_TYPE::EYE_TO_HAND)) {
        writeWeldPoint(outfile, X0, Y0, Z0, A0, B0, C0, moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
    } else {
        PLOGE << "机器人手眼关系错误";
    }

    // ###########################             ###########################
    // ########################### 写入左侧焊缝 ###########################
    // ###########################             ###########################
    double withdrawDis = settingPara.FrontBeamLeft_WithDrawDistance;
    std::array<double, 3> leftVector = abcToVector(leftA, leftB, leftC);  // 计算左侧焊缝后撤向量
    for (int j = 0; j < leftVector.size(); j++) leftVector[j] *= withdrawDis;
    PLOGD << "左侧焊缝偏移量: " << leftVector[0] << " " << leftVector[1] << " " << leftVector[2];

    std::array<double, 3> backVector = abcToVector(backBeamA, backBeamB, backBeamC);  // 计算背面横梁焊缝后撤向量
    for (int j = 0; j < backVector.size(); j++) backVector[j] *= 1;
    PLOGD << "背面横梁焊缝偏移量: " << backVector[0] << " " << backVector[1] << " " << backVector[2];

    // 提前判断需要摆焊的焊缝如何摆
    SWING_WELD_ACTION CURR_SWING_METHOD = SWING_WELD_ACTION::LINE_WELD;
    if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::FRONT)
        CURR_SWING_METHOD = SWING_WELD_ACTION::FRONT_LEFT_VERTICAL_SWING_WELD;
    else if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::LEFT)
        CURR_SWING_METHOD = SWING_WELD_ACTION::LEFT_LEFT_VERTICAL_SWING_WELD;
    else if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::RIGHT)
        CURR_SWING_METHOD = SWING_WELD_ACTION::RIGHT_LEFT_VERTICAL_SWING_WELD;

    int cornerIndex = -1, backBeamIndex = -1, frontBeamIndex = -1, horizontalIndex = -1, verticalIndex = -1;
    for (int i = 0; i <= endOfLeftSeams; ++i) {  // 焊缝编号归类预存
        if (weldSeamInfo[i]->detectSuccFlag == true && weldSeamInfo[i]->weldEndPointsInRobot != nullptr &&
            weldSeamInfo[i]->weldEndPointsInRobot->size() == 2) {
            if (weldSeamInfo[i]->weldType == WELD_TYPE::BACK_CORNER_BUTT || weldSeamInfo[i]->weldType == WELD_TYPE::FRONT_CORNER_BUTT) {  // 边角焊缝
                cornerIndex = i;
            } else if (weldSeamInfo[i]->weldType == WELD_TYPE::BACK_BEAM_BUTT) {  // 背面横梁对接
                backBeamIndex = i;
            } else if (weldSeamInfo[i]->weldType == WELD_TYPE::FRONT_BEAM_BUTT) {  // 正面横梁对接
                frontBeamIndex = i;
                // TODO 临时误差补偿
                if (weldSeamInfo[i]->detectSuccFlag && weldSeamInfo[i]->weldEndPointsInRobot && weldSeamInfo[i]->weldEndPointsInRobot->size() == 2) {
                    if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::LEFT) {
                        weldSeamInfo[i]->weldEndPointsInRobot->at(0).x += 6;
                        weldSeamInfo[i]->weldEndPointsInRobot->at(1).x += 6;
                    } else if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::RIGHT) {
                        weldSeamInfo[i]->weldEndPointsInRobot->at(0).x -= 5;
                        weldSeamInfo[i]->weldEndPointsInRobot->at(1).x -= 5;
                    }
                }
            } else if (weldSeamInfo[i]->weldType == WELD_TYPE::FRONT_HORIZONTAL_FILLET) {  // 水平角接
                horizontalIndex = i;
            } else if (weldSeamInfo[i]->weldType == WELD_TYPE::FRONT_VERTICAL_FILLET) {  // 竖直角接
                verticalIndex = i;
            }
        }
    }

    double width = 1.5, weldingSpeed = 5;
    // 写入边角焊缝
    if (cornerIndex != -1) {
        width = weldSeamInfo[cornerIndex]->width;  // clang-format off
        if (width <= 1) weldingSpeed = weldingSpeed0To1;
        else if (width <= 3) weldingSpeed = weldingSpeed1To3;
        else if (width <= 5) weldingSpeed = weldingSpeed3To5;
        else weldingSpeed = weldingSpeedDefault;  // clang-format on

        pcl::PointXYZ p1 = weldSeamInfo[cornerIndex]->weldEndPointsInRobot->at(0);
        pcl::PointXYZ p2 = weldSeamInfo[cornerIndex]->weldEndPointsInRobot->at(1);
        // 起始过渡点的八维坐标
        writeWeldPoint(outfile, p1.x - leftVector[0] / withdrawDis * 7, p1.y - leftVector[1] / withdrawDis * 7,
                       p1.z - leftVector[2] / withdrawDis * 7 + 50, leftA, leftB, leftC, moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent,
                       weldingVoltage);

        // 写入第一个点的八维信息
        writeWeldPoint(outfile, p1.x - leftVector[0] / withdrawDis * 7, p1.y - leftVector[1] / withdrawDis * 7,
                       p1.z - leftVector[2] / withdrawDis * 7, leftA, leftB, leftC, moveSpeed, ARC_START, LINE_WELD, weldingCurrent, weldingVoltage);
        // 写入第二个点的八维信息
        writeWeldPoint(outfile, p2.x - leftVector[0] / withdrawDis * 7, p2.y - leftVector[1] / withdrawDis * 7,
                       p2.z - leftVector[2] / withdrawDis * 7, leftA, leftB, leftC, weldingSpeed, ARC_STOP, LINE_WELD, weldingCurrent,
                       weldingVoltage);
        // 终止过渡点的八维坐标
        writeWeldPoint(outfile, p2.x - leftVector[0] / withdrawDis * 7, p2.y - leftVector[1] / withdrawDis * 7,
                       p2.z - leftVector[2] / withdrawDis * 7 + 50, leftA, leftB, leftC, moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent,
                       weldingVoltage);

        if (backBeamIndex != -1) {  // 如果后面还有横梁焊缝, 就加一个过渡点
            writeWeldPoint(outfile, X0, Y0, Z0, A0, B0, C0, moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
        }
    }

    // 写入背面横梁焊缝
    if (backBeamIndex != -1) {
        width = weldSeamInfo[backBeamIndex]->width;  // clang-format off
        if (width <= 1) weldingSpeed = weldingSpeed0To1;
        else if (width <= 3) weldingSpeed = weldingSpeed1To3;
        else if (width <= 5) weldingSpeed = weldingSpeed3To5;
        else weldingSpeed = weldingSpeedDefault;  // clang-format on

        pcl::PointXYZ p1 = weldSeamInfo[backBeamIndex]->weldEndPointsInRobot->at(0);
        pcl::PointXYZ p2 = weldSeamInfo[backBeamIndex]->weldEndPointsInRobot->at(1);
        // 起始过渡点的八维坐标
        writeWeldPoint(outfile, p1.x - backVector[0] * 3, p1.y - backVector[1] * 3, p1.z - backVector[2] * 3 + 50, backBeamA, backBeamB, backBeamC,
                       moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);

        // 写入第一个点的八维信息
        writeWeldPoint(outfile, p1.x - backVector[0] * 3, p1.y - backVector[1] * 3, p1.z - backVector[2] * 3, backBeamA, backBeamB, backBeamC,
                       moveSpeed, ARC_START, LINE_WELD, weldingCurrent, weldingVoltage);
        // 写入第二个点的八维信息
        writeWeldPoint(outfile, p2.x - backVector[0] * 3, p2.y - backVector[1] * 3, p2.z - backVector[2] * 3, backBeamA, backBeamB, backBeamC,
                       weldingSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);

        // 终止过渡点的八维坐标
        writeWeldPoint(outfile, p2.x - backVector[0] * 3, p2.y - backVector[1] * 3, p2.z - backVector[2] * 3 + 50, backBeamA, backBeamB, backBeamC,
                       moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
    }

    // 写入正面横梁处焊缝 1'2'3'12'13'23'123'-1
    if (frontBeamIndex != -1) {  // 存在正面横梁对接
        pcl::PointXYZ p1 = weldSeamInfo[frontBeamIndex]->weldEndPointsInRobot->at(0);
        pcl::PointXYZ p2 = weldSeamInfo[frontBeamIndex]->weldEndPointsInRobot->at(1);
        // 起始过渡点的八维坐标
        writeWeldPoint(outfile, p1.x + leftVector[0] / 5, p1.y + leftVector[1] / 5, p1.z + leftVector[2] / 5 + 50, leftA, leftB, leftC, moveSpeed,
                       ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
        // 写入第一个点的八维信息
        writeWeldPoint(outfile, p1.x - leftVector[0] / withdrawDis * 5, p1.y - leftVector[1] / withdrawDis * 5,
                       p1.z - leftVector[2] / withdrawDis * 5, leftA, leftB, leftC, moveSpeed, ARC_START, LINE_WELD, weldingCurrent, weldingVoltage);

        if (horizontalIndex != -1) {  // 并且存在水平焊缝, 先把横梁对接走完, 再走水平焊缝
            width = weldSeamInfo[frontBeamIndex]->width;
            // clang-format off
            if (width <= 1) weldingSpeed = weldingSpeed0To1;
            else if (width <= 3) weldingSpeed = weldingSpeed1To3;
            else if (width <= 5) weldingSpeed = weldingSpeed3To5;
            else weldingSpeed = weldingSpeedDefault;  // clang-format on

            // 写入第二个点的八维信息
            writeWeldPoint(outfile, p2.x - leftVector[0] / withdrawDis * 5, p2.y - leftVector[1] / withdrawDis * 5,
                           p2.z - leftVector[2] / withdrawDis * 5, leftA, leftB, leftC, weldingSpeed, ARC_START, LINE_WELD, weldingCurrent,
                           weldingVoltage);

            pcl::PointXYZ p3 = weldSeamInfo[horizontalIndex]->weldEndPointsInRobot->at(0);
            pcl::PointXYZ p4 = weldSeamInfo[horizontalIndex]->weldEndPointsInRobot->at(1);
            // 写入第三个点的八维信息
            writeWeldPoint(outfile, p3.x + leftVector[0] / 5, p3.y + leftVector[1] / 5, p3.z + leftVector[2] / 5, leftA, leftB, leftC, weldingSpeed,
                           ARC_START, LINE_WELD, weldingCurrent, weldingVoltage);

            if (verticalIndex != -1) {  // 并且还存在竖直焊缝, 先把水平焊缝走完, 再走竖直焊缝 这里由于摆焊，需要熄弧到点再起弧
                if (trajectoryConfig.robotType != MyToolFunc::getRobotTypeString(ROBOT_TYPE::BAO_YUAN) && settingPara.weldingVerticalWeld) {
                    // 写入第四个点的八维信息
                    writeWeldPoint(outfile, p4.x + leftVector[0] / 2, p4.y + leftVector[1] / 2, p4.z + leftVector[2] / 2, leftA, leftB, leftC,
                                   weldingSpeedHorizontal, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);

                    pcl::PointXYZ p5 = weldSeamInfo[verticalIndex]->weldEndPointsInRobot->at(0);
                    pcl::PointXYZ p6 = weldSeamInfo[verticalIndex]->weldEndPointsInRobot->at(1);

                    // 写入第五个点的八维信息
                    writeWeldPoint(outfile, p5.x + leftVector[0], p5.y + leftVector[1], p5.z + leftVector[2], leftA, leftB, leftC, moveSpeed,
                                   ARC_START, LINE_WELD, weldingCurrent_Vertical, weldingVoltage_Vertical);
                    // 写入第六个点的八维信息
                    writeWeldPoint(outfile, p6.x + leftVector[0], p6.y + leftVector[1], p6.z + leftVector[2], leftA, leftB, leftC,
                                   weldingSpeedVertical, ARC_STOP, CURR_SWING_METHOD, weldingCurrent, weldingVoltage);
                    // 终止过渡点的八维坐标
                    writeWeldPoint(outfile, p6.x + leftVector[0], p6.y + leftVector[1], p6.z + leftVector[2] + 50, leftA, leftB, leftC, moveSpeed,
                                   ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
                } else {
                    // 写入第四个点的八维信息
                    writeWeldPoint(outfile, p4.x + leftVector[0] / 2, p4.y + leftVector[1] / 2, p4.z + leftVector[2] / 2, leftA, leftB, leftC,
                                   weldingSpeedHorizontal, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
                    // 终止过渡点的八维信息
                    writeWeldPoint(outfile, p4.x + leftVector[0], p4.y + leftVector[1], p4.z + leftVector[2] + 50, leftA, leftB, leftC, moveSpeed,
                                   ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
                }
            } else {  // 没有竖直焊缝, 则只把水平焊缝走完
                // 写入第四个点的八维信息
                writeWeldPoint(outfile, p4.x + leftVector[0] / 2, p4.y + leftVector[1] / 2, p4.z + leftVector[2] / 2, leftA, leftB, leftC,
                               weldingSpeedHorizontal, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
                // 终止过渡点的八维信息
                writeWeldPoint(outfile, p4.x + leftVector[0], p4.y + leftVector[1], p4.z + leftVector[2] + 50, leftA, leftB, leftC, moveSpeed,
                               ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
            }
        } else {  // 不存在水平焊缝, 就继续把横梁对接走完, 再判断有没有竖直焊缝
            width = weldSeamInfo[frontBeamIndex]->width;
            // clang-format off
            if (width <= 1) weldingSpeed = weldingSpeed0To1;
            else if (width <= 3) weldingSpeed = weldingSpeed1To3;
            else if (width <= 5) weldingSpeed = weldingSpeed3To5;
            else weldingSpeed = weldingSpeedDefault;  // clang-format on

            // 写入第二个点的八维信息
            writeWeldPoint(outfile, p2.x - leftVector[0] / withdrawDis * 5, p2.y - leftVector[1] / withdrawDis * 5,
                           p2.z - leftVector[2] / withdrawDis * 5, leftA, leftB, leftC, weldingSpeed, ARC_STOP, LINE_WELD, weldingCurrent,
                           weldingVoltage);
            // 终止过渡点的八维坐标
            writeWeldPoint(outfile, p2.x + leftVector[0] / 5, p2.y + leftVector[1] / 5, p2.z + leftVector[2] / 5 + 50, leftA, leftB, leftC, moveSpeed,
                           ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);

            if (verticalIndex != -1) {
                if (trajectoryConfig.robotType != MyToolFunc::getRobotTypeString(ROBOT_TYPE::BAO_YUAN) && settingPara.weldingVerticalWeld) {
                    pcl::PointXYZ p5 = weldSeamInfo[verticalIndex]->weldEndPointsInRobot->at(0);
                    pcl::PointXYZ p6 = weldSeamInfo[verticalIndex]->weldEndPointsInRobot->at(1);

                    // 起始过渡点的八维坐标
                    writeWeldPoint(outfile, p5.x + leftVector[0], p5.y + leftVector[1], p5.z + leftVector[2] + 50, leftA, leftB, leftC, moveSpeed,
                                   ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
                    // 写入第五个点的八维信息
                    writeWeldPoint(outfile, p5.x + leftVector[0], p5.y + leftVector[1], p5.z + leftVector[2], leftA, leftB, leftC, moveSpeed,
                                   ARC_START, LINE_WELD, weldingCurrent_Vertical, weldingVoltage_Vertical);
                    // 写入第六个点的八维信息
                    writeWeldPoint(outfile, p6.x + leftVector[0], p6.y + leftVector[1], p6.z + leftVector[2], leftA, leftB, leftC,
                                   weldingSpeedVertical, ARC_STOP, CURR_SWING_METHOD, weldingCurrent, weldingVoltage);
                    // 终止过渡点的八维坐标
                    writeWeldPoint(outfile, p6.x + leftVector[0], p6.y + leftVector[1], p6.z + leftVector[2] + 50, leftA, leftB, leftC, moveSpeed,
                                   ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
                }
            }
        }
    } else if (horizontalIndex != -1) {  // 不存在横梁对接, 但存在水平焊缝
        pcl::PointXYZ p3 = weldSeamInfo[horizontalIndex]->weldEndPointsInRobot->at(0);
        pcl::PointXYZ p4 = weldSeamInfo[horizontalIndex]->weldEndPointsInRobot->at(1);
        // 起始过渡点的八维坐标
        writeWeldPoint(outfile, p3.x + leftVector[0] / 5, p3.y + leftVector[1] / 5, p3.z + leftVector[2] / 5 + 50, leftA, leftB, leftC, moveSpeed,
                       ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
        // 写入第三个点的八维信息
        writeWeldPoint(outfile, p3.x + leftVector[0] / 5, p3.y + leftVector[1] / 5, p3.z + leftVector[2] / 5, leftA, leftB, leftC, moveSpeed,
                       ARC_START, LINE_WELD, weldingCurrent, weldingVoltage);

        if (verticalIndex != -1) {  // 并且还存在竖直焊缝, 先把水平焊缝走完, 再走竖直焊缝
            if (trajectoryConfig.robotType != MyToolFunc::getRobotTypeString(ROBOT_TYPE::BAO_YUAN) && settingPara.weldingVerticalWeld) {
                // 写入第四个点的八维信息
                writeWeldPoint(outfile, p4.x + leftVector[0] / 2, p4.y + leftVector[1] / 2, p4.z + leftVector[2] / 2, leftA, leftB, leftC,
                               weldingSpeedHorizontal, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
                pcl::PointXYZ p5 = weldSeamInfo[verticalIndex]->weldEndPointsInRobot->at(0);
                pcl::PointXYZ p6 = weldSeamInfo[verticalIndex]->weldEndPointsInRobot->at(1);

                // 写入第五个点的八维信息

                writeWeldPoint(outfile, p5.x + leftVector[0], p5.y + leftVector[1], p5.z + leftVector[2] - 4, leftA, leftB, leftC, moveSpeed,
                               ARC_START, LINE_WELD, weldingCurrent_Vertical, weldingVoltage_Vertical);

                // 写入第六个点的八维信息
                writeWeldPoint(outfile, p6.x + leftVector[0], p6.y + leftVector[1], p6.z + leftVector[2] - 2, leftA, leftB, leftC,
                               weldingSpeedVertical, ARC_STOP, CURR_SWING_METHOD, weldingCurrent, weldingVoltage);

                // 终止过渡点的八维坐标
                writeWeldPoint(outfile, p6.x + leftVector[0], p6.y + leftVector[1], p6.z + leftVector[2] + 50, leftA, leftB, leftC, moveSpeed,
                               ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
            } else {
                // 写入第四个点的八维信息
                writeWeldPoint(outfile, p4.x + leftVector[0] / 2, p4.y + leftVector[1] / 2, p4.z + leftVector[2] / 2, leftA, leftB, leftC,
                               weldingSpeedHorizontal, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
                // 终止过渡点的八维信息
                writeWeldPoint(outfile, p4.x + leftVector[0], p4.y + leftVector[1], p4.z + leftVector[2] + 50, leftA, leftB, leftC, moveSpeed,
                               ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
            }
        } else {  // 没有竖直焊缝, 则只把水平焊缝走完
            // 写入第四个点的八维信息
            writeWeldPoint(outfile, p4.x + leftVector[0] / 2, p4.y + leftVector[1] / 2, p4.z + leftVector[2] / 2, leftA, leftB, leftC,
                           weldingSpeedHorizontal, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
            // 终止过渡点的八维信息
            writeWeldPoint(outfile, p4.x + leftVector[0], p4.y + leftVector[1], p4.z + leftVector[2] + 50, leftA, leftB, leftC, moveSpeed, ARC_STOP,
                           LINE_WELD, weldingCurrent, weldingVoltage);
        }
    } else if (verticalIndex != -1) {  // 不存在横梁对接和水平, 但存在竖直焊缝
        if (trajectoryConfig.robotType != MyToolFunc::getRobotTypeString(ROBOT_TYPE::BAO_YUAN) && settingPara.weldingVerticalWeld) {
            pcl::PointXYZ p5 = weldSeamInfo[verticalIndex]->weldEndPointsInRobot->at(0);
            pcl::PointXYZ p6 = weldSeamInfo[verticalIndex]->weldEndPointsInRobot->at(1);

            // 起始过渡点的八维坐标
            writeWeldPoint(outfile, p5.x + leftVector[0], p5.y + leftVector[1], p5.z + leftVector[2] + 50, leftA, leftB, leftC, moveSpeed, ARC_STOP,
                           LINE_WELD, weldingCurrent, weldingVoltage);
            // 写入第五个点的八维信息
            writeWeldPoint(outfile, p5.x + leftVector[0], p5.y + leftVector[1], p5.z + leftVector[2], leftA, leftB, leftC, moveSpeed, ARC_START,
                           LINE_WELD, weldingCurrent_Vertical, weldingVoltage_Vertical);
            // 写入第六个点的八维信息
            writeWeldPoint(outfile, p6.x + leftVector[0], p6.y + leftVector[1], p6.z + leftVector[2], leftA, leftB, leftC, weldingSpeedVertical,
                           ARC_STOP, CURR_SWING_METHOD, weldingCurrent, weldingVoltage);
            // 终止过渡点的八维坐标
            writeWeldPoint(outfile, p6.x + leftVector[0], p6.y + leftVector[1], p6.z + leftVector[2] + 50, leftA, leftB, leftC, moveSpeed, ARC_STOP,
                           LINE_WELD, weldingCurrent, weldingVoltage);
        }
    }

    // ###########################               ###########################
    // ########################### 写入中间过渡点 ###########################
    // ###########################               ###########################
    if (weldSeamInfo.size() - 1 > endOfLeftSeams) {
        writeWeldPoint(outfile, X0, Y0, Z0, A0, B0, C0, moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
    }

    // ###########################             ###########################
    // ########################### 写入右侧焊缝 ###########################
    // ###########################             ###########################
    withdrawDis = settingPara.FrontBeamRight_WithDrawDistance;
    std::array<double, 3> rightVector = abcToVector(rightA, rightB, rightC);  // 计算右侧焊缝后撤向量
    for (int j = 0; j < rightVector.size(); j++) rightVector[j] *= withdrawDis;
    PLOGD << "右侧焊缝偏移量: " << rightVector[0] << " " << rightVector[1] << " " << rightVector[2];

    // 提前判断需要摆焊的焊缝如何摆
    CURR_SWING_METHOD = SWING_WELD_ACTION::LINE_WELD;
    if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::FRONT)
        CURR_SWING_METHOD = SWING_WELD_ACTION::FRONT_RIGHT_VERTICAL_SWING_WELD;
    else if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::LEFT)
        CURR_SWING_METHOD = SWING_WELD_ACTION::LEFT_RIGHT_VERTICAL_SWING_WELD;
    else if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::RIGHT)
        CURR_SWING_METHOD = SWING_WELD_ACTION::RIGHT_RIGHT_VERTICAL_SWING_WELD;

    // clang-format off
    cornerIndex = -1; backBeamIndex = -1; frontBeamIndex = -1; horizontalIndex = -1; verticalIndex = -1;
    // clang-format on
    for (int i = endOfLeftSeams + 1; i < weldSeamInfo.size(); ++i) {  // 焊缝编号归类预存
        if (weldSeamInfo[i]->detectSuccFlag == true && weldSeamInfo[i]->weldEndPointsInRobot != nullptr &&
            weldSeamInfo[i]->weldEndPointsInRobot->size() == 2) {
            if (weldSeamInfo[i]->weldType == WELD_TYPE::BACK_CORNER_BUTT || weldSeamInfo[i]->weldType == WELD_TYPE::FRONT_CORNER_BUTT) {  // 边角焊缝
                cornerIndex = i;
            } else if (weldSeamInfo[i]->weldType == WELD_TYPE::BACK_BEAM_BUTT) {  // 背面横梁对接
                backBeamIndex = i;
            } else if (weldSeamInfo[i]->weldType == WELD_TYPE::FRONT_BEAM_BUTT) {  // 正面横梁对接
                frontBeamIndex = i;
                // TODO 临时误差补偿
                if (weldSeamInfo[i]->detectSuccFlag && weldSeamInfo[i]->weldEndPointsInRobot && weldSeamInfo[i]->weldEndPointsInRobot->size() == 2) {
                    if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::LEFT) {
                        weldSeamInfo[i]->weldEndPointsInRobot->at(0).x -= 5;
                        weldSeamInfo[i]->weldEndPointsInRobot->at(1).x -= 5;
                    } else if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::RIGHT) {
                        weldSeamInfo[i]->weldEndPointsInRobot->at(0).x += 5;
                        weldSeamInfo[i]->weldEndPointsInRobot->at(1).x += 5;
                    }
                }
            } else if (weldSeamInfo[i]->weldType == WELD_TYPE::FRONT_HORIZONTAL_FILLET) {  // 水平角接
                horizontalIndex = i;
            } else if (weldSeamInfo[i]->weldType == WELD_TYPE::FRONT_VERTICAL_FILLET) {  // 竖直角接
                verticalIndex = i;
            }
        }
    }

    // 写入边角焊缝
    if (cornerIndex != -1) {
        width = weldSeamInfo[cornerIndex]->width;  // clang-format off
        if (width <= 1) weldingSpeed = weldingSpeed0To1;
        else if (width <= 3) weldingSpeed = weldingSpeed1To3;
        else if (width <= 5) weldingSpeed = weldingSpeed3To5;
        else weldingSpeed = weldingSpeedDefault;  // clang-format on

        pcl::PointXYZ p1 = weldSeamInfo[cornerIndex]->weldEndPointsInRobot->at(0);
        pcl::PointXYZ p2 = weldSeamInfo[cornerIndex]->weldEndPointsInRobot->at(1);
        // 起始过渡点的八维坐标
        writeWeldPoint(outfile, p1.x - rightVector[0] / withdrawDis * 7, p1.y - rightVector[1] / withdrawDis * 7,
                       p1.z - rightVector[2] / withdrawDis * 7 + 50, rightA, rightB, rightC, moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent,
                       weldingVoltage);
        // 写入第一个点的八维信息
        writeWeldPoint(outfile, p1.x - rightVector[0] / withdrawDis * 7, p1.y - rightVector[1] / withdrawDis * 7,
                       p1.z - rightVector[2] / withdrawDis * 7, rightA, rightB, rightC, moveSpeed, ARC_START, LINE_WELD, weldingCurrent,
                       weldingVoltage);
        // 写入第二个点的八维信息
        writeWeldPoint(outfile, p2.x - rightVector[0] / withdrawDis * 7, p2.y - rightVector[1] / withdrawDis * 7,
                       p2.z - rightVector[2] / withdrawDis * 7, rightA, rightB, rightC, weldingSpeed, ARC_STOP, LINE_WELD, weldingCurrent,
                       weldingVoltage);
        // 终止过渡点的八维坐标
        writeWeldPoint(outfile, p2.x - rightVector[0] / withdrawDis * 7, p2.y - rightVector[1] / withdrawDis * 7,
                       p2.z - rightVector[2] / withdrawDis * 7 + 50, rightA, rightB, rightC, moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent,
                       weldingVoltage);

        if (backBeamIndex != -1) {  // 如果后面还有横梁焊缝, 就加一个过渡点
            writeWeldPoint(outfile, X0, Y0, Z0, A0, B0, C0, moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
        }
    }

    // 写入背面横梁焊缝
    if (backBeamIndex != -1) {
        width = weldSeamInfo[backBeamIndex]->width;  // clang-format off
        if (width <= 1) weldingSpeed = weldingSpeed0To1;
        else if (width <= 3) weldingSpeed = weldingSpeed1To3;
        else if (width <= 5) weldingSpeed = weldingSpeed3To5;
        else weldingSpeed = weldingSpeedDefault;  // clang-format on

        pcl::PointXYZ p1 = weldSeamInfo[backBeamIndex]->weldEndPointsInRobot->at(0);
        pcl::PointXYZ p2 = weldSeamInfo[backBeamIndex]->weldEndPointsInRobot->at(1);
        // 起始过渡点的八维坐标
        writeWeldPoint(outfile, p1.x - backVector[0] * 3, p1.y - backVector[1] * 3, p1.z - backVector[2] * 3 + 50, backBeamA, backBeamB, backBeamC,
                       moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
        // 写入第一个点的八维信息
        writeWeldPoint(outfile, p1.x - backVector[0] * 3, p1.y - backVector[1] * 3, p1.z - backVector[2] * 3, backBeamA, backBeamB, backBeamC,
                       moveSpeed, ARC_START, LINE_WELD, weldingCurrent, weldingVoltage);
        // 写入第二个点的八维信息
        writeWeldPoint(outfile, p2.x - backVector[0] * 3, p2.y - backVector[1] * 3, p2.z - backVector[2] * 3, backBeamA, backBeamB, backBeamC,
                       weldingSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
        // 终止过渡点的八维坐标
        writeWeldPoint(outfile, p2.x - backVector[0] * 3, p2.y - backVector[1] * 3, p2.z - backVector[2] * 3 + 50, backBeamA, backBeamB, backBeamC,
                       moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
    }

    // 写入正面横梁处焊缝
    if (frontBeamIndex != -1) {  // 存在正面横梁对接
        pcl::PointXYZ p1 = weldSeamInfo[frontBeamIndex]->weldEndPointsInRobot->at(0);
        pcl::PointXYZ p2 = weldSeamInfo[frontBeamIndex]->weldEndPointsInRobot->at(1);
        // 起始过渡点的八维坐标
        writeWeldPoint(outfile, p1.x + rightVector[0] / 5, p1.y + rightVector[1] / 5, p1.z + rightVector[2] / 5 + 50, rightA, rightB, rightC,
                       moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
        // 写入第一个点的八维信息
        writeWeldPoint(outfile, p1.x - rightVector[0] / withdrawDis * 5, p1.y - rightVector[1] / withdrawDis * 5,
                       p1.z - rightVector[2] / withdrawDis * 5, rightA, rightB, rightC, moveSpeed, ARC_START, LINE_WELD, weldingCurrent,
                       weldingVoltage);

        if (horizontalIndex != -1) {  // 并且存在水平焊缝, 先把横梁对接走完, 再走水平焊缝
            width = weldSeamInfo[frontBeamIndex]->width;
            // clang-format off
            if (width <= 1) weldingSpeed = weldingSpeed0To1;
            else if (width <= 3) weldingSpeed = weldingSpeed1To3;
            else if (width <= 5) weldingSpeed = weldingSpeed3To5;
            else weldingSpeed = weldingSpeedDefault;  // clang-format on

            // 写入第二个点的八维信息
            writeWeldPoint(outfile, p2.x - rightVector[0] / withdrawDis * 5, p2.y - rightVector[1] / withdrawDis * 5,
                           p2.z - rightVector[2] / withdrawDis * 5, rightA, rightB, rightC, weldingSpeed, ARC_START, LINE_WELD, weldingCurrent,
                           weldingVoltage);

            pcl::PointXYZ p3 = weldSeamInfo[horizontalIndex]->weldEndPointsInRobot->at(0);
            pcl::PointXYZ p4 = weldSeamInfo[horizontalIndex]->weldEndPointsInRobot->at(1);
            // 写入第三个点的八维信息
            writeWeldPoint(outfile, p3.x + rightVector[0] / 5, p3.y + rightVector[1] / 5, p3.z + rightVector[2] / 5, rightA, rightB, rightC,
                           weldingSpeed, ARC_START, LINE_WELD, weldingCurrent, weldingVoltage);

            if (verticalIndex != -1) {  // 并且还存在竖直焊缝, 先把水平焊缝走完, 再走竖直焊缝
                if (trajectoryConfig.robotType != MyToolFunc::getRobotTypeString(ROBOT_TYPE::BAO_YUAN) && settingPara.weldingVerticalWeld) {
                    // 写入第四个点的八维信息
                    writeWeldPoint(outfile, p4.x + rightVector[0] / 2, p4.y + rightVector[1] / 2, p4.z + rightVector[2] / 2, rightA, rightB, rightC,
                                   weldingSpeedHorizontal, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
                    pcl::PointXYZ p5 = weldSeamInfo[verticalIndex]->weldEndPointsInRobot->at(0);
                    pcl::PointXYZ p6 = weldSeamInfo[verticalIndex]->weldEndPointsInRobot->at(1);

                    // 写入第五个点的八维信息
                    writeWeldPoint(outfile, p5.x + rightVector[0], p5.y + rightVector[1], p5.z + rightVector[2], rightA, rightB, rightC, moveSpeed,
                                   ARC_START, LINE_WELD, weldingCurrent_Vertical, weldingVoltage_Vertical);
                    // 写入第六个点的八维信息
                    writeWeldPoint(outfile, p6.x + rightVector[0], p6.y + rightVector[1], p6.z + rightVector[2], rightA, rightB, rightC,
                                   weldingSpeedVertical, ARC_STOP, CURR_SWING_METHOD, weldingCurrent, weldingVoltage);
                    // 终止过渡点的八维坐标
                    writeWeldPoint(outfile, p6.x + rightVector[0], p6.y + rightVector[1], p6.z + rightVector[2] + 50, rightA, rightB, rightC,
                                   moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
                } else {
                    // 写入第四个点的八维信息
                    writeWeldPoint(outfile, p4.x + rightVector[0] / 2, p4.y + rightVector[1] / 2, p4.z + rightVector[2] / 2, rightA, rightB, rightC,
                                   weldingSpeedHorizontal, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
                    // 终止过渡点的八维信息
                    writeWeldPoint(outfile, p4.x + rightVector[0], p4.y + rightVector[1], p4.z + rightVector[2] + 50, rightA, rightB, rightC,
                                   moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
                }
            } else {  // 没有竖直焊缝, 则只把水平焊缝走完
                // 写入第四个点的八维信息
                writeWeldPoint(outfile, p4.x + rightVector[0] / 2, p4.y + rightVector[1] / 2, p4.z + rightVector[2] / 2, rightA, rightB, rightC,
                               weldingSpeedHorizontal, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
                // 终止过渡点的八维信息
                writeWeldPoint(outfile, p4.x + rightVector[0], p4.y + rightVector[1], p4.z + rightVector[2] + 50, rightA, rightB, rightC, moveSpeed,
                               ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
            }
        } else {  // 不存在水平焊缝, 就继续把横梁对接走完, 再判断有没有竖直焊缝
            width = weldSeamInfo[frontBeamIndex]->width;
            // clang-format off
            if (width <= 1) weldingSpeed = weldingSpeed0To1;
            else if (width <= 3) weldingSpeed = weldingSpeed1To3;
            else if (width <= 5) weldingSpeed = weldingSpeed3To5;
            else weldingSpeed = weldingSpeedDefault;  // clang-format on

            // 写入第二个点的八维信息
            writeWeldPoint(outfile, p2.x - rightVector[0] / withdrawDis * 5, p2.y - rightVector[1] / withdrawDis * 5,
                           p2.z - rightVector[2] / withdrawDis * 5, rightA, rightB, rightC, weldingSpeed, ARC_STOP, LINE_WELD, weldingCurrent,
                           weldingVoltage);
            // 终止过渡点的八维坐标
            writeWeldPoint(outfile, p2.x + rightVector[0] / 5, p2.y + rightVector[1] / 5, p2.z + rightVector[2] / 5 + 50, rightA, rightB, rightC,
                           moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);

            if (verticalIndex != -1) {
                if (trajectoryConfig.robotType != MyToolFunc::getRobotTypeString(ROBOT_TYPE::BAO_YUAN) && settingPara.weldingVerticalWeld) {
                    pcl::PointXYZ p5 = weldSeamInfo[verticalIndex]->weldEndPointsInRobot->at(0);
                    pcl::PointXYZ p6 = weldSeamInfo[verticalIndex]->weldEndPointsInRobot->at(1);

                    // 起始过渡点的八维坐标
                    writeWeldPoint(outfile, p5.x + rightVector[0], p5.y + rightVector[1], p5.z + rightVector[2] + 50, rightA, rightB, rightC,
                                   moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
                    // 写入第五个点的八维信息
                    writeWeldPoint(outfile, p5.x + rightVector[0], p5.y + rightVector[1], p5.z + rightVector[2], rightA, rightB, rightC, moveSpeed,
                                   ARC_START, LINE_WELD, weldingCurrent_Vertical, weldingVoltage_Vertical);
                    // 写入第六个点的八维信息
                    writeWeldPoint(outfile, p6.x + rightVector[0], p6.y + rightVector[1], p6.z + rightVector[2], rightA, rightB, rightC,
                                   weldingSpeedVertical, ARC_STOP, CURR_SWING_METHOD, weldingCurrent, weldingVoltage);
                    // 终止过渡点的八维坐标
                    writeWeldPoint(outfile, p6.x + rightVector[0], p6.y + rightVector[1], p6.z + rightVector[2] + 50, rightA, rightB, rightC,
                                   moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
                }
            }
        }
    } else if (horizontalIndex != -1) {  // 不存在横梁对接, 但存在水平焊缝
        pcl::PointXYZ p3 = weldSeamInfo[horizontalIndex]->weldEndPointsInRobot->at(0);
        pcl::PointXYZ p4 = weldSeamInfo[horizontalIndex]->weldEndPointsInRobot->at(1);
        // 起始过渡点的八维坐标
        writeWeldPoint(outfile, p3.x + rightVector[0] / 5, p3.y + rightVector[1] / 5, p3.z + rightVector[2] / 5 + 50, rightA, rightB, rightC,
                       moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
        // 写入第三个点的八维信息
        writeWeldPoint(outfile, p3.x + rightVector[0] / 5, p3.y + rightVector[1] / 5, p3.z + rightVector[2] / 5, rightA, rightB, rightC, moveSpeed,
                       ARC_START, LINE_WELD, weldingCurrent, weldingVoltage);

        if (verticalIndex != -1) {  // 并且还存在竖直焊缝, 先把水平焊缝走完, 再走竖直焊缝
            if (trajectoryConfig.robotType != MyToolFunc::getRobotTypeString(ROBOT_TYPE::BAO_YUAN) && settingPara.weldingVerticalWeld) {
                // 写入第四个点的八维信息
                writeWeldPoint(outfile, p4.x + rightVector[0] / 2, p4.y + rightVector[1] / 2, p4.z + rightVector[2] / 2, rightA, rightB, rightC,
                               weldingSpeedHorizontal, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);

                pcl::PointXYZ p5 = weldSeamInfo[verticalIndex]->weldEndPointsInRobot->at(0);
                pcl::PointXYZ p6 = weldSeamInfo[verticalIndex]->weldEndPointsInRobot->at(1);

                // 写入第五个点的八维信息
                writeWeldPoint(outfile, p5.x + rightVector[0], p5.y + rightVector[1], p5.z + rightVector[2] - 4, rightA, rightB, rightC, moveSpeed,
                               ARC_START, LINE_WELD, weldingCurrent_Vertical, weldingVoltage_Vertical);
                // 写入第六个点的八维信息
                writeWeldPoint(outfile, p6.x + rightVector[0], p6.y + rightVector[1], p6.z + rightVector[2] - 2, rightA, rightB, rightC,
                               weldingSpeedVertical, ARC_STOP, CURR_SWING_METHOD, weldingCurrent, weldingVoltage);
                // 终止过渡点的八维坐标
                writeWeldPoint(outfile, p6.x + rightVector[0], p6.y + rightVector[1], p6.z + rightVector[2] + 50, rightA, rightB, rightC, moveSpeed,
                               ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
            } else {
                // 写入第四个点的八维信息
                writeWeldPoint(outfile, p4.x + rightVector[0] / 2, p4.y + rightVector[1] / 2, p4.z + rightVector[2] / 2, rightA, rightB, rightC,
                               weldingSpeedHorizontal, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
                // 终止过渡点的八维信息
                writeWeldPoint(outfile, p4.x + rightVector[0], p4.y + rightVector[1], p4.z + rightVector[2] + 50, rightA, rightB, rightC, moveSpeed,
                               ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
            }
        } else {  // 没有竖直焊缝, 则只把水平焊缝走完
            // 写入第四个点的八维信息
            writeWeldPoint(outfile, p4.x + rightVector[0] / 2, p4.y + rightVector[1] / 2, p4.z + rightVector[2] / 2, rightA, rightB, rightC,
                           weldingSpeedHorizontal, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
            // 终止过渡点的八维信息
            writeWeldPoint(outfile, p4.x + rightVector[0], p4.y + rightVector[1], p4.z + rightVector[2] + 50, rightA, rightB, rightC, moveSpeed,
                           ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
        }
    } else if (verticalIndex != -1) {  // 不存在横梁对接和水平, 但存在竖直焊缝
        if (trajectoryConfig.robotType != MyToolFunc::getRobotTypeString(ROBOT_TYPE::BAO_YUAN) && settingPara.weldingVerticalWeld) {
            pcl::PointXYZ p5 = weldSeamInfo[verticalIndex]->weldEndPointsInRobot->at(0);
            pcl::PointXYZ p6 = weldSeamInfo[verticalIndex]->weldEndPointsInRobot->at(1);

            // 起始过渡点的八维坐标
            writeWeldPoint(outfile, p5.x + rightVector[0], p5.y + rightVector[1], p5.z + rightVector[2] + 50, rightA, rightB, rightC, moveSpeed,
                           ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
            // 写入第五个点的八维信息
            writeWeldPoint(outfile, p5.x + rightVector[0], p5.y + rightVector[1], p5.z + rightVector[2], rightA, rightB, rightC, moveSpeed, ARC_START,
                           LINE_WELD, weldingCurrent_Vertical, weldingVoltage_Vertical);
            // 写入第六个点的八维信息
            writeWeldPoint(outfile, p6.x + rightVector[0], p6.y + rightVector[1], p6.z + rightVector[2], rightA, rightB, rightC, weldingSpeedVertical,
                           ARC_STOP, CURR_SWING_METHOD, weldingCurrent, weldingVoltage);
            // 终止过渡点的八维坐标
            writeWeldPoint(outfile, p6.x + rightVector[0], p6.y + rightVector[1], p6.z + rightVector[2] + 50, rightA, rightB, rightC, moveSpeed,
                           ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
        }
    }

    // ########################### 眼在手上写入拍照点, 眼在手外写入零过渡点 ###########################
    if (trajectoryConfig.handEyeType == MyToolFunc::getHandTypeTypeString(HAND_EYE_TYPE::EYE_IN_HAND)) {
        outfile << X0 << " " << Y0 << " " << Z0 << " ";
        outfile << A0 << " " << B0 << " " << C0 << " " << moveSpeed << " " << ARC_STOP << " " << LINE_WELD << " " << weldingCurrent << " "
                << weldingVoltage << std::endl;
        outfile << takePhotoX0 << " " << takePhotoY0 << " " << takePhotoZ0 << " ";
        outfile << takePhotoA0 << " " << takePhotoB0 << " " << takePhotoC0 << " " << moveSpeed << " " << ARC_STOP << " " << LINE_WELD << " "
                << weldingCurrent << " " << weldingVoltage << std::endl;
        outfile << takePhotoX0 << " " << takePhotoY0 << " " << takePhotoZ0 << " ";
        outfile << takePhotoA0 << " " << takePhotoB0 << " " << takePhotoC0 << " " << moveSpeed << " " << ARC_STOP << " " << LINE_WELD << " "
                << weldingCurrent << " " << weldingVoltage << std::endl;
    } else if (trajectoryConfig.handEyeType == MyToolFunc::getHandTypeTypeString(HAND_EYE_TYPE::EYE_TO_HAND)) {
        outfile << X0 << " " << Y0 << " " << Z0 << " ";
        outfile << A0 << " " << B0 << " " << C0 << " " << moveSpeed << " " << ARC_STOP << " " << LINE_WELD << " " << weldingCurrent << " "
                << weldingVoltage << std::endl;
    } else {
        PLOGE << "机器人手眼关系错误";
    }

    if (outfile.fail()) {
        PLOGE << "轨迹规划焊缝写入文件失败";
    } else {
        PLOGD << "轨迹规划焊缝写入文件成功";
    }

    outfile.close();
    emit sendTrajectoryPlanOver();  // 发送轨迹规划完成
}

// 延长焊缝
void SteelAngleTrajectoryPlanning::extendSeams(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo, int endOfLeftSeams) {
    for (int i = 0; i < weldSeamInfo.size(); ++i) {
        if (weldSeamInfo[i]->detectSuccFlag == true && weldSeamInfo[i]->weldEndPointsInRobot != nullptr &&
            weldSeamInfo[i]->weldEndPointsInRobot->size() == 2) {
            // 计算焊缝单位方向向量
            double inside2outsideX = 0, inside2outsideY = 0, inside2outsideZ = 0;
            inside2outsideX = weldSeamInfo[i]->weldEndPointsInRobot->at(1).x - weldSeamInfo[i]->weldEndPointsInRobot->at(0).x;
            inside2outsideY = weldSeamInfo[i]->weldEndPointsInRobot->at(1).y - weldSeamInfo[i]->weldEndPointsInRobot->at(0).y;
            inside2outsideZ = weldSeamInfo[i]->weldEndPointsInRobot->at(1).z - weldSeamInfo[i]->weldEndPointsInRobot->at(0).z;
            double length = sqrt(pow(inside2outsideX, 2) + pow(inside2outsideY, 2) + pow(inside2outsideZ, 2));
            inside2outsideX /= length;
            inside2outsideY /= length;
            inside2outsideZ /= length;

            if (weldSeamInfo[i]->weldType == WELD_TYPE::BACK_CORNER_BUTT) {  // 背面边角对接焊缝
                if (i <= endOfLeftSeams) {
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).x -= inside2outsideX * settingPara.BackLeft_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).y -= inside2outsideY * settingPara.BackLeft_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).z -= inside2outsideZ * settingPara.BackLeft_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).x += inside2outsideX * settingPara.BackLeft_ExtendEnd;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).y += inside2outsideY * settingPara.BackLeft_ExtendEnd;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).z += inside2outsideZ * settingPara.BackLeft_ExtendEnd;
                } else {
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).x -= inside2outsideX * settingPara.BackRight_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).y -= inside2outsideY * settingPara.BackRight_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).z -= inside2outsideZ * settingPara.BackRight_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).x += inside2outsideX * settingPara.BackRight_ExtendEnd;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).y += inside2outsideY * settingPara.BackRight_ExtendEnd;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).z += inside2outsideZ * settingPara.BackRight_ExtendEnd;
                }
            } else if (weldSeamInfo[i]->weldType == WELD_TYPE::BACK_BEAM_BUTT) {  // 背面横梁对接焊缝
                if (i <= endOfLeftSeams) {
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).x -= inside2outsideX * settingPara.BackBeamLeft_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).y -= inside2outsideY * settingPara.BackBeamLeft_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).z -= inside2outsideZ * settingPara.BackBeamLeft_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).x += inside2outsideX * settingPara.BackBeamLeft_ExtendEnd;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).y += inside2outsideY * settingPara.BackBeamLeft_ExtendEnd;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).z += inside2outsideZ * settingPara.BackBeamLeft_ExtendEnd;
                } else {
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).x -= inside2outsideX * settingPara.BackBeamRight_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).y -= inside2outsideY * settingPara.BackBeamRight_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).z -= inside2outsideZ * settingPara.BackBeamRight_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).x += inside2outsideX * settingPara.BackBeamRight_ExtendEnd;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).y += inside2outsideY * settingPara.BackBeamRight_ExtendEnd;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).z += inside2outsideZ * settingPara.BackBeamRight_ExtendEnd;
                }
            } else if (weldSeamInfo[i]->weldType == WELD_TYPE::FRONT_CORNER_BUTT) {  // 正面边角对接焊缝
                if (i <= endOfLeftSeams) {
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).x -= inside2outsideX * settingPara.FrontLeft_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).y -= inside2outsideY * settingPara.FrontLeft_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).z -= inside2outsideZ * settingPara.FrontLeft_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).x += inside2outsideX * settingPara.FrontLeft_ExtendEnd;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).y += inside2outsideY * settingPara.FrontLeft_ExtendEnd;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).z += inside2outsideZ * settingPara.FrontLeft_ExtendEnd;
                } else {
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).x -= inside2outsideX * settingPara.FrontRight_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).y -= inside2outsideY * settingPara.FrontRight_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).z -= inside2outsideZ * settingPara.FrontRight_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).x += inside2outsideX * settingPara.FrontRight_ExtendEnd;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).y += inside2outsideY * settingPara.FrontRight_ExtendEnd;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).z += inside2outsideZ * settingPara.FrontRight_ExtendEnd;
                }
            } else if (weldSeamInfo[i]->weldType == WELD_TYPE::FRONT_BEAM_BUTT) {  // 正面横梁对接焊缝
                if (i <= endOfLeftSeams) {
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).x -= inside2outsideX * settingPara.FrontBeamLeft_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).y -= inside2outsideY * settingPara.FrontBeamLeft_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).z -= inside2outsideZ * settingPara.FrontBeamLeft_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).x += inside2outsideX * settingPara.FrontBeamLeft_ExtendEnd;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).y += inside2outsideY * settingPara.FrontBeamLeft_ExtendEnd;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).z += inside2outsideZ * settingPara.FrontBeamLeft_ExtendEnd;
                } else {
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).x -= inside2outsideX * settingPara.FrontBeamRight_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).y -= inside2outsideY * settingPara.FrontBeamRight_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).z -= inside2outsideZ * settingPara.FrontBeamRight_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).x += inside2outsideX * settingPara.FrontBeamRight_ExtendEnd;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).y += inside2outsideY * settingPara.FrontBeamRight_ExtendEnd;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).z += inside2outsideZ * settingPara.FrontBeamRight_ExtendEnd;
                }
            } else if (weldSeamInfo[i]->weldType == WELD_TYPE::FRONT_HORIZONTAL_FILLET) {  // 正面横梁水平角接焊缝
                if (i <= endOfLeftSeams) {
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).x -= inside2outsideX * settingPara.FrontHBeamLeft_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).y -= inside2outsideY * settingPara.FrontHBeamLeft_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).z -= inside2outsideZ * settingPara.FrontHBeamLeft_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).x += inside2outsideX * settingPara.FrontHBeamLeft_ExtendEnd;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).y += inside2outsideY * settingPara.FrontHBeamLeft_ExtendEnd;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).z += inside2outsideZ * settingPara.FrontHBeamLeft_ExtendEnd;
                } else {
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).x -= inside2outsideX * settingPara.FrontHBeamRight_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).y -= inside2outsideY * settingPara.FrontHBeamRight_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).z -= inside2outsideZ * settingPara.FrontHBeamRight_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).x += inside2outsideX * settingPara.FrontHBeamRight_ExtendEnd;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).y += inside2outsideY * settingPara.FrontHBeamRight_ExtendEnd;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).z += inside2outsideZ * settingPara.FrontHBeamRight_ExtendEnd;
                }
            } else if (weldSeamInfo[i]->weldType == WELD_TYPE::FRONT_VERTICAL_FILLET) {  // 正面横梁垂直角接焊缝-立焊交换起点终点
                if (i <= endOfLeftSeams) {
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).x -= inside2outsideX * settingPara.FrontVBeamLeft_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).y -= inside2outsideY * settingPara.FrontVBeamLeft_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).z -= inside2outsideZ * settingPara.FrontVBeamLeft_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).x += inside2outsideX * settingPara.FrontVBeamLeft_ExtendEnd;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).y += inside2outsideY * settingPara.FrontVBeamLeft_ExtendEnd;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).z += inside2outsideZ * settingPara.FrontVBeamLeft_ExtendEnd;
                } else {
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).x -= inside2outsideX * settingPara.FrontVBeamRight_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).y -= inside2outsideY * settingPara.FrontVBeamRight_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(0).z -= inside2outsideZ * settingPara.FrontVBeamRight_ExtendStart;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).x += inside2outsideX * settingPara.FrontVBeamRight_ExtendEnd;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).y += inside2outsideY * settingPara.FrontVBeamRight_ExtendEnd;
                    weldSeamInfo[i]->weldEndPointsInRobot->at(1).z += inside2outsideZ * settingPara.FrontVBeamRight_ExtendEnd;
                }
            }
        }
    }
}

// 修改焊缝方向 (在虚拟坐标系中操作)
void SteelAngleTrajectoryPlanning::transSeamsOri(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo, int endOfLeftSeams) {
    for (int i = 0; i < weldSeamInfo.size(); ++i) {
        if (weldSeamInfo[i]->detectSuccFlag == true && weldSeamInfo[i]->weldEndPointsInRobot != nullptr &&
            weldSeamInfo[i]->weldEndPointsInRobot->size() == 2) {
            if (weldSeamInfo[i]->weldType == WELD_TYPE::BACK_BEAM_BUTT || weldSeamInfo[i]->weldType == WELD_TYPE::BACK_CORNER_BUTT ||
                weldSeamInfo[i]->weldType == WELD_TYPE::FRONT_BEAM_BUTT ||
                weldSeamInfo[i]->weldType == WELD_TYPE::FRONT_CORNER_BUTT) {  // 不区分左右即可修改方向的焊缝
                if (weldSeamInfo[i]->weldEndPointsInRobot->at(0).y > weldSeamInfo[i]->weldEndPointsInRobot->at(1).y) {
                    std::swap(weldSeamInfo[i]->weldEndPointsInRobot->at(0), weldSeamInfo[i]->weldEndPointsInRobot->at(1));
                }
            } else if (weldSeamInfo[i]->weldType == WELD_TYPE::FRONT_HORIZONTAL_FILLET) {  // 水平焊缝
                if (i <= endOfLeftSeams) {  // 左侧焊缝                                              .
                    if (weldSeamInfo[i]->weldEndPointsInRobot->at(0).x < weldSeamInfo[i]->weldEndPointsInRobot->at(1).x) {
                        std::swap(weldSeamInfo[i]->weldEndPointsInRobot->at(0), weldSeamInfo[i]->weldEndPointsInRobot->at(1));
                    }
                } else {  // 右侧焊缝
                    if (weldSeamInfo[i]->weldEndPointsInRobot->at(0).x > weldSeamInfo[i]->weldEndPointsInRobot->at(1).x) {
                        std::swap(weldSeamInfo[i]->weldEndPointsInRobot->at(0), weldSeamInfo[i]->weldEndPointsInRobot->at(1));
                    }
                }
            } else if (weldSeamInfo[i]->weldType == WELD_TYPE::FRONT_VERTICAL_FILLET) {  // 竖直焊缝
                if (weldSeamInfo[i]->weldEndPointsInRobot->at(0).z < weldSeamInfo[i]->weldEndPointsInRobot->at(1).z) {
                    std::swap(weldSeamInfo[i]->weldEndPointsInRobot->at(0), weldSeamInfo[i]->weldEndPointsInRobot->at(1));
                }
            }
        }
    }
}

// 焊缝误差补偿 (真实坐标系)
void SteelAngleTrajectoryPlanning::seamsErrorCompensate(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo) {
    if (trajectoryConfig.robotType == MyToolFunc::getRobotTypeString(ROBOT_TYPE::BAO_YUAN)) {  // 宝元机器人
        // 与宝元版本代码分离
    } else if (trajectoryConfig.robotType == MyToolFunc::getRobotTypeString(ROBOT_TYPE::AN_CHUAN)) {  // 安川机器人
        /*
         *         YASKAWA (安川机器人)
         *
         *             ㊧ FRONT ㊨
         *                  ↑ X
         *                  |
         *                  |
         *   ㊨  Y          |             ㊧
         *  LEFT ←----------| 0         RIGHT
         *   ㊧                           ㊨
         *
         *               ☴ ☲ ☷
         *               ☳ ☯ ☱
         *               ☶ ☵ ☰
         *Region1 机器人工件的  ㊧
         *Region2 机器人工件的  ㊨
         */
        if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::FRONT) {
            for (auto& info : weldSeamInfo) {
                if (info->detectSuccFlag == true && info->weldEndPointsInRobot != nullptr && info->weldEndPointsInRobot->size() == 2) {
                    for (auto& end : *(info->weldEndPointsInRobot)) {
                        end = MyToolFunc::transformSinglePoint(end, trajectoryConfig.middleErrorCompensationMatrix);
                        // 不处理
                    }
                }
            }
        } else if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::LEFT) {
            for (auto& info : weldSeamInfo) {
                if (info->detectSuccFlag == true && info->weldEndPointsInRobot != nullptr && info->weldEndPointsInRobot->size() == 2) {
                    for (auto& end : *(info->weldEndPointsInRobot)) {
                        end = MyToolFunc::transformSinglePoint(end, trajectoryConfig.leftErrorCompensationMatrix);
                        if (info->weldType == WELD_TYPE::FRONT_CORNER_BUTT) {
                            if (end.x < 0) {
                                end.x += settingPara.Front_Region1_X_Shift;
                                end.y += settingPara.Front_Region1_Y_Shift;
                                end.z += settingPara.Front_Region1_Z_Shift;
                            } else {
                                end.x += settingPara.Front_Region2_X_Shift;
                                end.y += settingPara.Front_Region2_Y_Shift;
                                end.z += settingPara.Front_Region2_Z_Shift;
                            }
                        } else if (info->weldType == WELD_TYPE::BACK_CORNER_BUTT) {
                            if (end.x < 0) {
                                end.x += settingPara.Back_Region1_X_Shift;
                                end.y += settingPara.Back_Region1_Y_Shift;
                                end.z += settingPara.Back_Region1_Z_Shift;
                            } else {
                                end.x += settingPara.Back_Region2_X_Shift;
                                end.y += settingPara.Back_Region2_Y_Shift;
                                end.z += settingPara.Back_Region2_Z_Shift;
                            }
                        } else if (info->weldType == WELD_TYPE::FRONT_BEAM_BUTT && info->weldAreaType == WELD_AREA_TYPE::FRONT_UP_BEAM) {
                            if (end.x < 0) {
                                end.x += settingPara.Front_Beam_Region1_X_Shift;
                                end.y += settingPara.Front_Beam_Region1_Y_Shift;
                                end.z += settingPara.Front_Beam_Region1_Z_Shift;
                            } else {
                                end.x += settingPara.Front_Beam_Region2_X_Shift;
                                end.y += settingPara.Front_Beam_Region2_Y_Shift;
                                end.z += settingPara.Front_Beam_Region2_Z_Shift;
                            }
                        } else if (info->weldType == WELD_TYPE::FRONT_HORIZONTAL_FILLET && info->weldAreaType == WELD_AREA_TYPE::FRONT_UP_BEAM) {
                            if (end.x < 0) {
                                end.x += settingPara.Front_L_Beam_H_X_Shift;
                                end.y += settingPara.Front_L_Beam_H_Y_Shift;
                                end.z += settingPara.Front_L_Beam_H_Z_Shift;
                            } else {
                                end.x += settingPara.Front_R_Beam_H_X_Shift;
                                end.y += settingPara.Front_R_Beam_H_Y_Shift;
                                end.z += settingPara.Front_R_Beam_H_Z_Shift;
                            }
                        } else if (info->weldType == WELD_TYPE::FRONT_VERTICAL_FILLET && info->weldAreaType == WELD_AREA_TYPE::FRONT_UP_BEAM) {
                            if (end.x < 0) {
                                end.x += settingPara.Front_L_Beam_V_X_Shift;
                                end.y += settingPara.Front_L_Beam_V_Y_Shift;
                                end.z += settingPara.Front_L_Beam_V_Z_Shift;
                            } else {
                                end.x += settingPara.Front_R_Beam_V_X_Shift;
                                end.y += settingPara.Front_R_Beam_V_Y_Shift;
                                end.z += settingPara.Front_R_Beam_V_Z_Shift;
                            }
                        } else if (info->weldType == WELD_TYPE::FRONT_HORIZONTAL_FILLET && info->weldAreaType == WELD_AREA_TYPE::FRONT_DOWN_BEAM) {
                            if (end.x < 0) {
                                end.x += settingPara.Front_L_Beam_DH_X_Shift;
                                end.y += settingPara.Front_L_Beam_DH_Y_Shift;
                                end.z += settingPara.Front_L_Beam_DH_Z_Shift;
                            } else {
                                end.x += settingPara.Front_R_Beam_DH_X_Shift;
                                end.y += settingPara.Front_R_Beam_DH_Y_Shift;
                                end.z += settingPara.Front_R_Beam_DH_Z_Shift;
                            }
                        } else if (info->weldType == WELD_TYPE::FRONT_VERTICAL_FILLET && info->weldAreaType == WELD_AREA_TYPE::FRONT_DOWN_BEAM) {
                            if (end.x < 0) {
                                end.x += settingPara.Front_L_Beam_DV_X_Shift;
                                end.y += settingPara.Front_L_Beam_DV_Y_Shift;
                                end.z += settingPara.Front_L_Beam_DV_Z_Shift;
                            } else {
                                end.x += settingPara.Front_R_Beam_DV_X_Shift;
                                end.y += settingPara.Front_R_Beam_DV_Y_Shift;
                                end.z += settingPara.Front_R_Beam_DV_Z_Shift;
                            }
                        } else if (info->weldType == WELD_TYPE::BACK_BEAM_BUTT) {
                            if (end.x < 0) {
                                end.x += settingPara.Back_Beam_Region1_X_Shift;
                                end.y += settingPara.Back_Beam_Region1_Y_Shift;
                                end.z += settingPara.Back_Beam_Region1_Z_Shift;
                            } else {
                                end.x += settingPara.Back_Beam_Region2_X_Shift;
                                end.y += settingPara.Back_Beam_Region2_Y_Shift;
                                end.z += settingPara.Back_Beam_Region2_Z_Shift;
                            }
                        }
                        if (info->weldType == WELD_TYPE::FRONT_HORIZONTAL_FILLET) {
                            end.z -= 4;  // TODO 临时误差补偿
                        }
                    }
                }
            }
        } else if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::RIGHT) {
            for (auto& info : weldSeamInfo) {
                if (info->detectSuccFlag == true && info->weldEndPointsInRobot != nullptr && info->weldEndPointsInRobot->size() == 2) {
                    for (auto& end : *(info->weldEndPointsInRobot)) {
                        end = MyToolFunc::transformSinglePoint(end, trajectoryConfig.rightErrorCompensationMatrix);
                        if (info->weldType == WELD_TYPE::FRONT_CORNER_BUTT) {
                            if (end.x > 0) {
                                end.x += settingPara.Front_Region1_X_Shift_R;
                                end.y += settingPara.Front_Region1_Y_Shift_R;
                                end.z += settingPara.Front_Region1_Z_Shift_R;
                            } else {
                                end.x += settingPara.Front_Region2_X_Shift_R;
                                end.y += settingPara.Front_Region2_Y_Shift_R;
                                end.z += settingPara.Front_Region2_Z_Shift_R;
                            }
                        } else if (info->weldType == WELD_TYPE::BACK_CORNER_BUTT) {
                            if (end.x > 0) {
                                end.x += settingPara.Back_Region1_X_Shift_R;
                                end.y += settingPara.Back_Region1_Y_Shift_R;
                                end.z += settingPara.Back_Region1_Z_Shift_R;
                            } else {
                                end.x += settingPara.Back_Region2_X_Shift_R;
                                end.y += settingPara.Back_Region2_Y_Shift_R;
                                end.z += settingPara.Back_Region2_Z_Shift_R;
                            }
                        } else if (info->weldType == WELD_TYPE::FRONT_BEAM_BUTT || info->weldType == WELD_TYPE::FRONT_HORIZONTAL_FILLET ||
                                   info->weldType == WELD_TYPE::FRONT_VERTICAL_FILLET) {
                            if (end.x > 0) {
                                end.x += settingPara.Front_Beam_Region1_X_Shift_R;
                                end.y += settingPara.Front_Beam_Region1_Y_Shift_R;
                                end.z += settingPara.Front_Beam_Region1_Z_Shift_R;
                            } else {
                                end.x += settingPara.Front_Beam_Region2_X_Shift_R;
                                end.y += settingPara.Front_Beam_Region2_Y_Shift_R;
                                end.z += settingPara.Front_Beam_Region2_Z_Shift_R;
                            }
                        } else if (info->weldType == WELD_TYPE::BACK_BEAM_BUTT) {
                            if (end.x > 0) {
                                end.x += settingPara.Back_Beam_Region1_X_Shift_R;
                                end.y += settingPara.Back_Beam_Region1_Y_Shift_R;
                                end.z += settingPara.Back_Beam_Region1_Z_Shift_R;
                            } else {
                                end.x += settingPara.Back_Beam_Region2_X_Shift_R;
                                end.y += settingPara.Back_Beam_Region2_Y_Shift_R;
                                end.z += settingPara.Back_Beam_Region2_Z_Shift_R;
                            }
                        }
                        if (info->weldType == WELD_TYPE::FRONT_HORIZONTAL_FILLET) {
                            end.z -= 4;  // TODO 临时误差补偿
                        }
                    }
                }
            }
        }
    }
}

// 判断工件位于机器人的方位
void SteelAngleTrajectoryPlanning::determineWorkpieceOri(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo) {
    bool xAllMoreThan500 = true;
    bool xAllLessThanMinus500 = true;
    bool yAllMoreThan500 = true;
    bool yAllLessThanMinus500 = true;
    for (auto& info : weldSeamInfo) {
        if (info->detectSuccFlag == true && info->weldEndPointsInRobot != nullptr && info->weldEndPointsInRobot->size() == 2) {
            if (info->weldEndPointsInRobot->at(0).x > 500) {
                xAllLessThanMinus500 = false;
            } else if (info->weldEndPointsInRobot->at(0).x < -500) {
                xAllMoreThan500 = false;
            } else {
                xAllLessThanMinus500 = false;
                xAllMoreThan500 = false;
            }
            if (info->weldEndPointsInRobot->at(0).y > 500) {
                yAllLessThanMinus500 = false;
            } else if (info->weldEndPointsInRobot->at(0).y < -500) {
                yAllMoreThan500 = false;
            } else {
                yAllLessThanMinus500 = false;
                yAllMoreThan500 = false;
            }
        }
    }

    // 工件判面, 此处的前后左右是工件相对于机器人
    if (trajectoryConfig.robotType == MyToolFunc::getRobotTypeString(ROBOT_TYPE::BAO_YUAN)) {  // 宝元机器人
        if (yAllMoreThan500 == true) {
            workpieceSide = WORKPIECE_SIDE_OF_ROBOT::FRONT;
            PLOGD << "当前在焊接的工件方位判断为前方";
        } else if (xAllLessThanMinus500 == true) {
            workpieceSide = WORKPIECE_SIDE_OF_ROBOT::LEFT;
            PLOGD << "当前在焊接的工件方位判断为左侧";
        } else if (xAllMoreThan500 == true) {
            workpieceSide = WORKPIECE_SIDE_OF_ROBOT::RIGHT;
            PLOGD << "当前在焊接的工件方位判断为右侧";
        }
    } else if (trajectoryConfig.robotType == MyToolFunc::getRobotTypeString(ROBOT_TYPE::AN_CHUAN)) {  // 安川机器人
        if (yAllMoreThan500 == true) {
            workpieceSide = WORKPIECE_SIDE_OF_ROBOT::LEFT;
            PLOGD << "当前在焊接的工件方位判断为左侧";
        } else if (yAllLessThanMinus500 == true) {
            workpieceSide = WORKPIECE_SIDE_OF_ROBOT::RIGHT;
            PLOGD << "当前在焊接的工件方位判断为右侧";
        } else if (xAllMoreThan500 == true) {
            workpieceSide = WORKPIECE_SIDE_OF_ROBOT::FRONT;
            PLOGD << "当前在焊接的工件方位判断为前方";
        }
    }
}

// 将焊缝点转到机器人基坐标系
void SteelAngleTrajectoryPlanning::transSeams2Base(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo) {
    Eigen::Matrix4f T = trajectoryConfig.matrixEyeHand;
    if (trajectoryConfig.handEyeType == MyToolFunc::getHandTypeTypeString(HAND_EYE_TYPE::EYE_IN_HAND)) {
        T = trajectoryConfig.matrixEnd2Base * trajectoryConfig.matrixEyeHand;
    }
    for (auto& info : weldSeamInfo) {
        if (!info || !info->detectSuccFlag) continue;
        /* ---------- 焊缝点（同步） ---------- */

        if (info->weldEndPointsInCamera && info->weldEndPointsInCamera->size() == 2) {
            info->weldEndPointsInRobot = std::make_shared<std::vector<pcl::PointXYZ>>();
            info->weldEndPointsInRobotRaw = std::make_shared<std::vector<pcl::PointXYZ>>();
            info->weldEndPointsInRobot->reserve(info->weldEndPointsInCamera->size());
            info->weldEndPointsInRobotRaw->reserve(info->weldEndPointsInCamera->size());

            for (const auto& pt : *(info->weldEndPointsInCamera)) {
                const auto rawPoint = MyToolFunc::transformSinglePoint(pt, T);
                info->weldEndPointsInRobot->push_back(rawPoint);
                info->weldEndPointsInRobotRaw->push_back(rawPoint);
            }

        }
        /* ---------- 点云（异步） ---------- */

        if (info->weldAreaPointCloudInCamera && !info->weldAreaPointCloudInCamera->empty()) {
            auto cloud_in = info->weldAreaPointCloudInCamera;

            info->cloudFuture = QtConcurrent::run([cloud_in, T]() { return MyToolFunc::transformPointCloud(cloud_in, T); });
        }
    }
}

// 找到左右焊缝的分界线
int SteelAngleTrajectoryPlanning::findEndOfLeftSeams(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo) {
    int endOfLeftSeamSerial = 0;

    // ########################### 使用绝对位置判断左右 ###########################
    for (int i = 0; i < weldSeamInfo.size(); ++i) {
        if (weldSeamInfo[i]->detectSuccFlag == true && weldSeamInfo[i]->weldEndPointsInCamera != nullptr &&
            weldSeamInfo[i]->weldEndPointsInCamera->size() == 2) {
            if (weldSeamInfo[i]->weldEndPointsInCamera->at(0).x > -11) {
                endOfLeftSeamSerial = i - 1;
                break;
            } else {
                endOfLeftSeamSerial = i;
            }
        }
    }

    return endOfLeftSeamSerial;
}

// 将焊缝信息按照X值进行排序
void SteelAngleTrajectoryPlanning::sortSeamsWithX(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo) {
    std::sort(weldSeamInfo.begin(), weldSeamInfo.end(), [](const std::shared_ptr<WeldSeamInfo>& a, const std::shared_ptr<WeldSeamInfo>& b) {
        // 优先级1: detectSuccFlag 为 true 的排前面
        if (a->detectSuccFlag != b->detectSuccFlag) {
            return a->detectSuccFlag > b->detectSuccFlag;
        }

        // 两者 detectSuccFlag 状态相同
        if (!a->detectSuccFlag) {  // 都为 false, 保持原顺序
            return false;
        }

        // 检查指针和容器有效性
        auto& pointsA = a->weldEndPointsInCamera;
        auto& pointsB = b->weldEndPointsInCamera;
        if (!pointsA || !pointsB || pointsA->empty() || pointsB->empty()) {
            // 处理无效数据, 将无效项排在后面
            return (pointsA && !pointsA->empty()) > (pointsB && !pointsB->empty());
        }

        // 优先级2: 比较 x 坐标
        return pointsA->front().x < pointsB->front().x;
    });
}

// 真实坐标系转虚拟坐标系
void SteelAngleTrajectoryPlanning::real2Virtual(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo) {
    if (trajectoryConfig.robotType == MyToolFunc::getRobotTypeString(ROBOT_TYPE::BAO_YUAN)) {  // 宝元机器人
        if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::FRONT) {
            // 无需操作
        } else if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::LEFT) {
            backXrightY2rightXfrontY(weldSeamInfo);
        } else if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::RIGHT) {
            frontXleftY2rightXfrontY(weldSeamInfo);
        }
    } else if (trajectoryConfig.robotType == MyToolFunc::getRobotTypeString(ROBOT_TYPE::AN_CHUAN)) {  // 安川机器人
        if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::FRONT) {
            frontXleftY2rightXfrontY(weldSeamInfo);
        } else if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::LEFT) {
            // 无需操作
        } else if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::RIGHT) {
            leftXbackY2rightXfrontY(weldSeamInfo);
        }
    }
}

// 虚拟坐标系转真实坐标系
void SteelAngleTrajectoryPlanning::virtual2Real(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo) {
    if (trajectoryConfig.robotType == MyToolFunc::getRobotTypeString(ROBOT_TYPE::BAO_YUAN)) {  // 宝元机器人
        if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::FRONT) {
            // 无需操作
        } else if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::LEFT) {
            rightXfrontY2backXrightY(weldSeamInfo);
        } else if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::RIGHT) {
            rightXfrontY2frontXleftY(weldSeamInfo);
        }
    } else if (trajectoryConfig.robotType == MyToolFunc::getRobotTypeString(ROBOT_TYPE::AN_CHUAN)) {  // 安川机器人
        if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::FRONT) {
            rightXfrontY2frontXleftY(weldSeamInfo);
        } else if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::LEFT) {
            // 无需操作
        } else if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::RIGHT) {
            rightXfrontY2leftXbackY(weldSeamInfo);
        }
    }
}

// 前方X左方Y(真实坐标系) 转为 右方X前方Y(虚拟坐标系)
void SteelAngleTrajectoryPlanning::frontXleftY2rightXfrontY(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo) {
    for (auto& info : weldSeamInfo) {
        if (info->detectSuccFlag == true && info->weldEndPointsInRobot != nullptr && info->weldEndPointsInRobot->size() == 2) {
            std::vector<pcl::PointXYZ> tempPoints = *(info->weldEndPointsInRobot);
            info->weldEndPointsInRobot->at(0) = pcl::PointXYZ(-tempPoints[0].y, tempPoints[0].x, tempPoints[0].z);
            info->weldEndPointsInRobot->at(1) = pcl::PointXYZ(-tempPoints[1].y, tempPoints[1].x, tempPoints[1].z);
        }
    }
}

// 左方X后方Y(真实坐标系) 转为 右方X前方Y(虚拟坐标系)
void SteelAngleTrajectoryPlanning::leftXbackY2rightXfrontY(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo) {
    for (auto& info : weldSeamInfo) {
        if (info->detectSuccFlag == true && info->weldEndPointsInRobot != nullptr && info->weldEndPointsInRobot->size() == 2) {
            std::vector<pcl::PointXYZ> tempPoints = *(info->weldEndPointsInRobot);
            info->weldEndPointsInRobot->at(0) = pcl::PointXYZ(-tempPoints[0].x, -tempPoints[0].y, tempPoints[0].z);
            info->weldEndPointsInRobot->at(1) = pcl::PointXYZ(-tempPoints[1].x, -tempPoints[1].y, tempPoints[1].z);
        }
    }
}

// 后方X右方Y(真实坐标系) 转为 右方X前方Y(虚拟坐标系)
void SteelAngleTrajectoryPlanning::backXrightY2rightXfrontY(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo) {
    for (auto& info : weldSeamInfo) {
        if (info->detectSuccFlag == true && info->weldEndPointsInRobot != nullptr && info->weldEndPointsInRobot->size() == 2) {
            std::vector<pcl::PointXYZ> tempPoints = *(info->weldEndPointsInRobot);
            info->weldEndPointsInRobot->at(0) = pcl::PointXYZ(tempPoints[0].y, -tempPoints[0].x, tempPoints[0].z);
            info->weldEndPointsInRobot->at(1) = pcl::PointXYZ(tempPoints[1].y, -tempPoints[1].x, tempPoints[1].z);
        }
    }
}

// 右方X前方Y(虚拟坐标系) 转为 前方X左方Y(真实坐标系)
void SteelAngleTrajectoryPlanning::rightXfrontY2frontXleftY(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo) {
    for (auto& info : weldSeamInfo) {
        if (info->detectSuccFlag == true && info->weldEndPointsInRobot != nullptr && info->weldEndPointsInRobot->size() == 2) {
            std::vector<pcl::PointXYZ> tempPoints = *(info->weldEndPointsInRobot);
            info->weldEndPointsInRobot->at(0) = pcl::PointXYZ(tempPoints[0].y, -tempPoints[0].x, tempPoints[0].z);
            info->weldEndPointsInRobot->at(1) = pcl::PointXYZ(tempPoints[1].y, -tempPoints[1].x, tempPoints[1].z);
        }
    }
}

// 右方X前方Y(虚拟坐标系) 转为 左方X后方Y(真实坐标系)
void SteelAngleTrajectoryPlanning::rightXfrontY2leftXbackY(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo) {
    for (auto& info : weldSeamInfo) {
        if (info->detectSuccFlag == true && info->weldEndPointsInRobot != nullptr && info->weldEndPointsInRobot->size() == 2) {
            std::vector<pcl::PointXYZ> tempPoints = *(info->weldEndPointsInRobot);
            info->weldEndPointsInRobot->at(0) = pcl::PointXYZ(-tempPoints[0].x, -tempPoints[0].y, tempPoints[0].z);
            info->weldEndPointsInRobot->at(1) = pcl::PointXYZ(-tempPoints[1].x, -tempPoints[1].y, tempPoints[1].z);
        }
    }
}

// 右方X前方Y(虚拟坐标系) 转为 后方X右方Y(真实坐标系)
void SteelAngleTrajectoryPlanning::rightXfrontY2backXrightY(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo) {
    for (auto& info : weldSeamInfo) {
        if (info->detectSuccFlag == true && info->weldEndPointsInRobot != nullptr && info->weldEndPointsInRobot->size() == 2) {
            std::vector<pcl::PointXYZ> tempPoints = *(info->weldEndPointsInRobot);
            info->weldEndPointsInRobot->at(0) = pcl::PointXYZ(-tempPoints[0].y, tempPoints[0].x, tempPoints[0].z);
            info->weldEndPointsInRobot->at(1) = pcl::PointXYZ(-tempPoints[1].y, tempPoints[1].x, tempPoints[1].z);
        }
    }
}

// 角度转换为弧度
constexpr double SteelAngleTrajectoryPlanning::deg2rad(double degrees) { return degrees * M_PI / 180.0; }

// 生成绕 x 轴的旋转矩阵 (Roll)
std::array<std::array<double, 3>, 3> SteelAngleTrajectoryPlanning::getRollMatrix(double rollRad) {
    return {
        {{1.0, 0.0, 0.0}, {0.0, std::cos(rollRad), -std::sin(rollRad)}, {0.0, std::sin(rollRad), std::cos(rollRad)}}
    };
}

// 生成绕 y 轴的旋转矩阵 (Pitch)
std::array<std::array<double, 3>, 3> SteelAngleTrajectoryPlanning::getPitchMatrix(double pitchRad) {
    return {
        {{std::cos(pitchRad), 0.0, std::sin(pitchRad)}, {0.0, 1.0, 0.0}, {-std::sin(pitchRad), 0.0, std::cos(pitchRad)}}
    };
}

// 生成绕 z 轴的旋转矩阵 (Yaw)
std::array<std::array<double, 3>, 3> SteelAngleTrajectoryPlanning::getYawMatrix(double yawRad) {
    return {
        {{std::cos(yawRad), -std::sin(yawRad), 0.0}, {std::sin(yawRad), std::cos(yawRad), 0.0}, {0.0, 0.0, 1.0}}
    };
}

// 旋转向量
std::array<double, 3> SteelAngleTrajectoryPlanning::rotateVector(const std::array<std::array<double, 3>, 3>& mat, const std::array<double, 3>& vec) {
    std::array<double, 3> result = {0.0, 0.0, 0.0};

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result[i] += mat[i][j] * vec[j];
        }
    }

    return result;
}

// 将A、B、C应用到向量 (0, 0, 1)
std::array<double, 3> SteelAngleTrajectoryPlanning::abcToVector(double A, double B, double C) {
    if (trajectoryConfig.robotType == MyToolFunc::getRobotTypeString(ROBOT_TYPE::AN_CHUAN)) {
        double rollRad = deg2rad(A);  // 将角度转换为弧度
        double pitchRad = deg2rad(B);
        double yawRad = deg2rad(C);

        auto rollMatrix = getRollMatrix(rollRad);  // 计算旋转矩阵
        auto pitchMatrix = getPitchMatrix(pitchRad);
        auto yawMatrix = getYawMatrix(yawRad);

        std::array<double, 3> vector = {0.0, 0.0, -1.0};  // 初始单位向量 (0, 0, 1)

        // 依次应用 roll -> pitch -> yaw 旋转 (ZYX 先左乘X，最后Z)
        vector = rotateVector(rollMatrix, vector);
        vector = rotateVector(pitchMatrix, vector);
        vector = rotateVector(yawMatrix, vector);

        return vector;
    } else if (trajectoryConfig.robotType == MyToolFunc::getRobotTypeString(ROBOT_TYPE::BAO_YUAN)) {
        A = deg2rad(A);  // 计算时欧拉角以弧度为单位
        B = deg2rad(B);
        C = deg2rad(C);

        auto Rz_A = getYawMatrix(A);  // 计算旋转矩阵
        auto Rx_B = getRollMatrix(B);
        auto Rz_C = getYawMatrix(C);

        std::array<double, 3> vector = {0.0, 0.0, 1.0};  // 初始单位向量 (0, 0, -1)

        // 先应用 Rz(C)，然后是 Rx(B)，最后是 Rz(A)
        vector = rotateVector(Rz_C, vector);
        vector = rotateVector(Rx_B, vector);
        vector = rotateVector(Rz_A, vector);
        return vector;
    }
}
// // 初始化配置信息
// void SteelAngleTrajectoryPlanning::initConfig() {
//     // this->writeConfig();  // 写配置文件
//     this->readConfig();  // 读配置文件
//     // this->printConfig();  // 打印配置文件

//     TrajectoryPlanningConfig::getInstance().myDataStructure2LibDataStructure();  // 自定义数据类型转换为库数据类型
// }

// // 写配置文件
// void SteelAngleTrajectoryPlanning::writeConfig() {
//     {  // 写
//         std::ofstream os("./data/config/Trajectory_Planning_config.json");
//         cereal::JSONOutputArchive jsonOutputArchive(os);
//         jsonOutputArchive(cereal::make_nvp("config about Trajectory Planning", TrajectoryPlanningConfig::getInstance()));
//     }
// }

// // 读配置文件
// void SteelAngleTrajectoryPlanning::readConfig() {
//     {  // 读
//         std::ifstream is("./data/config/Trajectory_Planning_config.json");
//         cereal::JSONInputArchive inputArchive(is);
//         inputArchive(TrajectoryPlanningConfig::getInstance());
//         PLOGD << "轨迹规划配置文件读取成功";
//     }
// }

// // 打印配置文件
// void SteelAngleTrajectoryPlanning::printConfig() {
//     {  // 打印
//         cereal::JSONOutputArchive jsonOutputArchive(std::cout);
//         jsonOutputArchive(cereal::make_nvp("config about Trajectory Planning", TrajectoryPlanningConfig::getInstance()));
//         std::cout << std::endl;
//     }
// }
