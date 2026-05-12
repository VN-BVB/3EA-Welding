
#include "robotTrajectoryPlanning/GantrayFrameTrajectoryPlanning.h"

#include "plog/Log.h"
#include "robotTrajectoryPlanning/config/TrajectoryPlanningConfig.h"
#include "settingPara/SettingPara.h"
#include "utils/common/CommonFunc.h"
#include "utils/common/WeldSeamInfo.h"
#include "utils/pointCloud/PointCloudFunc.h"
GantrayFrameTrajectoryPlanning::GantrayFrameTrajectoryPlanning(QObject* parent) : AbstractTrajectoryPlanning(parent) {
    PLOGD << "龙门支架轨迹规划类初始化";
}

void GantrayFrameTrajectoryPlanning::whenPlanningTrajectory(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo) {
    PLOGD << "龙门支架轨迹规划类 收到焊缝数量: " << weldSeamInfo.size();
    for (auto& info : weldSeamInfo) {
        if (info->detectSuccFlag == true && info->weldEndPointsInCamera != nullptr && info->weldEndPointsInCamera->size() == 2) {
            PLOGD << info->weldEndPointsInCamera->at(0) << " " << info->weldEndPointsInCamera->at(1);
        }
    }
    // --------------------------- 标准化表面方向（相机坐标系） ---------------------------

    this->normalizeSurfaceDirectionInCamera(weldSeamInfo);

    // --------------------------- 转换坐标点到机器人基坐标系下 ---------------------------
    // 实现龙门支架的轨迹规划逻辑
    // std::cout << "当前机器人位姿矩阵" << trajectoryConfig.matrixEnd2Base;
    this->transSeams2Base(weldSeamInfo);
    PLOGD << "转到机器人基坐标系下后的焊缝点: ";  // 打印操作后的焊缝
    for (auto& info : weldSeamInfo) {
        if (info->detectSuccFlag == true && info->weldEndPointsInRobot != nullptr && info->weldEndPointsInRobot->size() == 2) {
            PLOGD << info->weldEndPointsInRobot->at(0) << " " << info->weldEndPointsInRobot->at(1);
        }
    }

    // --------------------------- 对工件摆放进行方位判别 ---------------------------
    this->determineWorkpieceOri(weldSeamInfo);
    if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::FRONT)
        PLOGD << "工件位于机器人前面";
    else if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::LEFT)
        PLOGD << "工件位于机器人左面";
    else if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::RIGHT)
        PLOGD << "工件位于机器人右面";
    else
        PLOGD << "工件判面失败";
    // --------------------------- 修改焊缝方向 ---------------------------
    this->transSeamsOri(weldSeamInfo);
    PLOGD << "修改焊缝方向后的焊缝点: ";  // 打印操作后的焊缝
    for (auto& info : weldSeamInfo) {
        if (info->detectSuccFlag == true && info->weldEndPointsInRobot != nullptr && info->weldEndPointsInRobot->size() == 2) {
            PLOGD << info->weldEndPointsInRobot->at(0) << " " << info->weldEndPointsInRobot->at(1);
        }
    }
    //---------------------------- 焊缝微调 ------------------------------

    this->compensateSeams(weldSeamInfo);

    // --------------------------- 生成焊接轨迹 ---------------------------

    this->generateWeldPose(weldSeamInfo);
    this->debugWeldingCollisionCheck(weldSeamInfo);          // 碰撞检测
    this->computeSwingReferencePointsForSeam(weldSeamInfo);  // 摆焊参考点计算

    // --------------------------- 打印焊接轨迹 ---------------------------
    PLOGD << "================ 焊接轨迹（robotWeldPose） ================";

    for (size_t i = 0; i < weldSeamInfo.size(); ++i) {
        auto& info = weldSeamInfo[i];
        if (!info || !info->detectSuccFlag) continue;

        PLOGD << "---- seam index: " << i;

        if (info->robotWeldPose.empty()) {
            PLOGD << "robotWeldPose is empty";
            continue;
        }

        for (size_t j = 0; j < info->robotWeldPose.size(); ++j) {
            const auto& pose = info->robotWeldPose[j];

            PLOGD << "[pose " << j << "] " << "x=" << pose.x_ << ", y=" << pose.y_ << ", z=" << pose.z_ << ", a=" << pose.a_ << ", b=" << pose.b_
                  << ", c=" << pose.c_;
        }
    }
    // --------------------------- 焊缝轨迹后撤(转移到写入文件部分) ---------------------------
    // --------------------------- 等待多线程点云缓存运行完成 ---------------------------
    for (auto& info : weldSeamInfo) {
        if (!info || !info->detectSuccFlag) continue;
        // 检查异步任务是否已启动

        if (info->cloudFuture.isRunning()) {
            info->cloudFuture.waitForFinished();
        }
        info->weldAreaPointCloudInRobot = info->cloudFuture.result();
    }
    PLOGD << "已发送";

    // 发出规划完成的焊缝
    emit sendPlannedSeams(weldSeamInfo);
}

void GantrayFrameTrajectoryPlanning::write2File(const std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo, int endOfLeftSeams) {
    // 实现龙门支架的文件写入逻辑
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

    moveSpeed = settingPara.Value_MoveSpeed * 60;
    weldingSpeedDefault = settingPara.Value_WeldingSpeed * 60;
    weldingCurrent = settingPara.Value_WeldingCurrent;
    weldingCurrent_Vertical = settingPara.Value_WeldingCurrent_Vertical;
    weldingVoltage = settingPara.Value_WeldingVoltage;
    weldingVoltage_Vertical = settingPara.Value_WeldingVoltage_Vertical;
    // 预读零点、拍照点、焊接时姿态
    float X0 = trajectoryConfig.zeroPointX, Y0 = trajectoryConfig.zeroPointY, Z0 = trajectoryConfig.zeroPointZ;
    float A0 = trajectoryConfig.zeroPointA, B0 = trajectoryConfig.zeroPointB, C0 = trajectoryConfig.zeroPointC;

    float takePhotoX0 = trajectoryConfig.takePhotoX, takePhotoY0 = trajectoryConfig.takePhotoY, takePhotoZ0 = trajectoryConfig.takePhotoZ;
    float takePhotoA0 = trajectoryConfig.takePhotoA, takePhotoB0 = trajectoryConfig.takePhotoB, takePhotoC0 = trajectoryConfig.takePhotoC;
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
    for (const auto& info : weldSeamInfo) {
        if (!info || info->robotWeldPose.empty()) continue;
        if (info->weldCollisionResult.size() != info->robotWeldPose.size()) {
            info->weldCollisionResult.resize(info->robotWeldPose.size());
        }

        if (info->weldType == TubeSide_Plate_F_H) {
            const robotPose& startPose = info->robotWeldPose[0];
            const robotPose& endPose = info->robotWeldPose[1];

            // ================= 起点过渡=================
            robotPose startTransition = startPose;
            applyWeldGunWithdraw(startTransition, 20.0);
            writeWeldPoint(outfile, startTransition.x_, startTransition.y_, startTransition.z_ + 20.0, startTransition.a_, startTransition.b_,
                           startTransition.c_, moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);

            // ================= 焊接起点=================
            robotPose startWeld = startPose;
            applyWeldGunWithdraw(startWeld, settingPara.TubeSidePlatFilletWithdrawDistance);
            writeWeldPoint(outfile, startWeld.x_, startWeld.y_, startWeld.z_, startWeld.a_, startWeld.b_, startWeld.c_, moveSpeed, ARC_START,
                           LINE_WELD, weldingCurrent, weldingVoltage);

            // ================= 焊接终点=================
            robotPose endWeld = endPose;
            applyWeldGunWithdraw(endWeld, settingPara.TubeSidePlatFilletWithdrawDistance);

            writeWeldPoint(outfile, endWeld.x_, endWeld.y_, endWeld.z_, endWeld.a_, endWeld.b_, endWeld.c_, weldingSpeedDefault, ARC_STOP, LINE_WELD,
                           weldingCurrent, weldingVoltage);

            // ================= 终点过渡=================
            robotPose endTransition = endPose;
            applyWeldGunWithdraw(endTransition, 20.0);
            writeWeldPoint(outfile, endTransition.x_, endTransition.y_, endTransition.z_ + 20.0, endTransition.a_, endTransition.b_, endTransition.c_,
                           moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);

        } else if (info->weldType == Plate_Plate_Fillet_V) {
            const robotPose& startPose = info->robotWeldPose[0];
            const robotPose& endPose = info->robotWeldPose[1];
            // if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::FRONT) {
            //     if (startPose.y_ >= 0) {
            //         CURR_WELD_METHOD = SWING_WELD_ACTION::FRONT_LEFT_VERTICAL_SWING_WELD;
            //     } else if (startPose.y_ < 0) {
            //         CURR_WELD_METHOD = SWING_WELD_ACTION::FRONT_RIGHT_VERTICAL_SWING_WELD;
            //     }
            // }
            // ================= 起点过渡=================
            robotPose startTransition = startPose;
            applyWeldGunWithdraw(startTransition, 40.0);
            writeWeldPoint(outfile, startTransition.x_, startTransition.y_, startTransition.z_ + 40.0, startTransition.a_, startTransition.b_,
                           startTransition.c_, moveSpeed / 5, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);

            // ================= 焊接起点=================
            robotPose startWeld = startPose;
            applyWeldGunWithdraw(startWeld, settingPara.PlatePlateFilletVerticalWithdrawDistance);
            writeWeldPoint(outfile, startWeld.x_, startWeld.y_, startWeld.z_, startWeld.a_, startWeld.b_, startWeld.c_, moveSpeed, ARC_START,
                           LINE_WELD, weldingCurrent, weldingVoltage);

            // ================= 焊接终点=================
            robotPose endWeld = endPose;
            applyWeldGunWithdraw(endWeld, settingPara.PlatePlateFilletVerticalWithdrawDistance);

            const auto& v = info->swingReferencePoints;

            SWING_WELD_ACTION CURR_WELD_METHOD = GANTRAY_FRAME_LINE_SWING_WELD;
            if (v.size() != 6) {
                CURR_WELD_METHOD = LINE_WELD;
            }
            // if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::FRONT) {
            //     if (startPose.y_ >= 0) {
            //         CURR_WELD_METHOD = SWING_WELD_ACTION::FRONT_LEFT_VERTICAL_SWING_WELD;
            //     } else if (startPose.y_ < 0) {
            //         CURR_WELD_METHOD = SWING_WELD_ACTION::FRONT_RIGHT_VERTICAL_SWING_WELD;
            //     }
            // }
            writeWeldPoint(outfile, endWeld.x_, endWeld.y_, endWeld.z_, endWeld.a_, endWeld.b_, endWeld.c_, weldingSpeedDefault, ARC_STOP,
                           CURR_WELD_METHOD, weldingCurrent, weldingVoltage, v[0], v[1], v[2], v[3], v[4], v[5]);

            // ================= 终点过渡=================
            robotPose endTransition = endPose;
            applyWeldGunWithdraw(endTransition, 40.0);
            // auto midABC = MyToolFunc::interpolateEulerZYX(endTransition.a_, endTransition.b_, endTransition.c_, A0, B0, C0, 0.5);
            writeWeldPoint(outfile, endTransition.x_, endTransition.y_, endTransition.z_ + 40.0, endTransition.a_, endTransition.b_, endTransition.c_,
                           moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
            // outfile << endTransition.x_ << " " << endTransition.y_ << " " << endTransition.z_ + 40.0 << " " << midABC[0] << " " << midABC[1] << " "
            //         << midABC[2] << " " << moveSpeed / 10 << " " << ARC_STOP << " " << LINE_WELD << " " << weldingCurrent << " " << weldingVoltage
            //         << std::endl;
        } else if (info->weldType == Plate_Plate_Fillet_H) {
            const robotPose& startPose = info->robotWeldPose[0];
            const robotPose& endPose = info->robotWeldPose[1];

            // ================= 起点过渡=================
            robotPose startTransition = startPose;
            applyWeldGunWithdraw(startTransition, 20.0);
            writeWeldPoint(outfile, startTransition.x_, startTransition.y_, startTransition.z_ + 20.0, startTransition.a_, startTransition.b_,
                           startTransition.c_, moveSpeed / 10.0, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);

            // ================= 焊接起点=================
            robotPose startWeld = startPose;
            applyWeldGunWithdraw(startWeld, settingPara.PlatePlateFilletHorizontalWithdrawDistance);

            writeWeldPoint(outfile, startWeld.x_, startWeld.y_, startWeld.z_, startWeld.a_, startWeld.b_, startWeld.c_, moveSpeed, ARC_START,
                           LINE_WELD, weldingCurrent, weldingVoltage);

            // ================= 焊接终点=================
            robotPose endWeld = endPose;
            applyWeldGunWithdraw(endWeld, settingPara.PlatePlateFilletHorizontalWithdrawDistance);
            writeWeldPoint(outfile, endWeld.x_, endWeld.y_, endWeld.z_, endWeld.a_, endWeld.b_, endWeld.c_, weldingSpeedDefault, ARC_STOP, LINE_WELD,
                           weldingCurrent, weldingVoltage);

            // ================= 终点过渡=================
            robotPose endTransition = endPose;
            applyWeldGunWithdraw(endTransition, 20.0);
            writeWeldPoint(outfile, endTransition.x_, endTransition.y_, endTransition.z_ + 20.0, endTransition.a_, endTransition.b_, endTransition.c_,
                           moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
        } else if (info->weldType == Tube_Plate_Fillet || info->weldType == Tube_Tube_Fillet) {
            if (info->robotWeldPose.size() < 2) continue;
            // NOTE 后撤临时赋值
            double withdrawBase =
                (info->weldType == Tube_Plate_Fillet) ? settingPara.TubePlateFilletWithdrawDistance : settingPara.TubeTubeFilletWithdrawDistance;
            const auto& v = info->swingReferencePoints;

            SWING_WELD_ACTION CURR_WELD_METHOD = GANTRAY_FRAME_CURVE_SWING_WELD;
            if (v.size() != 6) {
                CURR_WELD_METHOD = GANTRAY_FRAME_CURVE_WELD;
            }
            if (info->weldType == Tube_Tube_Fillet) CURR_WELD_METHOD = GANTRAY_FRAME_CURVE_WELD;  // NOTE 临时debug

            int N = info->robotWeldPose.size();

            // ================= 1. 起点过渡 =================
            robotPose startTransition = info->robotWeldPose[0];
            applyWeldGunWithdraw(startTransition, 60.0);

            writeWeldPoint(outfile, startTransition.x_, startTransition.y_, startTransition.z_ + 60.0, startTransition.a_, startTransition.b_,
                           startTransition.c_, moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);

            // ================= 2. 起弧点 =================
            robotPose startWeld = info->robotWeldPose[0];
            applyWeldGunWithdraw(startWeld, (withdrawBase + info->weldCollisionResult[0].extraOffset));

            writeWeldPoint(outfile, startWeld.x_, startWeld.y_, startWeld.z_, startWeld.a_, startWeld.b_, startWeld.c_, moveSpeed, ARC_START,
                           LINE_WELD, weldingCurrent, weldingVoltage);

            // ================= 3. 中间轨迹点 =================
            for (int i = 0; i < N - 1; i++) {
                robotPose midPose = info->robotWeldPose[i];
                float extra = info->weldCollisionResult[i].extraOffset;

                applyWeldGunWithdraw(midPose, (withdrawBase + extra));
                if (i == 0 && CURR_WELD_METHOD == GANTRAY_FRAME_CURVE_SWING_WELD) {
                    writeWeldPoint(outfile, midPose.x_, midPose.y_, midPose.z_, midPose.a_, midPose.b_, midPose.c_, weldingSpeedDefault, ARC_START,
                                   CURR_WELD_METHOD, weldingCurrent, weldingVoltage, v[0], v[1], v[2], v[3], v[4], v[5]);
                } else {
                    writeWeldPoint(outfile, midPose.x_, midPose.y_, midPose.z_, midPose.a_, midPose.b_, midPose.c_, weldingSpeedDefault, ARC_START,
                                   CURR_WELD_METHOD, weldingCurrent, weldingVoltage);
                }
            }
            robotPose endWeld = info->robotWeldPose[N - 1];
            applyWeldGunWithdraw(endWeld, (withdrawBase + info->weldCollisionResult[N - 1].extraOffset));
            // 轨迹终点--退出曲线运动标志点
            writeWeldPoint(outfile, endWeld.x_, endWeld.y_, endWeld.z_, endWeld.a_, endWeld.b_, endWeld.c_, weldingSpeedDefault, ARC_START, LINE_WELD,
                           weldingCurrent, weldingVoltage);
            // ================= 4. 熄弧终点 =================
            applyWeldGunWithdraw(endWeld, (withdrawBase + info->weldCollisionResult[N - 1].extraOffset));

            writeWeldPoint(outfile, endWeld.x_, endWeld.y_, endWeld.z_, endWeld.a_, endWeld.b_, endWeld.c_, weldingSpeedDefault, ARC_STOP, LINE_WELD,
                           weldingCurrent, weldingVoltage);

            // ================= 5. 终点过渡 =================
            robotPose endTransition = endWeld;
            applyWeldGunWithdraw(endTransition, 60.0);

            writeWeldPoint(outfile, endTransition.x_, endTransition.y_, endTransition.z_ + 60.0, endTransition.a_, endTransition.b_, endTransition.c_,
                           moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
            if (info->areaNum != weldSeamInfo.size()) {
                writeWeldPoint(outfile, X0, Y0, Z0, A0, B0, C0, moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
                writeWeldPoint(outfile, X0, Y0, Z0, A0, B0, C0, moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent,
                               weldingVoltage);  // 此处为了让targetNum偏移一位，是在调试阶段让两段曲线分别存在两个缓冲区而加的，可以删除；
            }
        }
    }
    // ########################### 眼在手上写入拍照点, 眼在手外写入零过渡点 ###########################
    if (trajectoryConfig.handEyeType == MyToolFunc::getHandTypeTypeString(HAND_EYE_TYPE::EYE_IN_HAND)) {
        writeWeldPoint(outfile, X0, Y0, Z0, A0, B0, C0, moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
        writeWeldPoint(outfile, takePhotoX0, takePhotoY0, takePhotoZ0, takePhotoA0, takePhotoB0, takePhotoC0, moveSpeed, ARC_STOP, LINE_WELD,
                       weldingCurrent, weldingVoltage);
        writeWeldPoint(outfile, takePhotoX0, takePhotoY0, takePhotoZ0, takePhotoA0, takePhotoB0, takePhotoC0, moveSpeed, ARC_STOP, LINE_WELD,
                       weldingCurrent, weldingVoltage);
    } else if (trajectoryConfig.handEyeType == MyToolFunc::getHandTypeTypeString(HAND_EYE_TYPE::EYE_TO_HAND)) {
        writeWeldPoint(outfile, X0, Y0, Z0, A0, B0, C0, moveSpeed, ARC_STOP, LINE_WELD, weldingCurrent, weldingVoltage);
    } else {
        PLOGE << "机器人手眼关系错误";
    }

    if (outfile.fail()) {
        PLOGE << "轨迹规划焊缝写入文件失败";
    } else {
        PLOGD << "轨迹规划焊缝写入文件成功";
    }

    outfile.close();
    emit sendTrajectoryPlanOver();
}
void GantrayFrameTrajectoryPlanning::normalizeSurfaceDirectionInCamera(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo) {
    const float EPS = 1e-6f;

    for (auto& info : weldSeamInfo) {
        if (!info || !info->detectSuccFlag) continue;

        // ================= weldPlane =================
        if (info->weldCoeff) {
            auto& v = info->weldCoeff->values;

            // ---------- 平面 ----------
            if (v.size() == 4) {
                Eigen::Vector3f n(v[0], v[1], v[2]);
                float d = v[3];

                if (n.norm() > EPS) {
                    // 平面上一点（离原点最近）
                    Eigen::Vector3f p = -d * n / n.squaredNorm();

                    // 指向相机原点
                    Eigen::Vector3f toCam = -p;

                    if (n.dot(toCam) < 0) {
                        v[0] *= -1;
                        v[1] *= -1;
                        v[2] *= -1;
                        v[3] *= -1;
                    }
                }
            }
            // ---------- 圆柱 ----------
            else if (v.size() == 7) {
                Eigen::Vector3f axis(v[3], v[4], v[5]);
                Eigen::Vector3f p(v[0], v[1], v[2]);  // 轴上一点

                if (axis.norm() > EPS) {
                    axis.normalize();

                    // 让轴方向尽量朝向相机
                    Eigen::Vector3f toCam = -p;

                    if (axis.dot(toCam) < 0) {
                        axis = -axis;
                    }

                    v[3] = axis.x();
                    v[4] = axis.y();
                    v[5] = axis.z();
                }
            }
        }

        // ================= otherSurface =================
        for (auto& surf : info->otherSurface) {
            if (!surf) continue;

            auto& v = surf->values;

            // ---------- 平面 ----------
            if (v.size() == 4) {
                Eigen::Vector3f n(v[0], v[1], v[2]);
                float d = v[3];

                if (n.norm() > EPS) {
                    Eigen::Vector3f p = -d * n / n.squaredNorm();
                    Eigen::Vector3f toCam = -p;

                    if (n.dot(toCam) < 0) {
                        v[0] *= -1;
                        v[1] *= -1;
                        v[2] *= -1;
                        v[3] *= -1;
                    }
                }
            }
            // ---------- 圆柱 ----------
            else if (v.size() == 7) {
                Eigen::Vector3f axis(v[3], v[4], v[5]);
                Eigen::Vector3f p(v[0], v[1], v[2]);

                if (axis.norm() > EPS) {
                    axis.normalize();

                    Eigen::Vector3f toCam = -p;

                    if (axis.dot(toCam) < 0) {
                        axis = -axis;
                    }

                    v[3] = axis.x();
                    v[4] = axis.y();
                    v[5] = axis.z();
                }
            }
        }
    }
}
// 将焊缝信息转到机器人基坐标系
void GantrayFrameTrajectoryPlanning::transSeams2Base(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo) {
    // ================= 1. 构建统一变换 =================
    const Eigen::Matrix4f& EyH = trajectoryConfig.matrixEyeHand;
    const Eigen::Matrix4f& EnB = trajectoryConfig.matrixEnd2Base;
    Eigen::Matrix4f T_cam2base = Eigen::Matrix4f::Identity();

    if (trajectoryConfig.handEyeType == MyToolFunc::getHandTypeTypeString(HAND_EYE_TYPE::EYE_IN_HAND)) {
        T_cam2base = EnB * EyH;
    } else {
        T_cam2base = EyH;
    }
    // ================= 2. 批量转换 =================
    for (auto& info : weldSeamInfo) {
        if (!info || !info->detectSuccFlag) continue;
        // ---------- 2.1 焊缝端点 ----------
        if (info->weldEndPointsInCamera && info->weldEndPointsInCamera->size() >= 2) {
            info->weldEndPointsInRobot = std::make_shared<std::vector<pcl::PointXYZ>>();

            info->weldEndPointsInRobot->reserve(info->weldEndPointsInCamera->size());

            for (const auto& pt : *(info->weldEndPointsInCamera)) {
                info->weldEndPointsInRobot->emplace_back(MyToolFunc::transformSinglePoint(pt, T_cam2base));
            }
        }
        // ---------- 2.2 （异步）焊缝区域点云 ----------

        if (info->weldAreaPointCloudInCamera && !info->weldAreaPointCloudInCamera->empty()) {
            auto cloud_in = info->weldAreaPointCloudInCamera;

            info->cloudFuture = QtConcurrent::run([cloud_in, T_cam2base]() { return MyToolFunc::transformPointCloud(cloud_in, T_cam2base); });
        }
        // ---------- 2.2 转换焊缝母材系数 -----------
        pcl::ModelCoefficients::Ptr plane_base_trans;
        pcl::ModelCoefficients::Ptr cylinder_base_trans;
        if (info->weldCoeff) {
            if (info->weldCoeff->values.size() == 4) {
                plane_base_trans = MyToolFunc::transformPlane(info->weldCoeff, T_cam2base);
                info->weldCoeff = plane_base_trans;
            } else if (info->weldCoeff->values.size() == 7) {
                cylinder_base_trans = MyToolFunc::transformCylinder(info->weldCoeff, T_cam2base);
                info->weldCoeff = cylinder_base_trans;
            } else {
                PLOGD << "转换失败，焊缝母材系数数量错误 ";
            }
        }
        for (auto& surface : info->otherSurface) {
            if (!surface) {
                PLOGE << "不存在其他焊缝母材系数";
                continue;
            }
            if (surface->values.size() == 4) {
                surface = MyToolFunc::transformPlane(surface, T_cam2base);
            } else if (surface->values.size() == 7) {
                surface = MyToolFunc::transformCylinder(surface, T_cam2base);
            } else {
                PLOGD << "转换失败，其他母材系数数量错误 ";
            }
        }
    }
}
// 判断工件位于机器人的方位
void GantrayFrameTrajectoryPlanning::determineWorkpieceOri(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo) {
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
void GantrayFrameTrajectoryPlanning::transSeamsOri(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo) {
    // ===== 参考点（机器人当前位置）=====
    Eigen::Vector3f ref(trajectoryConfig.matrixEnd2Base(0, 3), trajectoryConfig.matrixEnd2Base(1, 3), trajectoryConfig.matrixEnd2Base(2, 3));
    if (weldSeamInfo.empty()) return;
    // Eigen::Vector3f ref(0.0f, 0.0f, 0.0f);  // 基座
    for (auto& info : weldSeamInfo) {
        if (!info || !info->detectSuccFlag) continue;
        if (info->weldAreaType == TubeSide_Plate_F) {
            // 离ref近的为起点
            if (!info->weldEndPointsInRobot || info->weldEndPointsInRobot->size() != 2) continue;

            auto& pts = *(info->weldEndPointsInRobot);

            Eigen::Vector3f P0(pts[0].x, pts[0].y, pts[0].z);
            Eigen::Vector3f P1(pts[1].x, pts[1].y, pts[1].z);

            float d0 = (P0 - ref).squaredNorm();
            float d1 = (P1 - ref).squaredNorm();

            // 如果P0更远 → 交换
            if (d0 > d1) {
                std::swap(pts[0], pts[1]);
            }
        } else if (info->weldAreaType == Tube_Plate_F || info->weldAreaType == Tube_Tube_F) {
            if (!info->weldEndPointsInRobot || info->weldEndPointsInRobot->size() < 2) continue;

            auto& pts = *(info->weldEndPointsInRobot);

            const auto& start = pts.front();
            const auto& end = pts.back();

            // TODO 如果是倒装会不一样，目前Z大的应该是起点
            if (start.z < end.z) {
                std::reverse(pts.begin(), pts.end());
            }
        }
    }
    if (std::any_of(weldSeamInfo.begin(), weldSeamInfo.end(),
                    [](const std::shared_ptr<WeldSeamInfo>& s) { return s && s->weldAreaType == Plate_Plate_F; })) {
        planPlatePlateFilletSeamOrientation(weldSeamInfo);
    }
}
void GantrayFrameTrajectoryPlanning::generateWeldPose(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo) {
    for (auto& info : weldSeamInfo) {
        Eigen::Vector3f P0;
        Eigen::Vector3f P1;
        double a;
        double b;
        double c;
        robotPose pose_start, pose_end;
        if (!info || !info->detectSuccFlag) continue;

        if (!info->weldEndPointsInRobot || info->weldEndPointsInRobot->size() < 2) continue;
        if (info->weldType == TubeSide_Plate_F_H) {
            if (!info->weldCoeff || info->weldCoeff->values.size() != 4) continue;

            if (info->otherSurface.empty()) continue;

            // ================= 1. 取起点终点 =================
            P0 = Eigen::Vector3f(info->weldEndPointsInRobot->at(0).x, info->weldEndPointsInRobot->at(0).y, info->weldEndPointsInRobot->at(0).z);

            P1 = Eigen::Vector3f(info->weldEndPointsInRobot->at(1).x, info->weldEndPointsInRobot->at(1).y, info->weldEndPointsInRobot->at(1).z);

            Eigen::Vector3f mid = 0.5f * (P0 + P1);

            // ================= 2. 平面法向 =================
            Eigen::Vector3f n1(info->weldCoeff->values[0], info->weldCoeff->values[1], info->weldCoeff->values[2]);
            n1.normalize();

            // ================= 3. 圆柱法向 =================
            Eigen::Vector3f n2(0, 0, 0);

            for (auto& surf : info->otherSurface) {
                if (!surf || surf->values.size() != 7) continue;

                Eigen::Vector3f axis(surf->values[3], surf->values[4], surf->values[5]);
                axis.normalize();

                Eigen::Vector3f pointOnAxis(surf->values[0], surf->values[1], surf->values[2]);

                Eigen::Vector3f v = mid - pointOnAxis;
                n2 = v - v.dot(axis) * axis;

                if (n2.norm() > 1e-6) {
                    n2.normalize();
                    break;
                }
            }

            if (n2.norm() < 1e-6) n2 = n1;

            // ================= 4. Z轴 =================
            Eigen::Vector3f Z = (tubeSidePlateFilletPlanePoseW * n1 + (1.0f - tubeSidePlateFilletPlanePoseW) * n2).normalized();

            if (Z.dot(Eigen::Vector3f(0, 0, 1)) > 0) Z = -Z;

            // ================= 5. Y轴（焊缝方向） =================
            Eigen::Vector3f dir = (P1 - P0).normalized();

            Eigen::Vector3f Y = dir - dir.dot(Z) * Z;

            if (Y.norm() < 1e-6) {
                Eigen::Vector3f fallback(1, 0, 0);
                if (fabs(fallback.dot(Z)) > 0.9) fallback = Eigen::Vector3f(0, 1, 0);

                Y = fallback - fallback.dot(Z) * Z;
            }

            Y.normalize();

            if (Y.dot(Eigen::Vector3f(0, 1, 0)) > 0) Y = -Y;

            // ================= 6. X轴 =================
            Eigen::Vector3f X = Y.cross(Z).normalized();

            if (X.dot(Eigen::Vector3f(1, 0, 0)) < 0) {
                X = -X;
                Y = -Y;
            }

            // ================= 7. 重正交 =================
            Z = X.cross(Y).normalized();

            // ================= 8. 旋转矩阵 =================
            Eigen::Matrix3f R_ref;
            R_ref.col(0) = X;
            R_ref.col(1) = Y;
            R_ref.col(2) = Z;
            std::vector<double> currentABC = {trajectoryConfig.currentRobotPose.a_, trajectoryConfig.currentRobotPose.b_,
                                              trajectoryConfig.currentRobotPose.c_};
            std::vector<double> targetABC = MyToolFunc::extractEulerZYX(R_ref, currentABC);

            a = targetABC[0];
            b = targetABC[1];
            c = targetABC[2];
            // 起点
            pose_start.x_ = P0.x();
            pose_start.y_ = P0.y();
            pose_start.z_ = P0.z();
            pose_start.a_ = a;
            pose_start.b_ = b;
            pose_start.c_ = c;

            // 终点
            pose_end.x_ = P1.x();
            pose_end.y_ = P1.y();
            pose_end.z_ = P1.z();
            pose_end.a_ = a;
            pose_end.b_ = b;
            pose_end.c_ = c;
            info->robotWeldPose.clear();
            info->robotWeldPose.push_back(pose_start);
            info->robotWeldPose.push_back(pose_end);
        } else if (info->weldType == Plate_Plate_Fillet_H) {
            if (info->otherSurface.size() < 1) continue;

            // ===== 1. 端点 =====
            P0 = Eigen::Vector3f(info->weldEndPointsInRobot->at(0).x, info->weldEndPointsInRobot->at(0).y, info->weldEndPointsInRobot->at(0).z);

            P1 = Eigen::Vector3f(info->weldEndPointsInRobot->at(1).x, info->weldEndPointsInRobot->at(1).y, info->weldEndPointsInRobot->at(1).z);

            // ===== 2. 两个平面法向 =====
            Eigen::Vector3f n1(info->weldCoeff->values[0], info->weldCoeff->values[1], info->weldCoeff->values[2]);
            n1.normalize();

            Eigen::Vector3f n2(0, 0, 0);
            for (auto& surf : info->otherSurface) {
                if (!surf || surf->values.size() != 4) continue;

                n2 = Eigen::Vector3f(surf->values[0], surf->values[1], surf->values[2]);
                n2.normalize();
                break;
            }

            if (n2.norm() < 1e-6) n2 = n1;

            // ===== 3. Z轴（角平分方向）=====
            Eigen::Vector3f Z = (platePlateFilletPlanePoseW_H * n1 + (1.0f - platePlateFilletPlanePoseW_H) * n2).normalized();

            // 朝下
            if (Z.dot(Eigen::Vector3f(0, 0, 1)) > 0) Z = -Z;

            // ===== 4. Y轴（焊缝方向）=====
            Eigen::Vector3f dir = (P1 - P0).normalized();

            Eigen::Vector3f Y = dir - dir.dot(Z) * Z;

            if (Y.norm() < 1e-6) {
                Eigen::Vector3f fallback(1, 0, 0);
                if (fabs(fallback.dot(Z)) > 0.9) fallback = Eigen::Vector3f(0, 1, 0);

                Y = fallback - fallback.dot(Z) * Z;
            }

            Y.normalize();

            // Y反向
            if (Y.dot(Eigen::Vector3f(0, 1, 0)) > 0) Y = -Y;

            // ===== 5. X轴 =====
            Eigen::Vector3f X = Y.cross(Z).normalized();

            // X正向
            if (X.dot(Eigen::Vector3f(1, 0, 0)) < 0) {
                X = -X;
                Y = -Y;
            }

            // ===== 6. 重正交 =====
            Z = X.cross(Y).normalized();

            // ===== 当前姿态（用于欧拉解算连续性）=====
            std::vector<double> currentABC = {trajectoryConfig.currentRobotPose.a_, trajectoryConfig.currentRobotPose.b_,
                                              trajectoryConfig.currentRobotPose.c_};

            // ===== 倾斜角 =====
            float theta = platePlateFillettiltW_H * 45.0f * M_PI / 180.0f;

            // ================= 起点姿态 =================
            {
                float t = +theta;

                Eigen::Vector3f Y_s = std::cos(t) * Y + std::sin(t) * Z;
                Eigen::Vector3f Z_s = -std::sin(t) * Y + std::cos(t) * Z;

                Y_s.normalize();
                Z_s.normalize();

                Eigen::Vector3f X_s = Y_s.cross(Z_s).normalized();
                Z_s = X_s.cross(Y_s).normalized();

                Eigen::Matrix3f R;
                R.col(0) = X_s;
                R.col(1) = Y_s;
                R.col(2) = Z_s;

                auto abc = MyToolFunc::extractEulerZYX(R, currentABC);

                pose_start.x_ = P0.x();
                pose_start.y_ = P0.y();
                pose_start.z_ = P0.z();
                pose_start.a_ = abc[0];
                pose_start.b_ = abc[1];
                pose_start.c_ = abc[2];
            }

            // ================= 终点姿态 =================
            {
                float t = -theta;

                Eigen::Vector3f Y_e = std::cos(t) * Y + std::sin(t) * Z;
                Eigen::Vector3f Z_e = -std::sin(t) * Y + std::cos(t) * Z;

                Y_e.normalize();
                Z_e.normalize();

                Eigen::Vector3f X_e = Y_e.cross(Z_e).normalized();
                Z_e = X_e.cross(Y_e).normalized();

                Eigen::Matrix3f R;
                R.col(0) = X_e;
                R.col(1) = Y_e;
                R.col(2) = Z_e;

                auto abc = MyToolFunc::extractEulerZYX(R, currentABC);

                pose_end.x_ = P1.x();
                pose_end.y_ = P1.y();
                pose_end.z_ = P1.z();
                pose_end.a_ = abc[0];
                pose_end.b_ = abc[1];
                pose_end.c_ = abc[2];
            }
            info->robotWeldPose.clear();
            info->robotWeldPose.push_back(pose_start);
            info->robotWeldPose.push_back(pose_end);
        } else if (info->weldType == Plate_Plate_Fillet_V) {
            if (!info->weldCoeff || info->weldCoeff->values.size() != 4) continue;
            if (info->otherSurface.empty()) continue;

            //  1. 端点
            P0 = Eigen::Vector3f(info->weldEndPointsInRobot->at(0).x, info->weldEndPointsInRobot->at(0).y, info->weldEndPointsInRobot->at(0).z);

            P1 = Eigen::Vector3f(info->weldEndPointsInRobot->at(1).x, info->weldEndPointsInRobot->at(1).y, info->weldEndPointsInRobot->at(1).z);

            Eigen::Vector3f seamDir = (P0 - P1).normalized();

            //  2. 主平面法向
            Eigen::Vector3f n1(info->weldCoeff->values[0], info->weldCoeff->values[1], info->weldCoeff->values[2]);
            n1.normalize();

            //  3. otherSurface 法向
            Eigen::Vector3f n2(0, 0, 0);
            for (auto& surf : info->otherSurface) {
                if (!surf || surf->values.size() != 4) continue;

                n2 = Eigen::Vector3f(surf->values[0], surf->values[1], surf->values[2]);
                n2.normalize();
                break;
            }
            // PLOGD << "区域" << info->areaNum << "第一平面向量系数为" << n1;
            // PLOGD << "区域" << info->areaNum << "第二平面向量系数为:" << n2;
            // PLOGD << "区域" << info->areaNum << "焊缝向量系数为:" << seamDir;
            if (n2.norm() < 1e-6) n2 = n1;

            //  4. 第一层融合（平面角平分）

            Eigen::Vector3f N_mid = (platePlateFilletPlanePoseW_V * n1 + (1.0f - platePlateFilletPlanePoseW_V) * n2).normalized();

            //  5. 第二层融合（加入焊缝方向）

            Eigen::Vector3f Z = (platePlateFilletWeldPoseW_V * seamDir + (1.0f - platePlateFilletWeldPoseW_V) * N_mid).normalized();

            //  6. Z方向约束（朝下）
            if (Z.dot(Eigen::Vector3f(0, 0, 1)) > 0) Z = -Z;

            //  7. Y轴（用立板方向 n2）
            Eigen::Vector3f Y = n2;

            // 投影到垂直于Z
            Y = Y - Y.dot(Z) * Z;

            if (Y.norm() < 1e-6) {
                Y = seamDir - seamDir.dot(Z) * Z;
            }

            Y.normalize();

            // Y与机器人基座坐标系反向
            if (Y.dot(Eigen::Vector3f(0, 1, 0)) > 0) Y = -Y;

            //  8. X轴
            Eigen::Vector3f X = Y.cross(Z).normalized();

            // X正向
            if (X.dot(Eigen::Vector3f(1, 0, 0)) < 0) {
                X = -X;
                Y = -Y;
            }

            //  9. 重正交
            Z = X.cross(Y).normalized();

            //  10. 旋转矩阵
            Eigen::Matrix3f R_ref;
            R_ref.col(0) = X;
            R_ref.col(1) = Y;
            R_ref.col(2) = Z;

            std::vector<double> currentABC = {trajectoryConfig.currentRobotPose.a_, trajectoryConfig.currentRobotPose.b_,
                                              trajectoryConfig.currentRobotPose.c_};

            std::vector<double> targetABC = MyToolFunc::extractEulerZYX(R_ref, currentABC);

            a = targetABC[0];
            b = targetABC[1];
            c = targetABC[2];
            // 起点
            pose_start.x_ = P0.x();
            pose_start.y_ = P0.y();
            pose_start.z_ = P0.z();
            pose_start.a_ = a;
            pose_start.b_ = b;
            pose_start.c_ = c;

            // 终点
            pose_end.x_ = P1.x();
            pose_end.y_ = P1.y();
            pose_end.z_ = P1.z();
            pose_end.a_ = a;
            pose_end.b_ = b;
            pose_end.c_ = c;
            info->robotWeldPose.clear();
            info->robotWeldPose.push_back(pose_start);
            info->robotWeldPose.push_back(pose_end);
        } else if (info->weldType == Tube_Plate_Fillet) {
            if (!info->weldCoeff || info->weldEndPointsInRobot->empty() || info->otherSurface.size() == 0) continue;

            int N = info->weldEndPointsInRobot->size();

            // ===== 平面法向 =====
            Eigen::Vector3f n_plane(info->otherSurface[0]->values[0], info->otherSurface[0]->values[1], info->otherSurface[0]->values[2]);
            n_plane.normalize();
            // // ===== 强制指向世界 -X =====
            // if (n_plane.dot(Eigen::Vector3f(-1, 0, 0)) < 0) n_plane = -n_plane;
            info->robotWeldPose.clear();

            for (int i = 0; i < N; i++) {
                // ===== 当前点 =====
                Eigen::Vector3f P(info->weldEndPointsInRobot->at(i).x, info->weldEndPointsInRobot->at(i).y, info->weldEndPointsInRobot->at(i).z);

                // ===== 1. 切向 t =====
                Eigen::Vector3f t;
                if (i < N - 1) {
                    t = Eigen::Vector3f(info->weldEndPointsInRobot->at(i + 1).x, info->weldEndPointsInRobot->at(i + 1).y,
                                        info->weldEndPointsInRobot->at(i + 1).z) -
                        P;
                } else {
                    t = P - Eigen::Vector3f(info->weldEndPointsInRobot->at(i - 1).x, info->weldEndPointsInRobot->at(i - 1).y,
                                            info->weldEndPointsInRobot->at(i - 1).z);
                }
                t.normalize();

                // ===== 2. 圆柱法向（展开写）=====
                Eigen::Vector3f n_cyl(0, 0, 1);  // 默认值，防崩

                if (!info->weldCoeff || info->weldCoeff->values.size() < 6) {
                    // fallback：没有圆柱参数，直接用平面法向
                    n_cyl = n_plane;
                } else {
                    // 圆柱轴上一点 C
                    Eigen::Vector3f C(info->weldCoeff->values[0], info->weldCoeff->values[1], info->weldCoeff->values[2]);

                    // 圆柱轴方向 axis
                    Eigen::Vector3f axis(info->weldCoeff->values[3], info->weldCoeff->values[4], info->weldCoeff->values[5]);

                    if (axis.norm() < 1e-6) {
                        n_cyl = n_plane;  // fallback
                    } else {
                        axis.normalize();

                        // ===== 关键：点到轴的径向向量 =====
                        Eigen::Vector3f CP = P - C;

                        // 投影到轴上
                        Eigen::Vector3f proj = CP.dot(axis) * axis;

                        // 去掉轴向分量 → 得到径向
                        n_cyl = CP - proj;

                        if (n_cyl.norm() < 1e-6) {
                            n_cyl = n_plane;  // 极端情况 fallback
                        } else {
                            n_cyl.normalize();
                        }
                    }
                }

                // ===== 3. 第一层角平分（平面 + 圆柱）=====
                Eigen::Vector3f N_mid = (tubePlateFilletPlanePoseW * n_plane + (1.0f - tubePlateFilletPlanePoseW) * n_cyl).normalized();
                // ===== 4. 第二层（先不参与）=====
                Eigen::Vector3f Z = (tubePlateFilletWeldPoseW * N_mid + (1.0f - tubePlateFilletWeldPoseW) * t).normalized();

                // ===== 5. Z轴约束：必须向下 =====
                // if (Z.dot(Eigen::Vector3f(0, 0, 1)) > 0) Z = -Z;

                Eigen::Vector3f worldZ(0, 0, 1);

                // 当前焊点位于圆柱上/下半圆
                float hemi = n_cyl.dot(worldZ);
                // 正常情况下，在规定了母材方向后，这里就不用0, 0, 1硬约束了，直接反向径向就行；
                if (hemi > 0.0f) {
                    // 上半圆：焊枪朝下
                    if (Z.dot(worldZ) > 0) Z = -Z;
                } else if (hemi < 0.0f) {
                    // 下半圆：焊枪朝上
                    if (Z.dot(worldZ) < 0) Z = -Z;
                } else {
                    // 与径向相反（朝向圆柱）
                    if (Z.dot(n_cyl) > 0) Z = -Z;
                }

                // ===== 6. Y轴：沿切向，但与世界Y反向 =====
                Eigen::Vector3f Y = t;

                // 去掉Z分量（保证垂直）
                Y = Y - Y.dot(Z) * Z;

                if (Y.norm() < 1e-6) Y = Eigen::Vector3f(0, -1, 0);  // fallback

                Y.normalize();

                // 强制与世界Y反向
                if (Y.dot(Eigen::Vector3f(0, 1, 0)) > 0) Y = -Y;

                // ===== 7. X轴：强制与世界X同向 =====
                Eigen::Vector3f X = Y.cross(Z).normalized();

                // 强制X与世界X同向
                if (X.dot(Eigen::Vector3f(1, 0, 0)) < 0) {
                    X = -X;
                    Y = -Y;  // 保持右手系
                }

                // ===== 8. 重正交（防止数值误差）=====
                Z = X.cross(Y).normalized();

                // ===== 9. 构造旋转矩阵 =====
                Eigen::Matrix3f R;
                R.col(0) = X;
                R.col(1) = Y;
                R.col(2) = Z;

                // ===== 10. 转欧拉角 =====
                std::vector<double> currentABC = {trajectoryConfig.currentRobotPose.a_, trajectoryConfig.currentRobotPose.b_,
                                                  trajectoryConfig.currentRobotPose.c_};

                std::vector<double> targetABC = MyToolFunc::extractEulerZYX(R, currentABC);

                // ===== 11. 写入pose =====
                robotPose pose;
                pose.x_ = P.x();
                pose.y_ = P.y();
                pose.z_ = P.z();
                pose.a_ = targetABC[0];
                pose.b_ = targetABC[1];
                pose.c_ = targetABC[2];

                info->robotWeldPose.push_back(pose);
                // // ===== 打印平面母材 =====
                // PLOGI << "====== Plane (母材1) ======";
                // if (!info->otherSurface.empty() && info->otherSurface[0]->values.size() >= 4) {
                //     auto& plane = info->otherSurface[0]->values;
                //     PLOGI << "Plane coeff: A=" << plane[0] << " B=" << plane[1] << " C=" << plane[2] << " D=" << plane[3];
                // } else {
                //     PLOGW << "Plane coeff invalid!";
                // }

                // // ===== 打印圆柱母材 =====
                // PLOGI << "====== Cylinder (母材2) ======";

                // if (info->weldCoeff && info->weldCoeff->values.size() >= 6) {
                //     auto& cyl = info->weldCoeff->values;

                //     PLOGI << "Axis point: (" << cyl[0] << ", " << cyl[1] << ", " << cyl[2] << ")";
                //     PLOGI << "Axis dir  : (" << cyl[3] << ", " << cyl[4] << ", " << cyl[5] << ")";
                //     PLOGI << "Axis R :(" << cyl[6] << ")";
                //     ;
                // } else {
                //     PLOGW << "Cylinder coeff invalid!";
                // }
                // PLOGI << "------ Weld Point [" << i << "] ------";
                // PLOGI << "Position: (" << info->weldEndPointsInRobot->at(i).x << ", " << info->weldEndPointsInRobot->at(i).y << ", "
                //       << info->weldEndPointsInRobot->at(i).z << ")";
            }
        } else if (info->weldType == Tube_Tube_Fillet) {
            if (!info->weldCoeff || info->weldCoeff->values.size() < 6) continue;
            if (!info->weldEndPointsInRobot || info->weldEndPointsInRobot->size() < 2) continue;
            if (info->otherSurface.empty()) continue;

            int N = static_cast<int>(info->weldEndPointsInRobot->size());
            info->robotWeldPose.clear();

            // 圆柱1：默认使用 weldCoeff
            Eigen::Vector3f C1(info->weldCoeff->values[0], info->weldCoeff->values[1], info->weldCoeff->values[2]);

            Eigen::Vector3f axis1(info->weldCoeff->values[3], info->weldCoeff->values[4], info->weldCoeff->values[5]);

            if (axis1.norm() < 1e-6f) {
                PLOGE << "Tube_Tube_Fillet: cylinder1 axis invalid, area=" << info->areaNum;
                continue;
            }
            axis1.normalize();

            // 圆柱2：默认从 otherSurface 里找 size == 7 的圆柱
            bool hasCylinder2 = false;
            Eigen::Vector3f C2(0, 0, 0);
            Eigen::Vector3f axis2(0, 0, 1);

            for (auto& surf : info->otherSurface) {
                if (!surf || surf->values.size() < 6) continue;

                C2 = Eigen::Vector3f(surf->values[0], surf->values[1], surf->values[2]);

                axis2 = Eigen::Vector3f(surf->values[3], surf->values[4], surf->values[5]);

                if (axis2.norm() > 1e-6f) {
                    axis2.normalize();
                    hasCylinder2 = true;
                    break;
                }
            }

            if (!hasCylinder2) {
                PLOGE << "Tube_Tube_Fillet: cylinder2 not found, area=" << info->areaNum;
                continue;
            }

            // 权重保护
            // tubeTubeFilletCylinderPoseW 越大越偏第二圆柱
            // tubeTubeFilletWeldPoseW = 1 不向焊缝方向偏，越小越向 t 偏
            float cylW = tubeTubeFilletCylinderPoseW;
            if (cylW < 0.0f) cylW = 0.0f;
            if (cylW > 1.0f) cylW = 1.0f;

            float weldW = tubeTubeFilletWeldPoseW;
            if (weldW < 0.0f) weldW = 0.0f;
            if (weldW > 1.0f) weldW = 1.0f;

            for (int i = 0; i < N; ++i) {
                // 1. 当前焊点 P

                Eigen::Vector3f P(info->weldEndPointsInRobot->at(i).x, info->weldEndPointsInRobot->at(i).y, info->weldEndPointsInRobot->at(i).z);

                // 2. 焊缝切向 t

                Eigen::Vector3f t(1, 0, 0);

                if (N == 2) {
                    Eigen::Vector3f P0(info->weldEndPointsInRobot->at(0).x, info->weldEndPointsInRobot->at(0).y, info->weldEndPointsInRobot->at(0).z);

                    Eigen::Vector3f P1(info->weldEndPointsInRobot->at(1).x, info->weldEndPointsInRobot->at(1).y, info->weldEndPointsInRobot->at(1).z);

                    t = P1 - P0;
                } else {
                    if (i == 0) {
                        Eigen::Vector3f P_next(info->weldEndPointsInRobot->at(i + 1).x, info->weldEndPointsInRobot->at(i + 1).y,
                                               info->weldEndPointsInRobot->at(i + 1).z);

                        t = P_next - P;
                    } else if (i == N - 1) {
                        Eigen::Vector3f P_prev(info->weldEndPointsInRobot->at(i - 1).x, info->weldEndPointsInRobot->at(i - 1).y,
                                               info->weldEndPointsInRobot->at(i - 1).z);

                        t = P - P_prev;
                    } else {
                        Eigen::Vector3f P_prev(info->weldEndPointsInRobot->at(i - 1).x, info->weldEndPointsInRobot->at(i - 1).y,
                                               info->weldEndPointsInRobot->at(i - 1).z);

                        Eigen::Vector3f P_next(info->weldEndPointsInRobot->at(i + 1).x, info->weldEndPointsInRobot->at(i + 1).y,
                                               info->weldEndPointsInRobot->at(i + 1).z);

                        t = P_next - P_prev;
                    }
                }

                if (t.norm() < 1e-6f) {
                    PLOGE << "Tube_Tube_Fillet: tangent invalid, area=" << info->areaNum << ", idx=" << i;
                    continue;
                }
                t.normalize();

                // 3. 圆柱1径向向量 n1
                //    n = P - axisProjection(P)

                Eigen::Vector3f CP1 = P - C1;
                Eigen::Vector3f n1 = CP1 - CP1.dot(axis1) * axis1;

                if (n1.norm() < 1e-6f) {
                    PLOGE << "Tube_Tube_Fillet: radial1 invalid, area=" << info->areaNum << ", idx=" << i;
                    continue;
                }
                n1.normalize();

                // 4. 圆柱2径向向量 n2

                Eigen::Vector3f CP2 = P - C2;
                Eigen::Vector3f n2 = CP2 - CP2.dot(axis2) * axis2;

                if (n2.norm() < 1e-6f) {
                    PLOGE << "Tube_Tube_Fillet: radial2 invalid, area=" << info->areaNum << ", idx=" << i;
                    continue;
                }
                n2.normalize();

                // 5. 两个圆柱径向方向融合
                //    cylW = 0.5：角平分
                //    cylW 越大：越偏向第二圆柱径向 n2

                Eigen::Vector3f N_mid = ((1.0f - cylW) * n1 + cylW * n2);

                // 如果两个径向几乎反向，直接相加可能接近 0，此时退回到 n1 或 n2
                if (N_mid.norm() < 1e-6f) {
                    if (cylW >= 0.5f) {
                        N_mid = n2;
                    } else {
                        N_mid = n1;
                    }
                } else {
                    N_mid.normalize();
                }

                // 6. 加入焊缝方向偏移
                //    weldW = 1：不向 t 偏，完全使用 N_mid
                //    weldW 越小：越向焊缝方向 t 偏

                Eigen::Vector3f Z = weldW * N_mid + (1.0f - weldW) * t;

                if (Z.norm() < 1e-6f) {
                    Z = N_mid;
                } else {
                    Z.normalize();
                }

                // 7. Z方向约束
                // 7. Z方向约束：参考圆柱上/下半圆
                Eigen::Vector3f worldZ(0, 0, 1);

                // 用融合后的径向方向判断当前焊点处于整体上/下半圆
                float hemi = N_mid.dot(worldZ);

                if (hemi > 0.0f) {
                    // 上半圆：焊枪朝下
                    if (Z.dot(worldZ) > 0.0f) {
                        Z = -Z;
                    }
                } else if (hemi < 0.0f) {
                    // 下半圆：焊枪朝上
                    if (Z.dot(worldZ) < 0.0f) {
                        Z = -Z;
                    }
                } else {
                    // 一般状况都是：让Z与融合径向相反，朝向两个圆柱夹角内部
                    if (Z.dot(N_mid) > 0.0f) {
                        Z = -Z;
                    }
                }

                // 8. Y轴：沿焊缝切向，并投影到垂直于 Z 的平面

                Eigen::Vector3f Y = t - t.dot(Z) * Z;

                if (Y.norm() < 1e-6f) {
                    // fallback：用圆柱1轴向
                    Y = axis1 - axis1.dot(Z) * Z;
                }

                if (Y.norm() < 1e-6f) {
                    // 再 fallback：用圆柱2轴向
                    Y = axis2 - axis2.dot(Z) * Z;
                }

                if (Y.norm() < 1e-6f) {
                    Y = Eigen::Vector3f(0, -1, 0);
                    Y = Y - Y.dot(Z) * Z;
                }

                Y.normalize();

                // TODO Y 与世界 Y 反向
                if (Y.dot(Eigen::Vector3f(0, 1, 0)) > 0.0f) {
                    Y = -Y;
                }

                // 9. X轴：右手系

                Eigen::Vector3f X = Y.cross(Z);

                if (X.norm() < 1e-6f) {
                    PLOGE << "Tube_Tube_Fillet: X invalid, area=" << info->areaNum << ", idx=" << i;
                    continue;
                }
                X.normalize();

                // 保持和现有逻辑一致：X 尽量与世界 X 同向
                if (X.dot(Eigen::Vector3f(1, 0, 0)) < 0.0f) {
                    X = -X;
                    Y = -Y;
                }

                // 10. 重正交

                Z = X.cross(Y);

                if (Z.norm() < 1e-6f) {
                    PLOGE << "Tube_Tube_Fillet: Z re-orthogonal invalid, area=" << info->areaNum << ", idx=" << i;
                    continue;
                }
                Z.normalize();

                // 11. 构造旋转矩阵

                Eigen::Matrix3f R;
                R.col(0) = X;
                R.col(1) = Y;
                R.col(2) = Z;

                std::vector<double> currentABC = {trajectoryConfig.currentRobotPose.a_, trajectoryConfig.currentRobotPose.b_,
                                                  trajectoryConfig.currentRobotPose.c_};

                std::vector<double> targetABC = MyToolFunc::extractEulerZYX(R, currentABC);

                // 12. 写入机器人姿态

                robotPose pose;
                pose.x_ = P.x();
                pose.y_ = P.y();
                pose.z_ = P.z();
                pose.a_ = targetABC[0];
                pose.b_ = targetABC[1];
                pose.c_ = targetABC[2];

                info->robotWeldPose.push_back(pose);
            }
            PLOGD << "========== Tube_Tube_Fillet 母材数据 ==========";
            PLOGD << "areaNum = " << info->areaNum;

            PLOGD << "圆柱1 weldCoeff: " << "C=(" << C1.x() << ", " << C1.y() << ", " << C1.z() << "), " << "axis=(" << axis1.x() << ", " << axis1.y()
                  << ", " << axis1.z() << "), " << "R=" << info->weldCoeff->values[6];

            PLOGD << "圆柱2 otherSurface: " << "C=(" << C2.x() << ", " << C2.y() << ", " << C2.z() << "), " << "axis=(" << axis2.x() << ", "
                  << axis2.y() << ", " << axis2.z() << "), " << "R=" << (info->otherSurface.empty() ? -1.0f : info->otherSurface[0]->values[6]);
        }
    }
}

// 对所有焊缝进行后撤
void GantrayFrameTrajectoryPlanning::applyWeldGunWithdraw(robotPose& pose, double withdrawDistance) {
    // PLOGD << "withdrawDistance" << withdrawDistance;
    // 1. 计算方向
    Eigen::Vector3d dir = abcToDirection(pose.a_, pose.b_, pose.c_);

    // 2. 后撤
    Eigen::Vector3d offset = -dir * withdrawDistance;

    // 3. 更新
    pose.x_ += offset.x();
    pose.y_ += offset.y();
    pose.z_ += offset.z();
    // PLOGD << "================焊缝后撤 ================";
    // PLOGD << "x=" << pose.x_ << ", y=" << pose.y_ << ", z=" << pose.z_ << ", a=" << pose.a_ << ", b=" << pose.b_
    //       << ", c=" << pose.c_;
}
Eigen::Vector3d GantrayFrameTrajectoryPlanning::abcToDirection(double a, double b, double c) {
    // 角度转弧度
    double ra = a * M_PI / 180.0;
    double rb = b * M_PI / 180.0;
    double rc = c * M_PI / 180.0;

    // ZYX（工业机器人常见：yaw-pitch-roll）
    Eigen::AngleAxisd Rx(ra, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd Ry(rb, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rz(rc, Eigen::Vector3d::UnitZ());

    Eigen::Matrix3d R = (Rz * Ry * Rx).toRotationMatrix();

    // 工具坐标系 Z 方向（焊枪方向）
    Eigen::Vector3d toolZ(0, 0, 1);

    Eigen::Vector3d dir = R * toolZ;

    return dir.normalized();
}
void GantrayFrameTrajectoryPlanning::compensateSeams(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo) {
    for (auto& info : weldSeamInfo) {
        if (!info || !info->detectSuccFlag) continue;
        if (info->weldType == TubeSide_Plate_F_H) {
            if (!info->weldEndPointsInRobot || info->weldEndPointsInRobot->size() != 2) continue;
            if (!info->weldCoeff || info->weldCoeff->values.size() < 4) continue;

            auto& pts = *(info->weldEndPointsInRobot);

            Eigen::Vector3f P0(pts[0].x, pts[0].y, pts[0].z);
            Eigen::Vector3f P1(pts[1].x, pts[1].y, pts[1].z);

            if ((P1 - P0).norm() < 1e-6) continue;

            /* ================= 坐标系构建 ================= */

            Eigen::Vector3f xAxis = (P1 - P0).normalized();

            Eigen::Vector3f zAxis(info->weldCoeff->values[0], info->weldCoeff->values[1], info->weldCoeff->values[2]);
            zAxis.normalize();

            Eigen::Vector3f yAxis = zAxis.cross(xAxis).normalized();
            zAxis = xAxis.cross(yAxis).normalized();

            Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
            T.block<3, 3>(0, 0).col(0) = xAxis;
            T.block<3, 3>(0, 0).col(1) = yAxis;
            T.block<3, 3>(0, 0).col(2) = zAxis;
            T.block<3, 1>(0, 3) = P0;
            Eigen::Matrix4f T_inv = T.inverse();

            /* ================= 转到工具系 ================= */

            Eigen::Vector4f p0 = T_inv * Eigen::Vector4f(P0.x(), P0.y(), P0.z(), 1.0f);
            Eigen::Vector4f p1 = T_inv * Eigen::Vector4f(P1.x(), P1.y(), P1.z(), 1.0f);

            /* ================= 补偿策略 ================= */
            p0.x() += settingPara.TubeSidePlatFilletStart_X;
            p0.y() += settingPara.TubeSidePlatFilletStart_Y;
            p0.z() += settingPara.TubeSidePlatFilletStart_Z;
            p1.x() += settingPara.TubeSidePlatFilletEnd_X;
            p1.y() += settingPara.TubeSidePlatFilletEnd_Y;
            p1.z() += settingPara.TubeSidePlatFilletEnd_Z;

            /* ================= 转回基座 ================= */

            Eigen::Vector4f p0_new = T * p0;
            Eigen::Vector4f p1_new = T * p1;

            pts[0].x = p0_new.x();
            pts[0].y = p0_new.y();
            pts[0].z = p0_new.z();

            pts[1].x = p1_new.x();
            pts[1].y = p1_new.y();
            pts[1].z = p1_new.z();
        } else if (info->weldType == Plate_Plate_Fillet_H || info->weldType == Plate_Plate_Fillet_V) {
            if (!info->weldEndPointsInRobot || info->weldEndPointsInRobot->size() != 2) continue;
            if (!info->weldCoeff || info->weldCoeff->values.size() < 4) continue;

            auto& pts = *(info->weldEndPointsInRobot);

            Eigen::Vector3f P0(pts[0].x, pts[0].y, pts[0].z);
            Eigen::Vector3f P1(pts[1].x, pts[1].y, pts[1].z);

            if ((P1 - P0).norm() < 1e-6) continue;

            // ===== 焊缝方向 =====
            Eigen::Vector3f seamDir = (P0 - P1).normalized();

            // ===== 主平面法向 =====
            Eigen::Vector3f n1(info->weldCoeff->values[0], info->weldCoeff->values[1], info->weldCoeff->values[2]);
            n1.normalize();

            // ===== 次平面法向 =====
            Eigen::Vector3f n2(0, 0, 0);
            for (auto& surf : info->otherSurface) {
                if (!surf || surf->values.size() != 4) continue;

                n2 = Eigen::Vector3f(surf->values[0], surf->values[1], surf->values[2]);
                n2.normalize();
                break;
            }

            if (n2.norm() < 1e-6) n2 = n1;

            Eigen::Vector3f xAxis, yAxis, zAxis;
            bool debug = false;
            // ================= H型 =================
            if (info->weldType == Plate_Plate_Fillet_H) {
                xAxis = seamDir;
                yAxis = n1;
                zAxis = n2;

                // 正交化（非常重要）
                yAxis = yAxis - yAxis.dot(xAxis) * xAxis;
                yAxis.normalize();

                zAxis = xAxis.cross(yAxis).normalized();
                yAxis = zAxis.cross(xAxis).normalized();
                if (zAxis.dot(n2) < 0) {
                    zAxis = -zAxis;
                    yAxis = -yAxis;  // 保持右手系
                }
                debug = true;
            }

            // ================= V型 =================
            else if (info->weldType == Plate_Plate_Fillet_V) {
                zAxis = seamDir;
                yAxis = n1;
                xAxis = n2;

                // 正交化
                yAxis = yAxis - yAxis.dot(zAxis) * zAxis;
                yAxis.normalize();

                xAxis = yAxis.cross(zAxis).normalized();
                zAxis = xAxis.cross(yAxis).normalized();
            }

            // ===== 构建变换矩阵 =====
            Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
            T.block<3, 3>(0, 0).col(0) = xAxis;
            T.block<3, 3>(0, 0).col(1) = yAxis;
            T.block<3, 3>(0, 0).col(2) = zAxis;
            T.block<3, 1>(0, 3) = P0;

            Eigen::Matrix4f T_inv = T.inverse();

            // ===== 转到工具系 =====
            Eigen::Vector4f p0 = T_inv * Eigen::Vector4f(P0.x(), P0.y(), P0.z(), 1.0f);
            Eigen::Vector4f p1 = T_inv * Eigen::Vector4f(P1.x(), P1.y(), P1.z(), 1.0f);
            // ===== 补偿 =====
            // std::cout << "==== BEFORE ====" << std::endl;
            // std::cout << "p0(local): " << p0.transpose() << std::endl;
            // std::cout << "p1(local): " << p1.transpose() << std::endl;
            // ================= H型 =================
            if (info->weldType == Plate_Plate_Fillet_H) {
                p0.x() += settingPara.PlatePlateFilletHorizontalStart_X;
                p0.y() += settingPara.PlatePlateFilletHorizontalStart_Y;
                p0.z() += settingPara.PlatePlateFilletHorizontalStart_Z;

                p1.x() += settingPara.PlatePlateFilletHorizontalEnd_X;
                p1.y() += settingPara.PlatePlateFilletHorizontalEnd_Y;
                p1.z() += settingPara.PlatePlateFilletHorizontalEnd_Z;
            } else if (info->weldType == Plate_Plate_Fillet_V) {
                p0.x() += settingPara.PlatePlateFilletVerticalStart_X;
                p0.y() += settingPara.PlatePlateFilletVerticalStart_Y;
                p0.z() += settingPara.PlatePlateFilletVerticalStart_Z;

                p1.x() += settingPara.PlatePlateFilletVerticalEnd_X;
                p1.y() += settingPara.PlatePlateFilletVerticalEnd_Y;
                p1.z() += settingPara.PlatePlateFilletVerticalEnd_Z;
            }
            // std::cout << "==== AFTER OFFSET ====" << std::endl;
            // std::cout << "p0(local): " << p0.transpose() << std::endl;
            // std::cout << "p1(local): " << p1.transpose() << std::endl;

            // ===== 转回基座 =====
            Eigen::Vector4f p0_new = T * p0;
            Eigen::Vector4f p1_new = T * p1;
            // std::cout << "==== WORLD ====" << std::endl;
            // std::cout << "p0(world): " << p0_new.transpose() << std::endl;
            // std::cout << "p1(world): " << p1_new.transpose() << std::endl;
            // std::cout << "==== AXIS ====" << std::endl;
            // std::cout << "xAxis: " << xAxis.transpose() << std::endl;
            // std::cout << "yAxis: " << yAxis.transpose() << std::endl;
            // std::cout << "zAxis: " << zAxis.transpose() << std::endl;

            pts[0].x = p0_new.x();
            pts[0].y = p0_new.y();
            pts[0].z = p0_new.z();

            pts[1].x = p1_new.x();
            pts[1].y = p1_new.y();
            pts[1].z = p1_new.z();
        } else if (info->weldType == Tube_Plate_Fillet) {
            // TODO 管板角接微调未写
            if (info->weldEndPointsInRobot) {
                for (auto& pt : *(info->weldEndPointsInRobot)) {
                    pt.z += 0.0f;
                }
            }
        } else if (info->weldType == Tube_Tube_Fillet) {
            // TODO 管管角接微调未写
            if (info->weldEndPointsInRobot) {
                for (auto& pt : *(info->weldEndPointsInRobot)) {
                    pt.z += 0.0f;
                }
            }
        }
    }
}
void GantrayFrameTrajectoryPlanning::planPlatePlateFilletSeamOrientation(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo) {
    std::vector<std::shared_ptr<WeldSeamInfo>> subset;

    for (auto& s : weldSeamInfo) {
        if (!s || !s->detectSuccFlag) continue;

        if (!s->weldEndPointsInRobot || s->weldEndPointsInRobot->size() != 2) {
            PLOGD << "区域" << s->areaNum << "板板角接焊缝数量错误";
            continue;
        }

        if (s->weldAreaType == Plate_Plate_F) {
            subset.push_back(s);
        }
    }

    // ===== 分类 + 几何校验 =====
    std::vector<std::shared_ptr<WeldSeamInfo>> V_seams;
    std::vector<std::shared_ptr<WeldSeamInfo>> H_seams;
    for (auto& s : subset) {
        auto& pts = *(s->weldEndPointsInRobot);

        Eigen::Vector3f P0(pts[0].x, pts[0].y, pts[0].z);
        Eigen::Vector3f P1(pts[1].x, pts[1].y, pts[1].z);

        Eigen::Vector3f dir = P1 - P0;
        Eigen::Vector3f abs_dir = dir.cwiseAbs();

        bool isV_byGeom = (abs_dir.z() >= abs_dir.x() && abs_dir.z() >= abs_dir.y());

        if (isV_byGeom && s->weldType != Plate_Plate_Fillet_V) {
            PLOGE << "焊缝类型错误：应为V";
            return;
        }
        if (!isV_byGeom && s->weldType != Plate_Plate_Fillet_H) {
            PLOGE << "焊缝类型错误：应为H";
            return;
        }

        if (s->weldType == Plate_Plate_Fillet_V)
            V_seams.push_back(s);
        else if (s->weldType == Plate_Plate_Fillet_H)
            H_seams.push_back(s);
    }
    if (subset.size() <= 1) return;
    // =====================================================
    // ================== 情况1：两个焊缝 ===================
    // =====================================================
    if (subset.size() == 2) {
        // ===== 1V + 1H =====
        if (V_seams.size() == 1 && H_seams.size() == 1) {
            auto V = V_seams[0];
            auto H = H_seams[0];

            // ===== V方向：起点远离H平面，终点靠近 =====
            if (!H->otherSurface.empty()) {
                auto& planeData = H->otherSurface[0]->values;
                Eigen::Vector4f plane(planeData[0], planeData[1], planeData[2], planeData[3]);

                auto& ptsV = *(V->weldEndPointsInRobot);

                Eigen::Vector3f V0(ptsV[0].x, ptsV[0].y, ptsV[0].z);
                Eigen::Vector3f V1(ptsV[1].x, ptsV[1].y, ptsV[1].z);

                float d0 = fabs(plane.head<3>().dot(V0) + plane[3]);
                float d1 = fabs(plane.head<3>().dot(V1) + plane[3]);

                //  起点要远 → 如果P0更近，就交换
                if (d0 < d1) {
                    std::swap(ptsV[0], ptsV[1]);
                }
            }

            // ===== H 起点靠近 V 终点 =====
            auto& ptsH = *(H->weldEndPointsInRobot);
            auto& ptsV = *(V->weldEndPointsInRobot);

            Eigen::Vector3f Vend(ptsV[1].x, ptsV[1].y, ptsV[1].z);

            Eigen::Vector3f H0(ptsH[0].x, ptsH[0].y, ptsH[0].z);
            Eigen::Vector3f H1(ptsH[1].x, ptsH[1].y, ptsH[1].z);

            if ((H0 - Vend).squaredNorm() > (H1 - Vend).squaredNorm()) {
                std::swap(ptsH[0], ptsH[1]);
            }

            // ===== 顺序：V → H =====
            std::vector<std::shared_ptr<WeldSeamInfo>> newSubset{V, H};

            int idx = 0;
            for (auto& s : weldSeamInfo) {
                if (s && s->weldAreaType == Plate_Plate_F) {
                    s = newSubset[idx++];
                }
            }

            return;
        }

        // ===== 2个V：不处理 =====
        return;
    }

    // =====================================================
    // ================== 情况2：三个焊缝 ===================
    // =====================================================
    if (subset.size() == 3 && V_seams.size() == 2 && H_seams.size() == 1) {
        // ===== V 按 -Y 排序 =====
        std::sort(V_seams.begin(), V_seams.end(), [](const std::shared_ptr<WeldSeamInfo>& a, const std::shared_ptr<WeldSeamInfo>& b) {
            auto& pa = *(a->weldEndPointsInRobot);
            auto& pb = *(b->weldEndPointsInRobot);

            float ya = (pa[0].y + pa[1].y) * 0.5f;
            float yb = (pb[0].y + pb[1].y) * 0.5f;

            return ya > yb;
        });

        auto H = H_seams[0];

        // ===== V方向：起点远离H平面 =====
        if (!H->otherSurface.empty()) {
            auto& planeData = H->otherSurface[0]->values;
            Eigen::Vector4f plane(planeData[0], planeData[1], planeData[2], planeData[3]);
            // ===== 每条V内部方向 =====
            for (auto& V : V_seams) {
                auto& pts = *(V->weldEndPointsInRobot);

                Eigen::Vector3f V0(pts[0].x, pts[0].y, pts[0].z);
                Eigen::Vector3f V1(pts[1].x, pts[1].y, pts[1].z);

                float d0 = fabs(plane.head<3>().dot(V0) + plane[3]);
                float d1 = fabs(plane.head<3>().dot(V1) + plane[3]);

                if (d0 < d1) {
                    std::swap(pts[0], pts[1]);
                }
            }
        }

        // ===== H 起点靠近 V0 终点 =====
        auto& ptsH = *(H->weldEndPointsInRobot);
        auto& ptsV = *(V_seams[0]->weldEndPointsInRobot);

        Eigen::Vector3f Vend(ptsV[1].x, ptsV[1].y, ptsV[1].z);

        Eigen::Vector3f H0(ptsH[0].x, ptsH[0].y, ptsH[0].z);
        Eigen::Vector3f H1(ptsH[1].x, ptsH[1].y, ptsH[1].z);

        if ((H0 - Vend).squaredNorm() > (H1 - Vend).squaredNorm()) {
            std::swap(ptsH[0], ptsH[1]);
        }

        // ===== 顺序 =====
        std::vector<std::shared_ptr<WeldSeamInfo>> newSubset{V_seams[0], H, V_seams[1]};

        int idx = 0;
        for (auto& s : weldSeamInfo) {
            if (s && s->weldAreaType == Plate_Plate_F) {
                s = newSubset[idx++];
            }
        }

        return;
    }
}
void GantrayFrameTrajectoryPlanning::computeSwingReferencePointsForSeam(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo) {
    for (auto& info : weldSeamInfo) {
        if (!info) continue;

        info->swingReferencePoints.clear();

        if (!info->detectSuccFlag) continue;
        if (info->robotWeldPose.empty()) continue;

        if (info->weldType == Plate_Plate_Fillet_V) {
            robotPose startWeld = info->robotWeldPose[0];
            applyWeldGunWithdraw(startWeld, settingPara.PlatePlateFilletVerticalWithdrawDistance);

            std::vector<double> v;
            if (computePlatePlateFilletVerticalSwingPoints(info, startWeld, v) && v.size() == 6) {
                info->swingReferencePoints = v;
            } else {
                info->swingReferencePoints.clear();
                PLOGE << "板板竖直角接摆焊点计算失败。";
            }
        } else if (info->weldType == Tube_Plate_Fillet) {
            robotPose startWeld = info->robotWeldPose[0];

            float extraOffset = 0.0f;
            if (!info->weldCollisionResult.empty()) {
                extraOffset = info->weldCollisionResult[0].extraOffset;
            }

            applyWeldGunWithdraw(startWeld, settingPara.TubePlateFilletWithdrawDistance + extraOffset);

            std::vector<double> v;
            if (computeTubePlateFilletSwingPoints(info, startWeld, v) && v.size() == 6) {
                info->swingReferencePoints = v;
            } else {
                info->swingReferencePoints.clear();
                PLOGE << "管板角接摆焊点计算失败。";
            }
        } else if (info->weldType == Tube_Tube_Fillet) {
            robotPose startWeld = info->robotWeldPose[0];

            float extraOffset = 0.0f;
            if (!info->weldCollisionResult.empty()) {
                extraOffset = info->weldCollisionResult[0].extraOffset;
            }

            applyWeldGunWithdraw(startWeld, settingPara.TubeTubeFilletWithdrawDistance + extraOffset);

            std::vector<double> v;
            if (computeTubeTubeFilletSwingPoints(info, startWeld, v) && v.size() == 6) {
                info->swingReferencePoints = v;
            } else {
                info->swingReferencePoints.clear();
                PLOGE << "管管角接摆焊点计算失败。";
            }
        } else {
            info->swingReferencePoints.clear();
        }
    }
}
bool GantrayFrameTrajectoryPlanning::computePlatePlateFilletVerticalSwingPoints(const std::shared_ptr<WeldSeamInfo>& info, const robotPose& refPose,
                                                                                std::vector<double>& swingPoints) {
    swingPoints.clear();
    if (!info || !info->weldCoeff || info->weldCoeff->values.size() != 4) return false;

    if (info->otherSurface.empty()) return false;

    if (!info->weldEndPointsInRobot || info->weldEndPointsInRobot->size() < 2) return false;

    // ===== 1. 焊缝端点 → seamDir =====
    Eigen::Vector3f P0(info->weldEndPointsInRobot->at(0).x, info->weldEndPointsInRobot->at(0).y, info->weldEndPointsInRobot->at(0).z);

    Eigen::Vector3f P1(info->weldEndPointsInRobot->at(1).x, info->weldEndPointsInRobot->at(1).y, info->weldEndPointsInRobot->at(1).z);

    Eigen::Vector3f seamDir = (P0 - P1).normalized();

    // ===== 2. 主平面法向 n1 =====
    Eigen::Vector3f n1(info->weldCoeff->values[0], info->weldCoeff->values[1], info->weldCoeff->values[2]);
    n1.normalize();

    // ===== 3. 第二平面法向 n2 =====
    Eigen::Vector3f n2(0, 0, 0);
    for (auto& surf : info->otherSurface) {
        if (!surf || surf->values.size() != 4) continue;

        n2 = Eigen::Vector3f(surf->values[0], surf->values[1], surf->values[2]);
        n2.normalize();
        break;
    }

    if (n2.norm() < 1e-6) n2 = n1;

    // ===== 4. 参考点（后撤后的点）=====
    Eigen::Vector3f P_ref(refPose.x_, refPose.y_, refPose.z_);

    // ================= 第一参考点 =================
    // 主平面内 ⟂ seamDir
    Eigen::Vector3f seamDir_proj = seamDir - seamDir.dot(n1) * n1;
    if (seamDir_proj.norm() < 1e-6) return false;
    seamDir_proj.normalize();

    Eigen::Vector3f verticalDir = n1.cross(seamDir_proj);
    if (verticalDir.norm() < 1e-6) return false;
    verticalDir.normalize();

    // 朝 n2
    if (verticalDir.dot(n2) < 0) verticalDir = -verticalDir;

    float L1 = 1.0f;
    Eigen::Vector3f refPoint1 = P_ref + L1 * verticalDir;

    // ================= 第二参考点 =================
    Eigen::Vector3f seamDir_proj2 = seamDir - seamDir.dot(n2) * n2;
    if (seamDir_proj2.norm() < 1e-6) return false;
    seamDir_proj2.normalize();

    Eigen::Vector3f horizontalDir = n2.cross(seamDir_proj2);
    if (horizontalDir.norm() < 1e-6) return false;
    horizontalDir.normalize();

    // 朝 n1
    if (horizontalDir.dot(n1) < 0) horizontalDir = -horizontalDir;

    float L2 = 0.1f;
    Eigen::Vector3f refPoint2 = P_ref - L2 * horizontalDir;

    // ===== 5. 输出 =====
    swingPoints.reserve(6);

    swingPoints.push_back(refPoint1.x());
    swingPoints.push_back(refPoint1.y());
    swingPoints.push_back(refPoint1.z());

    swingPoints.push_back(refPoint2.x());
    swingPoints.push_back(refPoint2.y());
    swingPoints.push_back(refPoint2.z());
    // PLOGD << "========== Swing Debug (Plate_Plate_Fillet_V) ==========";

    // // 原点（后撤点）
    // PLOGD << "P_ref: " << P_ref.x() << ", " << P_ref.y() << ", " << P_ref.z();

    // // seamDir
    // PLOGD << "seamDir: " << seamDir.x() << ", " << seamDir.y() << ", " << seamDir.z();

    // // 法向
    // PLOGD << "n1 (main plane): " << n1.x() << ", " << n1.y() << ", " << n1.z();

    // PLOGD << "n2 (other plane): " << n2.x() << ", " << n2.y() << ", " << n2.z();

    // // 第一方向（主平面摆动方向）
    // PLOGD << "verticalDir (ref1 dir): " << verticalDir.x() << ", " << verticalDir.y() << ", " << verticalDir.z();

    // // 第二方向（另一板摆动方向）
    // PLOGD << "horizontalDir (ref2 dir): " << horizontalDir.x() << ", " << horizontalDir.y() << ", " << horizontalDir.z();

    // // 第一参考点
    // PLOGD << "refPoint1: " << refPoint1.x() << ", " << refPoint1.y() << ", " << refPoint1.z();

    // // 第二参考点
    // PLOGD << "refPoint2: " << refPoint2.x() << ", " << refPoint2.y() << ", " << refPoint2.z();

    // PLOGD << "=======================================================";

    return true;
}
bool GantrayFrameTrajectoryPlanning::computeTubePlateFilletSwingPoints(const std::shared_ptr<WeldSeamInfo>& info, const robotPose& basePose,
                                                                       std::vector<double>& swingPoints) {
    swingPoints.clear();

    if (!info) {
        PLOGE << "computeTubePlateFilletSwingPoints: info 为空";
        return false;
    }

    if (!info->weldCoeff || info->weldCoeff->values.size() < 7) {
        PLOGE << "computeTubePlateFilletSwingPoints: 圆柱参数无效";
        return false;
    }

    // ================= 圆柱参数 =================
    const auto& coeff = info->weldCoeff->values;

    Eigen::Vector3f cylCenter(coeff[0], coeff[1], coeff[2]);
    Eigen::Vector3f cylAxis(coeff[3], coeff[4], coeff[5]);
    float cylRadius = coeff[6];

    if (cylAxis.norm() < 1e-6f) {
        PLOGE << "computeTubePlateFilletSwingPoints: 圆柱轴方向长度过小";
        return false;
    }
    cylAxis.normalize();

    // ================= 基准点（已后撤点） =================
    Eigen::Vector3f P(basePose.x_, basePose.y_, basePose.z_);

    // ================= 计算径向方向 =================
    // 先求 P 在圆柱轴上的投影点
    float t = (P - cylCenter).dot(cylAxis);
    Eigen::Vector3f projOnAxis = cylCenter + t * cylAxis;

    Eigen::Vector3f radial = P - projOnAxis;
    float radialNorm = radial.norm();

    if (radialNorm < 1e-6f) {
        PLOGE << "computeTubePlateFilletSwingPoints: 基准点落在圆柱轴上，无法计算径向方向";
        return false;
    }
    radial /= radialNorm;

    // ================= 两个参考点 =================
    // 参考点1：沿圆柱径向延伸 1mm
    Eigen::Vector3f ref1 = P + 20.0f * radial;

    // 参考点2：沿圆柱轴延伸 1mm
    Eigen::Vector3f ref2 = P + 20.0f * cylAxis;

    swingPoints.resize(6);
    swingPoints[0] = ref1.x();
    swingPoints[1] = ref1.y();
    swingPoints[2] = ref1.z();
    swingPoints[3] = ref2.x();
    swingPoints[4] = ref2.y();
    swingPoints[5] = ref2.z();

    // 第一参考点
    PLOGD << "refPoint1: " << ref1.x() << ", " << ref1.y() << ", " << ref1.z();

    // 第二参考点
    PLOGD << "refPoint2: " << ref2.x() << ", " << ref2.y() << ", " << ref2.z();

    return true;
}
bool GantrayFrameTrajectoryPlanning::computeTubeTubeFilletSwingPoints(const std::shared_ptr<WeldSeamInfo>& info, const robotPose& basePose,
                                                                      std::vector<double>& swingPoints) {
    swingPoints.clear();

    if (!info) {
        PLOGE << "computeTubeTubeFilletSwingPoints: info 为空";
        return false;
    }

    // 主圆柱参数，默认使用 weldCoeff
    if (!info->weldCoeff || info->weldCoeff->values.size() < 7) {
        PLOGE << "computeTubeTubeFilletSwingPoints: 主圆柱参数无效";
        return false;
    }

    const auto& coeff = info->weldCoeff->values;

    Eigen::Vector3f cylCenter(coeff[0], coeff[1], coeff[2]);
    Eigen::Vector3f cylAxis(coeff[3], coeff[4], coeff[5]);
    float cylRadius = coeff[6];

    if (cylAxis.norm() < 1e-6f) {
        PLOGE << "computeTubeTubeFilletSwingPoints: 主圆柱轴方向长度过小";
        return false;
    }

    cylAxis.normalize();

    // 基准点：通常是已经后撤后的焊接起点
    Eigen::Vector3f P(basePose.x_, basePose.y_, basePose.z_);

    // 点 P 到圆柱主轴的径向向量
    Eigen::Vector3f CP = P - cylCenter;
    Eigen::Vector3f radial = CP - CP.dot(cylAxis) * cylAxis;

    if (radial.norm() < 1e-6f) {
        PLOGE << "computeTubeTubeFilletSwingPoints: 基准点落在主圆柱轴附近，无法计算径向方向";
        return false;
    }

    radial.normalize();

    // 参考点距离
    const float refDist = 20.0f;

    // 参考点1：主圆柱径向方向
    Eigen::Vector3f ref1 = P + refDist * radial;

    // 参考点2：主圆柱轴向方向
    Eigen::Vector3f ref2 = P + refDist * cylAxis;

    swingPoints.resize(6);
    swingPoints[0] = ref1.x();
    swingPoints[1] = ref1.y();
    swingPoints[2] = ref1.z();

    swingPoints[3] = ref2.x();
    swingPoints[4] = ref2.y();
    swingPoints[5] = ref2.z();

    PLOGD << "TubeTube refPoint1 : " << ref1.x() << ", " << ref1.y() << ", " << ref1.z();

    PLOGD << "TubeTube refPoint2 : " << ref2.x() << ", " << ref2.y() << ", " << ref2.z();

    return true;
}
void GantrayFrameTrajectoryPlanning::debugWeldingCollisionCheck(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo) {
    const float toolRadius = settingPara.toolRadius;
    const float tolOffset = 1e-3f;
    for (size_t s = 0; s < weldSeamInfo.size(); ++s) {
        auto& info = weldSeamInfo[s];

        if (!info->weldCoeff || !info->weldEndPointsInRobot || info->weldEndPointsInRobot->empty() || info->otherSurface.empty()) continue;

        if (info->weldType == Tube_Plate_Fillet) {
            const float offset0 = settingPara.wireCalibrationOffset + settingPara.TubePlateFilletWithdrawDistance;
            int N = info->weldEndPointsInRobot->size();

            // 防止越界
            if (info->robotWeldPose.size() != N) {
                std::cout << "!!! robotWeldPose size mismatch\n";
                continue;
            }

            info->weldCollisionResult.resize(N);

            // ================= 平面 =================
            Eigen::Vector3f n_plane(info->otherSurface[0]->values[0], info->otherSurface[0]->values[1], info->otherSurface[0]->values[2]);
            n_plane.normalize();
            float d_plane = info->otherSurface[0]->values[3];

            // ================= 圆柱 =================
            Eigen::Vector3f cylC(info->weldCoeff->values[0], info->weldCoeff->values[1], info->weldCoeff->values[2]);

            Eigen::Vector3f cylAxis(info->weldCoeff->values[3], info->weldCoeff->values[4], info->weldCoeff->values[5]);
            cylAxis.normalize();

            float cylRadius = info->weldCoeff->values[6];

            // ================= 从 robotPose 提取 Z =================
            std::vector<Eigen::Vector3f> Zlist(N);

            for (int i = 0; i < N; ++i) {
                const robotPose& pose = info->robotWeldPose[i];

                Eigen::Matrix4f T = MyToolFunc::createTransformationMatrixZYX(pose.x_, pose.y_, pose.z_, pose.a_, pose.b_, pose.c_);

                Zlist[i] = T.block<3, 1>(0, 2);
            }

            // #pragma omp parallel for schedule(dynamic)  //测试后单线程较快
            for (int i = 0; i < N; ++i) {
                Eigen::Vector3f P(info->weldEndPointsInRobot->at(i).x, info->weldEndPointsInRobot->at(i).y, info->weldEndPointsInRobot->at(i).z);

                Eigen::Vector3f Z = Zlist[i];

                CollisionResult col = checker.evalCollisionAtOffset(P, Z, offset0, n_plane, d_plane, cylC, cylAxis, cylRadius, toolRadius);

                if (col.isIntersect) {
                    float safeOffset =
                        checker.findSafeOffset(P, Z, offset0, n_plane, d_plane, cylC, cylAxis, cylRadius, toolRadius, maxExtraOffset, tolOffset);

                    if (std::isfinite(safeOffset)) {
                        col.safeOffset = static_cast<double>(safeOffset);
                        col.extraOffset = static_cast<double>(safeOffset - offset0);
                    } else {
                        col.safeOffset = std::numeric_limits<double>::quiet_NaN();
                        col.extraOffset = 0.0;
                    }
                }

                info->weldCollisionResult[i] = col;
            }

            // ================= 串行打印 =================
            if (settingPara.bool_save_model) {
                for (int i = 0; i < N; ++i) {
                    auto& col = info->weldCollisionResult[i];

                    Eigen::Vector3f P(info->weldEndPointsInRobot->at(i).x, info->weldEndPointsInRobot->at(i).y, info->weldEndPointsInRobot->at(i).z);

                    Eigen::Vector3f Z = Zlist[i];
                    Eigen::Vector3f base = P - offset0 * Z;

                    std::cout << "\n[SeamIdx " << s << " | Area " << info->areaNum << " | Point " << i + 1 << "]\n";

                    std::cout << "  Base = [" << base.x() << " " << base.y() << " " << base.z() << "]\n";
                    std::cout << "  dist_plane   = " << col.distSec << "\n";
                    std::cout << "  dist_cyl     = " << col.distMain << "\n";
                    std::cout << "  Intersect?   = " << (col.isIntersect ? 1 : 0) << "\n";

                    if (col.isIntersect) {
                        if (!std::isfinite(col.safeOffset)) {
                            std::cout << "  safeOffset   = NOT FOUND\n";
                        } else {
                            std::cout << "  safeOffset   = " << col.safeOffset << "\n";
                            std::cout << "  extraOffset  = " << col.extraOffset << "\n";
                        }
                    }
                }
            }
        } else if (info->weldType == Tube_Tube_Fillet) {
            const float offset0 = settingPara.wireCalibrationOffset + settingPara.TubeTubeFilletWithdrawDistance;
            int N = static_cast<int>(info->weldEndPointsInRobot->size());

            if (info->robotWeldPose.size() != N) {
                std::cout << "!!! robotWeldPose size mismatch\n";
                continue;
            }

            if (info->otherSurface.empty() || !info->otherSurface[0]) {
                std::cout << "!!! Tube_Tube_Fillet otherSurface empty\n";
                continue;
            }

            info->weldCollisionResult.resize(N);

            // ================= 圆柱1：weldCoeff =================
            Eigen::Vector3f cylC1(info->weldCoeff->values[0], info->weldCoeff->values[1], info->weldCoeff->values[2]);

            Eigen::Vector3f cylAxis1(info->weldCoeff->values[3], info->weldCoeff->values[4], info->weldCoeff->values[5]);
            cylAxis1.normalize();

            float cylRadius1 = info->weldCoeff->values[6];

            // ================= 圆柱2：otherSurface[0] =================
            Eigen::Vector3f cylC2(info->otherSurface[0]->values[0], info->otherSurface[0]->values[1], info->otherSurface[0]->values[2]);

            Eigen::Vector3f cylAxis2(info->otherSurface[0]->values[3], info->otherSurface[0]->values[4], info->otherSurface[0]->values[5]);
            cylAxis2.normalize();

            float cylRadius2 = info->otherSurface[0]->values[6];

            // ================= 从 robotPose 提取 Z =================
            std::vector<Eigen::Vector3f> Zlist(N);

            for (int i = 0; i < N; ++i) {
                const robotPose& pose = info->robotWeldPose[i];

                Eigen::Matrix4f T = MyToolFunc::createTransformationMatrixZYX(pose.x_, pose.y_, pose.z_, pose.a_, pose.b_, pose.c_);

                Zlist[i] = T.block<3, 1>(0, 2).normalized();

                // 和 MATLAB 保持一致：Z 与圆柱1径向向量相反
                Eigen::Vector3f P(info->weldEndPointsInRobot->at(i).x, info->weldEndPointsInRobot->at(i).y, info->weldEndPointsInRobot->at(i).z);

                Eigen::Vector3f v = P - cylC1;
                Eigen::Vector3f foot = cylC1 + v.dot(cylAxis1) * cylAxis1;
                Eigen::Vector3f radial1 = P - foot;

                if (radial1.norm() > 1e-6f) {
                    radial1.normalize();

                    if (Zlist[i].dot(radial1) > 0.0f) {
                        Zlist[i] = -Zlist[i];
                    }
                }
            }

            // ================= 碰撞检测 =================
            for (int i = 0; i < N; ++i) {
                Eigen::Vector3f P(info->weldEndPointsInRobot->at(i).x, info->weldEndPointsInRobot->at(i).y, info->weldEndPointsInRobot->at(i).z);

                Eigen::Vector3f Z = Zlist[i];

                CollisionResult col =
                    checker.evalCollisionTwoCylindersAtOffset(P, Z, offset0, cylC1, cylAxis1, cylRadius1, cylC2, cylAxis2, cylRadius2, toolRadius);

                if (col.isIntersect) {
                    float safeOffset = checker.findSafeOffsetTwoCylinders(P, Z, offset0, cylC1, cylAxis1, cylRadius1, cylC2, cylAxis2, cylRadius2,
                                                                          toolRadius, maxExtraOffset, tolOffset);

                    if (std::isfinite(safeOffset)) {
                        col.safeOffset = static_cast<double>(safeOffset);
                        col.extraOffset = static_cast<double>(safeOffset - offset0);
                    } else {
                        col.safeOffset = std::numeric_limits<double>::quiet_NaN();
                        col.extraOffset = 0.0;
                    }
                }

                info->weldCollisionResult[i] = col;
            }  // ================= 串行打印 =================
            if (1) {  // settingPara.bool_save_model
                for (int i = 0; i < N; ++i) {
                    auto& col = info->weldCollisionResult[i];

                    Eigen::Vector3f P(info->weldEndPointsInRobot->at(i).x, info->weldEndPointsInRobot->at(i).y, info->weldEndPointsInRobot->at(i).z);

                    Eigen::Vector3f Z = Zlist[i];
                    Eigen::Vector3f base = P - offset0 * Z;

                    // std::cout << "\n[SeamIdx " << s << " | Area " << info->areaNum << " | Point " << i + 1 << "]\n";

                    // std::cout << "  P = [" << P.x() << " " << P.y() << " " << P.z() << "]\n";
                    // std::cout << "  Z = [" << Z.x() << " " << Z.y() << " " << Z.z() << "]\n";
                    // std::cout << "  Base = [" << base.x() << " " << base.y() << " " << base.z() << "]\n";

                    // std::cout << "  dist_cyl1   = " << col.distMain << "\n";
                    // std::cout << "  dist_cyl2   = " << col.distSec << "\n";
                    // std::cout << "  Intersect?  = " << (col.isIntersect ? 1 : 0) << "\n";

                    // if (col.isIntersect) {
                    //     if (!std::isfinite(col.safeOffset)) {
                    //         std::cout << "  safeOffset  = NOT FOUND\n";
                    //     } else {
                    //         std::cout << "  safeOffset  = " << col.safeOffset << "\n";
                    //         std::cout << "  extraOffset = " << col.extraOffset << "\n";
                    //     }
                    // }
                }
            }
        }
    }
}
