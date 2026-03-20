#include "robotTrajectoryPlanning/LargeWorkpieceTrajectoryPlanning.h"

#include "plog/Log.h"
#include "robotTrajectoryPlanning/config/TrajectoryPlanningConfig.h"
#include "settingPara/SettingPara.h"
#include "utils/common/CommonFunc.h"
#include "utils/common/WeldSeamInfo.h"
#include "utils/pointCloud/PointCloudFunc.h"
LargeWorkpieceTrajectoryPlanning::LargeWorkpieceTrajectoryPlanning(QObject* parent) : AbstractTrajectoryPlanning(parent) {
    PLOGD << "大型工件轨迹规划类初始化";
}

void LargeWorkpieceTrajectoryPlanning::whenPlanningTrajectory(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo) {
    PLOGD << "大型工件轨迹规划类 收到焊缝数量: " << weldSeamInfo.size();
    for (auto& info : weldSeamInfo) {
        if (info->detectSuccFlag == true && info->weldEndPointsInCamera != nullptr && info->weldEndPointsInCamera->size() == 2) {
            PLOGD << info->weldEndPointsInCamera->at(0) << " " << info->weldEndPointsInCamera->at(1);
        }
    }
    // // --------------------------- 对焊缝点进行误差补偿 ---------------------------
    // this->seamsErrorCompensate(weldSeamInfo);
    // PLOGD << "误差补偿后的焊缝点: ";  // 打印操作后的焊缝
    // for (auto& info : weldSeamInfo) {
    //     if (info->detectSuccFlag == true && info->weldEndPointsInRobot != nullptr && info->weldEndPointsInRobot->size() == 2) {
    //         PLOGD << info->weldEndPointsInRobot->at(0) << " " << info->weldEndPointsInRobot->at(1);
    //     }
    // }
    // --------------------------- 标准化表面方向（相机坐标系） ---------------------------

    this->normalizeSurfaceDirectionInCamera(weldSeamInfo);

    // --------------------------- 转换坐标点到机器人基坐标系下 ---------------------------
    // 实现大型工件的轨迹规划逻辑
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
    // --------------------------- 生成焊接轨迹 ---------------------------
    this->generateWeldPose(weldSeamInfo);
    // --------------------------- 打印焊接轨迹 ---------------------------
    PLOGD << "================ 焊接轨迹（robotWeldPose） ================";

    for (size_t i = 0; i < weldSeamInfo.size(); ++i) {
        auto& info = weldSeamInfo[i];
        if (!info) continue;

        PLOGD << "---- seam index: " << i;

        if (info->robotWeldPose.empty()) {
            PLOGD << "robotWeldPose is empty";
            continue;
        }

        for (size_t j = 0; j < info->robotWeldPose.size(); ++j) {
            const auto& pose = info->robotWeldPose[j];

            PLOGD << "[pose " << j << "] " << "x=" << pose.x_ << ", y=" << pose.y_ << ", z=" << pose.z_ << ", a=" << pose.a_
                  << ", b=" << pose.b_ << ", c=" << pose.c_;
        }
    }
    // // --------------------------- 焊缝轨迹后撤(手动测试使用) ---------------------------
    // for (auto& info : weldSeamInfo) {
    //     if (!info) continue;

    //     for (auto& pose : info->robotWeldPose) {
    //         applyWeldGunWithdraw(pose, 3.0);
    //     }
    // }
    // PLOGD << "================焊缝后撤 ================";
    // for (size_t i = 0; i < weldSeamInfo.size(); ++i) {
    //     auto& info = weldSeamInfo[i];
    //     if (!info) continue;

    //     PLOGD << "---- seam index: " << i;

    //     if (info->robotWeldPose.empty()) {
    //         PLOGD << "robotWeldPose is empty";
    //         continue;
    //     }

    //     for (size_t j = 0; j < info->robotWeldPose.size(); ++j) {
    //         const auto& pose = info->robotWeldPose[j];

    //         PLOGD << "[pose " << j << "] " << "x=" << pose.x_ << ", y=" << pose.y_ << ", z=" << pose.z_ << ", a=" << pose.a_
    //               << ", b=" << pose.b_ << ", c=" << pose.c_;
    //     }
    // }
    // 发出规划完成的焊缝
    emit sendPlannedSeams(weldSeamInfo);
}

void LargeWorkpieceTrajectoryPlanning::write2File(const std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo,
                                                  int endOfLeftSeams) {
    // 实现大型工件的文件写入逻辑
    outfile.open(outfile_name, std::ios::out);
    if (!outfile.is_open()) {
        PLOGE << "无法打开输出文件: " << outfile_name;
        return;
    }
    if (weldSeamInfo.size() == 0) {
        PLOGE << "没有焊缝信息";
        outfile.close();
        // emit sendTrajectoryPlanOver();  // 但也发送轨迹规划完成
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

    float takePhotoX0 = trajectoryConfig.takePhotoX, takePhotoY0 = trajectoryConfig.takePhotoY,
          takePhotoZ0 = trajectoryConfig.takePhotoZ;
    float takePhotoA0 = trajectoryConfig.takePhotoA, takePhotoB0 = trajectoryConfig.takePhotoB,
          takePhotoC0 = trajectoryConfig.takePhotoC;
    // ########################### 眼在手上写入拍照点, 眼在手外写入零过渡点 ###########################
    if (trajectoryConfig.handEyeType == MyToolFunc::getHandTypeTypeString(HAND_EYE_TYPE::EYE_IN_HAND)) {
        if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::FRONT) {
            outfile << takePhotoX0 << " " << takePhotoY0 << " " << takePhotoZ0 << " ";
            outfile << takePhotoA0 << " " << takePhotoB0 << " " << takePhotoC0 << " " << moveSpeed << " " << ARC_STOP << " "
                    << LINE_WELD << " " << weldingCurrent << " " << weldingVoltage << std::endl;
        }
        outfile << X0 << " " << Y0 << " " << Z0 << " ";
        outfile << A0 << " " << B0 << " " << C0 << " " << moveSpeed << " " << ARC_STOP << " " << LINE_WELD << " "
                << weldingCurrent << " " << weldingVoltage << std::endl;
    } else if (trajectoryConfig.handEyeType == MyToolFunc::getHandTypeTypeString(HAND_EYE_TYPE::EYE_TO_HAND)) {
        outfile << X0 << " " << Y0 << " " << Z0 << " ";
        outfile << A0 << " " << B0 << " " << C0 << " " << moveSpeed << " " << ARC_STOP << " " << LINE_WELD << " "
                << weldingCurrent << " " << weldingVoltage << std::endl;
    } else {
        PLOGE << "机器人手眼关系错误";
    }
    for (const auto& info : weldSeamInfo) {
        if (!info || info->robotWeldPose.empty()) continue;
        if (info->weldType == TubeSide_Plate_F_H) {
            const robotPose& startPose = info->robotWeldPose[0];
            const robotPose& endPose = info->robotWeldPose[1];

            // ================= 起点过渡=================
            robotPose startTransition = startPose;
            applyWeldGunWithdraw(startTransition, 10.0);

            outfile << startTransition.x_ << " " << startTransition.y_ << " " << startTransition.z_ + 10.0 << " "
                    << startTransition.a_ << " " << startTransition.b_ << " " << startTransition.c_ << " " << moveSpeed << " "
                    << ARC_STOP << " " << LINE_WELD << " " << weldingCurrent << " " << weldingVoltage << std::endl;

            // ================= 焊接起点=================
            robotPose startWeld = startPose;
            applyWeldGunWithdraw(startWeld, 3.0);

            outfile << startWeld.x_ << " " << startWeld.y_ << " " << startWeld.z_ << " " << startWeld.a_ << " " << startWeld.b_
                    << " " << startWeld.c_ << " " << moveSpeed << " " << ARC_START << " " << LINE_WELD << " " << weldingCurrent
                    << " " << weldingVoltage << std::endl;

            // ================= 焊接终点=================
            robotPose endWeld = endPose;
            applyWeldGunWithdraw(endWeld, 3.0);

            outfile << endWeld.x_ << " " << endWeld.y_ << " " << endWeld.z_ << " " << endWeld.a_ << " " << endWeld.b_ << " "
                    << endWeld.c_ << " " << weldingSpeedDefault << " " << ARC_STOP << " " << LINE_WELD << " " << weldingCurrent
                    << " " << weldingVoltage << std::endl;

            // ================= 终点过渡=================
            robotPose endTransition = endPose;
            applyWeldGunWithdraw(endTransition, 10.0);

            outfile << endTransition.x_ << " " << endTransition.y_ << " " << endTransition.z_ + 10.0 << " " << endTransition.a_
                    << " " << endTransition.b_ << " " << endTransition.c_ << " " << moveSpeed << " " << ARC_STOP << " "
                    << LINE_WELD << " " << weldingCurrent << " " << weldingVoltage << std::endl;
        }
    }
    // ########################### 眼在手上写入拍照点, 眼在手外写入零过渡点 ###########################
    if (trajectoryConfig.handEyeType == MyToolFunc::getHandTypeTypeString(HAND_EYE_TYPE::EYE_IN_HAND)) {
        outfile << X0 << " " << Y0 << " " << Z0 << " ";
        outfile << A0 << " " << B0 << " " << C0 << " " << moveSpeed << " " << ARC_STOP << " " << LINE_WELD << " "
                << weldingCurrent << " " << weldingVoltage << std::endl;
        outfile << takePhotoX0 << " " << takePhotoY0 << " " << takePhotoZ0 << " ";
        outfile << takePhotoA0 << " " << takePhotoB0 << " " << takePhotoC0 << " " << moveSpeed << " " << ARC_STOP << " "
                << LINE_WELD << " " << weldingCurrent << " " << weldingVoltage << std::endl;
        outfile << takePhotoX0 << " " << takePhotoY0 << " " << takePhotoZ0 << " ";
        outfile << takePhotoA0 << " " << takePhotoB0 << " " << takePhotoC0 << " " << moveSpeed << " " << ARC_STOP << " "
                << LINE_WELD << " " << weldingCurrent << " " << weldingVoltage << std::endl;
    } else if (trajectoryConfig.handEyeType == MyToolFunc::getHandTypeTypeString(HAND_EYE_TYPE::EYE_TO_HAND)) {
        outfile << X0 << " " << Y0 << " " << Z0 << " ";
        outfile << A0 << " " << B0 << " " << C0 << " " << moveSpeed << " " << ARC_STOP << " " << LINE_WELD << " "
                << weldingCurrent << " " << weldingVoltage << std::endl;
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
void LargeWorkpieceTrajectoryPlanning::normalizeSurfaceDirectionInCamera(
    std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo) {
    const float EPS = 1e-6f;

    for (auto& info : weldSeamInfo) {
        if (!info) continue;

        // ================= weldPlane =================
        if (info->weldPlane) {
            auto& v = info->weldPlane->values;

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
void LargeWorkpieceTrajectoryPlanning::transSeams2Base(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo) {
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr weldAreaPointCloudBase;
        // ---------- 2.1 焊缝端点 ----------
        if (info->weldEndPointsInCamera && info->weldEndPointsInCamera->size() >= 2) {
            info->weldEndPointsInRobot = std::make_shared<std::vector<pcl::PointXYZ>>();

            info->weldEndPointsInRobot->reserve(info->weldEndPointsInCamera->size());

            for (const auto& pt : *(info->weldEndPointsInCamera)) {
                info->weldEndPointsInRobot->emplace_back(MyToolFunc::transformSinglePoint(pt, T_cam2base));
            }
        }
        // ---------- 2.2 （可选）焊缝区域点云 ----------

        if (info->weldAreaPointCloud && !info->weldAreaPointCloud->empty()) {
            weldAreaPointCloudBase = MyToolFunc::transformPointCloud(info->weldAreaPointCloud, T_cam2base);
            // weldAreaPointCloudBase->height = 1;
            // weldAreaPointCloudBase->width = static_cast<uint32_t>(weldAreaPointCloudBase->size());
            // pcl::io::savePCDFile("./data/seamDetWithPointCloud/tubeSidePlateFilletSeamsDet/weldAreaPointCloudInBase.pcd",
            //                      *weldAreaPointCloudBase);
        }
        // ---------- 2.2 转换焊缝母材系数 -----------
        pcl::ModelCoefficients::Ptr plane_base_trans;
        pcl::ModelCoefficients::Ptr cylinder_base_trans;
        if (info->weldPlane) {
            if (info->weldPlane->values.size() == 4) {
                plane_base_trans = MyToolFunc::transformPlane(info->weldPlane, T_cam2base);
                info->weldPlane = plane_base_trans;
            } else if (info->weldPlane->values.size() == 7) {
                cylinder_base_trans = MyToolFunc::transformCylinder(info->weldPlane, T_cam2base);
                info->weldPlane = cylinder_base_trans;
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
void LargeWorkpieceTrajectoryPlanning::determineWorkpieceOri(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo) {
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
void LargeWorkpieceTrajectoryPlanning::transSeamsOri(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo) {
    // ===== 参考点（机器人当前位置）=====
    Eigen::Vector3f ref(trajectoryConfig.matrixEnd2Base(0, 3), trajectoryConfig.matrixEnd2Base(1, 3),
                        trajectoryConfig.matrixEnd2Base(2, 3));

    for (auto& info : weldSeamInfo) {
        if (!info || !info->detectSuccFlag) continue;

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
    }
}
void LargeWorkpieceTrajectoryPlanning::generateWeldPose(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo) {
    for (auto& info : weldSeamInfo) {
        if (!info || !info->detectSuccFlag) continue;

        if (!info->weldEndPointsInRobot || info->weldEndPointsInRobot->size() < 2) continue;

        if (!info->weldPlane || info->weldPlane->values.size() != 4) continue;

        if (info->otherSurface.empty()) continue;

        // ================= 1. 取起点终点 =================
        Eigen::Vector3f P0(info->weldEndPointsInRobot->at(0).x, info->weldEndPointsInRobot->at(0).y,
                           info->weldEndPointsInRobot->at(0).z);

        Eigen::Vector3f P1(info->weldEndPointsInRobot->at(1).x, info->weldEndPointsInRobot->at(1).y,
                           info->weldEndPointsInRobot->at(1).z);

        Eigen::Vector3f mid = 0.5f * (P0 + P1);

        // ================= 2. 平面法向 =================
        Eigen::Vector3f n1(info->weldPlane->values[0], info->weldPlane->values[1], info->weldPlane->values[2]);
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
        Eigen::Vector3f Z = (tubeSidePlateFilletPlanePoseW * n1 + tubeSidePlateFilletCylinderPoseW * n2).normalized();

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

        double a = targetABC[0];
        double b = targetABC[1];
        double c = targetABC[2];

        // ================= 10. 写入两个点 =================
        robotPose pose_start, pose_end;

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

        // static int idx1 = 0;

        // std::string file = "./matlab_full_" + std::to_string(idx1++) + ".m";

        // pcl::ModelCoefficients::Ptr cylinder = nullptr;
        // for (auto& s : info->otherSurface) {
        //     if (s && s->values.size() == 7) {
        //         cylinder = s;
        //         break;
        //     }
        // }

        // saveToMatlabFull(file, P0, P1, mid, X, Y, Z, info->weldPlane, cylinder);
    }
}
void LargeWorkpieceTrajectoryPlanning::saveToMatlabFull(const std::string& filename, const Eigen::Vector3f& P0,
                                                        const Eigen::Vector3f& P1, const Eigen::Vector3f& mid,
                                                        const Eigen::Vector3f& X, const Eigen::Vector3f& Y,
                                                        const Eigen::Vector3f& Z, const pcl::ModelCoefficients::Ptr& plane,
                                                        const pcl::ModelCoefficients::Ptr& cylinder) {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) return;

    // ================= 基本设置 =================
    ofs << "figure; hold on; grid on; axis equal;\n";
    ofs << "view(3); rotate3d on;\n";
    ofs << "daspect([1 1 1]); axis auto;\n";

    // ================= 基坐标系 =================
    ofs << "quiver3(0,0,0,100,0,0,'r','LineWidth',2);\n";
    ofs << "quiver3(0,0,0,0,100,0,'g','LineWidth',2);\n";
    ofs << "quiver3(0,0,0,0,0,100,'b','LineWidth',2);\n";

    // ================= 点 =================
    ofs << "P0=[" << P0.x() << "," << P0.y() << "," << P0.z() << "];\n";
    ofs << "P1=[" << P1.x() << "," << P1.y() << "," << P1.z() << "];\n";
    ofs << "mid=[" << mid.x() << "," << mid.y() << "," << mid.z() << "];\n";

    ofs << "plot3([P0(1) P1(1)],[P0(2) P1(2)],[P0(3) P1(3)],'k','LineWidth',3);\n";
    ofs << "scatter3(P0(1),P0(2),P0(3),80,'filled');\n";
    ofs << "scatter3(P1(1),P1(2),P1(3),80,'filled');\n";

    // ================= 工具坐标系 =================
    float s = 80.0f;
    ofs << "X=[" << X.x() << "," << X.y() << "," << X.z() << "];\n";
    ofs << "Y=[" << Y.x() << "," << Y.y() << "," << Y.z() << "];\n";
    ofs << "Z=[" << Z.x() << "," << Z.y() << "," << Z.z() << "];\n";

    ofs << "quiver3(mid(1),mid(2),mid(3),X(1),X(2),X(3)," << s << ",'r','LineWidth',3);\n";
    ofs << "quiver3(mid(1),mid(2),mid(3),Y(1),Y(2),Y(3)," << s << ",'g','LineWidth',3);\n";
    ofs << "quiver3(mid(1),mid(2),mid(3),Z(1),Z(2),Z(3)," << s << ",'b','LineWidth',3);\n";

    // ================= 平面 =================
    if (plane && plane->values.size() == 4) {
        float a = plane->values[0];
        float b = plane->values[1];
        float c = plane->values[2];
        float d = plane->values[3];

        ofs << "[xx,yy]=meshgrid(-1000:50:1000);\n";
        ofs << "zz=(-" << a << "*xx-" << b << "*yy-" << d << ")/" << c << ";\n";
        ofs << "surf(xx,yy,zz,'FaceAlpha',0.3,'EdgeColor','none','FaceColor',[0.8 0.8 1]);\n";
    }

    // ================= 圆柱（任意方向） =================
    if (cylinder && cylinder->values.size() == 7) {
        float x0 = cylinder->values[0];
        float y0 = cylinder->values[1];
        float z0 = cylinder->values[2];

        float dx = cylinder->values[3];
        float dy = cylinder->values[4];
        float dz = cylinder->values[5];

        float r = cylinder->values[6];

        // 单位化
        Eigen::Vector3f axis(dx, dy, dz);
        axis.normalize();

        ofs << "axis_vec=[" << axis.x() << "," << axis.y() << "," << axis.z() << "];\n";

        ofs << "[theta,z]=meshgrid(linspace(0,2*pi,40),linspace(-200,200,40));\n";
        ofs << "r=" << r << ";\n";
        ofs << "Xc=r*cos(theta);\n";
        ofs << "Yc=r*sin(theta);\n";
        ofs << "Zc=z;\n";

        // Rodrigues旋转：Z轴 → axis_vec
        ofs << "k=[0 0 1];\n";
        ofs << "v=cross(k,axis_vec);\n";
        ofs << "s=norm(v);\n";
        ofs << "c=dot(k,axis_vec);\n";
        ofs << "vx=[0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];\n";
        ofs << "R=eye(3)+vx+vx*vx*((1-c)/(s^2+1e-8));\n";

        ofs << "pts=R*[Xc(:)';Yc(:)';Zc(:)'];\n";
        ofs << "Xr=reshape(pts(1,:),size(Xc))+" << x0 << ";\n";
        ofs << "Yr=reshape(pts(2,:),size(Yc))+" << y0 << ";\n";
        ofs << "Zr=reshape(pts(3,:),size(Zc))+" << z0 << ";\n";

        ofs << "surf(Xr,Yr,Zr,'FaceAlpha',0.3,'EdgeColor','none','FaceColor',[1 0.7 0.7]);\n";
    }

    // ================= 光照 =================
    ofs << "camlight;\n";
    ofs << "lighting gouraud;\n";

    // ================= 坐标标签 =================
    ofs << "xlabel('X'); ylabel('Y'); zlabel('Z');\n";
    ofs << "title('Weld Pose Visualization');\n";

    ofs.close();
}
// 对所有焊缝进行后撤
void LargeWorkpieceTrajectoryPlanning::applyWeldGunWithdraw(robotPose& pose, double withdrawDistance) {
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
Eigen::Vector3d LargeWorkpieceTrajectoryPlanning::abcToDirection(double a, double b, double c) {
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
// // 焊缝误差补偿 (真实坐标系)
// void LargeWorkpieceTrajectoryPlanning::seamsErrorCompensate(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo) {
//     if (trajectoryConfig.robotType == MyToolFunc::getRobotTypeString(ROBOT_TYPE::AN_CHUAN)) {
//         // 安川机器人
//         /*
//          *         YASKAWA (安川机器人)
//          *
//          *             ㊧ FRONT ㊨
//          *                  ↑ X
//          *                  |
//          *                  |
//          *   ㊨  Y          |             ㊧
//          *  LEFT ←----------| 0         RIGHT
//          *   ㊧                           ㊨
//          *
//          *               ☴ ☲ ☷
//          *               ☳ ☯ ☱
//          *               ☶ ☵ ☰
//          *Region1 机器人工件的  ㊧
//          *Region2 机器人工件的  ㊨
//          */
//         for (auto& info : weldSeamInfo) {
//             if (info->detectSuccFlag == true && info->weldEndPointsInRobot != nullptr &&
//                 info->weldEndPointsInRobot->size() == 2) {
//                 for (auto& end : *(info->weldEndPointsInRobot)) {
//                     end = MyToolFunc::transformSinglePoint(end, trajectoryConfig.leftErrorCompensationMatrix);
//                     if (info->weldType == WELD_TYPE::FRONT_CORNER_BUTT) {
//                         if (end.x < 0) {
//                             end.x += settingPara.Front_Region1_X_Shift;
//                             end.y += settingPara.Front_Region1_Y_Shift;
//                             end.z += settingPara.Front_Region1_Z_Shift;
//                         } else {
//                             end.x += settingPara.Front_Region2_X_Shift;
//                             end.y += settingPara.Front_Region2_Y_Shift;
//                             end.z += settingPara.Front_Region2_Z_Shift;
//                         }
//                     } else if (info->weldType == WELD_TYPE::BACK_CORNER_BUTT) {
//                         if (end.x < 0) {
//                             end.x += settingPara.Back_Region1_X_Shift;
//                             end.y += settingPara.Back_Region1_Y_Shift;
//                             end.z += settingPara.Back_Region1_Z_Shift;
//                         } else {
//                             end.x += settingPara.Back_Region2_X_Shift;
//                             end.y += settingPara.Back_Region2_Y_Shift;
//                             end.z += settingPara.Back_Region2_Z_Shift;
//                         }
//                     } else if (info->weldType == WELD_TYPE::FRONT_BEAM_BUTT &&
//                                info->weldAreaType == WELD_AREA_TYPE::FRONT_UP_BEAM) {
//                         if (end.x < 0) {
//                             end.x += settingPara.Front_Beam_Region1_X_Shift;
//                             end.y += settingPara.Front_Beam_Region1_Y_Shift;
//                             end.z += settingPara.Front_Beam_Region1_Z_Shift;
//                         } else {
//                             end.x += settingPara.Front_Beam_Region2_X_Shift;
//                             end.y += settingPara.Front_Beam_Region2_Y_Shift;
//                             end.z += settingPara.Front_Beam_Region2_Z_Shift;
//                         }
//                     } else if (info->weldType == WELD_TYPE::FRONT_HORIZONTAL_FILLET &&
//                                info->weldAreaType == WELD_AREA_TYPE::FRONT_UP_BEAM) {
//                         if (end.x < 0) {
//                             end.x += settingPara.Front_L_Beam_H_X_Shift;
//                             end.y += settingPara.Front_L_Beam_H_Y_Shift;
//                             end.z += settingPara.Front_L_Beam_H_Z_Shift;
//                         } else {
//                             end.x += settingPara.Front_R_Beam_H_X_Shift;
//                             end.y += settingPara.Front_R_Beam_H_Y_Shift;
//                             end.z += settingPara.Front_R_Beam_H_Z_Shift;
//                         }
//                     } else if (info->weldType == WELD_TYPE::FRONT_VERTICAL_FILLET &&
//                                info->weldAreaType == WELD_AREA_TYPE::FRONT_UP_BEAM) {
//                         if (end.x < 0) {
//                             end.x += settingPara.Front_L_Beam_V_X_Shift;
//                             end.y += settingPara.Front_L_Beam_V_Y_Shift;
//                             end.z += settingPara.Front_L_Beam_V_Z_Shift;
//                         } else {
//                             end.x += settingPara.Front_R_Beam_V_X_Shift;
//                             end.y += settingPara.Front_R_Beam_V_Y_Shift;
//                             end.z += settingPara.Front_R_Beam_V_Z_Shift;
//                         }
//                     } else if (info->weldType == WELD_TYPE::FRONT_HORIZONTAL_FILLET &&
//                                info->weldAreaType == WELD_AREA_TYPE::FRONT_DOWN_BEAM) {
//                         if (end.x < 0) {
//                             end.x += settingPara.Front_L_Beam_DH_X_Shift;
//                             end.y += settingPara.Front_L_Beam_DH_Y_Shift;
//                             end.z += settingPara.Front_L_Beam_DH_Z_Shift;
//                         } else {
//                             end.x += settingPara.Front_R_Beam_DH_X_Shift;
//                             end.y += settingPara.Front_R_Beam_DH_Y_Shift;
//                             end.z += settingPara.Front_R_Beam_DH_Z_Shift;
//                         }
//                     } else if (info->weldType == WELD_TYPE::FRONT_VERTICAL_FILLET &&
//                                info->weldAreaType == WELD_AREA_TYPE::FRONT_DOWN_BEAM) {
//                         if (end.x < 0) {
//                             end.x += settingPara.Front_L_Beam_DV_X_Shift;
//                             end.y += settingPara.Front_L_Beam_DV_Y_Shift;
//                             end.z += settingPara.Front_L_Beam_DV_Z_Shift;
//                         } else {
//                             end.x += settingPara.Front_R_Beam_DV_X_Shift;
//                             end.y += settingPara.Front_R_Beam_DV_Y_Shift;
//                             end.z += settingPara.Front_R_Beam_DV_Z_Shift;
//                         }
//                     } else if (info->weldType == WELD_TYPE::BACK_BEAM_BUTT) {
//                         if (end.x < 0) {
//                             end.x += settingPara.Back_Beam_Region1_X_Shift;
//                             end.y += settingPara.Back_Beam_Region1_Y_Shift;
//                             end.z += settingPara.Back_Beam_Region1_Z_Shift;
//                         } else {
//                             end.x += settingPara.Back_Beam_Region2_X_Shift;
//                             end.y += settingPara.Back_Beam_Region2_Y_Shift;
//                             end.z += settingPara.Back_Beam_Region2_Z_Shift;
//                         }
//                     }
//                     if (info->weldType == WELD_TYPE::FRONT_HORIZONTAL_FILLET) {
//                         end.z -= 4;  // TODO 临时误差补偿
//                     }
//                 }
//             }
//         }
//     } else if (workpieceSide == WORKPIECE_SIDE_OF_ROBOT::RIGHT) {
//         for (auto& info : weldSeamInfo) {
//             if (info->detectSuccFlag == true && info->weldEndPointsInRobot != nullptr &&
//                 info->weldEndPointsInRobot->size() == 2) {
//                 for (auto& end : *(info->weldEndPointsInRobot)) {
//                     end = MyToolFunc::transformSinglePoint(end, trajectoryConfig.rightErrorCompensationMatrix);
//                     if (info->weldType == WELD_TYPE::FRONT_CORNER_BUTT) {
//                         if (end.x > 0) {
//                             end.x += settingPara.Front_Region1_X_Shift_R;
//                             end.y += settingPara.Front_Region1_Y_Shift_R;
//                             end.z += settingPara.Front_Region1_Z_Shift_R;
//                         } else {
//                             end.x += settingPara.Front_Region2_X_Shift_R;
//                             end.y += settingPara.Front_Region2_Y_Shift_R;
//                             end.z += settingPara.Front_Region2_Z_Shift_R;
//                         }
//                     } else if (info->weldType == WELD_TYPE::BACK_CORNER_BUTT) {
//                         if (end.x > 0) {
//                             end.x += settingPara.Back_Region1_X_Shift_R;
//                             end.y += settingPara.Back_Region1_Y_Shift_R;
//                             end.z += settingPara.Back_Region1_Z_Shift_R;
//                         } else {
//                             end.x += settingPara.Back_Region2_X_Shift_R;
//                             end.y += settingPara.Back_Region2_Y_Shift_R;
//                             end.z += settingPara.Back_Region2_Z_Shift_R;
//                         }
//                     } else if (info->weldType == WELD_TYPE::FRONT_BEAM_BUTT ||
//                                info->weldType == WELD_TYPE::FRONT_HORIZONTAL_FILLET ||
//                                info->weldType == WELD_TYPE::FRONT_VERTICAL_FILLET) {
//                         if (end.x > 0) {
//                             end.x += settingPara.Front_Beam_Region1_X_Shift_R;
//                             end.y += settingPara.Front_Beam_Region1_Y_Shift_R;
//                             end.z += settingPara.Front_Beam_Region1_Z_Shift_R;
//                         } else {
//                             end.x += settingPara.Front_Beam_Region2_X_Shift_R;
//                             end.y += settingPara.Front_Beam_Region2_Y_Shift_R;
//                             end.z += settingPara.Front_Beam_Region2_Z_Shift_R;
//                         }
//                     } else if (info->weldType == WELD_TYPE::BACK_BEAM_BUTT) {
//                         if (end.x > 0) {
//                             end.x += settingPara.Back_Beam_Region1_X_Shift_R;
//                             end.y += settingPara.Back_Beam_Region1_Y_Shift_R;
//                             end.z += settingPara.Back_Beam_Region1_Z_Shift_R;
//                         } else {
//                             end.x += settingPara.Back_Beam_Region2_X_Shift_R;
//                             end.y += settingPara.Back_Beam_Region2_Y_Shift_R;
//                             end.z += settingPara.Back_Beam_Region2_Z_Shift_R;
//                         }
//                     }
//                     if (info->weldType == WELD_TYPE::FRONT_HORIZONTAL_FILLET) {
//                         end.z -= 4;  // TODO 临时误差补偿
//                     }
//                 }
//             }
//         }
//     }
// }
