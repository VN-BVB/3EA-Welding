#include "robotTrajectoryPlanning/LargeWorkpieceTrajectoryPlanning.h"

#include "plog/Log.h"
#include "robotFactory/AbstractRobot.h"
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

    // 发出规划完成的焊缝
    emit sendDetSeamWithSeg(weldSeamInfo);
}

void LargeWorkpieceTrajectoryPlanning::write2File(const std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo,
                                                  int endOfLeftSeams) {
    // 实现大型工件的文件写入逻辑
    // ...

    emit sendTrajectoryPlanOver();
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
