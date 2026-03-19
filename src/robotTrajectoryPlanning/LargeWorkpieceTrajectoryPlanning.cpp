#include "robotTrajectoryPlanning/LargeWorkpieceTrajectoryPlanning.h"

#include "plog/Log.h"
#include "robotTrajectoryPlanning/config/TrajectoryPlanningConfig.h"

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
    // ########################### 转换坐标点到机器人基坐标系下 ###########################
    // 实现大型工件的轨迹规划逻辑
    // std::cout << "当前机器人位姿矩阵" << trajectoryConfig.matrixEnd2Base;
    this->transSeams2Base(weldSeamInfo);
    PLOGD << "转到机器人基坐标系下后的焊缝点: ";  // 打印操作后的焊缝
    for (auto& info : weldSeamInfo) {
        if (info->detectSuccFlag == true && info->weldEndPointsInRobot != nullptr && info->weldEndPointsInRobot->size() == 2) {
            PLOGD << info->weldEndPointsInRobot->at(0) << " " << info->weldEndPointsInRobot->at(1);
        }
    }

    // 发出规划完成的焊缝
    // emit sendDetSeamWithSeg(weldSeamInfo);
}

void LargeWorkpieceTrajectoryPlanning::write2File(const std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo,
                                                  int endOfLeftSeams) {
    // 实现大型工件的文件写入逻辑
    // ...

    emit sendTrajectoryPlanOver();
}
// 将焊缝点转到机器人基坐标系
void LargeWorkpieceTrajectoryPlanning::transSeams2Base(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo) {
    for (auto& info : weldSeamInfo) {
        // 转换焊缝点
        if (info->detectSuccFlag == true && info->weldEndPointsInCamera != nullptr && info->weldEndPointsInCamera->size() >= 2) {
            info->weldEndPointsInRobot = std::make_shared<std::vector<pcl::PointXYZ>>();
            info->weldEndPointsInRobot->push_back(
                MyToolFunc::transformSinglePoint(info->weldEndPointsInCamera->at(0), trajectoryConfig.matrixEyeHand));
            info->weldEndPointsInRobot->push_back(
                MyToolFunc::transformSinglePoint(info->weldEndPointsInCamera->at(1), trajectoryConfig.matrixEyeHand));
        }
        // // 转换区域点
        // if (info->weldAreaPointCloud && info->weldAreaPointCloud->size() > 0) {
        //     info->weldAreaPointCloud = MyToolFunc::transformPointCloud(info->weldAreaPointCloud,
        //     trajectoryConfig.matrixEyeHand);
        // }
    }
    if (trajectoryConfig.handEyeType == MyToolFunc::getHandTypeTypeString(HAND_EYE_TYPE::EYE_IN_HAND)) {
        for (auto& info : weldSeamInfo) {
            // 转换焊缝点
            if (info->detectSuccFlag == true && info->weldEndPointsInRobot != nullptr &&
                info->weldEndPointsInRobot->size() == info->weldEndPointsInCamera->size()) {
                info->weldEndPointsInRobot->at(0) =
                    MyToolFunc::transformSinglePoint(info->weldEndPointsInRobot->at(0), trajectoryConfig.matrixEnd2Base);
                info->weldEndPointsInRobot->at(1) =
                    MyToolFunc::transformSinglePoint(info->weldEndPointsInRobot->at(1), trajectoryConfig.matrixEnd2Base);
            }
            // // 转换区域点
            // if (info->weldAreaPointCloud && info->weldAreaPointCloud->size() > 0) {
            //     info->weldAreaPointCloud =
            //         MyToolFunc::transformPointCloud(info->weldAreaPointCloud, trajectoryConfig.matrixEnd2Base);
            // }
        }
    }
    // weldSeamInfo[0]->weldAreaPointCloud->height = 1;
    // weldSeamInfo[0]->weldAreaPointCloud->width = static_cast<uint32_t>(weldSeamInfo[0]->weldAreaPointCloud->size());
    // pcl::io::savePCDFile("./data/seamDetWithPointCloud/tubeSidePlateFilletSeamsDet/weldAreaPointCloudInBase.pcd",
    //                      *weldSeamInfo[0]->weldAreaPointCloud);
}
