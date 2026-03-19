#include "SeamDetWithPointCloud.h"

#include "seamDetWithPointCloud/AbstractSeamDet.h"
#include "seamDetWithPointCloud/steelAngelDet/beamButtSeamsDet/BeamButtSeamsDet.h"
#include "seamDetWithPointCloud/steelAngelDet/cornerButtSeamsDet/CornerButtSeamsDet.h"
#include "seamDetWithPointCloud/steelAngelDet/downBeamFilletSeamsDet/DownBeamFilletSeamsDet.h"
#include "seamDetWithPointCloud/steelAngelDet/upBeamFilletSeamsDet/UpBeamFilletSeamsDet.h"
#include "seamDetWithPointCloud/steelDefaultDet/tubeSidePlateFilletSeamsDet/TubeSidePlateFilletSeamsDet.h"
#include "utils/common/WeldSeamInfo.h"

SeamDetWithPointCloud::SeamDetWithPointCloud(QObject* parent)
    : QObject{parent},
      beamButtSeamsDet(std::make_shared<BeamButtSeamsDet>(nullptr)),
      cornerButtSeamsDet(std::make_shared<CornerButtSeamsDet>(nullptr)),
      upBeamFilletSeamsDet(std::make_shared<UpBeamFilletSeamsDet>(nullptr)),
      downBeamFilletSeamsDet(std::make_shared<DownBeamFilletSeamsDet>(nullptr)),
      beamButtSeamsDetSec(std::make_shared<BeamButtSeamsDet>(nullptr)),
      cornerButtSeamsDetSec(std::make_shared<CornerButtSeamsDet>(nullptr)),
      upBeamFilletSeamsDetSec(std::make_shared<UpBeamFilletSeamsDet>(nullptr)),
      downBeamFilletSeamsDetSec(std::make_shared<DownBeamFilletSeamsDet>(nullptr)),
      tubePlateButtSeamsDet(std::make_shared<TubeSidePlateFilletSeamsDet>(nullptr)) {}

// 求解焊缝
void SeamDetWithPointCloud::whenDetSeamWithPointCloud(std::vector<std::shared_ptr<WeldSeamInfo>> weldAreaInfo) {
    PLOGD << "点云方法焊缝检测开始";

    // 清空各类别信息容器
    beamButtInfo.clear();
    cornerButtInfo.clear();
    beamUpFilletInfo.clear();
    beamDownFilletInfo.clear();
    beamButtInfoSec.clear();
    cornerButtInfoSec.clear();
    beamUpFilletInfoSec.clear();
    beamDownFilletInfoSec.clear();

    beamButtFlag = false;
    cornerButtFlag = false;
    downBeamFilletFlag = false;
    upBeamFilletFlag = false;

    beamButtFuture.clear();
    cornerButtFuture.clear();
    beamDownFilletFuture.clear();
    beamUpFilletFuture.clear();

    tempWeldSeamsInfo.clear();

    // 区域分类
    for (int i = 0; i < weldAreaInfo.size(); i++) {
        if (weldAreaInfo[i]->weldAreaType == WELD_AREA_TYPE::BACK_BEAM) {  // 背面横梁
            PLOGD << "背面横梁焊缝";
            if (beamButtFlag == false) {
                beamButtFlag = true;
                beamButtInfo.push_back(weldAreaInfo[i]);
            } else {
                beamButtInfoSec.push_back(weldAreaInfo[i]);
            }
        } else if (weldAreaInfo[i]->weldAreaType == WELD_AREA_TYPE::FRONT_UP_BEAM) {  // 正面正立横梁 (正立角接先不计算)
            PLOGD << "正面正立横梁焊缝";
            if (beamButtFlag == false) {
                beamButtFlag = true;
                beamButtInfo.push_back(weldAreaInfo[i]);
            } else {
                beamButtInfoSec.push_back(weldAreaInfo[i]);
            }
        } else if (weldAreaInfo[i]->weldAreaType == WELD_AREA_TYPE::BACK_CORNER ||
                   weldAreaInfo[i]->weldAreaType == WELD_AREA_TYPE::FRONT_CORNER) {  // 边角
            PLOGD << "边角焊缝";
            if (cornerButtFlag == false) {
                cornerButtFlag = true;
                cornerButtInfo.push_back(weldAreaInfo[i]);
            } else {
                cornerButtInfoSec.push_back(weldAreaInfo[i]);
            }
        } else if (weldAreaInfo[i]->weldAreaType == WELD_AREA_TYPE::FRONT_DOWN_BEAM) {  // 正面倒立横梁
            PLOGD << "正面倒立横梁焊缝";
            if (downBeamFilletFlag == false) {
                downBeamFilletFlag = true;
                beamDownFilletInfo.push_back(weldAreaInfo[i]);
            } else {
                beamDownFilletInfoSec.push_back(weldAreaInfo[i]);
            }
        }
    }

    if (!beamButtInfo.empty()) {
        beamButtFuture.push_back(threadPool->addTask([this]() { return beamButtSeamsDet->solveSeamsEndPoints(beamButtInfo); }));
    }
    if (!cornerButtInfo.empty()) {
        cornerButtFuture.push_back(
            threadPool->addTask([this]() { return cornerButtSeamsDet->solveSeamsEndPoints(cornerButtInfo); }));
    }
    if (!beamDownFilletInfo.empty()) {
        beamDownFilletFuture.push_back(
            threadPool->addTask([this]() { return downBeamFilletSeamsDet->solveSeamsEndPoints(beamDownFilletInfo); }));
    }

    if (!beamButtInfoSec.empty()) {
        beamButtFuture.push_back(
            threadPool->addTask([this]() { return beamButtSeamsDetSec->solveSeamsEndPoints(beamButtInfoSec); }));
    }
    if (!cornerButtInfoSec.empty()) {
        cornerButtFuture.push_back(
            threadPool->addTask([this]() { return cornerButtSeamsDetSec->solveSeamsEndPoints(cornerButtInfoSec); }));
    }
    if (!beamDownFilletInfoSec.empty()) {
        beamDownFilletFuture.push_back(
            threadPool->addTask([this]() { return downBeamFilletSeamsDetSec->solveSeamsEndPoints(beamDownFilletInfoSec); }));
    }

    // 等待所有任务完成
    threadPool->waitAll();

    // 取出线程池的计算结果
    for (auto& future : beamButtFuture) {
        if (future.valid()) {
            std::vector<std::shared_ptr<WeldSeamInfo>> res = future.get();
            for (auto& seam : res) {
                tempWeldSeamsInfo.push_back(seam);
            }
        }
    }
    for (auto& future : cornerButtFuture) {
        if (future.valid()) {
            std::vector<std::shared_ptr<WeldSeamInfo>> res = future.get();
            for (auto& seam : res) {
                tempWeldSeamsInfo.push_back(seam);
            }
        }
    }
    for (auto& future : beamDownFilletFuture) {
        if (future.valid()) {
            std::vector<std::shared_ptr<WeldSeamInfo>> res = future.get();
            for (auto& seam : res) {
                tempWeldSeamsInfo.push_back(seam);
            }
        }
    }

    // 正面横梁正立角接焊缝需要等到对应位置的对接焊缝求解完成后计算
    for (int i = 0; i < tempWeldSeamsInfo.size(); i++) {
        if (tempWeldSeamsInfo[i]->weldAreaType == WELD_AREA_TYPE::FRONT_UP_BEAM) {
            if (upBeamFilletFlag == false) {
                upBeamFilletFlag = true;
                beamUpFilletInfo.push_back(tempWeldSeamsInfo[i]);
            } else {
                beamUpFilletInfoSec.push_back(tempWeldSeamsInfo[i]);
            }
        }
    }

    if (!beamUpFilletInfo.empty()) {
        beamUpFilletFuture.push_back(
            threadPool->addTask([this]() { return upBeamFilletSeamsDet->solveSeamsEndPoints(beamUpFilletInfo); }));
    }
    if (!beamUpFilletInfoSec.empty()) {
        beamUpFilletFuture.push_back(
            threadPool->addTask([this]() { return upBeamFilletSeamsDetSec->solveSeamsEndPoints(beamUpFilletInfoSec); }));
    }

    // 等待所有任务完成
    threadPool->waitAll();

    // 取出线程池的计算结果
    for (auto& future : beamUpFilletFuture) {
        if (future.valid()) {
            std::vector<std::shared_ptr<WeldSeamInfo>> res = future.get();
            for (auto& seam : res) {
                tempWeldSeamsInfo.push_back(seam);
            }
        }
    }

    // 发出求解出的焊缝结果
    emit sendDetSeamWithPointCloud(tempWeldSeamsInfo);

    // 打印焊缝信息
    PLOGD << "共计算出" << tempWeldSeamsInfo.size() << "个焊缝信息, 具体信息: ";
    for (auto& info : tempWeldSeamsInfo) {
        PLOGD << "区域编号: " << info->areaNum << ", 焊缝类型: " << info->weldType << ", 检测是否成功: " << info->detectSuccFlag;
        if (info->detectSuccFlag == true) {
            PLOGD << "   焊缝端点坐标: (" << info->weldEndPointsInCamera->at(0).x << " " << info->weldEndPointsInCamera->at(0).y
                  << " " << info->weldEndPointsInCamera->at(0).z << ") (" << info->weldEndPointsInCamera->at(1).x << " "
                  << info->weldEndPointsInCamera->at(1).y << " " << info->weldEndPointsInCamera->at(1).z << ")";
            if (info->weldPlane != nullptr && info->weldPlane->values.size() == 4) {
                PLOGD << "   焊缝所在平面: (" << info->weldPlane->values[0] << " " << info->weldPlane->values[1] << " "
                      << info->weldPlane->values[2] << " " << info->weldPlane->values[3] << ")";
            }
            if (info->seamsLineToVal != nullptr && info->seamsLineToVal->values.size() == 6) {
                PLOGD << "   焊缝验证直线: (" << info->seamsLineToVal->values[0] << " " << info->seamsLineToVal->values[1] << " "
                      << info->seamsLineToVal->values[2] << " " << info->seamsLineToVal->values[3] << " "
                      << info->seamsLineToVal->values[4] << " " << info->seamsLineToVal->values[5] << ")";
            }
        }
    }
}
// 求解焊缝
void SeamDetWithPointCloud::whenDetSeamWithPointCloudLW(std::vector<std::shared_ptr<WeldSeamInfo>> weldAreaInfo) {
    PLOGD << "点云方法焊缝检测开始";

    // 清空各类别信息容器
    tubePlateButtInfo.clear();
    tubePlateButtFuture.clear();
    tempWeldSeamsInfo.clear();

    // 区域分类
    for (int i = 0; i < weldAreaInfo.size(); i++) {
        if (weldAreaInfo[i]->weldAreaType == WELD_AREA_TYPE::TubeSide_Plate_F) {  // 背面横梁
            PLOGD << "管侧板角接焊缝";
            tubePlateButtInfo.push_back(weldAreaInfo[i]);
        }
    }

    if (!tubePlateButtInfo.empty()) {
        for (size_t i = 0; i < tubePlateButtInfo.size(); ++i) {
            auto info = tubePlateButtInfo[i];

            tubePlateButtFuture.push_back(threadPool->addTask([this, info]() {
                std::vector<std::shared_ptr<WeldSeamInfo>> tmp;
                tmp.push_back(info);
                return tubePlateButtSeamsDet->solveSeamsEndPoints(tmp);
            }));
        }
    }

    // 等待所有任务完成
    threadPool->waitAll();

    // 取出线程池的计算结果
    for (auto& future : tubePlateButtFuture) {
        if (future.valid()) {
            std::vector<std::shared_ptr<WeldSeamInfo>> res = future.get();
            for (auto& seam : res) {
                tempWeldSeamsInfo.push_back(seam);
            }
        }
    }
    // 发出求解出的焊缝结果
    emit sendDetSeamWithPointCloud(tempWeldSeamsInfo);

    // 打印焊缝信息
    PLOGD << "共计算出" << tempWeldSeamsInfo.size() << "个焊缝信息, 具体信息: ";
    for (auto& info : tempWeldSeamsInfo) {
        PLOGD << "区域编号: " << info->areaNum << ", 焊缝类型: " << info->weldType << ", 检测是否成功: " << info->detectSuccFlag;
        if (info->detectSuccFlag == true) {
            PLOGD << "   焊缝端点坐标: (" << info->weldEndPointsInCamera->at(0).x << " " << info->weldEndPointsInCamera->at(0).y
                  << " " << info->weldEndPointsInCamera->at(0).z << ") (" << info->weldEndPointsInCamera->at(1).x << " "
                  << info->weldEndPointsInCamera->at(1).y << " " << info->weldEndPointsInCamera->at(1).z << ")";
            if (info->weldPlane != nullptr && info->weldPlane->values.size() == 4) {
                PLOGD << "   焊缝所在平面: (" << info->weldPlane->values[0] << " " << info->weldPlane->values[1] << " "
                      << info->weldPlane->values[2] << " " << info->weldPlane->values[3] << ")";
            }
            if (info->seamsLineToVal != nullptr && info->seamsLineToVal->values.size() == 6) {
                PLOGD << "   焊缝验证直线: (" << info->seamsLineToVal->values[0] << " " << info->seamsLineToVal->values[1] << " "
                      << info->seamsLineToVal->values[2] << " " << info->seamsLineToVal->values[3] << " "
                      << info->seamsLineToVal->values[4] << " " << info->seamsLineToVal->values[5] << ")";
            }
        }
    }
}
