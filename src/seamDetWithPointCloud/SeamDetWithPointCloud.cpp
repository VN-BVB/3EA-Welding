#include "SeamDetWithPointCloud.h"

#include "seamDetWithPointCloud/AbstractSeamDet.h"
#include "seamDetWithPointCloud/gantrayFrameDet/platePlateFilletSeamsDet/PlatePlateFilletSeamsDet.h"
#include "seamDetWithPointCloud/gantrayFrameDet/tubePlateFilletSeamsDet/TubePlateFilletSeamsDet.h"
#include "seamDetWithPointCloud/gantrayFrameDet/tubeSidePlateFilletSeamsDet/TubeSidePlateFilletSeamsDet.h"
#include "seamDetWithPointCloud/steelAngelDet/beamButtSeamsDet/BeamButtSeamsDet.h"
#include "seamDetWithPointCloud/steelAngelDet/cornerButtSeamsDet/CornerButtSeamsDet.h"
#include "seamDetWithPointCloud/steelAngelDet/downBeamFilletSeamsDet/DownBeamFilletSeamsDet.h"
#include "seamDetWithPointCloud/steelAngelDet/upBeamFilletSeamsDet/UpBeamFilletSeamsDet.h"
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
      downBeamFilletSeamsDetSec(std::make_shared<DownBeamFilletSeamsDet>(nullptr)) {
    initGantrayFrameSeamsDet();
}
void SeamDetWithPointCloud::initGantrayFrameSeamsDet() {
    gantrayFrameSeamsDet.clear();

    gantrayFrameSeamsDet[WELD_AREA_TYPE::Plate_Plate_F] = []() { return std::make_shared<PlatePlateFilletSeamsDet>(nullptr); };
    gantrayFrameSeamsDet[WELD_AREA_TYPE::TubeSide_Plate_F] = []() { return std::make_shared<TubeSidePlateFilletSeamsDet>(nullptr); };
    gantrayFrameSeamsDet[WELD_AREA_TYPE::Tube_Plate_F] = []() { return std::make_shared<TubePlateFilletSeamsDet>(nullptr); };
    // gantrayFrameSeamsDet[WELD_AREA_TYPE::Tube_Tube_F] = []() {
    //     return std::make_shared<TubeTubeFilletSeamsDet>(nullptr);
    // };
}
std::shared_ptr<AbstractSeamDet> SeamDetWithPointCloud::createSeamDet(WELD_AREA_TYPE type) {
    auto it = gantrayFrameSeamsDet.find(type);
    if (it == gantrayFrameSeamsDet.end()) {
        PLOGE << "未找到对应焊缝类型: " << static_cast<int>(type);
        return nullptr;
    }
    return it->second();
}
// 求解焊缝
void SeamDetWithPointCloud::whenDetSeamWithPointCloudGF(std::vector<std::shared_ptr<WeldSeamInfo>> weldAreaInfo) {
    // 清空各类别信息容器

    tempWeldSeamsInfo.clear();
    WeldSeamsfutures.clear();

    for (auto& info : weldAreaInfo) {
        if (info->weldAreaType == WELD_AREA_TYPE::TubeSide_Plate_F) {
            PLOGD << "管侧板角接焊缝";
        } else if (info->weldAreaType == WELD_AREA_TYPE::Plate_Plate_F) {
            PLOGD << "板板角接焊缝";
        } else if (info->weldAreaType == WELD_AREA_TYPE::Tube_Plate_F) {
            PLOGD << "管板角接焊缝";
        }
        WeldSeamsfutures.emplace_back(threadPool->addTask([this, info]() {
            auto det = createSeamDet(info->weldAreaType);
            std::vector<std::shared_ptr<WeldSeamInfo>> tmp{info};
            return det->solveSeamsEndPoints(tmp);
        }));
    }

    // 等待所有任务完成
    threadPool->waitAll();

    // 取出线程池的计算结果
    for (auto& future : WeldSeamsfutures) {
        if (future.valid()) {
            auto res = future.get();
            tempWeldSeamsInfo.insert(tempWeldSeamsInfo.end(), res.begin(), res.end());
        }
    }
    // 后处理：按类型拆分焊缝
    splitWeldSeamsInPlace(tempWeldSeamsInfo);
    PLOGD << "发出焊缝数量" << tempWeldSeamsInfo.size();
    // 发出求解出的焊缝结果
    emit sendDetSeamWithPointCloud(tempWeldSeamsInfo);

    // 打印焊缝信息
    PLOGD << "共计算出" << tempWeldSeamsInfo.size() << "个焊缝信息, 具体信息: ";
    for (auto& info : tempWeldSeamsInfo) {
        PLOGD << "区域编号: " << info->areaNum << ", 焊缝类型: " << info->weldType << ", 检测是否成功: " << info->detectSuccFlag;
        if (info->detectSuccFlag == true) {
            PLOGD << "   焊缝端点坐标: (" << info->weldEndPointsInCamera->at(0).x << " " << info->weldEndPointsInCamera->at(0).y << " "
                  << info->weldEndPointsInCamera->at(0).z << ") (" << info->weldEndPointsInCamera->at(1).x << " "
                  << info->weldEndPointsInCamera->at(1).y << " " << info->weldEndPointsInCamera->at(1).z << ")";
            if (info->weldCoeff != nullptr && info->weldCoeff->values.size() == 4) {
                PLOGD << "   焊缝所在平面: (" << info->weldCoeff->values[0] << " " << info->weldCoeff->values[1] << " " << info->weldCoeff->values[2]
                      << " " << info->weldCoeff->values[3] << ")";
            }
            if (info->seamsLineToVal != nullptr && info->seamsLineToVal->values.size() == 6) {
                PLOGD << "   焊缝验证直线: (" << info->seamsLineToVal->values[0] << " " << info->seamsLineToVal->values[1] << " "
                      << info->seamsLineToVal->values[2] << " " << info->seamsLineToVal->values[3] << " " << info->seamsLineToVal->values[4] << " "
                      << info->seamsLineToVal->values[5] << ")";
            }
        }
    }
}
void SeamDetWithPointCloud::splitWeldSeamsInPlace(std::vector<std::shared_ptr<WeldSeamInfo>>& infos) {
    std::vector<std::shared_ptr<WeldSeamInfo>> newInfos;
    newInfos.reserve(infos.size() * 2);
    for (auto& info : infos) {
        if (!info) continue;
        std::vector<std::shared_ptr<WeldSeamInfo>> splitRes;
        if (info->weldType == Tube_Plate_Fillet) {
            if (!info->weldEndPointsInCamera || info->weldEndPointsInCamera->size() < 3) {
                splitRes.push_back(info);
            } else {
                const auto& pts = *(info->weldEndPointsInCamera);
                int N = pts.size();

                // ---------- 1. 找 Z 最小 ----------
                int idx_min = 0;
                float z_min = pts[0].z;

                for (int i = 1; i < N; ++i) {
                    if (pts[i].z < z_min) {
                        z_min = pts[i].z;
                        idx_min = i;
                    }
                }

                // ---------- 2. 判断是否切分 ----------
                int margin = 1;

                if (idx_min <= margin || idx_min >= N - 1 - margin) {
                    splitRes.push_back(info);
                } else {
                    // ---------- 3. 构造两段 ----------

                    // start → a
                    auto info1 = info->clone();
                    info1->weldEndPointsInCamera = std::make_shared<std::vector<pcl::PointXYZ>>(pts.begin(), pts.begin() + idx_min + 1);

                    // a → end
                    auto info2 = info->clone();
                    info2->weldEndPointsInCamera = std::make_shared<std::vector<pcl::PointXYZ>>(pts.begin() + idx_min, pts.end());

                    splitRes.push_back(info1);
                    splitRes.push_back(info2);
                }
            }
        } else {
            splitRes.push_back(info);
        }

        newInfos.insert(newInfos.end(), splitRes.begin(), splitRes.end());
    }

    infos.swap(newInfos);
}
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
        cornerButtFuture.push_back(threadPool->addTask([this]() { return cornerButtSeamsDet->solveSeamsEndPoints(cornerButtInfo); }));
    }
    if (!beamDownFilletInfo.empty()) {
        beamDownFilletFuture.push_back(threadPool->addTask([this]() { return downBeamFilletSeamsDet->solveSeamsEndPoints(beamDownFilletInfo); }));
    }

    if (!beamButtInfoSec.empty()) {
        beamButtFuture.push_back(threadPool->addTask([this]() { return beamButtSeamsDetSec->solveSeamsEndPoints(beamButtInfoSec); }));
    }
    if (!cornerButtInfoSec.empty()) {
        cornerButtFuture.push_back(threadPool->addTask([this]() { return cornerButtSeamsDetSec->solveSeamsEndPoints(cornerButtInfoSec); }));
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
        beamUpFilletFuture.push_back(threadPool->addTask([this]() { return upBeamFilletSeamsDet->solveSeamsEndPoints(beamUpFilletInfo); }));
    }
    if (!beamUpFilletInfoSec.empty()) {
        beamUpFilletFuture.push_back(threadPool->addTask([this]() { return upBeamFilletSeamsDetSec->solveSeamsEndPoints(beamUpFilletInfoSec); }));
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
            PLOGD << "   焊缝端点坐标: (" << info->weldEndPointsInCamera->at(0).x << " " << info->weldEndPointsInCamera->at(0).y << " "
                  << info->weldEndPointsInCamera->at(0).z << ") (" << info->weldEndPointsInCamera->at(1).x << " "
                  << info->weldEndPointsInCamera->at(1).y << " " << info->weldEndPointsInCamera->at(1).z << ")";
            if (info->weldCoeff != nullptr && info->weldCoeff->values.size() == 4) {
                PLOGD << "   焊缝所在平面: (" << info->weldCoeff->values[0] << " " << info->weldCoeff->values[1] << " " << info->weldCoeff->values[2]
                      << " " << info->weldCoeff->values[3] << ")";
            }
            if (info->seamsLineToVal != nullptr && info->seamsLineToVal->values.size() == 6) {
                PLOGD << "   焊缝验证直线: (" << info->seamsLineToVal->values[0] << " " << info->seamsLineToVal->values[1] << " "
                      << info->seamsLineToVal->values[2] << " " << info->seamsLineToVal->values[3] << " " << info->seamsLineToVal->values[4] << " "
                      << info->seamsLineToVal->values[5] << ")";
            }
        }
    }
}
