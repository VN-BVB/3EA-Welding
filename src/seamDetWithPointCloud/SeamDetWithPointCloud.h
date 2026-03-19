#ifndef SEAMDETWITHPOINTCLOUD_H
#define SEAMDETWITHPOINTCLOUD_H

#include <plog/Log.h>

#include <QObject>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include "utils/common/ThreadPool.h"

class AbstractSeamDet;
class BeamButtSeamsDet;
class CornerButtSeamsDet;
class DownBeamFilletSeamsDet;
class UpBeamFilletSeamsDet;
class WeldSeamInfo;

class SeamDetWithPointCloud : public QObject {
    Q_OBJECT
public:
    explicit SeamDetWithPointCloud(QObject* parent = nullptr);

signals:
    void sendDetSeamWithPointCloud(std::vector<std::shared_ptr<WeldSeamInfo>> weldInfo);  // 发出求解完成的焊缝

public slots:
    void whenDetSeamWithPointCloud(std::vector<std::shared_ptr<WeldSeamInfo>> weldAreaInfo);    // 求解焊缝
    void whenDetSeamWithPointCloudLW(std::vector<std::shared_ptr<WeldSeamInfo>> weldAreaInfo);  // 求解焊缝
private:
    std::shared_ptr<AbstractSeamDet> beamButtSeamsDet{nullptr};        // 横梁对接求解类
    std::shared_ptr<AbstractSeamDet> cornerButtSeamsDet{nullptr};      // 边角对接求解类
    std::shared_ptr<AbstractSeamDet> downBeamFilletSeamsDet{nullptr};  // 倒立角接焊缝求解类
    std::shared_ptr<AbstractSeamDet> upBeamFilletSeamsDet{nullptr};    // 正立交接焊缝求解类

    std::shared_ptr<AbstractSeamDet> platePlateFilletSeamsDet{nullptr};  // 板对板角接焊缝求解类
    std::shared_ptr<AbstractSeamDet> tubePlateButtSeamsDet{nullptr};     // 管对板对接焊缝求解类
    std::shared_ptr<AbstractSeamDet> tubePlateFilletSeamsDet{nullptr};   // 管对板角接焊缝求解类
    std::shared_ptr<AbstractSeamDet> tubeTubeButtSeamsDet{nullptr};      // 管对管对接焊缝求解类
    std::shared_ptr<AbstractSeamDet> tubeTubeFilletSeamsDet{nullptr};    // 管对管角接焊缝求解类

    bool beamButtFlag = false;
    bool cornerButtFlag = false;
    bool downBeamFilletFlag = false;
    bool upBeamFilletFlag = false;

    std::shared_ptr<AbstractSeamDet> beamButtSeamsDetSec{nullptr};        // 横梁对接求解类
    std::shared_ptr<AbstractSeamDet> cornerButtSeamsDetSec{nullptr};      // 边角对接求解类
    std::shared_ptr<AbstractSeamDet> downBeamFilletSeamsDetSec{nullptr};  // 倒立角接焊缝求解类
    std::shared_ptr<AbstractSeamDet> upBeamFilletSeamsDetSec{nullptr};    // 正立交接焊缝求解类

    std::vector<std::shared_ptr<WeldSeamInfo>> tempWeldSeamsInfo;  // 临时焊缝信息

    std::vector<std::shared_ptr<WeldSeamInfo>> beamButtInfo;        // 横梁对接焊缝
    std::vector<std::shared_ptr<WeldSeamInfo>> cornerButtInfo;      // 边角对接焊缝
    std::vector<std::shared_ptr<WeldSeamInfo>> beamUpFilletInfo;    // 横梁正立角接焊缝
    std::vector<std::shared_ptr<WeldSeamInfo>> beamDownFilletInfo;  // 横梁倒立交接焊缝

    std::vector<std::shared_ptr<WeldSeamInfo>> beamButtInfoSec;        // 横梁对接焊缝
    std::vector<std::shared_ptr<WeldSeamInfo>> cornerButtInfoSec;      // 边角对接焊缝
    std::vector<std::shared_ptr<WeldSeamInfo>> beamUpFilletInfoSec;    // 横梁正立角接焊缝
    std::vector<std::shared_ptr<WeldSeamInfo>> beamDownFilletInfoSec;  // 横梁倒立交接焊缝

    std::vector<std::shared_ptr<WeldSeamInfo>> tubePlateButtInfo;  

    // 用于接收线程池返回值的future对象
    std::vector<std::future<std::vector<std::shared_ptr<WeldSeamInfo>>>> beamButtFuture;
    std::vector<std::future<std::vector<std::shared_ptr<WeldSeamInfo>>>> cornerButtFuture;
    std::vector<std::future<std::vector<std::shared_ptr<WeldSeamInfo>>>> beamDownFilletFuture;
    std::vector<std::future<std::vector<std::shared_ptr<WeldSeamInfo>>>> beamUpFilletFuture;
    std::vector<std::future<std::vector<std::shared_ptr<WeldSeamInfo>>>> tubePlateButtFuture;

    ThreadPool* threadPool = new ThreadPool(4);  // 线程池
};

#endif  // SEAMDETWITHPOINTCLOUD_H
