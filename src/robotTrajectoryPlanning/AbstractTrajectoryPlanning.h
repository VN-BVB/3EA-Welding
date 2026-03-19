#ifndef ABSTRACTTRAJECTORYPLANNING_H
#define ABSTRACTTRAJECTORYPLANNING_H

#include <QObject>
#include <memory>
#include <vector>

#include "utils/common/WeldSeamInfo.h"

// 前向声明
class TrajectoryPlanningConfig;
class SettingPara;

class AbstractTrajectoryPlanning : public QObject {
    Q_OBJECT
public:
    explicit AbstractTrajectoryPlanning(QObject* parent = nullptr);
    virtual ~AbstractTrajectoryPlanning() = default;
    virtual void write2File(const std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo,
                            int endOfLeftSeams) = 0;  // 焊缝写入文件
    int endOfLeftSeamSerial = 0;                      // 左侧最后一个焊缝索引
    // 纯虚函数，定义统一的轨迹规划接口
public slots:
    virtual void whenPlanningTrajectory(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo) = 0;

protected:
    // 配置初始化方法（供派生类调用）
    void initConfig();
    void readConfig();
    void writeConfig();
    void printConfig();

    // 配置成员变量（引用类型，所有派生类共享）
    TrajectoryPlanningConfig& trajectoryConfig;
    SettingPara& settingPara;

signals:
    void sendDetSeamWithSeg(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo);
    void sendTrajectoryPlanOver();

    friend class RailWeldingSystem;
};

#endif  // ABSTRACTTRAJECTORYPLANNING_H
