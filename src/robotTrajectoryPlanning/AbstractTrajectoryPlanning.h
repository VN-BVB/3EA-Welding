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
    enum class WORKPIECE_SIDE_OF_ROBOT {  // 当前工件位于机器人基座的方向
        FRONT,                            // 前方
        LEFT,                             // 左方
        RIGHT                             // 右方
    };
    enum SWING_WELD_ACTION {
        LINE_WELD = 0,                        // 直线焊接
        FRONT_LEFT_VERTICAL_SWING_WELD = 1,   // 机器人前方左侧竖直焊缝摆焊
        FRONT_RIGHT_VERTICAL_SWING_WELD = 2,  // 机器人前方右侧竖直焊缝摆焊
        LEFT_LEFT_VERTICAL_SWING_WELD = 3,    // 机器人左方左侧竖直焊缝摆焊
        LEFT_RIGHT_VERTICAL_SWING_WELD = 4,   // 机器人左方右侧竖直焊缝摆焊
        RIGHT_LEFT_VERTICAL_SWING_WELD = 5,   // 机器人右方左侧竖直焊缝摆焊
        RIGHT_RIGHT_VERTICAL_SWING_WELD = 6,  // 机器人右方右侧竖直焊缝摆焊
        LINE_HORIZONTAL_SWING_WELD = 10,      // 水平直线摆焊
        GANTRAY_FRAME_LINE_SWING_WELD = 100,  // 龙门支架直线摆焊（龙门支架改为用上位机提供的参考点）
        GANTRAY_FRAME_CURVE_WELD = 101,       // 龙门支架曲线焊接
        GANTRAY_FRAME_CURVE_SWING_WELD = 102  // 龙门支架曲线摆焊

    };
    // 配置初始化方法（供派生类调用）
    void initConfig();
    void readConfig();
    void writeConfig();
    void printConfig();
    void writeWeldPoint(std::fstream& outfile, double x = 0, double y = 0, double z = 0, double a = 0, double b = 0, double c = 0, double speed = 0,
                        ARC_ACTION arcAction = ARC_STOP, SWING_WELD_ACTION weldAction = LINE_WELD, double current = 0, double voltage = 0,
                        double p1x = 0, double p1y = 0, double p1z = 0, double p2x = 0, double p2y = 0, double p2z = 0);

    // 配置成员变量（引用类型，所有派生类共享）
    TrajectoryPlanningConfig& trajectoryConfig;
    SettingPara& settingPara;

signals:
    void sendPlannedSeams(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo);
    void sendTrajectoryPlanOver();

    friend class RailWeldingSystem;
};

#endif  // ABSTRACTTRAJECTORYPLANNING_H
