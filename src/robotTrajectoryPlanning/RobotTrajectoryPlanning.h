#ifndef ROBOTTRAJECTORYPLANNING_H
#define ROBOTTRAJECTORYPLANNING_H

#include <direct.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <plog/Log.h>

#include <QObject>
#include <algorithm>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class WeldSeamInfo;
class TrajectoryPlanningConfig;
class SettingPara;

enum class WORKPIECE_SIDE_OF_ROBOT {  // 当前工件位于机器人基座的方向
    FRONT,                            // 前方
    LEFT,                             // 左方
    RIGHT                             // 右方
};

class RobotTrajectoryPlanning : public QObject {
    Q_OBJECT
public:
    explicit RobotTrajectoryPlanning(QObject* parent = nullptr);

    void initConfig();   // 初始化配置信息
    void writeConfig();  // 写配置文件
    void readConfig();   // 读配置文件
    void printConfig();  // 打印配置文件
    void initPara();     // 初始化参数

    /*
     * 相对工作台来说, 机器人坐标系是几乎没有倾斜的, 而相机是倾斜安装的, 故焊缝方向调整、延长、姿态给定全都在机器人坐标系下进行;
     * 但由于不同机器人坐标系不统一, 同一个机器人也可能在不同方向上焊接工件, 具体情况繁多;
     * 所以将各种机器人坐标系下的焊缝统一到同一个虚拟坐标系(即右方X前方Y), 再进行后续相关操作, 操作完转回原本的坐标系.
     * 定义『焊接时』机械臂『后几个轴相对于基座』朝向的方向是前方, 并以此时的左右作为左方和右方, 『与机器人初始位置的前后左右无关』
     */
    void real2Virtual(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo);              // 真实坐标系转虚拟坐标系
    void virtual2Real(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo);              // 虚拟坐标系转真实坐标系
    void frontXleftY2rightXfrontY(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo);  // 前方X左方Y 转为 右方X前方Y
    void leftXbackY2rightXfrontY(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo);   // 左方X后方Y 转为 右方X前方Y
    void backXrightY2rightXfrontY(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo);  // 后方X右方Y 转为 右方X前方Y
    void rightXfrontY2frontXleftY(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo);  // 右方X前方Y 转为 前方X左方Y
    void rightXfrontY2leftXbackY(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo);   // 右方X前方Y 转为 左方X后方Y
    void rightXfrontY2backXrightY(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo);  // 右方X前方Y 转为 后方X右方Y

    void sortSeamsWithX(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo);                     // 将焊缝信息按照X值进行排序
    int findEndOfLeftSeams(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo);                  // 找到左右焊缝的分界线
    void transSeams2Base(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo);                    // 将焊缝点转到机器人基坐标系
    void determineWorkpieceOri(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo);              // 判断工件位于机器人的方位
    void seamsErrorCompensate(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo);               // 焊缝误差补偿
    void transSeamsOri(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo, int endOfLeftSeams);  // 修改焊缝方向
    void extendSeams(std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo, int endOfLeftSeams);    // 延长焊缝

    constexpr double deg2rad(double degrees);                              // 角度制转弧度制
    std::array<std::array<double, 3>, 3> getRollMatrix(double rollRad);    // 生成绕 x 轴的旋转矩阵 (Roll)
    std::array<std::array<double, 3>, 3> getPitchMatrix(double pitchRad);  // 生成绕 y 轴的旋转矩阵 (Pitch)
    std::array<std::array<double, 3>, 3> getYawMatrix(double yawRad);      // 生成绕 z 轴的旋转矩阵 (Yaw)
    std::array<double, 3> rotateVector(const std::array<std::array<double, 3>, 3>& mat, const std::array<double, 3>& vec);  // 旋转向量
    std::array<double, 3> abcToVector(double A, double B, double C);  // 将A、B、C应用到向量 (0, 0, 1)

    void write2File(const std::vector<std::shared_ptr<WeldSeamInfo>>& weldSeamInfo, int endOfLeftSeams);  // 焊缝写入文件

signals:
    void sendDetSeamWithSeg(std::vector<std::shared_ptr<WeldSeamInfo>> weldAreaInfo);  // 发送规划完成的焊缝
    void sendTrajectoryPlanOver();                                                     // 发送轨迹规划完成

public slots:
    void whenPlanningTrajectory(std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo);  // 规划焊缝轨迹

private:
    TrajectoryPlanningConfig& trajectoryConfig;
    SettingPara& settingPara;
    WORKPIECE_SIDE_OF_ROBOT workpieceSide = WORKPIECE_SIDE_OF_ROBOT::FRONT;  // 当前工件位于机器人基座的方向

    // 起弧熄弧动作控制
    enum ARC_ACTION {
        ARC_START = 1,  // 起弧
        ARC_STOP = 0    // 熄弧
    };

    // 摆焊动作控制 (机器人以及焊缝方位的定义参见seamsErrorCompensate函数)
    enum SWING_WELD_ACTION {
        LINE_WELD = 0,                        // 直线焊接
        FRONT_LEFT_VERTICAL_SWING_WELD = 1,   // 机器人前方左侧竖直焊缝摆焊
        FRONT_RIGHT_VERTICAL_SWING_WELD = 2,  // 机器人前方右侧竖直焊缝摆焊
        LEFT_LEFT_VERTICAL_SWING_WELD = 3,    // 机器人左方左侧竖直焊缝摆焊
        LEFT_RIGHT_VERTICAL_SWING_WELD = 4,   // 机器人左方右侧竖直焊缝摆焊
        RIGHT_LEFT_VERTICAL_SWING_WELD = 5,   // 机器人右方左侧竖直焊缝摆焊
        RIGHT_RIGHT_VERTICAL_SWING_WELD = 6   // 机器人右方右侧竖直焊缝摆焊
    };

    double moveSpeed = 170 * 60;             // 过渡运动速度
    double weldingSpeedDefault = 5 * 60;     // 焊接速度(默认速度，宽度检测失败时用这个速度)
    double weldingSpeed0To1 = 5 * 60;        // 焊接速度(焊缝宽度1mm以下用这个速度)
    double weldingSpeed1To3 = 5 * 60;        // 焊接速度(焊缝宽度1mm到3mm用这个速度)
    double weldingSpeed3To5 = 5 * 60;        // 焊接速度(焊缝宽度1mm到3mm用这个速度)
    double weldingSpeedHorizontal = 5 * 60;  // 焊接速度(水平焊缝的焊接速度)
    double weldingSpeedVertical = 5 * 60;    // 焊接速度(竖直焊缝的焊接速度)
    double weldingCurrent = 160;             // 焊接电流(默认焊接电流)
    double weldingCurrent_Vertical = 130;    // 焊接电流(竖直焊缝用这个电流)
    double weldingVoltage = 24;              // 焊接电压(默认焊接电压)
    double weldingVoltage_Vertical = 18;     // 焊接电压(竖直焊缝用这个电压)

    int endOfLeftSeamSerial = 0;  // 左侧最后一个焊缝索引

    // txt存储点
    std::fstream outfile;  // 读取存在mask坐标的txt文件
    std::string outfile_name = "./data/SeamCoordinate.txt";

    friend class RailWeldingSystem;
};

#endif  // ROBOTTRAJECTORYPLANNING_H
