#ifndef BAOYUANROBOT_H
#define BAOYUANROBOT_H

#include "robotFactory/AbstractRobot.h"
#include "stdafx.h"
#define __CLASS
#include "scif2.h"

class BaoYuanRobot : public AbstractRobot {
public:
    BaoYuanRobot(QObject *parent = nullptr);

    void someUses();  // 有用但是不知道有什么用的代码

signals:

public slots:
    bool connectRobot() override;                          // 连接机器人
    bool disconnectRobot() override;                       // 断开机器人
    bool welding() override;                               // 机器人焊接
    bool moveL(robotPose p, double speed) override;        // 机器人直线运动到指定位姿
    bool moveJ(robotJointAngle j, double speed) override;  // 机器人运动到指定关节角

    void whenRobotMoveJ2SouthWorkbench() override;  // 机器人运动到南工作台 (左侧)
    void whenRobotMoveJ2NorthWorkbench() override;  // 机器人运动到北工作台 (右侧)

protected:
private:
    SC2 sc;

    friend class RailWeldingMainWindow;
};

#endif  // BAOYUANROBOT_H
