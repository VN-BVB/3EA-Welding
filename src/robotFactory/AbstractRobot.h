#ifndef ABSTRACTROBOT_H
#define ABSTRACTROBOT_H

#include <plog/Log.h>

#include <QByteArray>
#include <QDebug>
#include <QMessageBox>
#include <QObject>
#include <QQueue>
#include <QString>
#include <QTime>
#include <QTimer>
#include <QWidget>
#include <QtConcurrent/QtConcurrent>
#include <QtNetwork/QNetworkInterface>
#include <QtNetwork/QTcpSocket>
#include <QtNetwork/QTcpserver>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

class robotJointAngle {
public:
    robotJointAngle() {}
    robotJointAngle(double j1, double j2, double j3, double j4, double j5, double j6)
        : joint1(j1), joint2(j2), joint3(j3), joint4(j4), joint5(j5), joint6(j6) {}
    double joint1 = 0;
    double joint2 = 0;
    double joint3 = 0;
    double joint4 = 0;
    double joint5 = 0;
    double joint6 = 0;
};

class robotPose {
public:
    robotPose() {}
    robotPose(double x, double y, double z, double a, double b, double c) : x_(x), y_(y), z_(z), a_(a), b_(b), c_(c) {}
    double x_ = 0;
    double y_ = 0;
    double z_ = 0;
    double a_ = 0;
    double b_ = 0;
    double c_ = 0;
};

enum ROBOT_WORK_MODE {  // 机器人运动方式
    SIMULATION_MODE,    // 模拟模式(只走轨迹)
    WELDING_MODE        // 焊接模式(起弧焊接)
};

class AbstractRobot : public QObject {
    Q_OBJECT
public:
    AbstractRobot(QObject *parent = nullptr);
    ~AbstractRobot();

signals:
    void sendRobotStatus(QString color);                 // 机器人设备状态信号
    void sendRobotWeldOver();                            // 发送机器人完成焊接信号
    void sendRobotMoveOver();                            // 发送机器人完成运动信号
    void sendRobotCurrentPose(robotPose p);              // 发送机器人当前位姿
    void sendRobotCurrentJointAngle(robotJointAngle j);  // 发送机器人当前关节

public slots:
    virtual bool connectRobot() = 0;                          // 连接机器人
    virtual bool disconnectRobot() = 0;                       // 断开机器人
    virtual bool welding() = 0;                               // 机器人焊接
    virtual bool moveL(robotPose p, double speed) = 0;        // 机器人直线运动到指定位姿
    virtual bool moveJ(robotJointAngle j, double speed) = 0;  // 机器人运动到指定关节角

    virtual void whenRobotMoveJ2SouthWorkbench() = 0;  // 机器人运动到南工作台 (左侧)
    virtual void whenRobotMoveJ2NorthWorkbench() = 0;  // 机器人运动到北工作台 (右侧)

protected:
    ROBOT_WORK_MODE robotWorkMode = SIMULATION_MODE;  // 默认模拟模式

private:
    friend class WeldingMainWindow;
};

#endif  // ABSTRACTROBOT_H
