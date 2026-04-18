#ifndef ANCHUANROBOT_H
#define ANCHUANROBOT_H

#include <robotFactory/AbstractRobot.h>

#include <QByteArray>
#include <QDebug>
#include <QMessageBox>
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
#include <iostream>
#include <vector>

class AnChuanRobot : public AbstractRobot {
public:
    AnChuanRobot(QObject *parent = nullptr);
    ~AnChuanRobot();

signals:

public slots:
    bool connectRobot() override;                          // 连接机器人
    bool disconnectRobot() override;                       // 断开机器人
    bool welding() override;                               // 机器人焊接
    bool moveL(robotPose p, double speed) override;        // 机器人直线运动到指定位姿
    bool moveJ(robotJointAngle j, double speed) override;  // 机器人运动到指定关节角

    void whenRobotMoveJ2SouthWorkbench() override;  // 机器人运动到南工作台 (左侧)
    void whenRobotMoveJ2NorthWorkbench() override;  // 机器人运动到北工作台 (右侧)

    void initSocket();  // 初始化网络通信

    // ============================== 通信建立/断开 ==================================
    void server_New_Connect();  // 获取客户端连接

    // ============================== 信息接收 ==================================
    void Read_Data();  // 从客户端接收到的消息

    // ============================== 镜像模型 ==================================
    void mirrorSimulate();  // 镜像运动
    void controlTimer();    // 镜像运动显示刷新
    void Unpack();          // 解包

    // ============================== 帮助算子 ==================================
    void Sleep(int msec);                                          // 延时一段时间（ms）
    void Btn_Send_loop(std::vector<QString> data, int sleepTime);  // 连续发送一组数据
private:
    std::vector<QString> packPoint(double x = 0, double y = 0, double z = 0, double rx = 0, double ry = 0, double rz = 0, double speed = 0,
                                   double arc = 0, double swing = 0, double current = 0, double voltage = 0, double p1x = 0, double p1y = 0,
                                   double p1z = 0, double p2x = 0, double p2y = 0, double p2z = 0);

private:
    const QByteArray BUF_INITIALIZATION = "1000";
    const QByteArray ASK_TO_SEND_DATA = "1001";
    const QByteArray STOP_TO_SEND_DATA = "1002";
    const QByteArray STOP_TO_ACCEPT_ASK = "1003";
    const QByteArray JBI_START_RUN = "1004";

    int port = 11000;  // 机器人端口号

    QTcpSocket *socket = nullptr;      // 通信数据收发实例
    QTcpServer *server = nullptr;      // 通信数据收发实例
    QList<QTcpSocket *> clientSocket;  // 客户端通信数据收发实例
    QByteArray buffer;                 // 接收到的数据实例
    QString cirTime = "10";            // 发送信息延时

    bool listenFlag = false;    // 当前是否正在侦听标志位
    bool running = false;       // 机器人是否正在运动
    bool weldOverFlag = false;  // 机器人焊接完成标志位
    bool moveOverFlag = false;  // 机器人运动完成标志位

    int b_servOpen = 0;
    QString messenges = 0;  // 调试信息
    int m_i_Pointnum = 0;   // 发送点数

    QQueue<QByteArray> q_oriData;     // 接收的原始数据流
    std::vector<double> q_pose;       // 位姿数据流
    QQueue<QByteArray> q_command;     // 指令数据流
    QQueue<double> q_interPose;       // 位姿插补数据流
    QQueue<QByteArray> q_upPackData;  // 已完成解包数据流

    int buffNum = 0;       // 位姿数据计数
    int faceFlag = 0;      // 正反面标志
    QTimer *update_timer;  // 仿真渲染界面更新定时器

    friend class WeldingMainWindow;
};

#endif  // ANCHUANROBOT_H
