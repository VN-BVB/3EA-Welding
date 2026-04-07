#include "AnChuanRobot.h"

#include "utils/stateLight/StateLight.h"

AnChuanRobot::AnChuanRobot(QObject *parent) { (void)parent; }

AnChuanRobot::~AnChuanRobot() {}

// 初始化网络通信
void AnChuanRobot::initSocket() {
    socket = new QTcpSocket(this);
    server = new QTcpServer(this);

    // 关联客户端连接信号newConnection
    connect(server, &QTcpServer::newConnection, this, &AnChuanRobot::server_New_Connect);

    // 渲染区更新定时器初始化
    update_timer = new QTimer(this);
    // Update the position once every 10ms
    update_timer->setInterval(10);

    // 机器人运动控制
    connect(update_timer, &QTimer::timeout, this, &AnChuanRobot::mirrorSimulate);
    update_timer->start();
}

// 连接机器人
bool AnChuanRobot::connectRobot() {
    if (!server) {  // 未初始化则初始化, 在此执行是为了在机器人线程创建server和socket实例
        initSocket();
    }

    if (listenFlag == false) {  // 当前没有在侦听
        // 侦听指定的端口
        if (!server->listen(QHostAddress::Any, port)) {
            emit sendRobotStatus(MY_COLOR::RED);
            return false;
        } else {
            listenFlag = true;  // 设置当前正在侦听
            emit sendRobotStatus(MY_COLOR::YELLOW);
            return true;
        }
    } else {  // 如果正在侦听
        if (socket->state() == QAbstractSocket::ConnectedState) {
            socket->disconnectFromHost();  // 关闭连接
        }

        server->close();     // 取消侦听
        listenFlag = false;  // 设置当前不在侦听
        emit sendRobotStatus(MY_COLOR::GRAY);
        return true;
    }
}

// 断开机器人
bool AnChuanRobot::disconnectRobot() {
    // 遍历寻找断开连接的是哪一个客户端
    for (int i = 0; i < clientSocket.length(); ++i) {
        if (clientSocket[i]->state() == QAbstractSocket::UnconnectedState) {
            // 删除存储在clientSocket列表中的客户端信息
            emit clientSocket[i]->destroyed();
            clientSocket.removeAt(i);
        }
    }

    emit sendRobotStatus(MY_COLOR::RED);
    return true;
}
std::vector<QString> AnChuanRobot::packPoint(double x, double y, double z, double rx, double ry, double rz, double speed, double arc, double swing,
                                             double current, double voltage, double p1x, double p1y, double p1z, double p2x, double p2y, double p2z) {
    return {
        // clang-format off
            QString::number(x * 1000) + "$$",
            QString::number(y * 1000) + "$$",
            QString::number(z * 1000) + "$$",
            QString::number(rx * 10000) + "$$",
            QString::number(ry * 10000) + "$$",
            QString::number(rz * 10000) + "$$",
            QString::number((int)speed) + "$$",
            QString::number(arc) + "$$",
            QString::number(swing) + "$$",
            QString::number(current) + "$$",
            QString::number(voltage * 10) + "$$",
            // ===== 摆焊参考点 =====
            QString::number(p1x * 1000) + "$$",
            QString::number(p1y * 1000) + "$$",
            QString::number(p1z * 1000) + "$$",
            QString::number(p2x * 1000) + "$$",
            QString::number(p2y * 1000) + "$$",
            QString::number(p2z * 1000) + "$$"
        // clang-format on
    };
}

// 机器人焊接
bool AnChuanRobot::welding() {
    if (clientSocket.size() == 0) {
        PLOGW << "安川机器人还未连接, 不可焊接... ...";
        return false;
    }

    weldOverFlag = false;                   // 机器人开始焊接, 标志位置为false
    running = true;                         // 进入运动模式
    std::ifstream infile;                   // 路点文件
    double x, y, z, rx, ry, rz;             // 路点的xyz型信息
    double moveSpeed, arc;                  // 焊接速度以及是否起弧（对安川机器人保留）
    double swingWeldAction;                 // 是否摆焊
    double weldingCurrent, weldingVoltage;  // 焊接电流电压
    double p1x, p1y, p1z;                   //  摆焊第一参考点
    double p2x, p2y, p2z;                   //  摆焊第二参考点 （或者是起点，就以上一个程序作为临近点）
    // std::vector<QString> datai;              // 存放每个路点具体信息的变量
    std::vector<std::vector<QString>> data;  // 存放路点的变量

    infile.open("./data/SeamCoordinate.txt");

    // 将路点信息读取到内存，等待发送
    while (infile >> x >> y >> z >> rx >> ry >> rz >> moveSpeed >> arc >> swingWeldAction >> weldingCurrent >> weldingVoltage >> p1x >> p1y >> p1z >>
           p2x >> p2y >> p2z) {
        data.push_back(packPoint(x, y, z, rx, ry, rz, moveSpeed / 6,
                                 (robotWorkMode == SIMULATION_MODE ? 0 : arc),  //
                                 swingWeldAction, weldingCurrent, weldingVoltage, p1x, p1y, p1z, p2x, p2y, p2z));
    }
    PLOGD << "共读取到 " << data.size() << " 个路点, 开始发送数据... ...";

    if (data.size() == 0) {
        infile.close();
        emit sendRobotWeldOver();  // 没有焊缝, 直接发送机器人完成焊接信号
        PLOGD << "###################### 没有焊缝, 直接发送机器人完成焊接信号";
        return true;
    }

    // 奇数补齐
    if (data.size() % 2 == 1) {  // 如果数据点数是奇数, 就把最后一个点多发一次
        data.push_back(data[data.size() - 1]);
    }

    controlTimer();  // 清空原始数据队列、位姿队列，buffNum置0
    PLOGD << "update_timer->isActive(): " << update_timer->isActive();

    while (true) {
        QByteArray command_temp = BUF_INITIALIZATION;
        if (q_command.size() > 0) {
            PLOGD << "q_command.size(): " << q_command.size();
        }
        if (q_command.size() >= 1) {
            command_temp = q_command.dequeue();
            qDebug() << "QByteArray command_temp" << command_temp;
        }
        if (JBI_START_RUN == command_temp) {  // JBI进入循环 1004
            command_temp = BUF_INITIALIZATION;
            Sleep(cirTime.toInt());

            QString start = "START$$";  // 通知motoplus焊接条件准备完成
            socket->write(start.toLatin1());
            PLOGD << "开始焊接... ...";

            Sleep(cirTime.toInt());

            QString pose = "POSE$$";  // 通知motoplus需要机器人当前位姿
            // ui->textEdit_Send->append(pose);
            socket->write(pose.toLatin1());
        }

        if (ASK_TO_SEND_DATA == command_temp) {  // 发送数据 1001
            command_temp = BUF_INITIALIZATION;
            Sleep(cirTime.toInt());
            if (m_i_Pointnum >= data.size()) continue;
            int swingType = data[m_i_Pointnum][8].toInt();

            if (swingType == 101) {
                int start = m_i_Pointnum;
                int end = start;
                // 找连续101
                while (end < data.size() && data[end][8].toInt() == 101) {
                    end++;
                }
                int count = end - start;
                PLOGD << "检测到曲线段，采样点数量: " << count;
                // // 发送模式头
                // QString header = QString("MODE_CURVE,%1$$").arg(count);
                // socket->write(header.toLatin1());
                for (int i = start; i < end; i++) {
                    Btn_Send_loop(data[i], cirTime.toInt());
                }
                m_i_Pointnum = end;
            } else {
                // 把data里的一组数据按cirTime[ms]的周期发送给客户端
                Btn_Send_loop(data[m_i_Pointnum], cirTime.toInt());
                m_i_Pointnum++;
            }
        }

        if (m_i_Pointnum == data.size()) {  // 全部发送完毕
            QString stop = "EXIT$$";        // 发送完毕通知
            // ui->textEdit_Send->append(stop);
            socket->write(stop.toLatin1());
            PLOGD << "所有焊缝点都已发送完毕... ...";
            running = false;  // 退出运动模式

            m_i_Pointnum = 0;
            break;
        }

        if (STOP_TO_ACCEPT_ASK == command_temp) {  // 退出访问 1003
            command_temp = BUF_INITIALIZATION;
            QString stop = "EXIT$$";  // 发送完毕通知
            // ui->textEdit_Send->append(stop);
            socket->write(stop.toLatin1());

            std::cout << "STOP_TO_ACCEPT_ASK == command_temp" << std::endl;
            running = false;  // 退出运动模式

            // ui->Btn_Recv_loop->setText(u8"接收访问");
            // ui->textEdit_Send->append("Accepting is stoped.");
            m_i_Pointnum = 0;
            break;
        }

        // QString Q_err_stop;
        // Q_err_stop = ui->err_stop->text();
        // if (Q_err_stop.toInt() == -1) {  // 主动退出访问
        //     QString stop = "EXIT$$";     // 通知out取消侦听
        //     // ui->textEdit_Send->append(stop);
        //     socket->write(stop.toLatin1());

        //     std::cout << "Q_err_stop.toInt() == -1" << std::endl;

        //     // ui->textEdit_Send->append("Accepting is err!");
        //     break;
        // }

        Sleep(100);
        // QApplication::processEvents();
    }

    infile.close();
    // ui->Btn_Recv_loop->setText(u8"接收访问");

    // QString stop = "EXIT$$";  // 通知out取消侦听
    // ui->textEdit_Send->append(stop);

    PLOGD << "所有焊接点发送完毕";
    weldOverFlag = true;  // 机器人焊接点发送完成 (但是轨迹不一定走完了), 标志位置为true

    // emit sendRobotWeldOver();  // 发送机器人完成焊接信号

    return true;
}

// 机器人直线运动到指定位姿
bool AnChuanRobot::moveL(robotPose p, double speed) {
    if (clientSocket.size() == 0) {
        PLOGW << "安川机器人还未连接, 不可运动... ...";
        return false;
    }

    moveOverFlag = false;  // 机器人开始运动, 标志位置为false
    running = true;        // 进入运动模式
    std::vector<QString> datai = packPoint(p.x_, p.y_, p.z_, p.a_, p.b_, p.c_, speed * 10.0,
                                           0,  // arc
                                           0,  // swing
                                           0,  // current
                                           0   // voltage
    );
    std::vector<std::vector<QString>> data;  // 存放路点的变量
    data.push_back(datai);
    data.push_back(datai);
    PLOGD << "收到一个位姿, 准备运动... ...";

    controlTimer();  // 清空原始数据队列、位姿队列，buffNum置0
    PLOGD << "update_timer->isActive(): " << update_timer->isActive();

    bool dataSent = false;  // 数据是否已经发送
    while (true) {
        QByteArray command_temp = BUF_INITIALIZATION;
        // if (q_command.size() > 0) {
        //     PLOGD << "q_command.size(): " << q_command.size();
        // }
        std::cout << "q_command.size(): " << q_command.size();
        if (q_command.size() >= 1) {
            command_temp = q_command.dequeue();
            PLOGD << "QByteArray command: " << command_temp.toStdString();
        }
        if (JBI_START_RUN == command_temp) {  // JBI已经进入循环 1004
            command_temp = BUF_INITIALIZATION;
            Sleep(cirTime.toInt());

            QString start = "START$$";  // 通知motoplus焊接条件准备完成
            socket->write(start.toLatin1());
            PLOGD << "开始运动... ... (JBI_START_RUN)";

            Sleep(cirTime.toInt());

            QString pose = "POSE$$";  // 通知motoplus需要机器人当前位姿
            // ui->textEdit_Send->append(pose);
            socket->write(pose.toLatin1());
        }
        if (ASK_TO_SEND_DATA == command_temp) {  // 发送数据 1001
            PLOGD << "发送数据... ... (ASK_TO_SEND_DATA)";
            command_temp = BUF_INITIALIZATION;
            Sleep(cirTime.toInt());
            // 把data里的一组数据按cirTime[ms]的周期发送给客户端
            Btn_Send_loop(data[m_i_Pointnum], cirTime.toInt());
            m_i_Pointnum++;

            dataSent = true;
        }
        if (m_i_Pointnum == data.size()) {  // 数据已经发送
            QString stop = "EXIT$$";        // 发送完毕通知
            socket->write(stop.toLatin1());
            PLOGD << "运动目标点已发送完毕... ... (EXIT$$)";
            running = false;  // 进入运动模式

            m_i_Pointnum = 0;
            break;
        }
        if (STOP_TO_ACCEPT_ASK == command_temp) {  // 退出访问 1003
            PLOGD << "退出访问... ... (STOP_TO_ACCEPT_ASK)";
            command_temp = BUF_INITIALIZATION;
            QString stop = "EXIT$$";  // 发送完毕通知
            socket->write(stop.toLatin1());
            running = false;  // 进入运动模式

            // std::cout << "STOP_TO_ACCEPT_ASK == command_temp" << std::endl;
            m_i_Pointnum = 0;
            break;
        }

        Sleep(100);
        // QApplication::processEvents();
    }
    PLOGD << "直线运动点发送完毕";
    moveOverFlag = true;  // 机器人直线运动发送完成 (但是轨迹不一定走完了), 标志位置为true

    return true;
}

// 机器人运动到指定关节角
bool AnChuanRobot::moveJ(robotJointAngle j, double speed) { return true; }

// 机器人运动到南工作台 (左侧)
void AnChuanRobot::whenRobotMoveJ2SouthWorkbench() {
    PLOGD << "左侧工作台... ...";
    Sleep(cirTime.toInt());
    QString moveJ2South = "MOVEJL$$";  // 发送完毕通知
    socket->write(moveJ2South.toLatin1());
    Sleep(cirTime.toInt());
}

// 机器人运动到北工作台 (右侧)
void AnChuanRobot::whenRobotMoveJ2NorthWorkbench() {
    PLOGD << "右侧工作台... ...";
    Sleep(cirTime.toInt());
    QString moveJ2North = "MOVEJR$$";  // 发送完毕通知
    socket->write(moveJ2North.toLatin1());
    Sleep(cirTime.toInt());
}

// 延时函数
void AnChuanRobot::Sleep(int msec) {
    QTime dieTime = QTime::currentTime().addMSecs(msec);
    while (QTime::currentTime() < dieTime) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
    }
}

// 连续发送一组数据
void AnChuanRobot::Btn_Send_loop(std::vector<QString> data, int sleepTime) {
    PLOGD << "开始写入数据... ...";
    // int row = ui->tableWidget->currentRow();
    QString wayPoint;
    for (int i = 0; i < data.size(); i++) {
        wayPoint += data[i];
    }
    if (data[0].isEmpty()) {
        // QMessageBox::information(this, u8"提示", u8"请输入发送内容！", QMessageBox::Yes);
        PLOGE << "安川机器人待发送数据为空";
    } else {
        // if (row >= 0) {
        for (int i = 0; i < clientSocket.length(); ++i) {
            // if (QString::number(clientSocket[i]->peerPort()) == ui->tableWidget->item(row, 2)->text()) {
            // // 以ASCII码形式发送文本框内容
            PLOGD << clientSocket[i]->write(wayPoint.toLatin1());
            PLOGD << wayPoint.toLatin1().data();
            /*机器人端在 socketRecv_Task() 中接收数据：
                bytesRecv = mpRecv(sockHandle, buff, BUFF_MAX, 0);*/
            // ui->textEdit_Send->append(wayPoint);
            // }
        }
        // } else {
        // socket->write(wayPoint.toLatin1());
        // ui->textEdit_Send->append(wayPoint);
        // }
    }

    (void)sleepTime;
}

// 镜像运动显示刷新
void AnChuanRobot::controlTimer() {
    if (update_timer->isActive()) {
        // update_timer->stop();
        // ui->mirrorStart->setText("镜像开始");
        q_oriData.clear();
        q_pose.clear();
        // q_command.clear();
        q_interPose.clear();
        buffNum = 0;
    } else {
        update_timer->start();
        PLOGD << "update_timer 定时器已开启...";
        // ui->mirrorStart->setText("镜像中...");
    }
}

// 获取客户端连接
void AnChuanRobot::server_New_Connect() {  // 获取客户端连接
    socket = server->nextPendingConnection();
    clientSocket.append(socket);

    // 连接QTcpSocket的信号槽，以读取新数据
    connect(socket, &QTcpSocket::readyRead, this, &AnChuanRobot::Read_Data);
    connect(socket, &QTcpSocket::disconnected, this, &AnChuanRobot::disconnectRobot);

    emit sendRobotStatus(MY_COLOR::GREEN);
}

// 从客户端接收到的消息
void AnChuanRobot::Read_Data() {
    // 由于 readyRead 信号并未提供 SocketDecriptor, 所以需要遍历所有客户端
    for (int i = 0; i < clientSocket.length(); ++i) {
        // 读取缓冲区数据
        buffer = clientSocket[i]->readAll();
        if (buffer.isEmpty()) {
            continue;
        }

        // 保存机器人当前发送过来的数据至队列oriData
        q_oriData.enqueue(buffer);
    }
}

void AnChuanRobot::mirrorSimulate() {
    // std::cout << "mirrorSimulate()... ..." << std::endl;
    // 数据解包
    Unpack();
}

void AnChuanRobot::Unpack() {
    // std::cout << "Unpack... ... q_oriData.size(): " << q_oriData.size() << std::endl;
    bool newPackFlag = false;  // 当前是否为新一组数据标志位

    // 读到一组数据后开始解包
    if (q_oriData.size() >= 1) {
        newPackFlag = true;
        // 收到一次就立马申请第二次
        QString pose = "POSE$$";  // 通知out取消侦听
        socket->write(pose.toLatin1());

        // 解包
        QList<QByteArray> recvMsgs;
        recvMsgs.append(q_oriData.dequeue());
        static QByteArray halfData;  // 上一次遗留的半包数据, static只分配一次内存防止被覆盖
        for (int var = 0; var < recvMsgs.size(); ++var) {
            auto recvMsg = recvMsgs.at(var);
            // 解包数据
            int idx = 0;
            while (idx != -1) {
                // 从idx位置开始查找$$的位置
                int postion = recvMsg.indexOf("$$", idx);  // 解决粘包问题 "1111$$2222$$xxxxx"
                if (postion != -1) {
                    // 获取$$前的数据, 不包括$$
                    auto byte = recvMsg.mid(idx, postion - idx);
                    if (!halfData.isEmpty()) {
                        byte = halfData + byte;
                        halfData.clear();
                    }
                    q_upPackData.enqueue(byte);
                    // qDebug() << byte;
                    postion += 2;
                } else {
                    // 如果有半包现象, 则把最后一个"$$"后的数据保存起来, 暂时不使用
                    if (idx < recvMsg.length()) {
                        halfData = recvMsg.mid(idx);
                    }
                }
                idx = postion;
            }
            // qDebug() << u8"剩下的半包数据:" << halfData;
        }
    }

    // 数据打包进队列
    if (q_upPackData.size() >= 1) {
        QByteArray qb_temp = q_upPackData.dequeue();
        if (qb_temp != BUF_INITIALIZATION && qb_temp != ASK_TO_SEND_DATA && qb_temp != STOP_TO_SEND_DATA && qb_temp != STOP_TO_ACCEPT_ASK &&
            qb_temp != JBI_START_RUN) {
            // 位姿数据进pose队列
            double data = qb_temp.toDouble();

            if (newPackFlag == true && q_pose.size() != 0) {
                q_pose.clear();
            }

            q_pose.push_back(data);
            // qDebug() << "************** q_pose =" << data << "  " << q_pose.size();

            // 每收到六个数据 (一组位姿, 就发出一次)
            if (q_pose.size() == 12) {
                emit sendRobotCurrentPose(
                    robotPose(q_pose[0] / 1000, q_pose[1] / 1000, q_pose[2] / 1000, q_pose[3] / 10000, q_pose[4] / 10000, q_pose[5] / 10000));
                emit sendRobotCurrentJointAngle(robotJointAngle(q_pose[6] / 10000, q_pose[7] / 10000, q_pose[8] / 10000, q_pose[9] / 10000,
                                                                q_pose[10] / 10000, q_pose[11] / 10000));
                q_pose.clear();
            }

            buffNum++;
        } else {
            // 指令数据进command队列
            if (qb_temp != "1003") {
                if (!(qb_temp == "1001" && running == false)) {  // 收到位姿请求, 但又不在运动模式中, 就忽略请求
                    q_command.enqueue(qb_temp);
                    qDebug() << "q_command= " << qb_temp;
                }
                if (qb_temp == "1004" && weldOverFlag == true) {  // JBI准备好下一次动作了, 才能说明上一次焊接任务完成了
                    weldOverFlag = false;
                    emit sendRobotWeldOver();
                }
                if (qb_temp == "1004" && moveOverFlag == true) {  // JBI准备好下一次动作了, 才能说明上一次运动任务完成了
                    moveOverFlag = false;
                    emit sendRobotMoveOver();
                }
            }
        }
    }
}
