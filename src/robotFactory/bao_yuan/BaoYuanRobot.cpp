#include "BaoYuanRobot.h"

#include "utils/stateLight/StateLight.h"

BaoYuanRobot::BaoYuanRobot(QObject *parent) {}

// 连接机器人
bool BaoYuanRobot::connectRobot() {
    int bool_connected = false;  // 连接成功标志位

    DLL_USE_SETTING DllSetting;
    DllSetting.SoftwareType = 2;
    DllSetting.ConnectNum = 1;   // 连接数量
    DllSetting.MemSizeI = 4096;  // 各项资源使用数量
    DllSetting.MemSizeO = 4096;
    DllSetting.MemSizeC = 4096;
    DllSetting.MemSizeS = 4096;
    DllSetting.MemSizeA = 4096;
    DllSetting.MemSizeR = 100000;  // 為節省內  宣告100000個 R值的鏡射記憶體空間
    DllSetting.MemSizeF = 100000;

    int rt;
    char EncString[] = "C4B6F4BBD567825AE16BB5E10F44B1426EDEF7F4B13D9FC2";  // 密码
    char ip_addr[] = "192.168.100.5";                                       // 机器人IP
    rt = sc.LibraryInitial(&DllSetting, 706995, EncString);                 // 初始化 (第二个参数是制造商编号)
    // printf("rt:%d\n", rt);
    int concount = sc.LocalReadControllerCount();
    // printf("concount:%d\n", concount);
    int count = sc.LocalDetectControllers();
    // printf("count:%d\n", count);
    int state = sc.ConnectLocalIP(0, ip_addr);  // 连接机器人控制器
    // printf("state:%d\n", state);
    (void)rt;
    (void)concount;
    (void)count;
    (void)state;

    // 设定循环命令
    for (int i = 0; i < 1; i++) {
        sc.LReadBegin(i);
        sc.LReadNR(i, 23004, 2);
        sc.LReadNR(i, 23160, 7);
        sc.LReadEnd(i);
    }

    // 查询机器人连接状态，自己写的新程序,多次循环查询
    int try_connect_num = 1;  // 查询次数
    for (int i = 0; i < 10; i++) {
        Sleep(50);
        sc.MainProcess();  // 呼叫函式库主程序
        PLOGD << "宝元机器人已尝试连接 " << try_connect_num << " 次";
        int ConnStatus = sc.GetConnectionMsg(0, 2);  // 连接状态
        // cout << "ConnStatus:" << ConnStatus << endl;
        if (ConnStatus == SC_CONN_STATE_OK) {
            PLOGD << "机器人连接成功";
            bool_connected = true;
            break;
        }
        try_connect_num++;
    }

    if (bool_connected) {
        emit sendRobotStatus(MY_COLOR::GREEN);
    } else {
        emit sendRobotStatus(MY_COLOR::RED);
    }

    return bool_connected;
}

// 断开机器人
bool BaoYuanRobot::disconnectRobot() {
    int bool_disconnected = 0;  // 断开成功标志位

    for (int i = 0; i < 5; i++) {
        sc.MainProcess();  // 必须包含此指令
        sc.Disconnect(0);
        int ConnStatus = sc.GetConnectionMsg(0, 2);  // 连接状态
        if (ConnStatus == SC_CONN_STATE_DISCONNECT) {
            PLOGD << "宝元机器人断开连接成功";
            bool_disconnected = 1;
            break;
        }
    }

    emit sendRobotStatus(MY_COLOR::RED);

    return bool_disconnected;
}

// 机器人焊接
bool BaoYuanRobot::welding() {
    // *************************** 将路点信息读取到内存 ***************************
    std::ifstream infile;                    // 路点文件
    double x, y, z, rx, ry, rz, speed, arc;  // 路点的 xyz 型信息
    std::vector<int> datai;                  // 存放每个路点具体信息的变量
    std::vector<std::vector<int>> wayPoint;  // 存放路点的变量
    infile.open("./data/SeamCoordinate.txt");
    while (infile >> x >> y >> z >> rx >> ry >> rz >> speed >> arc) {
        datai.push_back(x * 100000);
        datai.push_back(y * 100000);
        datai.push_back(z * 100000);
        datai.push_back(rx * 100000);
        datai.push_back(ry * 100000);
        datai.push_back(rz * 100000);
        datai.push_back(speed);
        datai.push_back(arc);

        wayPoint.push_back(datai);
        datai.clear();
    }
    infile.close();

    // 机器人必须连接, 才执行焊接
    if (sc.GetConnectionMsg(0, 2) != 3) {
        PLOGE << "机器人没有连接，不进行焊接";
        return false;
    }

    // 焊缝数量非0, 才可执行焊接
    int Seam_Num = wayPoint.size();  // 识别出的焊缝数量
    if (Seam_Num == 0) {
        return false;
    }

    // 需多次执行, 才可通讯成功
    for (int i = 0; i < 3; i++) {
        while (true) {
            Sleep(50);
            sc.MainProcess();  // 呼叫函式库主程序
            int StateCorrect;
            StateCorrect = 0;
            if (StateCorrect == 0) {
                int mode;
                sc.DWriteBegin(0);
                mode = sc.DWrite1R(0, 0, wayPoint[0][0]);  // 往指定地址写入指定数值（过渡点） 返回0即为失败

                // 将焊接数据写入机器人指定地址
                for (int j = 0; j < wayPoint.size(); ++j) {
                    for (int k = 0; k < wayPoint[j].size(); ++k) {
                        if (!(j == 0 && k == 0)) {
                            mode = sc.DWrite1R(0, j * 10 + k, wayPoint[j][k]);  // 往指定写入数值
                            if (mode == 0) {
                                PLOGE << "地址" << j * 10 + k << "写入数据失败";
                            }
                        }
                    }
                }

                sc.DWrite1R(0, 250, Seam_Num);  // 焊缝数量

                sc.DWriteEnd(0);
                sc.DWaitDone(0, 100);
                if (mode != 0) {
                    StateCorrect++;
                    break;
                }
            }
        }
        someUses();  // 有用但是不知道有什么用的代码
    }

    PLOGD << "焊接完成";
    emit sendRobotWeldOver();  // 发送机器人完成焊接信号

    return true;
}

// 机器人直线运动到指定位姿
bool BaoYuanRobot::moveL(robotPose p, double speed) {
    // 机器人必须连接, 才执行运动
    if (sc.GetConnectionMsg(0, 2) != 3) {
        PLOGE << "机器人没有连接, 不进行运动";
        return false;
    }

    // 用于运动的数据
    std::vector<int> data = {
        static_cast<int>(p.x_ * 100000), static_cast<int>(p.y_ * 100000), static_cast<int>(p.z_ * 100000), static_cast<int>(p.a_ * 100000),
        static_cast<int>(p.b_ * 100000), static_cast<int>(p.c_ * 100000), static_cast<int>(speed),         0};
    PLOGD << "收到一个位姿" << data << ", 准备运动... ...";

    // 需多次执行, 才可通讯成功
    for (int i = 0; i < 3; i++) {
        while (true) {
            Sleep(50);
            sc.MainProcess();  // 呼叫函式库主程序
            int StateCorrect;
            StateCorrect = 0;
            if (StateCorrect == 0) {
                int mode;
                sc.DWriteBegin(0);
                mode = sc.DWrite1R(0, 0, data[0]);  // 往指定地址写入指定数值（过渡点） 返回0即为失败

                // 将焊接数据写入机器人指定地址
                for (int k = 0; k < data.size(); ++k) {
                    if (!(k == 0)) {
                        mode = sc.DWrite1R(0, k, data[k]);  // 往指定写入数值
                        if (mode == 0) {
                            PLOGE << "地址" << k << "写入数据失败";
                        }
                    }
                }

                sc.DWrite1R(0, 250, 1);  // 位置数量

                sc.DWriteEnd(0);
                sc.DWaitDone(0, 100);
                if (mode != 0) {
                    StateCorrect++;
                    break;
                }
            }
        }
        someUses();  // 有用但是不知道有什么用的代码
    }

    return true;
}

// 机器人运动到指定关节角
bool BaoYuanRobot::moveJ(robotJointAngle j, double speed) { return true; }

void BaoYuanRobot::whenRobotMoveJ2SouthWorkbench()
{

}

void BaoYuanRobot::whenRobotMoveJ2NorthWorkbench()
{

}

// 有用但是不知道有什么用的代码
void BaoYuanRobot::someUses() {
    while (true) {
        Sleep(50);
        sc.MainProcess();
        int StateCorrect;
        StateCorrect = 0;
        if (StateCorrect == 0) {
            int mode;
            sc.DWriteBegin(0);
            mode = sc.DWrite1A(0, 1804, 1);  // 返回0即为失败
            sc.DWriteEnd(0);
            sc.DWaitDone(0, 100);
            // printf("mode: %d\n", mode);
            if (mode != 0) {
                int mode1;
                sc.DWriteBegin(0);
                mode1 = sc.DWrite1A(0, 1804, 0);
                sc.DWriteEnd(0);
                sc.DWaitDone(0, 100);
                // printf("mode1: %d\n", mode1);
            }
            int start;
            sc.DWriteBegin(0);
            start = sc.DWrite1A(0, 52, 1);
            sc.DWriteEnd(0);
            sc.DWaitDone(0, 100);
            // printf("start: %d\n", start);
            if (start != 0) {
                sc.DWriteBegin(0);
                start = sc.DWrite1A(0, 52, 0);
                sc.DWriteEnd(0);
                sc.DWaitDone(0, 100);
                StateCorrect++;
                break;
            }
        }
    }
}
