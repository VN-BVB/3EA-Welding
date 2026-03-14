/* mp_main.c - MotoPlus Test Application for Real Time Process */
#include "motoPlus.h"

extern int setValP(const float *target, int pointNum);
extern int setValP_Pulse(const long *target, int pointNum);
extern int GetBVar(UINT16 index, long *value);
extern int SetBVar(UINT16 index, long value);
extern int SetIVar(UINT16 index, long value);
extern int cmp_usr_var_info(MP_USR_VAR_INFO *info1, MP_USR_VAR_INFO *info2);
extern int cmp_pos_var_info(MP_P_VAR_BUFF *buf1, MP_P_VAR_BUFF *buf2);
extern int readPos(MP_CART_POS_RSP_DATA *rData);

// FUNCTION PROTOTYPES
void CmdRcv_Task(void);
void CorrPath_Task(void);
void socketRecv_Task(void);

void hd_ConnectServer();
void hd_AskPathServer();
void hd_StartSprayServer();
void hd_GetPosServer();

#define PORT 11000  // 系统软件用的端口是9900~10499 禁止使用，推荐用20000~23000
#define BUFF_MAX 1023
#define COORD_NUM 6  // X, Y, Z, Rx, Ry, Rz

char Process = 0;

int sockHandle;

int connect_flag = 0;
int ask_flag = 0;
int spray_flag = 0;
int pos_flag = 0;

// 摆焊动作控制 (机器人以及焊缝方位的定义参见上位机代码中的seamsErrorCompensate函数)
enum SWING_WELD_ACTION {
    LINE_WELD = 0,                        // 直线焊接
    FRONT_LEFT_VERTICAL_SWING_WELD = 1,   // 机器人前方左侧竖直焊缝摆焊
    FRONT_RIGHT_VERTICAL_SWING_WELD = 2,  // 机器人前方右侧竖直焊缝摆焊
    LEFT_LEFT_VERTICAL_SWING_WELD = 3,    // 机器人左方左侧竖直焊缝摆焊
    LEFT_RIGHT_VERTICAL_SWING_WELD = 4,   // 机器人左方右侧竖直焊缝摆焊
    RIGHT_LEFT_VERTICAL_SWING_WELD = 5,   // 机器人右方左侧竖直焊缝摆焊
    RIGHT_RIGHT_VERTICAL_SWING_WELD = 6   // 机器人右方右侧竖直焊缝摆焊
};

// 等待来自程序的 skill 指令， 根据指令的内容（cmd）决定执行的处理的任务。
void CmdRcv_Task(void) {
    SYS2MP_SENS_MSG msg;
    memset(&msg, CLEAR, sizeof(SYS2MP_SENS_MSG));

    FOREVER {  // 同while(1)
        mpEndSkillCommandProcess(MP_SL_ID1, &msg);  // 给机器人 1（R1）指定 MP_SL_ID1
        /* 接收通过 SKILLSND 命令执行发送的指令（传感器指令）
        STATUS mpReceiveSkillCommand (
             int sl_id, 与系统的通信区域指定
             SYS2MP_SENS_MSG *msg_p 传感器指令数据结构体
        ) */
        if (mpReceiveSkillCommand(MP_SL_ID1, &msg) == ERROR) {
            // printf("mpReceiveSkillCommand Error\n\r");
            mpTaskDelay(1000);
        }
        // printf("main_comm %d\n\r", msg.main_comm);
        // printf("sub_comm %d\n\r", msg.sub_comm);
        // printf("exe_tsk %d\n\r", msg.exe_tsk);
        // printf("exe_apl %d\n\r", msg.exe_apl);
        // printf("comm %s\n\r", msg.cmd);

        // 以下描述指令处理
        switch (msg.main_comm) {  // 主指令
        case MP_SKILL_COMM: // MP_SKILL_COMM = 0 Skill指令
            switch (msg.sub_comm) {
            case MP_SKILL_SEND: // MP_SKILL_SEND = 1 SKILLSEND指令
                if (strcmp(msg.cmd, "askpath") == 0 && Process == 0) {
                    // printf("Command Recive ASK_PATH\n\r");
                    Process = 1;
                } else if (strcmp(msg.cmd, "connect") == 0 && Process == 0) {
                    // printf("Command Recive connect\n\r");
                    Process = 2;
                } else if (strcmp(msg.cmd, "revstart") == 0 && Process == 0) {
                    // printf("Command Recive SpdOverride_off\n\r");
                    Process = 3;
                } else if (strcmp(msg.cmd, "forcepathend") == 0 && Process == 0) {
                    // printf("Command Recive ForcePathEnd\n\r");
                    Process = 4;
                } else if (strcmp(msg.cmd, "startspray") == 0 && Process == 0) {
                    // printf("Command Recive startspray\n\r");
                    Process = 5;
                } else {
                    Process = 0;
                    // printf("Unknown Command\n\r");
                }
                break;

            case MP_SKILL_END: // MP_SKILL_END = 2 SKILL指令强制结束
                Process = 0;
                // printf("MP_SKILL_END\n\r");
                break;
            default:
                // printf("Unknown Sub Command\n\r");
                break;
            }
            break;
        case MP_SL_RST_COMM:
            Process = 0;
            switch (msg.sub_comm) {
            case MP_SL_SOFTWARE_RST:
                // printf("MP_SL_SOFTWARE_RST\n\r");
                break;
            case MP_SL_ALM_RST:
                // printf("MP_SL_ALM_RST\n\r");
                break;
            case MP_START_SEG_CLK:
                // printf("MP_START_SEG_CLK\n\r");
                break;
            case MP_SE_PRM_TRANS:
                // printf("MP_SE_PRM_TRANS\n\r");
                break;
            default:
                // printf("Unknown Sub Command\n\r");
                break;
            }
            break;
        default:
            // printf("Unknown Main Command\n\r");
            break;
        }
    }
}

/* 执行 CmdRcv_Task 决定的处理的任务。
每个运行插补周期都需要执行轨迹修正和速度修正等运行条件变更处理。
因此，设定此任务的优先级为 MP_PRI_IO_CLK_TIME，每个插补周期用 mpClkAnnounce 接收通知，执行处理。 */
void CorrPath_Task(void) {
    int ret;
    long dy = 0;
    int cnt = 0;

    MP_POS_DATA corrpath_src_p;
    CTRLG_T ctrl_grp = 1;
    long spd_src_p[MP_GRP_NUM];

    memset(&corrpath_src_p, CLEAR, sizeof(MP_POS_DATA));
    memset(&spd_src_p, CLEAR, sizeof(long) * MP_GRP_NUM);

    // 修正量数据的初始化
    corrpath_src_p.ctrl_grp = 1;
    corrpath_src_p.grp_pos_info[0].pos_tag.data[0] = 0x3f;  // 有效控制轴（6轴）
    corrpath_src_p.grp_pos_info[0].pos_tag.data[2] = 0;
    corrpath_src_p.grp_pos_info[0].pos_tag.data[3] = MP_CORR_RF_DTYPE;  // 机器人坐标系数据
    corrpath_src_p.grp_pos_info[0].pos[0] = 0;  // 修正量X
    corrpath_src_p.grp_pos_info[0].pos[1] = 0;  // 修正量Y
    corrpath_src_p.grp_pos_info[0].pos[2] = 0;  // 修正量Z
    corrpath_src_p.grp_pos_info[0].pos[3] = 0;  // 修正量RX
    corrpath_src_p.grp_pos_info[0].pos[4] = 0;  // 修正量RY
    corrpath_src_p.grp_pos_info[0].pos[5] = 0;  // 修正量RZ
    spd_src_p[0] = 15000;
    FOREVER {
        /*通知 I/O 周期或插补周期
        STATUS mpClkAnnounce
        (
             int clk_id
        )
        调出这个函数的任务都会在通知各时钟（事件）之前被暂停（同步信号量）。 */
        mpClkAnnounce(MP_INTERPOLATION_CLK); // 按插补周期执行以下处理
        switch (Process) {  // 接收到的JBI指令
        case 0:
            cnt++;
            if (cnt > 30) {
                cnt = 0;
                SetBVar(0, 1);
                SetBVar(1, 1);
                SetBVar(8, 1);
                if (pos_flag == 1) {
                    pos_flag = 0;
                    hd_GetPosServer();
                }
            }
            break;
        // PutCorrPath
        case 1:
            hd_AskPathServer();
            if (ask_flag == 1) {
                SetBVar(0, 0);
            }
            break;
        // SpdOverride_on
        case 2:
            hd_ConnectServer();
            if (connect_flag == 1) {
                SetBVar(1, 0);
            }
            break;
        // SpdOverride_off
        case 3:
            cnt++;
            // wait 1s
            if (cnt > 250) {
                cnt = 0;
                Process = 0;
            }
            break;
        // ForcePathEnd
        case 4:
            cnt++;
            // wait 3s
            if (cnt > 750) {
                /*强制结束执行中的移动命令，切换至下一程序点运行
                int mpMeiPutForcePathEnd
                (
                     int sl_id, 与系统的通信区域指定
                     CTRLG_ T ctrl_grp
                ) */
                ret = mpMeiPutForcePathEnd(MP_SL_ID1, ctrl_grp);
                if (ret != 0) {
                    // printf("mpMeiPutForcePathEnd Error ret = %d\n\r", ret);
                } else {
                    // printf("mpMeiPutForcePathEnd done\n\r");
                }
                Process = 0;
                cnt = 0;
            }
            break;
        case 5:
            hd_StartSprayServer();
            if (spray_flag == 1) {
                SetBVar(8, 0);
            }
            break;
        default:
            dy = 0;
            cnt = 0;
            break;
        }
    }
}

void hd_ConnectServer() {  // 连接服务器
    Process = 0;
    connect_flag = 0;
    struct sockaddr_in serverSockAddr;

    mpClose(sockHandle);  // close first
    sockHandle = mpSocket(AF_INET, SOCK_STREAM, 0);
    if (sockHandle < 0) {
        connect_flag = 0;
        return;
    }

    memset(&serverSockAddr, 0, sizeof(serverSockAddr));
    serverSockAddr.sin_family = AF_INET;
    serverSockAddr.sin_addr.s_addr = mpInetAddr("192.168.100.1");
    serverSockAddr.sin_port = mpHtons(11000);

    int ret = mpConnect(sockHandle, (struct sockaddr *)&serverSockAddr, sizeof(serverSockAddr));
    if (ret < 0) {
        connect_flag = 0;
        return;
    }

    connect_flag = 1;
}

void hd_AskPathServer() {  // 请求路点
    Process = 0;
    int bytesSend = mpSend(sockHandle, "1001$$", 6, 0);

    if (bytesSend < 0) {
        ask_flag = 0;
        return;
    }
    ask_flag = 1;
}

void hd_StartSprayServer() {  // 开始喷涂
    Process = 0;
    int bytesSend = mpSend(sockHandle, "1004$$", 6, 0);

    if (bytesSend < 0) {
        spray_flag = 0;
        return;
    }
    spray_flag = 1;
}

void hd_GetPosServer() {  // 获取机器人当前正交位姿以及关节角
    SetBVar(10, 1);

    // 获取位姿
    MP_CTRL_GRP_SEND_DATA sData;
    MP_CART_POS_RSP_DATA rData;
    char pos[BUFF_MAX + 1], pos2[BUFF_MAX + 1];
    sData.sCtrlGrp = 0;  // 指定获取第一个机器人的信息
    int res = mpGetCartPos(&sData, &rData);

    long l_pos = rData.lPos[0];
    sprintf(pos, "%ld", l_pos);
    strcat(pos, "$$");
    int i = 0;
    for (i = 1; i < 6; i++) {
        long l_temp = rData.lPos[i];
        sprintf(pos2, "%ld", l_temp);
        strcat(pos2, "$$");
        strcat(pos, pos2);
    }

    // 获取关节角
    MP_CTRL_GRP_SEND_DATA sDataJoint;
    MP_DEG_POS_RSP_DATA rDataJoint;
    sDataJoint.sCtrlGrp = 0;  // 指定获取第一个机器人的信息
    mpGetDegPos(&sDataJoint, &rDataJoint);
    for (i = 0; i < 6; i++) {
        long l_temp = rDataJoint.lDegPos[i];
        sprintf(pos2, "%ld", l_temp);
        strcat(pos2, "$$");
        strcat(pos, pos2);
    }

    int bytesSend = mpSend(sockHandle, &pos, strlen(pos), 0);
}

void socketRecv_Task(void) {  // 套接字接收
    while (1) {
        int bytesRecv;
        int bytesSend;
        int valRet;
        float target[COORD_NUM + 1];  // 当前目标点
        int buffNum = 0;  // 当前接收数据点数
        int targetNum = 0;  // 当前目标点数
        int flagNum = 0;  // 当前字符在字符串中的位置
        int charNum = 0;
        float prevTarget[COORD_NUM + 1];  // 存放上一个目标点，用于生成摆焊上个点以及摆焊参考点
        float swingRefTarget[COORD_NUM + 1];  // 存放摆焊参考点
        float swingPrevTarget[COORD_NUM + 1];  // 存放摆焊所需的上一点

        SetBVar(3, 1);

        while (1) {
            char buff[BUFF_MAX + 1]; // buff[1023+1]
            memset(buff, 0, sizeof(buff));
            char data[BUFF_MAX + 1];
            memset(data, 0, sizeof(data));

            bytesRecv = mpRecv(sockHandle, buff, BUFF_MAX, 0);
            if (bytesRecv < 0) {
                SetBVar(2, 0);
                break;
            }

            // bytesSend = mpSend(sockHandle, buff, bytesRecv, 0);
            // if (bytesSend != bytesRecv) {SetBVar(2, 0);break;}

            SetBVar(2, 1);
            SetBVar(3, 0);

            int i = 0;
            // 开始解包
            for (i = 0; i < strlen(buff); i++) {
                // 判定结束标志位
                if (buff[i] == '$') {
                    flagNum++;
                    // 连续收到2个标志位才证明该数据发送完毕，判定数据类型并进行相应处理
                    if (flagNum == 2) {
                        data[charNum] = '\0';
                        charNum = 0;
                        flagNum = 0;
                        // 长度限定的字符串比较函数，即如果发送了除START或者EXIT以外的路点数据，则开始打包
                        if (strncmp(data, "EXIT", 4) != 0 && strncmp(data, "START", 5) != 0 && strncmp(data, "POSE", 4) != 0
                            && strncmp(data, "MOVEJL", 6) != 0 && strncmp(data, "MOVEJR", 6) != 0) {
                            float fbuff = atof(data);
                            int CURR_SWING_METHOD = LINE_WELD;  // 当前摆焊方式, 默认不摆焊

                            if (buffNum < 6) {  // 如果目标点数据还没到6个, 就继续存入数组
                                target[buffNum] = fbuff;
                                buffNum++;
                            } else if (buffNum < 7) {  // 如果到了6个没到7个, 说明当前数据是速度, 就存到速度处
                                if (targetNum % 2 == 0) {
                                    SetIVar(1, (int)fbuff);  // 1，3，5...的速度存进I001
                                } else {
                                    SetIVar(2, (int)fbuff);  // 2，4，6...的速度存进I002
                                }
                                buffNum++;
                            } else if (buffNum < 8) {  // 如果到了7个没到8个, 说明当前数据是起弧指令, 就存到起弧处
                                if (targetNum % 2 == 0) {
                                    SetBVar(11, (int)fbuff);  // 1，3，5...的起弧存进B011
                                } else {
                                    SetBVar(12, (int)fbuff);  // 2，4，6...的起弧存进B012
                                }
                                buffNum++;
                            } else if (buffNum < 9)  {  // 如果到了9个, 说明当前数据是摆焊指令, 就存到摆焊处
                                CURR_SWING_METHOD = (int)fbuff;

                                if ((targetNum != 0) && (CURR_SWING_METHOD != LINE_WELD)) {  // 也就是prevTarget不为空且确实需要摆焊
                                    int j = 0;
                                    for (j = 0; j < COORD_NUM; j++) {  // 摆焊参考点和上一点以上一次运动目标点为基准
                                        swingRefTarget[j] = prevTarget[j];
                                        swingPrevTarget[j] = prevTarget[j];
                                    }
                                    // 根据工件/焊缝的不同位置计算相应的摆焊参考点和上一点, 0 1 2分别为点的X Y Z, 参考点规则参见安川手册，有点歧义，前一接近点swingPrevTarget作为水平的方向以实际实验为准
                                    switch (CURR_SWING_METHOD) {
                                        case FRONT_LEFT_VERTICAL_SWING_WELD:
                                            swingRefTarget[1] -= 10 * 1000;
                                            swingPrevTarget[0] += 0.1 * 1000;
                                            swingPrevTarget[1] += 0.1 * 1000;
                                            break;
                                        case FRONT_RIGHT_VERTICAL_SWING_WELD:
                                            swingRefTarget[1] += 10 * 1000;
                                            swingPrevTarget[0] += 0.1 * 1000;
                                            swingPrevTarget[1] -= 0.1 * 1000;
                                            break;
                                        case LEFT_LEFT_VERTICAL_SWING_WELD:
                                            swingRefTarget[0] += 10 * 1000;
                                            swingPrevTarget[0] -= 0.1 * 1000;
                                            swingPrevTarget[1] += 0.1 * 1000;
                                            break;
                                        case LEFT_RIGHT_VERTICAL_SWING_WELD:
                                            swingRefTarget[0] -= 10 * 1000;
                                            swingPrevTarget[0] += 0.1 * 1000;
                                            swingPrevTarget[1] += 0.1 * 1000;
                                            break;
                                        case RIGHT_LEFT_VERTICAL_SWING_WELD:
                                            swingRefTarget[0] -= 10 * 1000;
                                            swingPrevTarget[0] += 0.1 * 1000;
                                            swingPrevTarget[1] -= 0.1 * 1000;
                                            break;
                                        case RIGHT_RIGHT_VERTICAL_SWING_WELD:
                                            swingRefTarget[0] += 10 * 1000;
                                            swingPrevTarget[0] -= 0.1 * 1000;
                                            swingPrevTarget[1] -= 0.1 * 1000;
                                            break;
                                    }

                                    if (targetNum % 2 == 0) {
                                        valRet = setValP(swingRefTarget, 7);  // 1，3，5...的摆焊参考点存进P007
                                        if (valRet < 0) {
                                            SetBVar(2, 0);
                                            break;
                                        }
                                        valRet = setValP(swingPrevTarget, 9);  // 1，3，5...的摆焊上一点存进P009
                                        if (valRet < 0) {
                                            SetBVar(2, 0);
                                            break;
                                        }
                                        SetBVar(15, 1);
                                        SetIVar(3, (int)fbuff);  // 1，3，5...的摆焊类型存进I003
                                    } else {
                                        valRet = setValP(swingRefTarget, 8);  // 2，4，6...的摆焊参考点存进P008
                                        if (valRet < 0) {
                                            SetBVar(2, 0);
                                            break;
                                        }
                                        valRet = setValP(swingPrevTarget, 10);  // 2，4，6...的摆焊上一点存进P010
                                        if (valRet < 0) {
                                            SetBVar(2, 0);
                                            break;
                                        }
                                        SetBVar(15, 1);
                                        SetIVar(4, (int)fbuff);  // 2，4，6...的摆焊类型存进I004
                                    }
                                }

                                // 存储摆焊类型
                                if (targetNum % 2 == 0) {
                                        SetIVar(3, (int)fbuff);  // 1，3，5...的摆焊类型存进I003
                                    } else {
                                        SetIVar(4, (int)fbuff);  // 2，4，6...的摆焊类型存进I004
                                    }
                                buffNum++;
                            } else if (buffNum < 10) {  // 如果到了10个, 说明当前数据是电流指令。
                                 if (targetNum % 2 == 0) {
                                    SetIVar(5, (int)fbuff);  // 1，3，5...的起弧存进B011
                                } else {
                                    SetIVar(7, (int)fbuff);  // 2，4，6...的起弧存进B012
                                }
                                buffNum++;
                            } else if (buffNum < 11) {  // 如果到了11个, 说明当前数据是电压指令。
                                 if (targetNum % 2 == 0) {
                                    SetIVar(6, (int)fbuff);  // 1，3，5...的起弧存进B011
                                } else {
                                    SetIVar(8, (int)fbuff);  // 2，4，6...的起弧存进B012
                                }
                                buffNum++;
                            }

                            // 每解包出9组数据打包一次 (第7个数为速度, 第8个数为是否起弧, 第9个数为摆焊类型)
                            if (buffNum == 11) {
                                targetNum++;

                                int j = 0;
                                for (j = 0; j < COORD_NUM; j++) {
                                    prevTarget[j] = target[j];
                                }

                                if (targetNum % 2 != 0) {
                                    valRet = setValP(target, 1);  // 1，3，5...打包进P001
                                } else {
                                    valRet = setValP(target, 2);  // 2，4，6...打包进P002
                                }
                                if (valRet < 0) {
                                    SetBVar(2, 0);
                                    break;
                                }

                                buffNum = 0;
                                SetBVar(5, 1);
                            }
                        }

                        // 长度限定的字符串比较函数，即如果发送了POSE或者pose，则发送一组当前位姿
                        if (strncmp(data, "POSE", 4) == 0 || strncmp(data, "pose", 4) == 0) {
                            pos_flag = 1;
                            SetBVar(9, 1);
                        }

                        // 长度限定的字符串比较函数，即如果发送了START或者start，则跳出WAIT开始进行新一轮循环
                        if (strncmp(data, "START", 5) == 0 || strncmp(data, "start", 5) == 0) {
                            SetBVar(7, 1);
                        }
                        // 长度限定的字符串比较函数，即如果发送了exit或者EXIT，则停止接收信息
                        if (strncmp(data, "EXIT", 4) == 0 || strncmp(data, "exit", 4) == 0) {
                            bytesSend = mpSend(sockHandle, "1003$$", 6, 0);
                            SetBVar(4, 1);  // B004=1
                            connect_flag = 0;
                            // mpClose(sockHandle);  //++
                            // targetNum = 0;
                            break;
                        }
                        // 长度限定的字符串比较函数，即如果发送了MOVEJL或者movejl，则运动到左工作台状态
                        if (strncmp(data, "MOVEJL", 6) == 0 || strncmp(data, "movejl", 6) == 0) {
                            long targetPulse[COORD_NUM + 1];
                            targetPulse[0] = -130609;  // 右侧抬起姿态
                            targetPulse[1] = -22371;
                            targetPulse[2] = -12795;
                            targetPulse[3] = 1144;
                            targetPulse[4] = -25933;
                            targetPulse[5] = -1626;
                            setValP_Pulse(targetPulse, 4);
                            targetPulse[0] = 129732;  // 左侧抬起姿态
                            targetPulse[1] = -22371;
                            targetPulse[2] = -12793;
                            targetPulse[3] = 1146;
                            targetPulse[4] = -25932;
                            targetPulse[5] = -1625;
                            setValP_Pulse(targetPulse, 5);
                            targetPulse[0] = 122774;  // 左侧放下姿态
                            targetPulse[1] = -29490;
                            targetPulse[2] = -42311;
                            targetPulse[3] = 825;
                            targetPulse[4] = -77619;
                            targetPulse[5] = 650;
                            setValP_Pulse(targetPulse, 6);

                            SetBVar(13, 1);  // B013=1
                            SetBVar(7, 1);  // B007=1
                        }
                        // 长度限定的字符串比较函数，即如果发送了MOVEJR或者movejr，则运动到右工作台状态
                        if (strncmp(data, "MOVEJR", 6) == 0 || strncmp(data, "movejr", 6) == 0) {
                            long targetPulse[COORD_NUM + 1];
                            targetPulse[0] = 129732;  // 左侧抬起姿态
                            targetPulse[1] = -22371;
                            targetPulse[2] = -12793;
                            targetPulse[3] = 1146;
                            targetPulse[4] = -25932;
                            targetPulse[5] = -1625;
                            setValP_Pulse(targetPulse, 4);
                            targetPulse[0] = -130609;  // 右侧抬起姿态
                            targetPulse[1] = -22371;
                            targetPulse[2] = -12795;
                            targetPulse[3] = 1144;
                            targetPulse[4] = -25933;
                            targetPulse[5] = -1626;
                            setValP_Pulse(targetPulse, 5);
                            targetPulse[0] = -128543;  // 右侧放下姿态
                            targetPulse[1] = -21267;
                            targetPulse[2] = -36717;
                            targetPulse[3] = 1148;
                            targetPulse[4] = -75360;
                            targetPulse[5] = -1628;
                            setValP_Pulse(targetPulse, 6);

                            SetBVar(14, 1);  // B014=1
                            SetBVar(7, 1);  // B007=1
                        }
                    }
                } else {
                    // 储存解包数据
                    data[charNum] = buff[i];
                    charNum++;
                }
            }
        }
    }
}
