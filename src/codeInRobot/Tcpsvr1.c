/* TcpSvr1.c */
/* Copyright 2009 YASKAWA ELECTRIC All Rights reserved. */
#include "motoPlus.h"

// for GLOBAL DATA DEFINITIONS
SEM_ID semid;

// for IMPORT API & FUNCTIONS
extern void CmdRcv_Task(void);  // 接受机器人的指令
extern void CorrPath_Task(void);
extern void socketRecv_Task(void);

// for LOCAL DEFINITIONS
int nTaskID1;
int nTaskID2;
int nTaskID3;

// 该函数在系统启动时被调用，用于创建任务和初始化资源，其中创建的任务在宏观上是一个线程，motoPluss是一个RTOS（实时操作系统）。
// 该线程的优先级和堆栈大小可以通过参数进行设置。
// MP_PRI_TIME_NORMAL: 任务优先级为正常时间优先级（优先级3）
// MP_PRI_IP_CLK_TAKE: 任务优先级为插补周期时间优先级（优先级1）
void mpUsrRoot(int arg1, int arg2, int arg3, int arg4, int arg5,
	       int arg6, int arg7, int arg8, int arg9, int arg10) {
    // 任务启动
    nTaskID1 = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, (FUNCPTR)CmdRcv_Task, 
                            arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10);
                       
    nTaskID2 = mpCreateTask(MP_PRI_IP_CLK_TAKE, MP_STACK_SIZE, (FUNCPTR)CorrPath_Task, 
                            arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10);
                       
    nTaskID3 = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, (FUNCPTR)socketRecv_Task, 
                            arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10);
	// 初始化 0 的二进制信号量
	semid = mpSemBCreate(SEM_Q_FIFO, SEM_EMPTY);	
	puts("Exit mpUsrRoot!");
	mpExitUsrRoot;  // (or) mpSuspendSelf;
}
