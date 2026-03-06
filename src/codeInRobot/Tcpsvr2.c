/* TcpSvr2.c */
/*与PC进行socket通信，实现变量打包*/

#include "motoPlus.h"

extern int setValP(const float *target,int pointNum);
extern int GetBVar(UINT16 index, long *value);
extern int SetBVar(UINT16 index, long value);
extern int cmp_usr_var_info(MP_USR_VAR_INFO *info1, MP_USR_VAR_INFO *info2);
extern int cmp_pos_var_info(MP_P_VAR_BUFF *buf1, MP_P_VAR_BUFF *buf2);
extern int readPos(MP_CART_POS_RSP_DATA *rData);

// for API & FUNCTIONS
void socketTcp_Ask(void);

#define PORT      11000  // 系统软件用的端口是9900~10499 禁止使用，推荐用20000~23000
#define BUFF_MAX  1023
#define COORD_NUM 6  // X, Y, Z, Rx, Ry, Rz

extern int sockHandle;
extern int connect_flag;

void socketTcp_Ask(void) {
    //puts("Activate moto_plus0_task!");

    ap_TCP_Sserver(PORT);

    mpSuspendSelf;
}


