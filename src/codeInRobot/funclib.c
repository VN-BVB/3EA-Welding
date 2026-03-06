#include "motoPlus.h"

int setValP(const float *target, int pointNum);
int setValP_Pulse(const long *target, int pointNum);
int GetBVar(UINT16 index, long *value);
int SetBVar(UINT16 index, long value);
int cmp_usr_var_info(MP_USR_VAR_INFO *info1, MP_USR_VAR_INFO *info2);
int cmp_pos_var_info(MP_P_VAR_BUFF *buf1, MP_P_VAR_BUFF *buf2);
int readPos(MP_CART_POS_RSP_DATA *rData);

#define COORD_NUM 6  // X, Y, Z, Rx, Ry, Rz

int setValP(const float *target, int pointNum) {
    int i = CLEAR;
	STATUS status = ERROR;
    MP_USR_VAR_INFO Info;
    MP_USR_VAR_INFO rInfo;
    MP_CART_POS_RSP_DATA rData;

    memset(&Info, 0, sizeof(Info));
    memset(&rInfo, 0, sizeof(rInfo));

    int retPos = readPos(&rData);
    if(retPos < 0) {
       return -1;
    }

    Info.var_type = rInfo.var_type = MP_VAR_P;
	Info.var_no = rInfo.var_no = pointNum;
	Info.val.p.dtype = MP_ROBO_DTYPE;  // 机器人坐标系
	Info.val.p.tool_no = 0;
	Info.val.p.fig_ctrl = rData.sConfig;
	for (i = 0; i < COORD_NUM; i++) {
		Info.val.p.data[i] = target[i];
	}

	status = mpPutUserVars(&Info);  // 将设置好的数据按变量索引传入机器人用户变量中
	if (status != OK) {
		return -1;
	}

	return 0;
}

int setValP_Pulse(const long *target, int pointNum) {
	int i = 0;
	STATUS status = ERROR;
	MP_USR_VAR_INFO Info;
	MP_CART_POS_RSP_DATA rData;

	memset(&Info, 0, sizeof(Info));

	int retPos = readPos(&rData);
    if(retPos < 0) {
       return -1;
    }

	Info.var_type = MP_VAR_P;
	Info.var_no = pointNum;
	Info.val.p.dtype = MP_PULSE_DTYPE;  // 脉冲坐标系
	Info.val.p.tool_no = 0;
	Info.val.p.fig_ctrl = rData.sConfig;
	for (i = 0; i < COORD_NUM; i++) {
		Info.val.p.data[i] = target[i];
	}

	status = mpPutUserVars(&Info);  // 将设置好的数据按变量索引传入机器人用户变量中
	if (status != OK) {
		return -1;
	}

	return 0;
}

// 设定数据与读取数据的比较函数
int cmp_usr_var_info(MP_USR_VAR_INFO *info1, MP_USR_VAR_INFO *info2) {
    int ret = OK;
	ret = cmp_pos_var_info(&(info1->val.p), &(info2->val.p)); //坐标型变量另设计一个比较函数

	return (ret);
}

// 位置型变量的比较函数
int cmp_pos_var_info(MP_P_VAR_BUFF *buf1, MP_P_VAR_BUFF *buf2) {
	int i = CLEAR;
	int ret = OK;

	if (buf1->dtype != buf2->dtype) {
		ret = ERROR;
	}
	if ((buf1->dtype == MP_USER_DTYPE) &&
	    (buf1->uf_no != buf2->uf_no)) {
		ret = ERROR;
	}
	if (buf1->tool_no != buf2->tool_no) {
		ret = ERROR;
	}
	if ((buf1->dtype >= MP_BASE_DTYPE) &&
	    (buf1->fig_ctrl != buf2->fig_ctrl)) {
		ret = ERROR;
	}
	for (i = CLEAR; i < MP_GRP_AXES_NUM; i++) {
		if (buf1->data[i] != buf2->data[i]) {
			ret = ERROR;
			break;
		}
	}

	return (ret);
}

int GetBVar(UINT16 index, long *value) {
    MP_VAR_INFO info;

    info.usType = MP_RESTYPE_VAR_B;  // 字节型
    info.usIndex = index; 

    return mpGetVarData(&info, value, 1);
}

int SetBVar(UINT16 index, long value) {
    MP_VAR_DATA info;

    info.usType = MP_RESTYPE_VAR_B;
    info.usIndex = index;
    info.ulValue = value;

    return mpPutVarData(&info, 1);
}

int SetIVar(UINT16 index, long value) {
    MP_VAR_DATA info;

    info.usType = MP_RESTYPE_VAR_I;
    info.usIndex = index;
    info.ulValue = value;

    return mpPutVarData(&info, 1);
}

int readPos(MP_CART_POS_RSP_DATA *rData) {
    MP_CTRL_GRP_SEND_DATA sData;  // 控制组发送数据

    sData.sCtrlGrp = 0;	 // 机器人R1
   	return mpGetCartPos(&sData, rData);  // 把R1的正交坐标读进接收变量
}