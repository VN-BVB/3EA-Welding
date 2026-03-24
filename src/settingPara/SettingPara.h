#ifndef SETTINGPARA_H
#define SETTINGPARA_H

#include <plog/Log.h>

#include <QSettings>
#include <memory>
#include <mutex>

class SettingPara {
public:
    static SettingPara &getInstance();  // 获取单例的方法

    SettingPara(const SettingPara &) = delete;             // 禁止拷贝构造
    SettingPara &operator=(const SettingPara &) = delete;  // 禁止赋值操作

    void loadSetting();  // 载入配置文件

    // 相机相关
    int camera_exposure = 20000;         // 相机曝光
    int coarse_camera_exposure = 88888;  // 相机曝光
    int camera_gain = 5;                 // 相机增益

    // 投影仪相关
    int projector_brightness = 100;  // 投影仪亮度
    int projector_fps = 20;          // 投影仪帧率
    int projector_num = 21;          // 投影仪图片数量

    double modulation_threshold = 2;  // 调制度
    bool bool_save_model = 0;         // 是否保存重建的点云模型
    bool bool_save_picture = 0;       // 是否保存拍摄的图片
    /*------------------------------龙门支架------------------------------*/
    double TubeSidePlatFilletStart_X = 0;
    double TubeSidePlatFilletStart_Y = 0;
    double TubeSidePlatFilletStart_Z = 0;
    double TubeSidePlatFilletEnd_X = 0;
    double TubeSidePlatFilletEnd_Y = 0;
    double TubeSidePlatFilletEnd_Z = 0;
    double TubeSidePlatFilletWithdrawDistance = 0;
    /*------------------------------角钢------------------------------*/
    // 工件正面的焊缝起点和终点延长
    double FrontLeft_ExtendStart = -5;   // 正面左焊缝起点(内点)
    double FrontLeft_ExtendEnd = -5;     // 正面左焊缝终点(外点)
    double FrontRight_ExtendStart = -5;  // 正面右焊缝起点(内点)
    double FrontRight_ExtendEnd = -5;    // 正面右焊缝终点(外点)

    double FrontBeamLeft_ExtendStart = 0;   // 正面横梁对接左焊缝起点
    double FrontBeamLeft_ExtendEnd = 0;     // 正面横梁对接左焊缝终点
    double FrontBeamRight_ExtendStart = 0;  // 正面横梁对接右焊缝起点
    double FrontBeamRight_ExtendEnd = 0;    // 正面横梁对接右焊缝终点

    double FrontHBeamLeft_ExtendStart = 0;   // 正面横梁水平左焊缝起点
    double FrontHBeamLeft_ExtendEnd = -2;    // 正面横梁水平左焊缝终点
    double FrontHBeamRight_ExtendStart = 0;  // 正面横梁水平右焊缝起点
    double FrontHBeamRight_ExtendEnd = -2;   // 正面横梁水平右焊缝终点

    double FrontVBeamLeft_ExtendStart = 0;   // 正面横梁竖直左焊缝起点
    double FrontVBeamLeft_ExtendEnd = -2;    // 正面横梁竖直左焊缝终点
    double FrontVBeamRight_ExtendStart = 0;  // 正面横梁竖直右焊缝起点
    double FrontVBeamRight_ExtendEnd = -2;   // 正面横梁竖直右焊缝终点

    double FrontBeamLeft_WithDrawDistance = 12;   // 正面左侧焊枪后撤
    double FrontBeamRight_WithDrawDistance = 12;  // 正面右侧焊枪后撤

    // 工件反面的焊缝起点和终点延长
    double BackLeft_ExtendStart = -0.5;   // 反面左焊缝起点(内点)
    double BackLeft_ExtendEnd = -0.7;     // 反面左焊缝终点(外点)
    double BackRight_ExtendStart = -0.5;  // 反面右焊缝起点(内点)
    double BackRight_ExtendEnd = -0.7;    // 反面右焊缝终点(外点)

    double BackBeamLeft_ExtendStart = 0;   // 反面横梁对接左焊缝起点
    double BackBeamLeft_ExtendEnd = 0;     // 反面横梁对接左焊缝终点
    double BackBeamRight_ExtendStart = 0;  // 反面横梁对接右焊缝起点
    double BackBeamRight_ExtendEnd = 0;    // 反面横梁对接右焊缝终点

    // 边界端点到交点的距离阈值, 用于判断焊缝拍摄是否完整
    double MinDth_IntersectionToStartPoint = 3.5;
    // 南工作台 (机器人左侧)
    // 工件正面『边角』偏移设置
    double Front_Region1_X_Shift = 0, Front_Region1_Y_Shift = 0, Front_Region1_Z_Shift = 0;
    double Front_Region2_X_Shift = 0, Front_Region2_Y_Shift = 0, Front_Region2_Z_Shift = 0;
    // 工件反面『边角』偏移设置
    double Back_Region1_X_Shift = 0, Back_Region1_Y_Shift = 0, Back_Region1_Z_Shift = 0;
    double Back_Region2_X_Shift = 0, Back_Region2_Y_Shift = 0, Back_Region2_Z_Shift = 0;
    // 工件正面『横梁』偏移设置
    double Front_Beam_Region1_X_Shift = 0, Front_Beam_Region1_Y_Shift = 0, Front_Beam_Region1_Z_Shift = 0;
    double Front_Beam_Region2_X_Shift = 0, Front_Beam_Region2_Y_Shift = 0, Front_Beam_Region2_Z_Shift = 0;
    double Front_L_Beam_H_X_Shift = 0, Front_L_Beam_H_Y_Shift = 0, Front_L_Beam_H_Z_Shift = 0;
    double Front_L_Beam_V_X_Shift = 0, Front_L_Beam_V_Y_Shift = 0, Front_L_Beam_V_Z_Shift = 0;
    double Front_L_Beam_DH_X_Shift = 0, Front_L_Beam_DH_Y_Shift = 0, Front_L_Beam_DH_Z_Shift = 0;
    double Front_L_Beam_DV_X_Shift = 0, Front_L_Beam_DV_Y_Shift = 0, Front_L_Beam_DV_Z_Shift = 0;
    // 工件反面『横梁』偏移设置
    double Back_Beam_Region1_X_Shift = 0, Back_Beam_Region1_Y_Shift = 0, Back_Beam_Region1_Z_Shift = 0;
    double Back_Beam_Region2_X_Shift = 0, Back_Beam_Region2_Y_Shift = 0, Back_Beam_Region2_Z_Shift = 0;

    // 北工作台 (机器人右侧)
    // 工件正面『边角』偏移设置
    double Front_Region1_X_Shift_R = 0, Front_Region1_Y_Shift_R = 0, Front_Region1_Z_Shift_R = 0;
    double Front_Region2_X_Shift_R = 0, Front_Region2_Y_Shift_R = 0, Front_Region2_Z_Shift_R = 0;
    // 工件反面『边角』偏移设置
    double Back_Region1_X_Shift_R = 0, Back_Region1_Y_Shift_R = 0, Back_Region1_Z_Shift_R = 0;
    double Back_Region2_X_Shift_R = 0, Back_Region2_Y_Shift_R = 0, Back_Region2_Z_Shift_R = 0;
    // 工件正面『横梁』偏移设置
    double Front_Beam_Region1_X_Shift_R = 0, Front_Beam_Region1_Y_Shift_R = 0, Front_Beam_Region1_Z_Shift_R = 0;
    double Front_Beam_Region2_X_Shift_R = 0, Front_Beam_Region2_Y_Shift_R = 0, Front_Beam_Region2_Z_Shift_R = 0;
    double Front_R_Beam_H_X_Shift = 0, Front_R_Beam_H_Y_Shift = 0, Front_R_Beam_H_Z_Shift = 0;
    double Front_R_Beam_V_X_Shift = 0, Front_R_Beam_V_Y_Shift = 0, Front_R_Beam_V_Z_Shift = 0;
    double Front_R_Beam_DH_X_Shift = 0, Front_R_Beam_DH_Y_Shift = 0, Front_R_Beam_DH_Z_Shift = 0;
    double Front_R_Beam_DV_X_Shift = 0, Front_R_Beam_DV_Y_Shift = 0, Front_R_Beam_DV_Z_Shift = 0;
    // 工件反面『横梁』偏移设置
    double Back_Beam_Region1_X_Shift_R = 0, Back_Beam_Region1_Y_Shift_R = 0, Back_Beam_Region1_Z_Shift_R = 0;
    double Back_Beam_Region2_X_Shift_R = 0, Back_Beam_Region2_Y_Shift_R = 0, Back_Beam_Region2_Z_Shift_R = 0;
    /*---------------------------------------------------------------*/
    // 焊接机器人相关
    double Value_MoveSpeed = 170;                // 过渡运动速度
    double Value_WeldingSpeed = 5;               // 焊接速度(默认焊接速度, 焊缝宽度检测失败用这个速度)
    double Value_WeldingSpeed0To1 = 5;           // 焊接速度(焊缝宽度1mm以下用这个速度)
    double Value_WeldingSpeed1To3 = 5;           // 焊接速度(焊缝宽度1mm到3mm用这个速度)
    double Value_WeldingSpeed3To5 = 5;           // 焊接速度(焊缝宽度3mm到5mm用这个速度)
    double Value_WeldingSpeedHorizontal = 5;     // 焊接速度(水平焊缝用这个速度)
    double Value_WeldingSpeedVertical = 5;       // 焊接速度(水平焊缝用这个速度)
    double Value_WeldingCurrent = 160;           // 焊接电流(默认焊接电流)
    double Value_WeldingCurrent_Vertical = 130;  // 焊接电流(竖直焊缝用这个电流)
    double Value_WeldingVoltage = 24;            // 焊接电压(默认焊接电压)
    double Value_WeldingVoltage_Vertical = 18;   // 焊接电压(竖直焊缝用这个电压)

    bool weldingVerticalWeld = 0;  // 是否焊接竖直焊缝

    // 点云预处理参数
    double passthrough_Min = 0, passthrough_Max = 1000;  // 直通滤波参数
    double statistical_Pts = 20, statistical_Std = 3;    // 统计滤波参数

    // 深度学习相关
    double DL_PredictThreshold = 0.2;  // 深度学习预测阈值

    // 点云放缩平移
    double scaleOfPointX = 1.0;  // X坐标的放缩量
    double transOfPointX = 0.0;  // X坐标的平移量
    double scaleOfPointY = 1.0;  // Y坐标的放缩量
    double transOfPointY = 0.0;  // Y坐标的平移量

private:
    SettingPara();
    ~SettingPara();

    static SettingPara *instance;     // 静态实例指针
    static std::mutex instanceMutex;  // 保护静态实例的互斥锁

    std::shared_ptr<QSettings> qSetting{nullptr};  // Qt配置类

    friend class SettingWidget;  // 友元类
};

#endif  // SETTINGPARA_H
