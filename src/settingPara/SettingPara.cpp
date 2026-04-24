#include "SettingPara.h"

// 初始化静态成员
SettingPara *SettingPara::instance = nullptr;  // 静态实例指针
std::mutex SettingPara::instanceMutex;         // 保护静态实例的互斥锁

// 获取单例的方法
SettingPara &SettingPara::getInstance() {
    std::lock_guard<std::mutex> lock(instanceMutex);  // 锁定互斥锁
    if (instance == nullptr) {                        // 如果实例为空
        instance = new SettingPara();                 // 创建实例
    }
    return *instance;  // 返回实例引用
}

SettingPara::SettingPara() : qSetting(std::make_shared<QSettings>("./data/setting/Parematers.ini", QSettings::IniFormat)) { this->loadSetting(); }

// 载入配置参数
void SettingPara::loadSetting() {
    if (qSetting != nullptr) {
        // 相机曝光和增益
        camera_exposure = qSetting->value("Camera/camera_exposure", camera_exposure).toInt();
        coarse_camera_exposure = qSetting->value("Camera/coarse_camera_exposure", camera_exposure).toInt();
        camera_gain = qSetting->value("Camera/camera_gain", camera_gain).toInt();
        camera_gain = std::min(camera_gain, 3);
        camera_gain = std::max(camera_gain, 0);

        // 投影仪亮度、帧率、投图片数量
        projector_brightness = qSetting->value("Projector/projector_brightness", projector_brightness).toInt();
        projector_fps = qSetting->value("Projector/projector_fps", projector_fps).toInt();
        projector_num = qSetting->value("Projector/projector_num", projector_num).toInt();

        // 调制度
        modulation_threshold = qSetting->value("Reconstruction/modulation_threshold", modulation_threshold).toDouble();
        // 龙门支架焊缝调整参数
        TubeSidePlatFilletStart_X = qSetting->value("GFSeamPosition/TubeSidePlatFilletStart_X", TubeSidePlatFilletStart_X).toDouble();
        TubeSidePlatFilletStart_Y = qSetting->value("GFSeamPosition/TubeSidePlatFilletStart_Y", TubeSidePlatFilletStart_Y).toDouble();
        TubeSidePlatFilletStart_Z = qSetting->value("GFSeamPosition/TubeSidePlatFilletStart_Z", TubeSidePlatFilletStart_Z).toDouble();
        TubeSidePlatFilletEnd_X = qSetting->value("GFSeamPosition/TubeSidePlatFilletEnd_X", TubeSidePlatFilletEnd_X).toDouble();
        TubeSidePlatFilletEnd_Y = qSetting->value("GFSeamPosition/TubeSidePlatFilletEnd_Y", TubeSidePlatFilletEnd_Y).toDouble();
        TubeSidePlatFilletEnd_Z = qSetting->value("GFSeamPosition/TubeSidePlatFilletEnd_Z", TubeSidePlatFilletEnd_Z).toDouble();
        TubeSidePlatFilletWithdrawDistance =
            qSetting->value("GFSeamPosition/TubeSidePlatFilletWithdrawDistance", TubeSidePlatFilletWithdrawDistance).toDouble();
        PlatePlateFilletHorizontalStart_X =
            qSetting->value("GFSeamPosition/PlatePlateFilletHorizontalStart_X", PlatePlateFilletHorizontalStart_X).toDouble();
        PlatePlateFilletHorizontalStart_Y =
            qSetting->value("GFSeamPosition/PlatePlateFilletHorizontalStart_Y", PlatePlateFilletHorizontalStart_Y).toDouble();
        PlatePlateFilletHorizontalStart_Z =
            qSetting->value("GFSeamPosition/PlatePlateFilletHorizontalStart_Z", PlatePlateFilletHorizontalStart_Z).toDouble();
        PlatePlateFilletHorizontalEnd_X =
            qSetting->value("GFSeamPosition/PlatePlateFilletHorizontalEnd_X", PlatePlateFilletHorizontalEnd_X).toDouble();
        PlatePlateFilletHorizontalEnd_Y =
            qSetting->value("GFSeamPosition/PlatePlateFilletHorizontalEnd_Y", PlatePlateFilletHorizontalEnd_Y).toDouble();
        PlatePlateFilletHorizontalEnd_Z =
            qSetting->value("GFSeamPosition/PlatePlateFilletHorizontalEnd_Z", PlatePlateFilletHorizontalEnd_Z).toDouble();
        PlatePlateFilletHorizontalWithdrawDistance =
            qSetting->value("GFSeamPosition/PlatePlateFilletHorizontalWithdrawDistance", PlatePlateFilletHorizontalWithdrawDistance).toDouble();
        PlatePlateFilletVerticalStart_X =
            qSetting->value("GFSeamPosition/PlatePlateFilletVerticalStart_X", PlatePlateFilletVerticalStart_X).toDouble();
        PlatePlateFilletVerticalStart_Y =
            qSetting->value("GFSeamPosition/PlatePlateFilletVerticalStart_Y", PlatePlateFilletVerticalStart_Y).toDouble();
        PlatePlateFilletVerticalStart_Z =
            qSetting->value("GFSeamPosition/PlatePlateFilletVerticalStart_Z", PlatePlateFilletVerticalStart_Z).toDouble();
        PlatePlateFilletVerticalEnd_X = qSetting->value("GFSeamPosition/PlatePlateFilletVerticalEnd_X", PlatePlateFilletVerticalEnd_X).toDouble();
        PlatePlateFilletVerticalEnd_Y = qSetting->value("GFSeamPosition/PlatePlateFilletVerticalEnd_Y", PlatePlateFilletVerticalEnd_Y).toDouble();
        PlatePlateFilletVerticalEnd_Z = qSetting->value("GFSeamPosition/PlatePlateFilletVerticalEnd_Z", PlatePlateFilletVerticalEnd_Z).toDouble();
        PlatePlateFilletVerticalWithdrawDistance =
            qSetting->value("GFSeamPosition/PlatePlateFilletVerticalWithdrawDistance", PlatePlateFilletVerticalWithdrawDistance).toDouble();
        TubePlateFilletStartOffset = qSetting->value("GFSeamPosition/TubePlateFilletStartOffset", TubePlateFilletStartOffset).toDouble();
        TubePlateFilletEndOffset = qSetting->value("GFSeamPosition/TubePlateFilletEndOffset", TubePlateFilletEndOffset).toDouble();
        TubePlateFilletWithdrawDistance = qSetting->value("GFSeamPosition/TubePlateFilletWithdrawDistance", TubePlateFilletWithdrawDistance).toDouble();
        TubeTubeFilletStartOffset = qSetting->value("GFSeamPosition/TubeTubeFilletStartOffset", TubeTubeFilletStartOffset).toDouble();
        TubeTubeFilletEndOffset = qSetting->value("GFSeamPosition/TubeTubeFilletEndOffset", TubeTubeFilletEndOffset).toDouble();
        TubeTubeFilletWithdrawDistance = qSetting->value("GFSeamPosition/TubeTubeFilletWithdrawDistance", TubeTubeFilletWithdrawDistance).toDouble();

        wireCalibrationOffset = qSetting->value("Welding/wireCalibrationOffset", wireCalibrationOffset).toDouble();
        toolRadius = qSetting->value("Welding/toolRadius", toolRadius).toDouble();

        // ----------------------------------------------------角钢-------------------------------------------------------------
        // 正面边角延长
        FrontLeft_ExtendStart = qSetting->value("SeamPosition/FrontLeft_ExtendStart", FrontLeft_ExtendStart).toDouble();
        FrontLeft_ExtendEnd = qSetting->value("SeamPosition/FrontLeft_ExtendEnd", FrontLeft_ExtendEnd).toDouble();
        FrontRight_ExtendStart = qSetting->value("SeamPosition/FrontRight_ExtendStart", FrontRight_ExtendStart).toDouble();
        FrontRight_ExtendEnd = qSetting->value("SeamPosition/FrontRight_ExtendEnd", FrontRight_ExtendEnd).toDouble();

        // 正面横梁对接延长
        FrontBeamLeft_ExtendStart = qSetting->value("SeamPosition/FrontBeamLeft_ExtendStart", FrontBeamLeft_ExtendStart).toDouble();
        FrontBeamLeft_ExtendEnd = qSetting->value("SeamPosition/FrontBeamLeft_ExtendEnd", FrontBeamLeft_ExtendEnd).toDouble();
        FrontBeamRight_ExtendStart = qSetting->value("SeamPosition/FrontBeamRight_ExtendStart", FrontBeamRight_ExtendStart).toDouble();
        FrontBeamRight_ExtendEnd = qSetting->value("SeamPosition/FrontBeamRight_ExtendEnd", FrontBeamRight_ExtendEnd).toDouble();

        // 正面横梁水平延长
        FrontHBeamLeft_ExtendStart = qSetting->value("SeamPosition/FrontHBeamLeft_ExtendStart", FrontHBeamLeft_ExtendStart).toDouble();
        FrontHBeamLeft_ExtendEnd = qSetting->value("SeamPosition/FrontHBeamLeft_ExtendEnd", FrontHBeamLeft_ExtendEnd).toDouble();
        FrontHBeamRight_ExtendStart = qSetting->value("SeamPosition/FrontHBeamRight_ExtendStart", FrontHBeamRight_ExtendStart).toDouble();
        FrontHBeamRight_ExtendEnd = qSetting->value("SeamPosition/FrontHBeamRight_ExtendEnd", FrontHBeamRight_ExtendEnd).toDouble();
        // 正面横梁竖直延长
        FrontVBeamLeft_ExtendStart = qSetting->value("SeamPosition/FrontVBeamLeft_ExtendStart", FrontVBeamLeft_ExtendStart).toDouble();
        FrontVBeamLeft_ExtendEnd = qSetting->value("SeamPosition/FrontVBeamLeft_ExtendEnd", FrontVBeamLeft_ExtendEnd).toDouble();
        FrontVBeamRight_ExtendStart = qSetting->value("SeamPosition/FrontVBeamRight_ExtendStart", FrontVBeamRight_ExtendStart).toDouble();
        FrontVBeamRight_ExtendEnd = qSetting->value("SeamPosition/FrontVBeamRight_ExtendEnd", FrontVBeamRight_ExtendEnd).toDouble();
        // 焊枪后撤
        FrontBeamLeft_WithDrawDistance =
            qSetting->value("SeamPosition/FrontBeamLeft_WithDrawDistance", FrontBeamLeft_WithDrawDistance).toDouble();  // 正面左侧焊枪后撤
        FrontBeamRight_WithDrawDistance =
            qSetting->value("SeamPosition/FrontBeamRight_WithDrawDistance", FrontBeamRight_WithDrawDistance).toDouble();  // 正面右侧焊枪后撤

        // 反面边角焊缝延长
        BackLeft_ExtendStart = qSetting->value("SeamPosition/BackLeft_ExtendStart", BackLeft_ExtendStart).toDouble();
        BackLeft_ExtendEnd = qSetting->value("SeamPosition/BackLeft_ExtendEnd", BackLeft_ExtendEnd).toDouble();
        BackRight_ExtendStart = qSetting->value("SeamPosition/BackRight_ExtendStart", BackRight_ExtendStart).toDouble();
        BackRight_ExtendEnd = qSetting->value("SeamPosition/BackRight_ExtendEnd", BackRight_ExtendEnd).toDouble();

        // 反面横梁焊缝延长
        BackBeamLeft_ExtendStart = qSetting->value("SeamPosition/BackBeamLeft_ExtendStart", BackBeamLeft_ExtendStart).toDouble();
        BackBeamLeft_ExtendEnd = qSetting->value("SeamPosition/BackBeamLeft_ExtendEnd", BackBeamLeft_ExtendEnd).toDouble();
        BackBeamRight_ExtendStart = qSetting->value("SeamPosition/BackBeamRight_ExtendStart", BackBeamRight_ExtendStart).toDouble();
        BackBeamRight_ExtendEnd = qSetting->value("SeamPosition/BackBeamRight_ExtendEnd", BackBeamRight_ExtendEnd).toDouble();

        // 边界端点到交点的距离阈值, 用于判断焊缝拍摄是否完整
        MinDth_IntersectionToStartPoint = qSetting->value("SeamPosition/MinDth_IntersectionToStartPoint", MinDth_IntersectionToStartPoint).toDouble();

        // 南工作台 (机器人左侧)
        // 工件正面偏移
        Front_Region1_X_Shift = qSetting->value("SeamPosition/Front_Region1_X_Shift", Front_Region1_X_Shift).toDouble();
        Front_Region1_Y_Shift = qSetting->value("SeamPosition/Front_Region1_Y_Shift", Front_Region1_Y_Shift).toDouble();
        Front_Region1_Z_Shift = qSetting->value("SeamPosition/Front_Region1_Z_Shift", Front_Region1_Z_Shift).toDouble();
        Front_Region2_X_Shift = qSetting->value("SeamPosition/Front_Region2_X_Shift", Front_Region2_X_Shift).toDouble();
        Front_Region2_Y_Shift = qSetting->value("SeamPosition/Front_Region2_Y_Shift", Front_Region2_Y_Shift).toDouble();
        Front_Region2_Z_Shift = qSetting->value("SeamPosition/Front_Region2_Z_Shift", Front_Region2_Z_Shift).toDouble();
        // 工件反面偏移
        Back_Region1_X_Shift = qSetting->value("SeamPosition/Back_Region1_X_Shift", Back_Region1_X_Shift).toDouble();
        Back_Region1_Y_Shift = qSetting->value("SeamPosition/Back_Region1_Y_Shift", Back_Region1_Y_Shift).toDouble();
        Back_Region1_Z_Shift = qSetting->value("SeamPosition/Back_Region1_Z_Shift", Back_Region1_Z_Shift).toDouble();
        Back_Region2_X_Shift = qSetting->value("SeamPosition/Back_Region2_X_Shift", Back_Region2_X_Shift).toDouble();
        Back_Region2_Y_Shift = qSetting->value("SeamPosition/Back_Region2_Y_Shift", Back_Region2_Y_Shift).toDouble();
        Back_Region2_Z_Shift = qSetting->value("SeamPosition/Back_Region2_Z_Shift", Back_Region2_Z_Shift).toDouble();
        // 工件正面横梁偏移
        Front_Beam_Region1_X_Shift = qSetting->value("SeamPosition/Front_Beam_Region1_X_Shift", Front_Beam_Region1_X_Shift).toDouble();
        Front_Beam_Region1_Y_Shift = qSetting->value("SeamPosition/Front_Beam_Region1_Y_Shift", Front_Beam_Region1_Y_Shift).toDouble();
        Front_Beam_Region1_Z_Shift = qSetting->value("SeamPosition/Front_Beam_Region1_Z_Shift", Front_Beam_Region1_Z_Shift).toDouble();
        Front_L_Beam_H_X_Shift = qSetting->value("SeamPosition/Front_L_Beam_H_X_Shift", Front_L_Beam_H_X_Shift).toDouble();
        Front_L_Beam_H_Y_Shift = qSetting->value("SeamPosition/Front_L_Beam_H_Y_Shift", Front_L_Beam_H_Y_Shift).toDouble();
        Front_L_Beam_H_Z_Shift = qSetting->value("SeamPosition/Front_L_Beam_H_Z_Shift", Front_L_Beam_H_Z_Shift).toDouble();
        Front_L_Beam_V_X_Shift = qSetting->value("SeamPosition/Front_L_Beam_V_X_Shift", Front_L_Beam_V_X_Shift).toDouble();
        Front_L_Beam_V_Y_Shift = qSetting->value("SeamPosition/Front_L_Beam_V_Y_Shift", Front_L_Beam_V_Y_Shift).toDouble();
        Front_L_Beam_V_Z_Shift = qSetting->value("SeamPosition/Front_L_Beam_V_Z_Shift", Front_L_Beam_V_Z_Shift).toDouble();
        Front_L_Beam_DH_X_Shift = qSetting->value("SeamPosition/Front_L_Beam_DH_X_Shift", Front_L_Beam_DH_X_Shift).toDouble();
        Front_L_Beam_DH_Y_Shift = qSetting->value("SeamPosition/Front_L_Beam_DH_Y_Shift", Front_L_Beam_DH_Y_Shift).toDouble();
        Front_L_Beam_DH_Z_Shift = qSetting->value("SeamPosition/Front_L_Beam_DH_Z_Shift", Front_L_Beam_DH_Z_Shift).toDouble();
        Front_L_Beam_DV_X_Shift = qSetting->value("SeamPosition/Front_L_Beam_DV_X_Shift", Front_L_Beam_DV_X_Shift).toDouble();
        Front_L_Beam_DV_Y_Shift = qSetting->value("SeamPosition/Front_L_Beam_DV_Y_Shift", Front_L_Beam_DV_Y_Shift).toDouble();
        Front_L_Beam_DV_Z_Shift = qSetting->value("SeamPosition/Front_L_Beam_DV_Z_Shift", Front_L_Beam_DV_Z_Shift).toDouble();

        Front_Beam_Region2_X_Shift = qSetting->value("SeamPosition/Front_Beam_Region2_X_Shift", Front_Beam_Region2_X_Shift).toDouble();
        Front_Beam_Region2_Y_Shift = qSetting->value("SeamPosition/Front_Beam_Region2_Y_Shift", Front_Beam_Region2_Y_Shift).toDouble();
        Front_Beam_Region2_Z_Shift = qSetting->value("SeamPosition/Front_Beam_Region2_Z_Shift", Front_Beam_Region2_Z_Shift).toDouble();
        Front_R_Beam_H_X_Shift = qSetting->value("SeamPosition/Front_R_Beam_H_X_Shift", Front_R_Beam_H_X_Shift).toDouble();
        Front_R_Beam_H_Y_Shift = qSetting->value("SeamPosition/Front_R_Beam_H_Y_Shift", Front_R_Beam_H_Y_Shift).toDouble();
        Front_R_Beam_H_Z_Shift = qSetting->value("SeamPosition/Front_R_Beam_H_Z_Shift", Front_R_Beam_H_Z_Shift).toDouble();
        Front_R_Beam_V_X_Shift = qSetting->value("SeamPosition/Front_R_Beam_V_X_Shift", Front_R_Beam_V_X_Shift).toDouble();
        Front_R_Beam_V_Y_Shift = qSetting->value("SeamPosition/Front_R_Beam_V_Y_Shift", Front_R_Beam_V_Y_Shift).toDouble();
        Front_R_Beam_V_Z_Shift = qSetting->value("SeamPosition/Front_R_Beam_V_Z_Shift", Front_R_Beam_V_Z_Shift).toDouble();
        Front_R_Beam_DH_X_Shift = qSetting->value("SeamPosition/Front_R_Beam_DH_X_Shift", Front_R_Beam_DH_X_Shift).toDouble();
        Front_R_Beam_DH_Y_Shift = qSetting->value("SeamPosition/Front_R_Beam_DH_Y_Shift", Front_R_Beam_DH_Y_Shift).toDouble();
        Front_R_Beam_DH_Z_Shift = qSetting->value("SeamPosition/Front_R_Beam_DH_Z_Shift", Front_R_Beam_DH_Z_Shift).toDouble();
        Front_R_Beam_DV_X_Shift = qSetting->value("SeamPosition/Front_R_Beam_DV_X_Shift", Front_R_Beam_DV_X_Shift).toDouble();
        Front_R_Beam_DV_Y_Shift = qSetting->value("SeamPosition/Front_R_Beam_DV_Y_Shift", Front_R_Beam_DV_Y_Shift).toDouble();
        Front_R_Beam_DV_Z_Shift = qSetting->value("SeamPosition/Front_R_Beam_DV_Z_Shift", Front_R_Beam_DV_Z_Shift).toDouble();

        // 工件反面横梁偏移
        Back_Beam_Region1_X_Shift = qSetting->value("SeamPosition/Back_Beam_Region1_X_Shift", Back_Beam_Region1_X_Shift).toDouble();
        Back_Beam_Region1_Y_Shift = qSetting->value("SeamPosition/Back_Beam_Region1_Y_Shift", Back_Beam_Region1_Y_Shift).toDouble();
        Back_Beam_Region1_Z_Shift = qSetting->value("SeamPosition/Back_Beam_Region1_Z_Shift", Back_Beam_Region1_Z_Shift).toDouble();
        Back_Beam_Region2_X_Shift = qSetting->value("SeamPosition/Back_Beam_Region2_X_Shift", Back_Beam_Region2_X_Shift).toDouble();
        Back_Beam_Region2_Y_Shift = qSetting->value("SeamPosition/Back_Beam_Region2_Y_Shift", Back_Beam_Region2_Y_Shift).toDouble();
        Back_Beam_Region2_Z_Shift = qSetting->value("SeamPosition/Back_Beam_Region2_Z_Shift", Back_Beam_Region2_Z_Shift).toDouble();

        // 北工作台 (机器人右侧)
        // 工件正面偏移
        Front_Region1_X_Shift_R = qSetting->value("SeamPosition/Front_Region1_X_Shift_R", Front_Region1_X_Shift_R).toDouble();
        Front_Region1_Y_Shift_R = qSetting->value("SeamPosition/Front_Region1_Y_Shift_R", Front_Region1_Y_Shift_R).toDouble();
        Front_Region1_Z_Shift_R = qSetting->value("SeamPosition/Front_Region1_Z_Shift_R", Front_Region1_Z_Shift_R).toDouble();
        Front_Region2_X_Shift_R = qSetting->value("SeamPosition/Front_Region2_X_Shift_R", Front_Region2_X_Shift_R).toDouble();
        Front_Region2_Y_Shift_R = qSetting->value("SeamPosition/Front_Region2_Y_Shift_R", Front_Region2_Y_Shift_R).toDouble();
        Front_Region2_Z_Shift_R = qSetting->value("SeamPosition/Front_Region2_Z_Shift_R", Front_Region2_Z_Shift_R).toDouble();
        // 工件反面偏移
        Back_Region1_X_Shift_R = qSetting->value("SeamPosition/Back_Region1_X_Shift_R", Back_Region1_X_Shift_R).toDouble();
        Back_Region1_Y_Shift_R = qSetting->value("SeamPosition/Back_Region1_Y_Shift_R", Back_Region1_Y_Shift_R).toDouble();
        Back_Region1_Z_Shift_R = qSetting->value("SeamPosition/Back_Region1_Z_Shift_R", Back_Region1_Z_Shift_R).toDouble();
        Back_Region2_X_Shift_R = qSetting->value("SeamPosition/Back_Region2_X_Shift_R", Back_Region2_X_Shift_R).toDouble();
        Back_Region2_Y_Shift_R = qSetting->value("SeamPosition/Back_Region2_Y_Shift_R", Back_Region2_Y_Shift_R).toDouble();
        Back_Region2_Z_Shift_R = qSetting->value("SeamPosition/Back_Region2_Z_Shift_R", Back_Region2_Z_Shift_R).toDouble();
        // 工件正面横梁偏移
        Front_Beam_Region1_X_Shift_R = qSetting->value("SeamPosition/Front_Beam_Region1_X_Shift_R", Front_Beam_Region1_X_Shift_R).toDouble();
        Front_Beam_Region1_Y_Shift_R = qSetting->value("SeamPosition/Front_Beam_Region1_Y_Shift_R", Front_Beam_Region1_Y_Shift_R).toDouble();
        Front_Beam_Region1_Z_Shift_R = qSetting->value("SeamPosition/Front_Beam_Region1_Z_Shift_R", Front_Beam_Region1_Z_Shift_R).toDouble();
        Front_Beam_Region2_X_Shift_R = qSetting->value("SeamPosition/Front_Beam_Region2_X_Shift_R", Front_Beam_Region2_X_Shift_R).toDouble();
        Front_Beam_Region2_Y_Shift_R = qSetting->value("SeamPosition/Front_Beam_Region2_Y_Shift_R", Front_Beam_Region2_Y_Shift_R).toDouble();
        Front_Beam_Region2_Z_Shift_R = qSetting->value("SeamPosition/Front_Beam_Region2_Z_Shift_R", Front_Beam_Region2_Z_Shift_R).toDouble();

        // 工件反面横梁偏移
        Back_Beam_Region1_X_Shift_R = qSetting->value("SeamPosition/Back_Beam_Region1_X_Shift_R", Back_Beam_Region1_X_Shift_R).toDouble();
        Back_Beam_Region1_Y_Shift_R = qSetting->value("SeamPosition/Back_Beam_Region1_Y_Shift_R", Back_Beam_Region1_Y_Shift_R).toDouble();
        Back_Beam_Region1_Z_Shift_R = qSetting->value("SeamPosition/Back_Beam_Region1_Z_Shift_R", Back_Beam_Region1_Z_Shift_R).toDouble();
        Back_Beam_Region2_X_Shift_R = qSetting->value("SeamPosition/Back_Beam_Region2_X_Shift_R", Back_Beam_Region2_X_Shift_R).toDouble();
        Back_Beam_Region2_Y_Shift_R = qSetting->value("SeamPosition/Back_Beam_Region2_Y_Shift_R", Back_Beam_Region2_Y_Shift_R).toDouble();
        Back_Beam_Region2_Z_Shift_R = qSetting->value("SeamPosition/Back_Beam_Region2_Z_Shift_R", Back_Beam_Region2_Z_Shift_R).toDouble();

        // 过渡速度
        Value_MoveSpeed = qSetting->value("Welding/Value_MoveSpeed", Value_MoveSpeed).toDouble();
        Value_MoveSpeed = std::min(Value_MoveSpeed, 300.0);  // 最大不能超过300
        Value_MoveSpeed = std::max(Value_MoveSpeed, 1.0);    // 最小不能小过1
        // 默认焊接速度
        Value_WeldingSpeed = qSetting->value("Welding/Value_WeldingSpeed", Value_WeldingSpeed).toDouble();
        Value_WeldingSpeed = std::min(Value_WeldingSpeed, 20.0);  // 最大不能超过20
        Value_WeldingSpeed = std::max(Value_WeldingSpeed, 1.0);   // 最小不能小过1

        // 不同焊缝焊接速度
        Value_WeldingSpeed0To1 =
            qSetting->value("Welding/Value_WeldingSpeed0To1", Value_WeldingSpeed).toDouble();  // 焊接速度(焊缝宽度1mm以下用这个速度)
        Value_WeldingSpeed0To1 = std::min(Value_WeldingSpeed0To1, 20.0);                       // 最大不能超过20m
        Value_WeldingSpeed0To1 = std::max(Value_WeldingSpeed0To1, 1.0);                        // 最小不能小过1
        Value_WeldingSpeed1To3 =
            qSetting->value("Welding/Value_WeldingSpeed1To3", Value_WeldingSpeed).toDouble();  // 焊接速度(焊缝宽度1mm到3mm用这个速度)
        Value_WeldingSpeed1To3 = std::min(Value_WeldingSpeed1To3, 20.0);                       // 最大不能超过20
        Value_WeldingSpeed1To3 = std::max(Value_WeldingSpeed1To3, 1.0);                        // 最小不能小过1
        Value_WeldingSpeed3To5 =
            qSetting->value("Welding/Value_WeldingSpeed3To5", Value_WeldingSpeed).toDouble();  // 焊接速度(焊缝宽度3mm到5mm用这个速度)
        Value_WeldingSpeed3To5 = std::min(Value_WeldingSpeed3To5, 20.0);                       // 最大不能超过20
        Value_WeldingSpeed3To5 = std::max(Value_WeldingSpeed3To5, 1.0);                        // 最小不能小过1
        Value_WeldingSpeedHorizontal =
            qSetting->value("Welding/Value_WeldingSpeedHorizontal", Value_WeldingSpeed).toDouble();  // 焊接速度(水平焊缝用这个速度)
        Value_WeldingSpeedHorizontal = std::min(Value_WeldingSpeedHorizontal, 20.0);                 // 最大不能超过20
        Value_WeldingSpeedHorizontal = std::max(Value_WeldingSpeedHorizontal, 1.0);                  // 最小不能小过1
        Value_WeldingSpeedVertical =
            qSetting->value("Welding/Value_WeldingSpeedVertical", Value_WeldingSpeed).toDouble();  // 焊接速度(竖直焊缝用这个速度)
        Value_WeldingSpeedVertical = std::min(Value_WeldingSpeedVertical, 20.0);                   // 最大不能超过20
        Value_WeldingSpeedVertical = std::max(Value_WeldingSpeedVertical, 1.0);                    // 最小不能小过1
        Value_WeldingCurrent = qSetting->value("Welding/Value_WeldingCurrent", Value_WeldingCurrent).toDouble();  // 焊接电流
        Value_WeldingCurrent = std::min(Value_WeldingCurrent, 300.0);                                             // 最大不能超过
        Value_WeldingCurrent = std::max(Value_WeldingCurrent, 1.0);                                               // 最小不能小过1
        Value_WeldingCurrent_Vertical =
            qSetting->value("Welding/Value_WeldingCurrent_Vertical", Value_WeldingCurrent_Vertical).toDouble();  // 焊接电流(竖直焊缝用这个电流)
        Value_WeldingCurrent_Vertical = std::min(Value_WeldingCurrent_Vertical, 300.0);                          // 最大不能超过
        Value_WeldingCurrent_Vertical = std::max(Value_WeldingCurrent_Vertical, 1.0);                            // 最小不能小过1
        Value_WeldingVoltage = qSetting->value("Welding/Value_WeldingVoltage", Value_WeldingVoltage).toDouble();  // 焊接电压
        Value_WeldingVoltage = std::min(Value_WeldingVoltage, 100.0);                                             // 最大不能超过
        Value_WeldingVoltage = std::max(Value_WeldingVoltage, 1.0);                                               // 最小不能小过1
        Value_WeldingVoltage_Vertical =
            qSetting->value("Welding/Value_WeldingVoltage_Vertical", Value_WeldingVoltage_Vertical).toDouble();  // 焊接电压(竖直焊缝用这个电压)
        Value_WeldingVoltage_Vertical = std::min(Value_WeldingVoltage_Vertical, 100.0);                          // 最大不能超过
        Value_WeldingVoltage_Vertical = std::max(Value_WeldingVoltage_Vertical, 1.0);                            // 最小不能小过1

        // 是否焊接竖直焊缝
        weldingVerticalWeld = qSetting->value("Welding/weldingVerticalWeld", weldingVerticalWeld).toBool();

        // 点云处理相关参数
        passthrough_Min = qSetting->value("pointCloudProcessing/passthrough_Min", passthrough_Min).toDouble();
        passthrough_Max = qSetting->value("pointCloudProcessing/passthrough_Max", passthrough_Max).toDouble();
        statistical_Pts = qSetting->value("pointCloudProcessing/statistical_Pts", statistical_Pts).toDouble();
        statistical_Std = qSetting->value("pointCloudProcessing/statistical_Std", statistical_Std).toDouble();

        // 深度学习相关
        DL_PredictThreshold = qSetting->value("deepLearning/DL_PredictThreshold", DL_PredictThreshold).toDouble();

        // 点云放缩相关
        scaleOfPointX = qSetting->value("Scale/scaleOfPointX", scaleOfPointX).toDouble();
        transOfPointX = qSetting->value("Scale/transOfPointX", transOfPointX).toDouble();
        scaleOfPointY = qSetting->value("Scale/scaleOfPointY", scaleOfPointY).toDouble();
        transOfPointY = qSetting->value("Scale/transOfPointY", transOfPointY).toDouble();

        PLOGD << "QSetting配置数据读取成功";
    } else {
        PLOGE << "QSetting类未初始化";
    }
}
