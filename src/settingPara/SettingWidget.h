#ifndef SETTINGWIDGET_H
#define SETTINGWIDGET_H

#include <QWidget>
#include <memory>

#include "cameraFactory/AbstractCamera.h"

namespace Ui {
class SettingWidget;
}

class SettingPara;
class WeldingMainWindow;
class StructLightCamera;

class SettingWidget : public QWidget {
    Q_OBJECT

public:
    explicit SettingWidget(QWidget *parent = nullptr);
    ~SettingWidget();

    void initSetting();   // 初始化配置页面
    void renewSetting();  // 更新显示参数

signals:
    void sendCameraExposure(int exposure);                               // 发出曝光时间
    void sendCoarseCameraExposure(int exposure);                         // 发出曝光时间
    void sendCameraGain(int gain);                                       // 发出增益
    void sendCameraWorkMode(CAMERA_WORK_MODE workMode);                  // 发出相机工作模式
    void sendProjectorBrightness(const char *nameNode, int brightness);  // 发出投影仪亮度
    void sendProjectorFps(const char *nameNode, int fps);                // 发出投影仪帧率
    void sendProjectorNum(const char *nameNode, int num);                // 发出投影仪图片数量
    void sendProjectorLedOn();                                           // 打开投影仪LED
    void sendProjectorLedOff();                                          // 关闭投影仪LED

private slots:
    void on_spinBoxExp_editingFinished();
    void on_spinBoxGain_editingFinished();
    // void on_btnHardwareTrigger_clicked();
    // void on_btnSoftwareTrigger_clicked();
    void on_spinBoxProjBrightness_editingFinished();
    void on_spinBoxProjFps_editingFinished();
    // void on_spinBoxProjNum_editingFinished();
    void on_LED_ON_clicked();
    void on_LED_OFF_clicked();

    // void on_spinBox_Modulation_editingFinished();
    void on_comboBox_SaveModel_activated(int index);
    // void on_lineEdit_Dth_IntersectionToStartPoint_editingFinished();
    // void on_lineEdit_passthrough_Min_editingFinished();
    // void on_lineEdit_passthrough_Max_editingFinished();
    // void on_lineEdit_StatisticalFilter_Pts_editingFinished();
    // void on_lineEdit_StatisticalFilter_Std_editingFinished();

    void on_lineEdit_Value_MoveSpeed_editingFinished();
    void on_lineEdit_Value_WeldingSpeed_editingFinished();
    void on_lineEdit_Value_WeldingSpeed_0To1_editingFinished();
    void on_lineEdit_Value_WeldingSpeed_1To3_editingFinished();
    void on_lineEdit_Value_WeldingSpeed_3To5_editingFinished();
    void on_lineEdit_Value_WeldingSpeed_Horizontal_editingFinished();

    void on_comboBoxWeldingVerticalWeld_activated(int index);

    void on_lineEditScaleOfPointX_editingFinished();
    void on_lineEditTransOfPointX_editingFinished();
    void on_lineEditScaleOfPointY_editingFinished();
    void on_lineEditTransOfPointY_editingFinished();

    void on_lineEdit_FrontLeft_ExtendStart_editingFinished();
    void on_lineEdit_FrontLeft_ExtendEnd_editingFinished();
    void on_lineEdit_FrontRight_ExtendStart_editingFinished();
    void on_lineEdit_FrontRight_ExtendEnd_editingFinished();
    void on_lineEdit_FrontBeamLeft_ExtendStart_editingFinished();
    void on_lineEdit_FrontBeamLeft_ExtendEnd_editingFinished();
    void on_lineEdit_FrontBeamRight_ExtendStart_editingFinished();
    void on_lineEdit_FrontBeamRight_ExtendEnd_editingFinished();
    void on_lineEdit_FrontHBeamLeft_ExtendStart_editingFinished();
    void on_lineEdit_FrontHBeamLeft_ExtendEnd_editingFinished();
    void on_lineEdit_FrontHBeamRight_ExtendStart_editingFinished();
    void on_lineEdit_FrontHBeamRight_ExtendEnd_editingFinished();

    void on_lineEdit_FrontBeamLeft_WithDrawDistance_editingFinished();
    void on_lineEdit_FrontBeamRight_WithDrawDistance_editingFinished();

    void on_lineEdit_BackLeft_ExtendStart_editingFinished();
    void on_lineEdit_BackLeft_ExtendEnd_editingFinished();
    void on_lineEdit_BackRight_ExtendStart_editingFinished();
    void on_lineEdit_BackRight_ExtendEnd_editingFinished();
    void on_lineEdit_BackBeamLeft_ExtendStart_editingFinished();
    void on_lineEdit_BackBeamLeft_ExtendEnd_editingFinished();
    void on_lineEdit_BackBeamRight_ExtendStart_editingFinished();
    void on_lineEdit_BackBeamRight_ExtendEnd_editingFinished();

    // 南工作台 (机器人左侧)
    void on_lineEdit_Front_Region1_X_Shift_editingFinished();  //
    void on_lineEdit_Front_Region1_Y_Shift_editingFinished();
    void on_lineEdit_Front_Region1_Z_Shift_editingFinished();
    void on_lineEdit_Front_Region2_X_Shift_editingFinished();
    void on_lineEdit_Front_Region2_Y_Shift_editingFinished();
    void on_lineEdit_Front_Region2_Z_Shift_editingFinished();
    void on_lineEdit_Back_Region1_X_Shift_editingFinished();  //
    void on_lineEdit_Back_Region1_Y_Shift_editingFinished();
    void on_lineEdit_Back_Region1_Z_Shift_editingFinished();
    void on_lineEdit_Back_Region2_X_Shift_editingFinished();
    void on_lineEdit_Back_Region2_Y_Shift_editingFinished();
    void on_lineEdit_Back_Region2_Z_Shift_editingFinished();
    void on_lineEdit_Front_Beam_Region1_X_Shift_editingFinished();  //
    void on_lineEdit_Front_Beam_Region1_Y_Shift_editingFinished();
    void on_lineEdit_Front_Beam_Region1_Z_Shift_editingFinished();
    void on_lineEdit_Front_Beam_Region2_X_Shift_editingFinished();
    void on_lineEdit_Front_Beam_Region2_Y_Shift_editingFinished();
    void on_lineEdit_Front_Beam_Region2_Z_Shift_editingFinished();
    void on_lineEdit_Back_Beam_Region1_X_Shift_editingFinished();  //
    void on_lineEdit_Back_Beam_Region1_Y_Shift_editingFinished();
    void on_lineEdit_Back_Beam_Region1_Z_Shift_editingFinished();
    void on_lineEdit_Back_Beam_Region2_X_Shift_editingFinished();
    void on_lineEdit_Back_Beam_Region2_Y_Shift_editingFinished();
    void on_lineEdit_Back_Beam_Region2_Z_Shift_editingFinished();

    // 北工作台 (机器人右侧)
    void on_lineEdit_Front_Region1_X_Shift_R_editingFinished();  //
    void on_lineEdit_Front_Region1_Y_Shift_R_editingFinished();
    void on_lineEdit_Front_Region1_Z_Shift_R_editingFinished();
    void on_lineEdit_Front_Region2_X_Shift_R_editingFinished();
    void on_lineEdit_Front_Region2_Y_Shift_R_editingFinished();
    void on_lineEdit_Front_Region2_Z_Shift_R_editingFinished();
    void on_lineEdit_Back_Region1_X_Shift_R_editingFinished();  //
    void on_lineEdit_Back_Region1_Y_Shift_R_editingFinished();
    void on_lineEdit_Back_Region1_Z_Shift_R_editingFinished();
    void on_lineEdit_Back_Region2_X_Shift_R_editingFinished();
    void on_lineEdit_Back_Region2_Y_Shift_R_editingFinished();
    void on_lineEdit_Back_Region2_Z_Shift_R_editingFinished();
    void on_lineEdit_Front_Beam_Region1_X_Shift_R_editingFinished();  //
    void on_lineEdit_Front_Beam_Region1_Y_Shift_R_editingFinished();
    void on_lineEdit_Front_Beam_Region1_Z_Shift_R_editingFinished();
    void on_lineEdit_Front_Beam_Region2_X_Shift_R_editingFinished();
    void on_lineEdit_Front_Beam_Region2_Y_Shift_R_editingFinished();
    void on_lineEdit_Front_Beam_Region2_Z_Shift_R_editingFinished();
    void on_lineEdit_Back_Beam_Region1_X_Shift_R_editingFinished();  //
    void on_lineEdit_Back_Beam_Region1_Y_Shift_R_editingFinished();
    void on_lineEdit_Back_Beam_Region1_Z_Shift_R_editingFinished();
    void on_lineEdit_Back_Beam_Region2_X_Shift_R_editingFinished();
    void on_lineEdit_Back_Beam_Region2_Y_Shift_R_editingFinished();
    void on_lineEdit_Back_Beam_Region2_Z_Shift_R_editingFinished();
    void on_coarseSpinBoxExp_editingFinished();
    void on_lineEdit_Front_L_Beam_H_X_Shift_editingFinished();
    void on_lineEdit_Front_L_Beam_H_Y_Shift_editingFinished();
    void on_lineEdit_Front_L_Beam_H_Z_Shift_editingFinished();
    void on_lineEdit_Front_L_Beam_V_X_Shift_editingFinished();
    void on_lineEdit_Front_L_Beam_V_Y_Shift_editingFinished();
    void on_lineEdit_Front_L_Beam_V_Z_Shift_editingFinished();
    void on_lineEdit_Front_L_Beam_DH_X_Shift_editingFinished();
    void on_lineEdit_Front_L_Beam_DH_Y_Shift_editingFinished();
    void on_lineEdit_Front_L_Beam_DH_Z_Shift_editingFinished();
    void on_lineEdit_Front_L_Beam_DV_X_Shift_editingFinished();
    void on_lineEdit_Front_L_Beam_DV_Y_Shift_editingFinished();
    void on_lineEdit_Front_L_Beam_DV_Z_Shift_editingFinished();
    void on_lineEdit_Front_R_Beam_H_X_Shift_editingFinished();
    void on_lineEdit_Front_R_Beam_H_Y_Shift_editingFinished();
    void on_lineEdit_Front_R_Beam_H_Z_Shift_editingFinished();
    void on_lineEdit_Front_R_Beam_V_X_Shift_editingFinished();
    void on_lineEdit_Front_R_Beam_V_Y_Shift_editingFinished();
    void on_lineEdit_Front_R_Beam_V_Z_Shift_editingFinished();
    void on_lineEdit_Front_R_Beam_DH_X_Shift_editingFinished();
    void on_lineEdit_Front_R_Beam_DH_Y_Shift_editingFinished();
    void on_lineEdit_Front_R_Beam_DH_Z_Shift_editingFinished();
    void on_lineEdit_Front_R_Beam_DV_X_Shift_editingFinished();
    void on_lineEdit_Front_R_Beam_DV_Y_Shift_editingFinished();
    void on_lineEdit_Front_R_Beam_DV_Z_Shift_editingFinished();
    void on_lineEdit_FrontVBeamLeft_ExtendStart_editingFinished();
    void on_lineEdit_FrontVBeamLeft_ExtendEnd_editingFinished();
    void on_lineEdit_FrontVBeamRight_ExtendStart_editingFinished();
    void on_lineEdit_FrontVBeamRight_ExtendEnd_editingFinished();
    void on_lineEdit_Value_WeldingSpeed_Vertical_editingFinished();
    void on_lineEdit_Value_WeldingCurrent_editingFinished();
    void on_lineEdit_Value_WeldingVoltage_editingFinished();
    void on_lineEdit_Value_WeldingCurrent_Vertical_editingFinished();
    void on_lineEdit_Value_WeldingVoltage_Vertical_editingFinished();

    void on_lineEdit_TSPFHStart_X_Shift_editingFinished();

    void on_lineEdit_TSPFHStart_Y_Shift_editingFinished();

    void on_lineEdit_TSPFHStart_Z_Shift_editingFinished();

    void on_lineEdit_TSPFHEnd_X_Shift_editingFinished();

    void on_lineEdit_TSPFHEnd_Y_Shift_editingFinished();

    void on_lineEdit_TSPFHEnd_Z_Shift_editingFinished();

    void on_lineEdit_TSPFHWeld_WithdrawDistance_editingFinished();

    void on_lineEdit_PPFHStart_X_Shift_editingFinished();

    void on_lineEdit_PPFHStart_Y_Shift_editingFinished();

    void on_lineEdit_PPFHStart_Z_Shift_editingFinished();

    void on_lineEdit_PPFHEnd_X_Shift_editingFinished();

    void on_lineEdit_PPFHEnd_Y_Shift_editingFinished();

    void on_lineEdit_PPFHEnd_Z_Shift_editingFinished();

    void on_lineEdit_PPFHWeld_WithdrawDistance_editingFinished();

    void on_lineEdit_PPFVStart_X_Shift_editingFinished();

    void on_lineEdit_PPFVStart_Y_Shift_editingFinished();

    void on_lineEdit_PPFVStart_Z_Shift_editingFinished();

    void on_lineEdit_PPFVEnd_X_Shift_editingFinished();

    void on_lineEdit_PPFVEnd_Y_Shift_editingFinished();

    void on_lineEdit_PPFVEnd_Z_Shift_editingFinished();

    void on_lineEdit_PPFVWeld_WithdrawDistance_editingFinished();

private:
    Ui::SettingWidget *ui;

    SettingPara *settingPara = nullptr;                             // 参数配置类
    std::shared_ptr<StructLightCamera> structLightCamera{nullptr};  // 结构光相机

    friend class WeldingMainWindow;  // 友元类
};

#endif  // SETTINGWIDGET_H
