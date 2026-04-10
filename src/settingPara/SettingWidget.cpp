#include "SettingWidget.h"

#include "projectFactory/AbstractProjector.h"
#include "settingPara/SettingPara.h"
#include "structLightCamera/StructLightCamera.h"
#include "ui_SettingWidget.h"
// #include "workpieceCoarseLocalization/WorkpieceCoarseLocalization.h"

SettingWidget::SettingWidget(QWidget *parent) : QWidget(parent), ui(new Ui::SettingWidget), settingPara(&SettingPara::getInstance()) {
    ui->setupUi(this);

    // // 相机触发开关和模式
    // ui->comboBox_mode->addItem("Off");  // 触发开关
    // ui->comboBox_mode->addItem("On");
    // ui->comboBox_mode->setCurrentIndex(1);  // 默认触发开关打开

    // // 采集用途选择(重建/标定)
    // ui->comboBox_capture_purpose->addItem("Reconstruction");
    // ui->comboBox_capture_purpose->addItem("Calibration");
    // ui->comboBox_capture_purpose->setCurrentIndex(0);  // 默认选择重建

    // 是否保存重建的模型
    ui->comboBox_SaveModel->addItem(QStringLiteral("不保存"));
    ui->comboBox_SaveModel->addItem(QStringLiteral("保存"));
    ui->comboBox_SaveModel->setCurrentIndex(0);  // 默认选择

    // 是否保存重建的模型
    ui->comboBoxWeldingVerticalWeld->addItem(QStringLiteral("否"));
    ui->comboBoxWeldingVerticalWeld->addItem(QStringLiteral("是"));
    ui->comboBoxWeldingVerticalWeld->setCurrentIndex(0);  // 默认选择

    ui->tabWidget->tabBar()->setExpanding(true);

    renewSetting();  // 更新显示参数
}

SettingWidget::~SettingWidget() { delete ui; }

// 初始化配置页面
void SettingWidget::initSetting() {
    // 由于相机线程一直在采图, 参数设置功能在结构光线程进行
    connect(this, &SettingWidget::sendCameraExposure, structLightCamera.get(), &StructLightCamera::whenGetCameraExposure);
    connect(this, &SettingWidget::sendCameraGain, structLightCamera.get(), &StructLightCamera::whenGetCameraGain);
    connect(this, &SettingWidget::sendCameraWorkMode, structLightCamera.get(), &StructLightCamera::whenGetCameraWorkMode);
    // 投影仪参数设置在投影仪线程进行
    connect(this, &SettingWidget::sendProjectorBrightness, structLightCamera->projector.get(), &AbstractProjector::setPara);
    connect(this, &SettingWidget::sendProjectorFps, structLightCamera->projector.get(), &AbstractProjector::setPara);
    connect(this, &SettingWidget::sendProjectorNum, structLightCamera->projector.get(), &AbstractProjector::setPara);
    connect(this, &SettingWidget::sendProjectorLedOn, structLightCamera->projector.get(), &AbstractProjector::openLed);
    connect(this, &SettingWidget::sendProjectorLedOff, structLightCamera->projector.get(), &AbstractProjector::closeLed);
}

// 更新显示参数
void SettingWidget::renewSetting() {
    // 相机曝光、增益参数
    ui->spinBoxExp->setValue(settingPara->camera_exposure);
    ui->spinBoxGain->setValue(settingPara->camera_gain);
    ui->coarseSpinBoxExp->setValue(settingPara->coarse_camera_exposure);

    // // 调制度
    // ui->spinBox_Modulation->setValue(settingPara->modulation_threshold);

    // // 直通滤波和统计滤波参数
    // ui->lineEdit_passthrough_Min->setText(QString::number(settingPara->passthrough_Min));
    // ui->lineEdit_passthrough_Max->setText(QString::number(settingPara->passthrough_Max));
    // ui->lineEdit_StatisticalFilter_Pts->setText(QString::number(settingPara->statistical_Pts));
    // ui->lineEdit_StatisticalFilter_Std->setText(QString::number(settingPara->statistical_Std));
    // 龙门支架
    ui->lineEdit_TSPFHStart_X_Shift->setText(QString::number(settingPara->TubeSidePlatFilletStart_X));
    ui->lineEdit_TSPFHStart_Y_Shift->setText(QString::number(settingPara->TubeSidePlatFilletStart_Y));
    ui->lineEdit_TSPFHStart_Z_Shift->setText(QString::number(settingPara->TubeSidePlatFilletStart_Z));
    ui->lineEdit_TSPFHEnd_X_Shift->setText(QString::number(settingPara->TubeSidePlatFilletEnd_X));
    ui->lineEdit_TSPFHEnd_Y_Shift->setText(QString::number(settingPara->TubeSidePlatFilletEnd_Y));
    ui->lineEdit_TSPFHEnd_Z_Shift->setText(QString::number(settingPara->TubeSidePlatFilletEnd_Z));
    ui->lineEdit_TSPFHWeld_WithdrawDistance->setText(QString::number(settingPara->TubeSidePlatFilletWithdrawDistance));
    ui->lineEdit_PPFHStart_X_Shift->setText(QString::number(settingPara->PlatePlateFilletHorizontalStart_X));
    ui->lineEdit_PPFHStart_Y_Shift->setText(QString::number(settingPara->PlatePlateFilletHorizontalStart_Y));
    ui->lineEdit_PPFHStart_Z_Shift->setText(QString::number(settingPara->PlatePlateFilletHorizontalStart_Z));
    ui->lineEdit_PPFHEnd_X_Shift->setText(QString::number(settingPara->PlatePlateFilletHorizontalEnd_X));
    ui->lineEdit_PPFHEnd_Y_Shift->setText(QString::number(settingPara->PlatePlateFilletHorizontalEnd_Y));
    ui->lineEdit_PPFHEnd_Z_Shift->setText(QString::number(settingPara->PlatePlateFilletHorizontalEnd_Z));
    ui->lineEdit_PPFHWeld_WithdrawDistance->setText(QString::number(settingPara->PlatePlateFilletHorizontalWithdrawDistance));
    ui->lineEdit_PPFVStart_X_Shift->setText(QString::number(settingPara->PlatePlateFilletVerticalStart_X));
    ui->lineEdit_PPFVStart_Y_Shift->setText(QString::number(settingPara->PlatePlateFilletVerticalStart_Y));
    ui->lineEdit_PPFVStart_Z_Shift->setText(QString::number(settingPara->PlatePlateFilletVerticalStart_Z));
    ui->lineEdit_PPFVEnd_X_Shift->setText(QString::number(settingPara->PlatePlateFilletVerticalEnd_X));
    ui->lineEdit_PPFVEnd_Y_Shift->setText(QString::number(settingPara->PlatePlateFilletVerticalEnd_Y));
    ui->lineEdit_PPFVEnd_Z_Shift->setText(QString::number(settingPara->PlatePlateFilletVerticalEnd_Z));
    ui->lineEdit_PPFVWeld_WithdrawDistance->setText(QString::number(settingPara->PlatePlateFilletVerticalWithdrawDistance));
    ui->lineEdit_TPFStartOffset->setText(QString::number(settingPara->TubePlatFilletStartOffset));
    ui->lineEdit_TPFEndOffset->setText(QString::number(settingPara->TubePlatFilletEndOffset));
    ui->lineEdit_TPFWeld_WithdrawDistance->setText(QString::number(settingPara->TubePlatFilletWithdrawDistance));

    ui->lineEdit_WireCalibrationOffset->setText(QString::number(settingPara->wireCalibrationOffset));
    ui->lineEdit_ToolRadius->setText(QString::number(settingPara->toolRadius));

    // 工件正面, 焊缝延长
    ui->lineEdit_FrontLeft_ExtendStart->setText(QString::number(settingPara->FrontLeft_ExtendStart));
    ui->lineEdit_FrontLeft_ExtendEnd->setText(QString::number(settingPara->FrontLeft_ExtendEnd));
    ui->lineEdit_FrontRight_ExtendStart->setText(QString::number(settingPara->FrontRight_ExtendStart));
    ui->lineEdit_FrontRight_ExtendEnd->setText(QString::number(settingPara->FrontRight_ExtendEnd));

    ui->lineEdit_FrontBeamLeft_ExtendStart->setText(QString::number(settingPara->FrontBeamLeft_ExtendStart));
    ui->lineEdit_FrontBeamLeft_ExtendEnd->setText(QString::number(settingPara->FrontBeamLeft_ExtendEnd));
    ui->lineEdit_FrontBeamRight_ExtendStart->setText(QString::number(settingPara->FrontBeamRight_ExtendStart));
    ui->lineEdit_FrontBeamRight_ExtendEnd->setText(QString::number(settingPara->FrontBeamRight_ExtendEnd));

    ui->lineEdit_FrontHBeamLeft_ExtendStart->setText(QString::number(settingPara->FrontHBeamLeft_ExtendStart));
    ui->lineEdit_FrontHBeamLeft_ExtendEnd->setText(QString::number(settingPara->FrontHBeamLeft_ExtendEnd));
    ui->lineEdit_FrontHBeamRight_ExtendStart->setText(QString::number(settingPara->FrontHBeamRight_ExtendStart));
    ui->lineEdit_FrontHBeamRight_ExtendEnd->setText(QString::number(settingPara->FrontHBeamRight_ExtendEnd));

    ui->lineEdit_FrontVBeamLeft_ExtendStart->setText(QString::number(settingPara->FrontVBeamLeft_ExtendStart));
    ui->lineEdit_FrontVBeamLeft_ExtendEnd->setText(QString::number(settingPara->FrontVBeamLeft_ExtendEnd));
    ui->lineEdit_FrontVBeamRight_ExtendStart->setText(QString::number(settingPara->FrontVBeamRight_ExtendStart));
    ui->lineEdit_FrontVBeamRight_ExtendEnd->setText(QString::number(settingPara->FrontVBeamRight_ExtendEnd));
    // 焊枪后撤
    ui->lineEdit_FrontBeamLeft_WithDrawDistance->setText(QString::number(settingPara->FrontBeamLeft_WithDrawDistance));
    ui->lineEdit_FrontBeamRight_WithDrawDistance->setText(QString::number(settingPara->FrontBeamRight_WithDrawDistance));

    // 工件反面, 焊缝延长
    ui->lineEdit_BackLeft_ExtendStart->setText(QString::number(settingPara->BackLeft_ExtendStart));
    ui->lineEdit_BackLeft_ExtendEnd->setText(QString::number(settingPara->BackLeft_ExtendEnd));
    ui->lineEdit_BackRight_ExtendStart->setText(QString::number(settingPara->BackRight_ExtendStart));
    ui->lineEdit_BackRight_ExtendEnd->setText(QString::number(settingPara->BackRight_ExtendEnd));

    ui->lineEdit_BackBeamLeft_ExtendStart->setText(QString::number(settingPara->BackBeamLeft_ExtendStart));
    ui->lineEdit_BackBeamLeft_ExtendEnd->setText(QString::number(settingPara->BackBeamLeft_ExtendEnd));
    ui->lineEdit_BackBeamRight_ExtendStart->setText(QString::number(settingPara->BackBeamRight_ExtendStart));
    ui->lineEdit_BackBeamRight_ExtendEnd->setText(QString::number(settingPara->BackBeamRight_ExtendEnd));

    // // 边界端点到交点的距离阈值, 用于判断焊缝拍摄是否完整
    // ui->lineEdit_Dth_IntersectionToStartPoint->setText(QString::number(settingPara->MinDth_IntersectionToStartPoint));

    // 南工作台 (机器人左侧)
    // 工件正面边角----左右的XYZ方向偏移
    ui->lineEdit_Front_Region1_X_Shift->setText(QString::number(settingPara->Front_Region1_X_Shift));
    ui->lineEdit_Front_Region1_Y_Shift->setText(QString::number(settingPara->Front_Region1_Y_Shift));
    ui->lineEdit_Front_Region1_Z_Shift->setText(QString::number(settingPara->Front_Region1_Z_Shift));
    ui->lineEdit_Front_Region2_X_Shift->setText(QString::number(settingPara->Front_Region2_X_Shift));
    ui->lineEdit_Front_Region2_Y_Shift->setText(QString::number(settingPara->Front_Region2_Y_Shift));
    ui->lineEdit_Front_Region2_Z_Shift->setText(QString::number(settingPara->Front_Region2_Z_Shift));
    // 工件反面边角----左右的XYZ方向偏移
    ui->lineEdit_Back_Region1_X_Shift->setText(QString::number(settingPara->Back_Region1_X_Shift));
    ui->lineEdit_Back_Region1_Y_Shift->setText(QString::number(settingPara->Back_Region1_Y_Shift));
    ui->lineEdit_Back_Region1_Z_Shift->setText(QString::number(settingPara->Back_Region1_Z_Shift));
    ui->lineEdit_Back_Region2_X_Shift->setText(QString::number(settingPara->Back_Region2_X_Shift));
    ui->lineEdit_Back_Region2_Y_Shift->setText(QString::number(settingPara->Back_Region2_Y_Shift));
    ui->lineEdit_Back_Region2_Z_Shift->setText(QString::number(settingPara->Back_Region2_Z_Shift));
    // 工件正面横梁----左右的XYZ方向偏移
    ui->lineEdit_Front_Beam_Region1_X_Shift->setText(QString::number(settingPara->Front_Beam_Region1_X_Shift));
    ui->lineEdit_Front_Beam_Region1_Y_Shift->setText(QString::number(settingPara->Front_Beam_Region1_Y_Shift));
    ui->lineEdit_Front_Beam_Region1_Z_Shift->setText(QString::number(settingPara->Front_Beam_Region1_Z_Shift));
    ui->lineEdit_Front_L_Beam_H_X_Shift->setText(QString::number(settingPara->Front_L_Beam_H_X_Shift));
    ui->lineEdit_Front_L_Beam_H_Y_Shift->setText(QString::number(settingPara->Front_L_Beam_H_Y_Shift));
    ui->lineEdit_Front_L_Beam_H_Z_Shift->setText(QString::number(settingPara->Front_L_Beam_H_Z_Shift));
    ui->lineEdit_Front_L_Beam_V_X_Shift->setText(QString::number(settingPara->Front_L_Beam_V_X_Shift));
    ui->lineEdit_Front_L_Beam_V_Y_Shift->setText(QString::number(settingPara->Front_L_Beam_V_Y_Shift));
    ui->lineEdit_Front_L_Beam_V_Z_Shift->setText(QString::number(settingPara->Front_L_Beam_V_Z_Shift));
    ui->lineEdit_Front_L_Beam_DH_X_Shift->setText(QString::number(settingPara->Front_L_Beam_DH_X_Shift));
    ui->lineEdit_Front_L_Beam_DH_Y_Shift->setText(QString::number(settingPara->Front_L_Beam_DH_Y_Shift));
    ui->lineEdit_Front_L_Beam_DH_Z_Shift->setText(QString::number(settingPara->Front_L_Beam_DH_Z_Shift));
    ui->lineEdit_Front_L_Beam_DV_X_Shift->setText(QString::number(settingPara->Front_L_Beam_DV_X_Shift));
    ui->lineEdit_Front_L_Beam_DV_Y_Shift->setText(QString::number(settingPara->Front_L_Beam_DV_Y_Shift));
    ui->lineEdit_Front_L_Beam_DV_Z_Shift->setText(QString::number(settingPara->Front_L_Beam_DV_Z_Shift));

    ui->lineEdit_Front_Beam_Region2_X_Shift->setText(QString::number(settingPara->Front_Beam_Region2_X_Shift));
    ui->lineEdit_Front_Beam_Region2_Y_Shift->setText(QString::number(settingPara->Front_Beam_Region2_Y_Shift));
    ui->lineEdit_Front_Beam_Region2_Z_Shift->setText(QString::number(settingPara->Front_Beam_Region2_Z_Shift));
    ui->lineEdit_Front_R_Beam_H_X_Shift->setText(QString::number(settingPara->Front_R_Beam_H_X_Shift));
    ui->lineEdit_Front_R_Beam_H_Y_Shift->setText(QString::number(settingPara->Front_R_Beam_H_Y_Shift));
    ui->lineEdit_Front_R_Beam_H_Z_Shift->setText(QString::number(settingPara->Front_R_Beam_H_Z_Shift));
    ui->lineEdit_Front_R_Beam_V_X_Shift->setText(QString::number(settingPara->Front_R_Beam_V_X_Shift));
    ui->lineEdit_Front_R_Beam_V_Y_Shift->setText(QString::number(settingPara->Front_R_Beam_V_Y_Shift));
    ui->lineEdit_Front_R_Beam_V_Z_Shift->setText(QString::number(settingPara->Front_R_Beam_V_Z_Shift));
    ui->lineEdit_Front_R_Beam_DH_X_Shift->setText(QString::number(settingPara->Front_R_Beam_DH_X_Shift));
    ui->lineEdit_Front_R_Beam_DH_Y_Shift->setText(QString::number(settingPara->Front_R_Beam_DH_Y_Shift));
    ui->lineEdit_Front_R_Beam_DH_Z_Shift->setText(QString::number(settingPara->Front_R_Beam_DH_Z_Shift));
    ui->lineEdit_Front_R_Beam_DV_X_Shift->setText(QString::number(settingPara->Front_R_Beam_DV_X_Shift));
    ui->lineEdit_Front_R_Beam_DV_Y_Shift->setText(QString::number(settingPara->Front_R_Beam_DV_Y_Shift));
    ui->lineEdit_Front_R_Beam_DV_Z_Shift->setText(QString::number(settingPara->Front_R_Beam_DV_Z_Shift));
    // 工件反面横梁----左右的XYZ方向偏移
    ui->lineEdit_Back_Beam_Region1_X_Shift->setText(QString::number(settingPara->Back_Beam_Region1_X_Shift));
    ui->lineEdit_Back_Beam_Region1_Y_Shift->setText(QString::number(settingPara->Back_Beam_Region1_Y_Shift));
    ui->lineEdit_Back_Beam_Region1_Z_Shift->setText(QString::number(settingPara->Back_Beam_Region1_Z_Shift));
    ui->lineEdit_Back_Beam_Region2_X_Shift->setText(QString::number(settingPara->Back_Beam_Region2_X_Shift));
    ui->lineEdit_Back_Beam_Region2_Y_Shift->setText(QString::number(settingPara->Back_Beam_Region2_Y_Shift));
    ui->lineEdit_Back_Beam_Region2_Z_Shift->setText(QString::number(settingPara->Back_Beam_Region2_Z_Shift));

    // 北工作台 (机器人右侧)
    // 工件正面边角----左右的XYZ方向偏移
    ui->lineEdit_Front_Region1_X_Shift_R->setText(QString::number(settingPara->Front_Region1_X_Shift_R));
    ui->lineEdit_Front_Region1_Y_Shift_R->setText(QString::number(settingPara->Front_Region1_Y_Shift_R));
    ui->lineEdit_Front_Region1_Z_Shift_R->setText(QString::number(settingPara->Front_Region1_Z_Shift_R));
    ui->lineEdit_Front_Region2_X_Shift_R->setText(QString::number(settingPara->Front_Region2_X_Shift_R));
    ui->lineEdit_Front_Region2_Y_Shift_R->setText(QString::number(settingPara->Front_Region2_Y_Shift_R));
    ui->lineEdit_Front_Region2_Z_Shift_R->setText(QString::number(settingPara->Front_Region2_Z_Shift_R));
    // 工件反面边角----左右的XYZ方向偏移
    ui->lineEdit_Back_Region1_X_Shift_R->setText(QString::number(settingPara->Back_Region1_X_Shift_R));
    ui->lineEdit_Back_Region1_Y_Shift_R->setText(QString::number(settingPara->Back_Region1_Y_Shift_R));
    ui->lineEdit_Back_Region1_Z_Shift_R->setText(QString::number(settingPara->Back_Region1_Z_Shift_R));
    ui->lineEdit_Back_Region2_X_Shift_R->setText(QString::number(settingPara->Back_Region2_X_Shift_R));
    ui->lineEdit_Back_Region2_Y_Shift_R->setText(QString::number(settingPara->Back_Region2_Y_Shift_R));
    ui->lineEdit_Back_Region2_Z_Shift_R->setText(QString::number(settingPara->Back_Region2_Z_Shift_R));
    // 工件正面横梁----左右的XYZ方向偏移
    ui->lineEdit_Front_Beam_Region1_X_Shift_R->setText(QString::number(settingPara->Front_Beam_Region1_X_Shift_R));
    ui->lineEdit_Front_Beam_Region1_Y_Shift_R->setText(QString::number(settingPara->Front_Beam_Region1_Y_Shift_R));
    ui->lineEdit_Front_Beam_Region1_Z_Shift_R->setText(QString::number(settingPara->Front_Beam_Region1_Z_Shift_R));
    ui->lineEdit_Front_Beam_Region2_X_Shift_R->setText(QString::number(settingPara->Front_Beam_Region2_X_Shift_R));
    ui->lineEdit_Front_Beam_Region2_Y_Shift_R->setText(QString::number(settingPara->Front_Beam_Region2_Y_Shift_R));
    ui->lineEdit_Front_Beam_Region2_Z_Shift_R->setText(QString::number(settingPara->Front_Beam_Region2_Z_Shift_R));
    // 工件反面横梁----左右的XYZ方向偏移
    ui->lineEdit_Back_Beam_Region1_X_Shift_R->setText(QString::number(settingPara->Back_Beam_Region1_X_Shift_R));
    ui->lineEdit_Back_Beam_Region1_Y_Shift_R->setText(QString::number(settingPara->Back_Beam_Region1_Y_Shift_R));
    ui->lineEdit_Back_Beam_Region1_Z_Shift_R->setText(QString::number(settingPara->Back_Beam_Region1_Z_Shift_R));
    ui->lineEdit_Back_Beam_Region2_X_Shift_R->setText(QString::number(settingPara->Back_Beam_Region2_X_Shift_R));
    ui->lineEdit_Back_Beam_Region2_Y_Shift_R->setText(QString::number(settingPara->Back_Beam_Region2_Y_Shift_R));
    ui->lineEdit_Back_Beam_Region2_Z_Shift_R->setText(QString::number(settingPara->Back_Beam_Region2_Z_Shift_R));

    // 点云放缩平移
    ui->lineEditScaleOfPointX->setText(QString::number(settingPara->scaleOfPointX));
    ui->lineEditTransOfPointX->setText(QString::number(settingPara->transOfPointX));
    ui->lineEditScaleOfPointY->setText(QString::number(settingPara->scaleOfPointY));
    ui->lineEditTransOfPointY->setText(QString::number(settingPara->transOfPointY));

    // 机器人焊接以及过渡速度显示
    ui->lineEdit_Value_MoveSpeed->setText(QString::number(settingPara->Value_MoveSpeed));
    ui->lineEdit_Value_WeldingSpeed->setText(QString::number(settingPara->Value_WeldingSpeed));
    ui->lineEdit_Value_WeldingSpeed_0To1->setText(QString::number(settingPara->Value_WeldingSpeed0To1));
    ui->lineEdit_Value_WeldingSpeed_1To3->setText(QString::number(settingPara->Value_WeldingSpeed1To3));
    ui->lineEdit_Value_WeldingSpeed_3To5->setText(QString::number(settingPara->Value_WeldingSpeed3To5));
    ui->lineEdit_Value_WeldingSpeed_Horizontal->setText(QString::number(settingPara->Value_WeldingSpeedHorizontal));
    ui->lineEdit_Value_WeldingSpeed_Vertical->setText(QString::number(settingPara->Value_WeldingSpeedVertical));
    ui->lineEdit_Value_WeldingCurrent->setText(QString::number(settingPara->Value_WeldingCurrent));
    ui->lineEdit_Value_WeldingCurrent_Vertical->setText(QString::number(settingPara->Value_WeldingCurrent_Vertical));
    ui->lineEdit_Value_WeldingVoltage->setText(QString::number(settingPara->Value_WeldingVoltage));
    ui->lineEdit_Value_WeldingVoltage_Vertical->setText(QString::number(settingPara->Value_WeldingVoltage_Vertical));

    // 是否焊接竖直焊缝
    ui->comboBoxWeldingVerticalWeld->setCurrentIndex(settingPara->weldingVerticalWeld);
}

// 相机曝光
void SettingWidget::on_spinBoxExp_editingFinished() {
    int value = ui->spinBoxExp->value();
    settingPara->camera_exposure = value;
    settingPara->qSetting->setValue("Camera/camera_exposure", value);

    emit sendCameraExposure(value);
    PLOGD << "ExposureTimeRaw: " << value;
}

void SettingWidget::on_coarseSpinBoxExp_editingFinished() {
    int value = ui->coarseSpinBoxExp->value();
    settingPara->coarse_camera_exposure = value;
    settingPara->qSetting->setValue("Camera/coarse_camera_exposure", value);

    emit sendCoarseCameraExposure(value);
    PLOGD << "ExposureTimeRaw: " << value;
}

// 相机增益
void SettingWidget::on_spinBoxGain_editingFinished() {
    int value = ui->spinBoxGain->value();
    settingPara->camera_gain = value;
    settingPara->qSetting->setValue("Camera/camera_gain", value);

    emit sendCameraGain(value);
    PLOGD << "GainRaw: " << value;
}

// // 设置相机硬件触发
// void SettingWidget::on_btnHardwareTrigger_clicked() {
//     emit sendCameraWorkMode(CAMERA_WORK_MODE::HARDWARE_TRIGGER);
//     PLOGD << "设置相机硬件触发";
// }

// // 设置相机软件触发
// void SettingWidget::on_btnSoftwareTrigger_clicked() {
//     emit sendCameraWorkMode(CAMERA_WORK_MODE::SOFTWARE_TRIGGER);
//     PLOGD << "设置相机软件触发";
// }

// 投影仪亮度
void SettingWidget::on_spinBoxProjBrightness_editingFinished() {
    int value = ui->spinBoxProjBrightness->value();
    settingPara->projector_brightness = value;
    settingPara->qSetting->setValue("Projector/projector_brightness", value);

    emit sendProjectorBrightness("brightness", value);
    PLOGD << "ProjBrightness: " << value;
}

// 投影仪帧率
void SettingWidget::on_spinBoxProjFps_editingFinished() {
    int value = ui->spinBoxProjFps->value();
    settingPara->projector_fps = value;
    settingPara->qSetting->setValue("Projector/projector_fps", value);

    emit sendProjectorFps("fps", value);
    PLOGD << "ProjFps: " << value;
}

// // 投影仪投影图片
// void SettingWidget::on_spinBoxProjNum_editingFinished() {
//     int value = ui->spinBoxProjNum->value();
//     settingPara->projector_num = value;
//     settingPara->qSetting->setValue("Projector/projector_num", value);

//     emit sendProjectorNum("projNum", value);
//     PLOGD << "ProjNum: " << value;
// }

// 开投影仪LED
void SettingWidget::on_LED_ON_clicked() { emit sendProjectorLedOn(); }

// 关投影仪LED
void SettingWidget::on_LED_OFF_clicked() { emit sendProjectorLedOff(); }

// // 调制度
// void SettingWidget::on_spinBox_Modulation_editingFinished() {
//     int value = ui->spinBox_Modulation->value();
//     settingPara->modulation_threshold = value;
//     settingPara->qSetting->setValue("Reconstruction/modulation_threshold", value);
//     PLOGD << "Modulation: " << value;
// }

// // 直通滤波最小值
// void SettingWidget::on_lineEdit_passthrough_Min_editingFinished() {
//     settingPara->passthrough_Min = ui->lineEdit_passthrough_Min->text().toDouble();
//     settingPara->qSetting->setValue("pointCloudProcessing/passthrough_Min", settingPara->passthrough_Min);
//     PLOGD << "passthrough_Min: " << settingPara->passthrough_Min;
// }

// // 直通滤波最大值
// void SettingWidget::on_lineEdit_passthrough_Max_editingFinished() {
//     settingPara->passthrough_Max = ui->lineEdit_passthrough_Max->text().toDouble();
//     settingPara->qSetting->setValue("pointCloudProcessing/passthrough_Max", settingPara->passthrough_Max);
//     PLOGD << "passthrough_Max: " << settingPara->passthrough_Max;
// }

// // 统计滤波点数
// void SettingWidget::on_lineEdit_StatisticalFilter_Pts_editingFinished() {
//     settingPara->statistical_Pts = ui->lineEdit_StatisticalFilter_Pts->text().toDouble();
//     settingPara->qSetting->setValue("pointCloudProcessing/statistical_Pts", settingPara->statistical_Pts);
//     PLOGD << "StatisticalFilter_Pts: " << settingPara->statistical_Pts;
// }

// // 统计滤波标准差
// void SettingWidget::on_lineEdit_StatisticalFilter_Std_editingFinished() {
//     settingPara->statistical_Std = ui->lineEdit_StatisticalFilter_Std->text().toDouble();
//     settingPara->qSetting->setValue("pointCloudProcessing/statistical_Std", settingPara->statistical_Std);
//     PLOGD << "StatisticalFilter_Std: " << settingPara->statistical_Std;
// }

// 是否保存重建的模型
void SettingWidget::on_comboBox_SaveModel_activated(int index) {
    settingPara->bool_save_model = ui->comboBox_SaveModel->currentIndex();  // 获取当前保存设置（是/否）
    PLOGD << "是否保存重建的模型: " << settingPara->bool_save_model;
    (void)index;
}

// // 边界端点到交点的距离阈值, 用于判断焊缝拍摄是否完整
// void SettingWidget::on_lineEdit_Dth_IntersectionToStartPoint_editingFinished() {
//     settingPara->MinDth_IntersectionToStartPoint = ui->lineEdit_Dth_IntersectionToStartPoint->text().toDouble();
//     settingPara->qSetting->setValue("SeamPosition/MinDth_IntersectionToStartPoint",
//     settingPara->MinDth_IntersectionToStartPoint); PLOGD << "MinDth_IntersectionToStartPoint: " <<
//     settingPara->MinDth_IntersectionToStartPoint;
// }

// 机器人过渡速度
void SettingWidget::on_lineEdit_Value_MoveSpeed_editingFinished() {
    settingPara->Value_MoveSpeed = ui->lineEdit_Value_MoveSpeed->text().toDouble();
    settingPara->qSetting->setValue("Welding/Value_MoveSpeed", settingPara->Value_MoveSpeed);
    PLOGD << "Value_MoveSpeed: " << settingPara->Value_MoveSpeed;
}

// 机器人默认焊接速度
void SettingWidget::on_lineEdit_Value_WeldingSpeed_editingFinished() {
    settingPara->Value_WeldingSpeed = ui->lineEdit_Value_WeldingSpeed->text().toDouble();
    settingPara->qSetting->setValue("Welding/Value_WeldingSpeed", settingPara->Value_WeldingSpeed);
    PLOGD << "Value_WeldingSpeed: " << settingPara->Value_WeldingSpeed;
}

// 宽度0到1焊接速度
void SettingWidget::on_lineEdit_Value_WeldingSpeed_0To1_editingFinished() {
    settingPara->Value_WeldingSpeed0To1 = ui->lineEdit_Value_WeldingSpeed_0To1->text().toDouble();
    settingPara->qSetting->setValue("Welding/Value_WeldingSpeed0To1", settingPara->Value_WeldingSpeed0To1);
    PLOGD << "Value_WeldingSpeed0To1: " << settingPara->Value_WeldingSpeed0To1;
}

// 宽度1到3焊接速度
void SettingWidget::on_lineEdit_Value_WeldingSpeed_1To3_editingFinished() {
    settingPara->Value_WeldingSpeed1To3 = ui->lineEdit_Value_WeldingSpeed_1To3->text().toDouble();
    settingPara->qSetting->setValue("Welding/Value_WeldingSpeed1To3", settingPara->Value_WeldingSpeed1To3);
    PLOGD << "Value_WeldingSpeed1To3: " << settingPara->Value_WeldingSpeed1To3;
}

// 宽度3到5焊接速度
void SettingWidget::on_lineEdit_Value_WeldingSpeed_3To5_editingFinished() {
    settingPara->Value_WeldingSpeed3To5 = ui->lineEdit_Value_WeldingSpeed_3To5->text().toDouble();
    settingPara->qSetting->setValue("Welding/Value_WeldingSpeed3To5", settingPara->Value_WeldingSpeed3To5);
    PLOGD << "Value_WeldingSpeed3To5: " << settingPara->Value_WeldingSpeed3To5;
}

// 水平焊缝焊接速度
void SettingWidget::on_lineEdit_Value_WeldingSpeed_Horizontal_editingFinished() {
    settingPara->Value_WeldingSpeedHorizontal = ui->lineEdit_Value_WeldingSpeed_Horizontal->text().toDouble();
    settingPara->qSetting->setValue("Welding/Value_WeldingSpeedHorizontal", settingPara->Value_WeldingSpeedHorizontal);
    PLOGD << "Value_WeldingSpeedHorizontal: " << settingPara->Value_WeldingSpeedHorizontal;
}
// 竖直焊缝焊接速度
void SettingWidget::on_lineEdit_Value_WeldingSpeed_Vertical_editingFinished() {
    settingPara->Value_WeldingSpeedVertical = ui->lineEdit_Value_WeldingSpeed_Vertical->text().toDouble();
    settingPara->qSetting->setValue("Welding/Value_WeldingSpeedVertical", settingPara->Value_WeldingSpeedVertical);
    PLOGD << "Value_WeldingSpeedVertical: " << settingPara->Value_WeldingSpeedVertical;
}

// 是否焊接竖直焊缝
void SettingWidget::on_comboBoxWeldingVerticalWeld_activated(int index) {
    settingPara->weldingVerticalWeld = ui->comboBoxWeldingVerticalWeld->currentIndex();
    settingPara->qSetting->setValue("Welding/weldingVerticalWeld", settingPara->weldingVerticalWeld);
    PLOGD << "是否焊接竖直焊缝: " << settingPara->weldingVerticalWeld;
    (void)index;
}
// 焊接电流电压
void SettingWidget::on_lineEdit_Value_WeldingCurrent_editingFinished() {
    settingPara->Value_WeldingCurrent = ui->lineEdit_Value_WeldingCurrent->text().toDouble();
    settingPara->qSetting->setValue("Welding/Value_WeldingCurrent", settingPara->Value_WeldingCurrent);
    PLOGD << "Value_WeldingCurrent: " << settingPara->Value_WeldingCurrent;
}

void SettingWidget::on_lineEdit_Value_WeldingVoltage_editingFinished() {
    settingPara->Value_WeldingVoltage = ui->lineEdit_Value_WeldingVoltage->text().toDouble();
    settingPara->qSetting->setValue("Welding/Value_WeldingVoltage", settingPara->Value_WeldingVoltage);
    PLOGD << "Value_WeldingVoltage: " << settingPara->Value_WeldingVoltage;
}

void SettingWidget::on_lineEdit_Value_WeldingCurrent_Vertical_editingFinished() {
    settingPara->Value_WeldingCurrent_Vertical = ui->lineEdit_Value_WeldingCurrent_Vertical->text().toDouble();
    settingPara->qSetting->setValue("Welding/Value_WeldingCurrent_Vertical", settingPara->Value_WeldingCurrent_Vertical);
    PLOGD << "Value_WeldingCurrent_Vertical: " << settingPara->Value_WeldingCurrent_Vertical;
}

void SettingWidget::on_lineEdit_Value_WeldingVoltage_Vertical_editingFinished() {
    settingPara->Value_WeldingVoltage_Vertical = ui->lineEdit_Value_WeldingVoltage_Vertical->text().toDouble();
    settingPara->qSetting->setValue("Welding/Value_WeldingVoltage_Vertical", settingPara->Value_WeldingVoltage_Vertical);
    PLOGD << "Value_WeldingVoltage_Vertical: " << settingPara->Value_WeldingVoltage_Vertical;
}
// 点云X方向放缩
void SettingWidget::on_lineEditScaleOfPointX_editingFinished() {
    settingPara->scaleOfPointX = ui->lineEditScaleOfPointX->text().toDouble();
    settingPara->qSetting->setValue("Scale/scaleOfPointX", settingPara->scaleOfPointX);
    PLOGD << "scaleOfPointX: " << settingPara->scaleOfPointX;
}

// 点云X方向平移
void SettingWidget::on_lineEditTransOfPointX_editingFinished() {
    settingPara->transOfPointX = ui->lineEditTransOfPointX->text().toDouble();
    settingPara->qSetting->setValue("Scale/transOfPointX", settingPara->transOfPointX);
    PLOGD << "transOfPointX: " << settingPara->transOfPointX;
}

// 点云Y方向放缩
void SettingWidget::on_lineEditScaleOfPointY_editingFinished() {
    settingPara->scaleOfPointY = ui->lineEditScaleOfPointY->text().toDouble();
    settingPara->qSetting->setValue("Scale/scaleOfPointY", settingPara->scaleOfPointY);
    PLOGD << "scaleOfPointY: " << settingPara->scaleOfPointY;
}

// 点云Y方向平移
void SettingWidget::on_lineEditTransOfPointY_editingFinished() {
    settingPara->transOfPointY = ui->lineEditTransOfPointY->text().toDouble();
    settingPara->qSetting->setValue("Scale/transOfPointY", settingPara->transOfPointY);
    PLOGD << "transOfPointY: " << settingPara->transOfPointY;
}

// 正面左侧边角焊缝起点延长
void SettingWidget::on_lineEdit_FrontLeft_ExtendStart_editingFinished() {
    settingPara->FrontLeft_ExtendStart = ui->lineEdit_FrontLeft_ExtendStart->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/FrontLeft_ExtendStart", settingPara->FrontLeft_ExtendStart);
    PLOGD << "FrontLeft_ExtendStart: " << settingPara->FrontLeft_ExtendStart;
}

// 正面左侧边角焊缝终点延长
void SettingWidget::on_lineEdit_FrontLeft_ExtendEnd_editingFinished() {
    settingPara->FrontLeft_ExtendEnd = ui->lineEdit_FrontLeft_ExtendEnd->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/FrontLeft_ExtendEnd", settingPara->FrontLeft_ExtendEnd);
    PLOGD << "FrontLeft_ExtendEnd: " << settingPara->FrontLeft_ExtendEnd;
}

// 正面右侧边角焊缝起点延长
void SettingWidget::on_lineEdit_FrontRight_ExtendStart_editingFinished() {
    settingPara->FrontRight_ExtendStart = ui->lineEdit_FrontRight_ExtendStart->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/FrontRight_ExtendStart", settingPara->FrontRight_ExtendStart);
    PLOGD << "FrontRight_ExtendStart: " << settingPara->FrontRight_ExtendStart;
}

// 正面右侧边角焊缝终点延长
void SettingWidget::on_lineEdit_FrontRight_ExtendEnd_editingFinished() {
    settingPara->FrontRight_ExtendEnd = ui->lineEdit_FrontRight_ExtendEnd->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/FrontRight_ExtendEnd", settingPara->FrontRight_ExtendEnd);
    PLOGD << "FrontRight_ExtendEnd: " << settingPara->FrontRight_ExtendEnd;
}

// 反面左侧横梁焊缝起点延长
void SettingWidget::on_lineEdit_FrontBeamLeft_ExtendStart_editingFinished() {
    settingPara->FrontBeamLeft_ExtendStart = ui->lineEdit_FrontBeamLeft_ExtendStart->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/FrontBeamLeft_ExtendStart", settingPara->FrontBeamLeft_ExtendStart);
    PLOGD << "FrontBeamLeft_ExtendStart: " << settingPara->FrontBeamLeft_ExtendStart;
}

// 反面左侧横梁焊缝终点延长
void SettingWidget::on_lineEdit_FrontBeamLeft_ExtendEnd_editingFinished() {
    settingPara->FrontBeamLeft_ExtendEnd = ui->lineEdit_FrontBeamLeft_ExtendEnd->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/FrontBeamLeft_ExtendEnd", settingPara->FrontBeamLeft_ExtendEnd);
    PLOGD << "FrontBeamLeft_ExtendEnd: " << settingPara->FrontBeamLeft_ExtendEnd;
}

// 反面右侧横梁焊缝起点延长
void SettingWidget::on_lineEdit_FrontBeamRight_ExtendStart_editingFinished() {
    settingPara->FrontBeamRight_ExtendStart = ui->lineEdit_FrontBeamRight_ExtendStart->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/FrontBeamRight_ExtendStart", settingPara->FrontBeamRight_ExtendStart);
    PLOGD << "FrontBeamRight_ExtendStart: " << settingPara->FrontBeamRight_ExtendStart;
}

// 反面右侧横梁焊缝终点延长
void SettingWidget::on_lineEdit_FrontBeamRight_ExtendEnd_editingFinished() {
    settingPara->FrontBeamRight_ExtendEnd = ui->lineEdit_FrontBeamRight_ExtendEnd->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/FrontBeamRight_ExtendEnd", settingPara->FrontBeamRight_ExtendEnd);
    PLOGD << "FrontBeamRight_ExtendEnd: " << settingPara->FrontBeamRight_ExtendEnd;
}

// 正面左侧横梁水平焊缝起点延长
void SettingWidget::on_lineEdit_FrontHBeamLeft_ExtendStart_editingFinished() {
    settingPara->FrontHBeamLeft_ExtendStart = ui->lineEdit_FrontHBeamLeft_ExtendStart->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/FrontHBeamLeft_ExtendStart", settingPara->FrontHBeamLeft_ExtendStart);
    PLOGD << "FrontHBeamLeft_ExtendStart: " << settingPara->FrontHBeamLeft_ExtendStart;
}

// 正面左侧横梁水平焊缝终点延长
void SettingWidget::on_lineEdit_FrontHBeamLeft_ExtendEnd_editingFinished() {
    settingPara->FrontHBeamLeft_ExtendEnd = ui->lineEdit_FrontHBeamLeft_ExtendEnd->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/FrontHBeamLeft_ExtendEnd", settingPara->FrontHBeamLeft_ExtendEnd);
    PLOGD << "FrontHBeamLeft_ExtendEnd: " << settingPara->FrontHBeamLeft_ExtendEnd;
}

// 正面右侧横梁水平焊缝起点延长
void SettingWidget::on_lineEdit_FrontHBeamRight_ExtendStart_editingFinished() {
    settingPara->FrontHBeamRight_ExtendStart = ui->lineEdit_FrontHBeamRight_ExtendStart->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/FrontHBeamRight_ExtendStart", settingPara->FrontHBeamRight_ExtendStart);
    PLOGD << "FrontHBeamRight_ExtendStart: " << settingPara->FrontHBeamRight_ExtendStart;
}

// 正面右侧横梁水平焊缝终点延长
void SettingWidget::on_lineEdit_FrontHBeamRight_ExtendEnd_editingFinished() {
    settingPara->FrontHBeamRight_ExtendEnd = ui->lineEdit_FrontHBeamRight_ExtendEnd->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/FrontHBeamRight_ExtendEnd", settingPara->FrontHBeamRight_ExtendEnd);
    PLOGD << "FrontHBeamRight_ExtendEnd: " << settingPara->FrontHBeamRight_ExtendEnd;
}

// 正面左侧横梁焊缝后撤距离
void SettingWidget::on_lineEdit_FrontBeamLeft_WithDrawDistance_editingFinished() {
    settingPara->FrontBeamLeft_WithDrawDistance = ui->lineEdit_FrontBeamLeft_WithDrawDistance->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/FrontBeamLeft_WithDrawDistance", settingPara->FrontBeamLeft_WithDrawDistance);
    PLOGD << "FrontBeamLeft_WithDrawDistance: " << settingPara->FrontBeamLeft_WithDrawDistance;
}

// 正面右侧横梁焊缝后撤距离
void SettingWidget::on_lineEdit_FrontBeamRight_WithDrawDistance_editingFinished() {
    settingPara->FrontBeamRight_WithDrawDistance = ui->lineEdit_FrontBeamRight_WithDrawDistance->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/FrontBeamRight_WithDrawDistance", settingPara->FrontBeamRight_WithDrawDistance);
    PLOGD << "FrontBeamRight_WithDrawDistance: " << settingPara->FrontBeamRight_WithDrawDistance;
}

// 反面左侧边角焊缝起点延长
void SettingWidget::on_lineEdit_BackLeft_ExtendStart_editingFinished() {
    settingPara->BackLeft_ExtendStart = ui->lineEdit_BackLeft_ExtendStart->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/BackLeft_ExtendStart", settingPara->BackLeft_ExtendStart);
    PLOGD << "BackLeft_ExtendStart: " << settingPara->BackLeft_ExtendStart;
}

// 反面左侧边角焊缝终点延长
void SettingWidget::on_lineEdit_BackLeft_ExtendEnd_editingFinished() {
    settingPara->BackLeft_ExtendEnd = ui->lineEdit_BackLeft_ExtendEnd->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/BackLeft_ExtendEnd", settingPara->BackLeft_ExtendEnd);
    PLOGD << "BackLeft_ExtendEnd: " << settingPara->BackLeft_ExtendEnd;
}

// 反面右侧边角焊缝起点延长
void SettingWidget::on_lineEdit_BackRight_ExtendStart_editingFinished() {
    settingPara->BackRight_ExtendStart = ui->lineEdit_BackRight_ExtendStart->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/BackRight_ExtendStart", settingPara->BackRight_ExtendStart);
    PLOGD << "BackRight_ExtendStart: " << settingPara->BackRight_ExtendStart;
}

// 反面右侧边角焊缝终点延长
void SettingWidget::on_lineEdit_BackRight_ExtendEnd_editingFinished() {
    settingPara->BackRight_ExtendEnd = ui->lineEdit_BackRight_ExtendEnd->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/BackRight_ExtendEnd", settingPara->BackRight_ExtendEnd);
    PLOGD << "BackRight_ExtendEnd: " << settingPara->BackRight_ExtendEnd;
}

// 反面左侧横梁焊缝起点延长
void SettingWidget::on_lineEdit_BackBeamLeft_ExtendStart_editingFinished() {
    settingPara->BackBeamLeft_ExtendStart = ui->lineEdit_BackBeamLeft_ExtendStart->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/BackBeamLeft_ExtendStart", settingPara->BackBeamLeft_ExtendStart);
    PLOGD << "BackBeamLeft_ExtendStart: " << settingPara->BackBeamLeft_ExtendStart;
}

// 反面左侧横梁焊缝终点延长
void SettingWidget::on_lineEdit_BackBeamLeft_ExtendEnd_editingFinished() {
    settingPara->BackBeamLeft_ExtendEnd = ui->lineEdit_BackBeamLeft_ExtendEnd->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/BackBeamLeft_ExtendEnd", settingPara->BackBeamLeft_ExtendEnd);
    PLOGD << "BackBeamLeft_ExtendEnd: " << settingPara->BackBeamLeft_ExtendEnd;
}

// 反面右侧横梁焊缝起点延长
void SettingWidget::on_lineEdit_BackBeamRight_ExtendStart_editingFinished() {
    settingPara->BackBeamRight_ExtendStart = ui->lineEdit_BackBeamRight_ExtendStart->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/BackBeamRight_ExtendStart", settingPara->BackBeamRight_ExtendStart);
    PLOGD << "BackBeamRight_ExtendStart: " << settingPara->BackBeamRight_ExtendStart;
}

// 反面右侧横梁焊缝终点延长
void SettingWidget::on_lineEdit_BackBeamRight_ExtendEnd_editingFinished() {
    settingPara->BackBeamRight_ExtendEnd = ui->lineEdit_BackBeamRight_ExtendEnd->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/BackBeamRight_ExtendEnd", settingPara->BackBeamRight_ExtendEnd);
    PLOGD << "BackBeamRight_ExtendEnd: " << settingPara->BackBeamRight_ExtendEnd;
}

// 机器人左侧, 工件正面区域1边角焊缝, X方向偏移
void SettingWidget::on_lineEdit_Front_Region1_X_Shift_editingFinished() {
    settingPara->Front_Region1_X_Shift = ui->lineEdit_Front_Region1_X_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_Region1_X_Shift", settingPara->Front_Region1_X_Shift);
    PLOGD << "Front_Region1_X_Shift: " << settingPara->Front_Region1_X_Shift;
}

// 机器人左侧, 工件正面区域1边角焊缝, Y方向偏移
void SettingWidget::on_lineEdit_Front_Region1_Y_Shift_editingFinished() {
    settingPara->Front_Region1_Y_Shift = ui->lineEdit_Front_Region1_Y_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_Region1_Y_Shift", settingPara->Front_Region1_Y_Shift);
    PLOGD << "Front_Region1_Y_Shift: " << settingPara->Front_Region1_Y_Shift;
}

// 机器人左侧, 工件正面区域1边角焊缝, Z方向偏移
void SettingWidget::on_lineEdit_Front_Region1_Z_Shift_editingFinished() {
    settingPara->Front_Region1_Z_Shift = ui->lineEdit_Front_Region1_Z_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_Region1_Z_Shift", settingPara->Front_Region1_Z_Shift);
    PLOGD << "Front_Region1_Z_Shift: " << settingPara->Front_Region1_Z_Shift;
}

// 机器人左侧, 工件正面区域2边角焊缝, X方向偏移
void SettingWidget::on_lineEdit_Front_Region2_X_Shift_editingFinished() {
    settingPara->Front_Region2_X_Shift = ui->lineEdit_Front_Region2_X_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_Region2_X_Shift", settingPara->Front_Region2_X_Shift);
    PLOGD << "Front_Region2_X_Shift: " << settingPara->Front_Region2_X_Shift;
}

// 机器人左侧, 工件正面区域2边角焊缝, Y方向偏移
void SettingWidget::on_lineEdit_Front_Region2_Y_Shift_editingFinished() {
    settingPara->Front_Region2_Y_Shift = ui->lineEdit_Front_Region2_Y_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_Region2_Y_Shift", settingPara->Front_Region2_Y_Shift);
    PLOGD << "Front_Region2_Y_Shift: " << settingPara->Front_Region2_Y_Shift;
}

// 机器人左侧, 工件正面区域2边角焊缝, Z方向偏移
void SettingWidget::on_lineEdit_Front_Region2_Z_Shift_editingFinished() {
    settingPara->Front_Region2_Z_Shift = ui->lineEdit_Front_Region2_Z_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_Region2_Z_Shift", settingPara->Front_Region2_Z_Shift);
    PLOGD << "Front_Region2_Z_Shift: " << settingPara->Front_Region2_Z_Shift;
}

// 机器人左侧, 工件反面区域1边角焊缝, X方向偏移
void SettingWidget::on_lineEdit_Back_Region1_X_Shift_editingFinished() {
    settingPara->Back_Region1_X_Shift = ui->lineEdit_Back_Region1_X_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Back_Region1_X_Shift", settingPara->Back_Region1_X_Shift);
    PLOGD << "Back_Region1_X_Shift: " << settingPara->Back_Region1_X_Shift;
}

// 机器人左侧, 工件反面区域1边角焊缝, Y方向偏移
void SettingWidget::on_lineEdit_Back_Region1_Y_Shift_editingFinished() {
    settingPara->Back_Region1_Y_Shift = ui->lineEdit_Back_Region1_Y_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Back_Region1_Y_Shift", settingPara->Back_Region1_Y_Shift);
    PLOGD << "Back_Region1_Y_Shift: " << settingPara->Back_Region1_Y_Shift;
}

// 机器人左侧, 工件反面区域1边角焊缝, Z方向偏移
void SettingWidget::on_lineEdit_Back_Region1_Z_Shift_editingFinished() {
    settingPara->Back_Region1_Z_Shift = ui->lineEdit_Back_Region1_Z_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Back_Region1_Z_Shift", settingPara->Back_Region1_Z_Shift);
    PLOGD << "Back_Region1_Z_Shift: " << settingPara->Back_Region1_Z_Shift;
}

// 机器人左侧, 工件反面区域2边角焊缝, X方向偏移
void SettingWidget::on_lineEdit_Back_Region2_X_Shift_editingFinished() {
    settingPara->Back_Region2_X_Shift = ui->lineEdit_Back_Region2_X_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Back_Region2_X_Shift", settingPara->Back_Region2_X_Shift);
    PLOGD << "Back_Region2_X_Shift: " << settingPara->Back_Region2_X_Shift;
}

// 机器人左侧, 工件反面区域2边角焊缝, Y方向偏移
void SettingWidget::on_lineEdit_Back_Region2_Y_Shift_editingFinished() {
    settingPara->Back_Region2_Y_Shift = ui->lineEdit_Back_Region2_Y_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Back_Region2_Y_Shift", settingPara->Back_Region2_Y_Shift);
    PLOGD << "Back_Region2_Y_Shift: " << settingPara->Back_Region2_Y_Shift;
}

// 机器人左侧, 工件反面区域2边角焊缝, Z方向偏移
void SettingWidget::on_lineEdit_Back_Region2_Z_Shift_editingFinished() {
    settingPara->Back_Region2_Z_Shift = ui->lineEdit_Back_Region2_Z_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Back_Region2_Z_Shift", settingPara->Back_Region2_Z_Shift);
    PLOGD << "Back_Region2_Z_Shift: " << settingPara->Back_Region2_Z_Shift;
}

// 机器人左侧, 工件正面区域1横梁焊缝, X方向偏移
void SettingWidget::on_lineEdit_Front_Beam_Region1_X_Shift_editingFinished() {
    settingPara->Front_Beam_Region1_X_Shift = ui->lineEdit_Front_Beam_Region1_X_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_Beam_Region1_X_Shift", settingPara->Front_Beam_Region1_X_Shift);
    PLOGD << "Front_Beam_Region1_X_Shift: " << settingPara->Front_Beam_Region1_X_Shift;
}

// 机器人左侧, 工件正面区域1横梁焊缝, Y方向偏移
void SettingWidget::on_lineEdit_Front_Beam_Region1_Y_Shift_editingFinished() {
    settingPara->Front_Beam_Region1_Y_Shift = ui->lineEdit_Front_Beam_Region1_Y_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_Beam_Region1_Y_Shift", settingPara->Front_Beam_Region1_Y_Shift);
    PLOGD << "Front_Beam_Region1_Y_Shift: " << settingPara->Front_Beam_Region1_Y_Shift;
}

// 机器人左侧, 工件正面区域1横梁焊缝, Z方向偏移
void SettingWidget::on_lineEdit_Front_Beam_Region1_Z_Shift_editingFinished() {
    settingPara->Front_Beam_Region1_Z_Shift = ui->lineEdit_Front_Beam_Region1_Z_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_Beam_Region1_Z_Shift", settingPara->Front_Beam_Region1_Z_Shift);
    PLOGD << "Front_Beam_Region1_Z_Shift: " << settingPara->Front_Beam_Region1_Z_Shift;
}

// 机器人左侧, 工件正面区域2横梁焊缝, X方向偏移
void SettingWidget::on_lineEdit_Front_Beam_Region2_X_Shift_editingFinished() {
    settingPara->Front_Beam_Region2_X_Shift = ui->lineEdit_Front_Beam_Region2_X_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_Beam_Region2_X_Shift", settingPara->Front_Beam_Region2_X_Shift);
    PLOGD << "Front_Beam_Region2_X_Shift: " << settingPara->Front_Beam_Region2_X_Shift;
}

// 机器人左侧, 工件正面区域2横梁焊缝, Y方向偏移
void SettingWidget::on_lineEdit_Front_Beam_Region2_Y_Shift_editingFinished() {
    settingPara->Front_Beam_Region2_Y_Shift = ui->lineEdit_Front_Beam_Region2_Y_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_Beam_Region2_Y_Shift", settingPara->Front_Beam_Region2_Y_Shift);
    PLOGD << "Front_Beam_Region2_Y_Shift: " << settingPara->Front_Beam_Region2_Y_Shift;
}

// 机器人左侧, 工件正面区域2横梁焊缝, Z方向偏移
void SettingWidget::on_lineEdit_Front_Beam_Region2_Z_Shift_editingFinished() {
    settingPara->Front_Beam_Region2_Z_Shift = ui->lineEdit_Front_Beam_Region2_Z_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_Beam_Region2_Z_Shift", settingPara->Front_Beam_Region2_Z_Shift);
    PLOGD << "Front_Beam_Region2_Z_Shift: " << settingPara->Front_Beam_Region2_Z_Shift;
}

// 机器人左侧, 工件反面区域1横梁焊缝, X方向偏移
void SettingWidget::on_lineEdit_Back_Beam_Region1_X_Shift_editingFinished() {
    settingPara->Back_Beam_Region1_X_Shift = ui->lineEdit_Back_Beam_Region1_X_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Back_Beam_Region1_X_Shift", settingPara->Back_Beam_Region1_X_Shift);
    PLOGD << "Back_Beam_Region1_X_Shift: " << settingPara->Back_Beam_Region1_X_Shift;
}

// 机器人左侧, 工件反面区域1横梁焊缝, Y方向偏移
void SettingWidget::on_lineEdit_Back_Beam_Region1_Y_Shift_editingFinished() {
    settingPara->Back_Beam_Region1_Y_Shift = ui->lineEdit_Back_Beam_Region1_Y_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Back_Beam_Region1_Y_Shift", settingPara->Back_Beam_Region1_Y_Shift);
    PLOGD << "Back_Beam_Region1_Y_Shift: " << settingPara->Back_Beam_Region1_Y_Shift;
}

// 机器人左侧, 工件反面区域1横梁焊缝, Z方向偏移
void SettingWidget::on_lineEdit_Back_Beam_Region1_Z_Shift_editingFinished() {
    settingPara->Back_Beam_Region1_Z_Shift = ui->lineEdit_Back_Beam_Region1_Z_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Back_Beam_Region1_Z_Shift", settingPara->Back_Beam_Region1_Z_Shift);
    PLOGD << "Back_Beam_Region1_Z_Shift: " << settingPara->Back_Beam_Region1_Z_Shift;
}

// 机器人左侧, 工件反面区域2横梁焊缝, X方向偏移
void SettingWidget::on_lineEdit_Back_Beam_Region2_X_Shift_editingFinished() {
    settingPara->Back_Beam_Region2_X_Shift = ui->lineEdit_Back_Beam_Region2_X_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Back_Beam_Region2_X_Shift", settingPara->Back_Beam_Region2_X_Shift);
    PLOGD << "Back_Beam_Region2_X_Shift: " << settingPara->Back_Beam_Region2_X_Shift;
}

// 机器人左侧, 工件反面区域2横梁焊缝, Y方向偏移
void SettingWidget::on_lineEdit_Back_Beam_Region2_Y_Shift_editingFinished() {
    settingPara->Back_Beam_Region2_Y_Shift = ui->lineEdit_Back_Beam_Region2_Y_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Back_Beam_Region2_Y_Shift", settingPara->Back_Beam_Region2_Y_Shift);
    PLOGD << "Back_Beam_Region2_Y_Shift: " << settingPara->Back_Beam_Region2_Y_Shift;
}

// 机器人左侧, 工件反面区域2横梁焊缝, Z方向偏移
void SettingWidget::on_lineEdit_Back_Beam_Region2_Z_Shift_editingFinished() {
    settingPara->Back_Beam_Region2_Z_Shift = ui->lineEdit_Back_Beam_Region2_Z_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Back_Beam_Region2_Z_Shift", settingPara->Back_Beam_Region2_Z_Shift);
    PLOGD << "Back_Beam_Region2_Z_Shift: " << settingPara->Back_Beam_Region2_Z_Shift;
}

// 机器人右侧, 工件正面区域1边角焊缝, X方向偏移
void SettingWidget::on_lineEdit_Front_Region1_Y_Shift_R_editingFinished() {
    settingPara->Front_Region1_Y_Shift_R = ui->lineEdit_Front_Region1_Y_Shift_R->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_Region1_Y_Shift_R", settingPara->Front_Region1_Y_Shift_R);
    PLOGD << "Front_Region1_Y_Shift_R: " << settingPara->Front_Region1_Y_Shift_R;
}

// 机器人右侧, 工件正面区域1边角焊缝, Y方向偏移
void SettingWidget::on_lineEdit_Front_Region1_X_Shift_R_editingFinished() {
    settingPara->Front_Region1_X_Shift_R = ui->lineEdit_Front_Region1_X_Shift_R->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_Region1_X_Shift_R", settingPara->Front_Region1_X_Shift_R);
    PLOGD << "Front_Region1_X_Shift_R: " << settingPara->Front_Region1_X_Shift_R;
}

// 机器人右侧, 工件正面区域1边角焊缝, Z方向偏移
void SettingWidget::on_lineEdit_Front_Region1_Z_Shift_R_editingFinished() {
    settingPara->Front_Region1_Z_Shift_R = ui->lineEdit_Front_Region1_Z_Shift_R->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_Region1_Z_Shift_R", settingPara->Front_Region1_Z_Shift_R);
    PLOGD << "Front_Region1_Z_Shift_R: " << settingPara->Front_Region1_Z_Shift_R;
}

// 机器人右侧, 工件正面区域2边角焊缝, X方向偏移
void SettingWidget::on_lineEdit_Front_Region2_X_Shift_R_editingFinished() {
    settingPara->Front_Region2_X_Shift_R = ui->lineEdit_Front_Region2_X_Shift_R->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_Region2_X_Shift_R", settingPara->Front_Region2_X_Shift_R);
    PLOGD << "Front_Region2_X_Shift_R: " << settingPara->Front_Region2_X_Shift_R;
}

// 机器人右侧, 工件正面区域2边角焊缝, Y方向偏移
void SettingWidget::on_lineEdit_Front_Region2_Y_Shift_R_editingFinished() {
    settingPara->Front_Region2_Y_Shift_R = ui->lineEdit_Front_Region2_Y_Shift_R->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_Region2_Y_Shift_R", settingPara->Front_Region2_Y_Shift_R);
    PLOGD << "Front_Region2_Y_Shift_R: " << settingPara->Front_Region2_Y_Shift_R;
}

// 机器人右侧, 工件正面区域2边角焊缝, Z方向偏移
void SettingWidget::on_lineEdit_Front_Region2_Z_Shift_R_editingFinished() {
    settingPara->Front_Region2_Z_Shift_R = ui->lineEdit_Front_Region2_Z_Shift_R->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_Region2_Z_Shift_R", settingPara->Front_Region2_Z_Shift_R);
    PLOGD << "Front_Region2_Z_Shift_R: " << settingPara->Front_Region2_Z_Shift_R;
}

// 机器人右侧, 工件反面区域1边角焊缝, X方向偏移
void SettingWidget::on_lineEdit_Back_Region1_X_Shift_R_editingFinished() {
    settingPara->Back_Region1_X_Shift_R = ui->lineEdit_Back_Region1_X_Shift_R->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Back_Region1_X_Shift_R", settingPara->Back_Region1_X_Shift_R);
    PLOGD << "Back_Region1_X_Shift_R: " << settingPara->Back_Region1_X_Shift_R;
}

// 机器人右侧, 工件反面区域1边角焊缝, Y方向偏移
void SettingWidget::on_lineEdit_Back_Region1_Y_Shift_R_editingFinished() {
    settingPara->Back_Region1_Y_Shift_R = ui->lineEdit_Back_Region1_Y_Shift_R->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Back_Region1_Y_Shift_R", settingPara->Back_Region1_Y_Shift_R);
    PLOGD << "Back_Region1_Y_Shift_R: " << settingPara->Back_Region1_Y_Shift_R;
}

// 机器人右侧, 工件反面区域1边角焊缝, Z方向偏移
void SettingWidget::on_lineEdit_Back_Region1_Z_Shift_R_editingFinished() {
    settingPara->Back_Region1_Z_Shift_R = ui->lineEdit_Back_Region1_Z_Shift_R->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Back_Region1_Z_Shift_R", settingPara->Back_Region1_Z_Shift_R);
    PLOGD << "Back_Region1_Z_Shift_R: " << settingPara->Back_Region1_Z_Shift_R;
}

// 机器人右侧, 工件反面区域2边角焊缝, X方向偏移
void SettingWidget::on_lineEdit_Back_Region2_X_Shift_R_editingFinished() {
    settingPara->Back_Region2_X_Shift_R = ui->lineEdit_Back_Region2_X_Shift_R->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Back_Region2_X_Shift_R", settingPara->Back_Region2_X_Shift_R);
    PLOGD << "Back_Region2_X_Shift_R: " << settingPara->Back_Region2_X_Shift_R;
}

// 机器人右侧, 工件反面区域2边角焊缝, Y方向偏移
void SettingWidget::on_lineEdit_Back_Region2_Y_Shift_R_editingFinished() {
    settingPara->Back_Region2_Y_Shift_R = ui->lineEdit_Back_Region2_Y_Shift_R->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Back_Region2_Y_Shift_R", settingPara->Back_Region2_Y_Shift_R);
    PLOGD << "Back_Region2_Y_Shift_R: " << settingPara->Back_Region2_Y_Shift_R;
}

// 机器人右侧, 工件反面区域2边角焊缝, Z方向偏移
void SettingWidget::on_lineEdit_Back_Region2_Z_Shift_R_editingFinished() {
    settingPara->Back_Region2_Z_Shift_R = ui->lineEdit_Back_Region2_Z_Shift_R->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Back_Region2_Z_Shift_R", settingPara->Back_Region2_Z_Shift_R);
    PLOGD << "Back_Region2_Z_Shift_R: " << settingPara->Back_Region2_Z_Shift_R;
}

// 机器人右侧, 工件正面区域1横梁焊缝, X方向偏移
void SettingWidget::on_lineEdit_Front_Beam_Region1_X_Shift_R_editingFinished() {
    settingPara->Front_Beam_Region1_X_Shift_R = ui->lineEdit_Front_Beam_Region1_X_Shift_R->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_Beam_Region1_X_Shift_R", settingPara->Front_Beam_Region1_X_Shift_R);
    PLOGD << "Front_Beam_Region1_X_Shift_R: " << settingPara->Front_Beam_Region1_X_Shift_R;
}

// 机器人右侧, 工件正面区域1横梁焊缝, Y方向偏移
void SettingWidget::on_lineEdit_Front_Beam_Region1_Y_Shift_R_editingFinished() {
    settingPara->Front_Beam_Region1_Y_Shift_R = ui->lineEdit_Front_Beam_Region1_Y_Shift_R->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_Beam_Region1_Y_Shift_R", settingPara->Front_Beam_Region1_Y_Shift_R);
    PLOGD << "Front_Beam_Region1_Y_Shift_R: " << settingPara->Front_Beam_Region1_Y_Shift_R;
}

// 机器人右侧, 工件正面区域1横梁焊缝, Z方向偏移
void SettingWidget::on_lineEdit_Front_Beam_Region1_Z_Shift_R_editingFinished() {
    settingPara->Front_Beam_Region1_Z_Shift_R = ui->lineEdit_Front_Beam_Region1_Z_Shift_R->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_Beam_Region1_Z_Shift_R", settingPara->Front_Beam_Region1_Z_Shift_R);
    PLOGD << "Front_Beam_Region1_Z_Shift_R: " << settingPara->Front_Beam_Region1_Z_Shift_R;
}

// 机器人右侧, 工件正面区域2横梁焊缝, X方向偏移
void SettingWidget::on_lineEdit_Front_Beam_Region2_X_Shift_R_editingFinished() {
    settingPara->Front_Beam_Region2_X_Shift_R = ui->lineEdit_Front_Beam_Region2_X_Shift_R->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_Beam_Region2_X_Shift_R", settingPara->Front_Beam_Region2_X_Shift_R);
    PLOGD << "Front_Beam_Region2_X_Shift_R: " << settingPara->Front_Beam_Region2_X_Shift_R;
}

// 机器人右侧, 工件正面区域2横梁焊缝, Y方向偏移
void SettingWidget::on_lineEdit_Front_Beam_Region2_Y_Shift_R_editingFinished() {
    settingPara->Front_Beam_Region2_Y_Shift_R = ui->lineEdit_Front_Beam_Region2_Y_Shift_R->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_Beam_Region2_Y_Shift_R", settingPara->Front_Beam_Region2_Y_Shift_R);
    PLOGD << "Front_Beam_Region2_Y_Shift_R: " << settingPara->Front_Beam_Region2_Y_Shift_R;
}

// 机器人右侧, 工件正面区域2横梁焊缝, Z方向偏移
void SettingWidget::on_lineEdit_Front_Beam_Region2_Z_Shift_R_editingFinished() {
    settingPara->Front_Beam_Region2_Z_Shift_R = ui->lineEdit_Front_Beam_Region2_Z_Shift_R->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_Beam_Region2_Z_Shift_R", settingPara->Front_Beam_Region2_Z_Shift_R);
    PLOGD << "Front_Beam_Region2_Z_Shift_R: " << settingPara->Front_Beam_Region2_Z_Shift_R;
}

// 机器人右侧, 工件反面区域1横梁焊缝, X方向偏移
void SettingWidget::on_lineEdit_Back_Beam_Region1_X_Shift_R_editingFinished() {
    settingPara->Back_Beam_Region1_X_Shift_R = ui->lineEdit_Back_Beam_Region1_X_Shift_R->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Back_Beam_Region1_X_Shift_R", settingPara->Back_Beam_Region1_X_Shift_R);
    PLOGD << "Back_Beam_Region1_X_Shift_R: " << settingPara->Back_Beam_Region1_X_Shift_R;
}

// 机器人右侧, 工件反面区域1横梁焊缝, Y方向偏移
void SettingWidget::on_lineEdit_Back_Beam_Region1_Y_Shift_R_editingFinished() {
    settingPara->Back_Beam_Region1_Y_Shift_R = ui->lineEdit_Back_Beam_Region1_Y_Shift_R->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Back_Beam_Region1_Y_Shift_R", settingPara->Back_Beam_Region1_Y_Shift_R);
    PLOGD << "Back_Beam_Region1_Y_Shift_R: " << settingPara->Back_Beam_Region1_Y_Shift_R;
}

// 机器人右侧, 工件反面区域1横梁焊缝, Z方向偏移
void SettingWidget::on_lineEdit_Back_Beam_Region1_Z_Shift_R_editingFinished() {
    settingPara->Back_Beam_Region1_Z_Shift_R = ui->lineEdit_Back_Beam_Region1_Z_Shift_R->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Back_Beam_Region1_Z_Shift_R", settingPara->Back_Beam_Region1_Z_Shift_R);
    PLOGD << "Back_Beam_Region1_Z_Shift_R: " << settingPara->Back_Beam_Region1_Z_Shift_R;
}

// 机器人右侧, 工件反面区域2横梁焊缝, X方向偏移
void SettingWidget::on_lineEdit_Back_Beam_Region2_X_Shift_R_editingFinished() {
    settingPara->Back_Beam_Region2_X_Shift_R = ui->lineEdit_Back_Beam_Region2_X_Shift_R->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Back_Beam_Region2_X_Shift_R", settingPara->Back_Beam_Region2_X_Shift_R);
    PLOGD << "Back_Beam_Region2_X_Shift_R: " << settingPara->Back_Beam_Region2_X_Shift_R;
}

// 机器人右侧, 工件反面区域2横梁焊缝, Y方向偏移
void SettingWidget::on_lineEdit_Back_Beam_Region2_Y_Shift_R_editingFinished() {
    settingPara->Back_Beam_Region2_Y_Shift_R = ui->lineEdit_Back_Beam_Region2_Y_Shift_R->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Back_Beam_Region2_Y_Shift_R", settingPara->Back_Beam_Region2_Y_Shift_R);
    PLOGD << "Back_Beam_Region2_Y_Shift_R: " << settingPara->Back_Beam_Region2_Y_Shift_R;
}

// 机器人右侧, 工件反面区域2横梁焊缝, Z方向偏移
void SettingWidget::on_lineEdit_Back_Beam_Region2_Z_Shift_R_editingFinished() {
    settingPara->Back_Beam_Region2_Z_Shift_R = ui->lineEdit_Back_Beam_Region2_Z_Shift_R->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Back_Beam_Region2_Z_Shift_R", settingPara->Back_Beam_Region2_Z_Shift_R);
    PLOGD << "Back_Beam_Region2_Z_Shift_R: " << settingPara->Back_Beam_Region2_Z_Shift_R;
}

void SettingWidget::on_lineEdit_Front_L_Beam_H_X_Shift_editingFinished() {
    settingPara->Front_L_Beam_H_X_Shift = ui->lineEdit_Front_L_Beam_H_X_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_L_Beam_H_X_Shift", settingPara->Front_L_Beam_H_X_Shift);
    PLOGD << "Front_L_Beam_H_X_Shift: " << settingPara->Front_L_Beam_H_X_Shift;
}

void SettingWidget::on_lineEdit_Front_L_Beam_H_Y_Shift_editingFinished() {
    settingPara->Front_L_Beam_H_Y_Shift = ui->lineEdit_Front_L_Beam_H_Y_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_L_Beam_H_Y_Shift", settingPara->Front_L_Beam_H_Y_Shift);
    PLOGD << "Front_L_Beam_H_Y_Shift: " << settingPara->Front_L_Beam_H_Y_Shift;
}

void SettingWidget::on_lineEdit_Front_L_Beam_H_Z_Shift_editingFinished() {
    settingPara->Front_L_Beam_H_Z_Shift = ui->lineEdit_Front_L_Beam_H_Z_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_L_Beam_H_Z_Shift", settingPara->Front_L_Beam_H_Z_Shift);
    PLOGD << "Front_L_Beam_H_Z_Shift: " << settingPara->Front_L_Beam_H_Z_Shift;
}

void SettingWidget::on_lineEdit_Front_L_Beam_V_X_Shift_editingFinished() {
    settingPara->Front_L_Beam_V_X_Shift = ui->lineEdit_Front_L_Beam_V_X_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_L_Beam_V_X_Shift", settingPara->Front_L_Beam_V_X_Shift);
    PLOGD << "Front_L_Beam_V_X_Shift: " << settingPara->Front_L_Beam_V_X_Shift;
}

void SettingWidget::on_lineEdit_Front_L_Beam_V_Y_Shift_editingFinished() {
    settingPara->Front_L_Beam_V_Y_Shift = ui->lineEdit_Front_L_Beam_V_Y_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_L_Beam_V_Y_Shift", settingPara->Front_L_Beam_V_Y_Shift);
    PLOGD << "Front_L_Beam_V_Y_Shift: " << settingPara->Front_L_Beam_V_Y_Shift;
}

void SettingWidget::on_lineEdit_Front_L_Beam_V_Z_Shift_editingFinished() {
    settingPara->Front_L_Beam_V_Z_Shift = ui->lineEdit_Front_L_Beam_V_Z_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_L_Beam_V_Z_Shift", settingPara->Front_L_Beam_V_Z_Shift);
    PLOGD << "Front_L_Beam_V_Z_Shift: " << settingPara->Front_L_Beam_V_Z_Shift;
}

void SettingWidget::on_lineEdit_Front_L_Beam_DH_X_Shift_editingFinished() {
    settingPara->Front_L_Beam_DH_X_Shift = ui->lineEdit_Front_L_Beam_DH_X_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_L_Beam_DH_X_Shift", settingPara->Front_L_Beam_DH_X_Shift);
    PLOGD << "Front_L_Beam_DH_X_Shift: " << settingPara->Front_L_Beam_DH_X_Shift;
}

void SettingWidget::on_lineEdit_Front_L_Beam_DH_Y_Shift_editingFinished() {
    settingPara->Front_L_Beam_DH_Y_Shift = ui->lineEdit_Front_L_Beam_DH_Y_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_L_Beam_DH_Y_Shift", settingPara->Front_L_Beam_DH_Y_Shift);
    PLOGD << "Front_L_Beam_DH_Y_Shift: " << settingPara->Front_L_Beam_DH_Y_Shift;
}

void SettingWidget::on_lineEdit_Front_L_Beam_DH_Z_Shift_editingFinished() {
    settingPara->Front_L_Beam_DH_Z_Shift = ui->lineEdit_Front_L_Beam_DH_Z_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_L_Beam_DH_Z_Shift", settingPara->Front_L_Beam_DH_Z_Shift);
    PLOGD << "Front_L_Beam_DH_Z_Shift: " << settingPara->Front_L_Beam_DH_Z_Shift;
}

void SettingWidget::on_lineEdit_Front_L_Beam_DV_X_Shift_editingFinished() {
    settingPara->Front_L_Beam_DV_X_Shift = ui->lineEdit_Front_L_Beam_DV_X_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_L_Beam_DV_X_Shift", settingPara->Front_L_Beam_DV_X_Shift);
    PLOGD << "Front_L_Beam_DV_X_Shift: " << settingPara->Front_L_Beam_DV_X_Shift;
}

void SettingWidget::on_lineEdit_Front_L_Beam_DV_Y_Shift_editingFinished() {
    settingPara->Front_L_Beam_DV_Y_Shift = ui->lineEdit_Front_L_Beam_DV_Y_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_L_Beam_DV_Y_Shift", settingPara->Front_L_Beam_DV_Y_Shift);
    PLOGD << "Front_L_Beam_DV_Y_Shift: " << settingPara->Front_L_Beam_DV_Y_Shift;
}

void SettingWidget::on_lineEdit_Front_L_Beam_DV_Z_Shift_editingFinished() {
    settingPara->Front_L_Beam_DV_Z_Shift = ui->lineEdit_Front_L_Beam_DV_Z_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_L_Beam_DV_Z_Shift", settingPara->Front_L_Beam_DV_Z_Shift);
    PLOGD << "Front_L_Beam_DV_Z_Shift: " << settingPara->Front_L_Beam_DV_Z_Shift;
}

void SettingWidget::on_lineEdit_Front_R_Beam_H_X_Shift_editingFinished() {
    settingPara->Front_R_Beam_H_X_Shift = ui->lineEdit_Front_R_Beam_H_X_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_R_Beam_H_X_Shift", settingPara->Front_R_Beam_H_X_Shift);
    PLOGD << "Front_R_Beam_H_X_Shift: " << settingPara->Front_R_Beam_H_X_Shift;
}

void SettingWidget::on_lineEdit_Front_R_Beam_H_Y_Shift_editingFinished() {
    settingPara->Front_R_Beam_H_Y_Shift = ui->lineEdit_Front_R_Beam_H_Y_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_R_Beam_H_Y_Shift", settingPara->Front_R_Beam_H_Y_Shift);
    PLOGD << "Front_R_Beam_H_Y_Shift: " << settingPara->Front_R_Beam_H_Y_Shift;
}

void SettingWidget::on_lineEdit_Front_R_Beam_H_Z_Shift_editingFinished() {
    settingPara->Front_R_Beam_H_Z_Shift = ui->lineEdit_Front_R_Beam_H_Z_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_R_Beam_H_Z_Shift", settingPara->Front_R_Beam_H_Z_Shift);
    PLOGD << "Front_R_Beam_H_Z_Shift: " << settingPara->Front_R_Beam_H_Z_Shift;
}

void SettingWidget::on_lineEdit_Front_R_Beam_V_X_Shift_editingFinished() {
    settingPara->Front_R_Beam_V_X_Shift = ui->lineEdit_Front_R_Beam_V_X_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_R_Beam_V_X_Shift", settingPara->Front_R_Beam_V_X_Shift);
    PLOGD << "Front_R_Beam_V_X_Shift: " << settingPara->Front_R_Beam_V_X_Shift;
}

void SettingWidget::on_lineEdit_Front_R_Beam_V_Y_Shift_editingFinished() {
    settingPara->Front_R_Beam_V_Y_Shift = ui->lineEdit_Front_R_Beam_V_Y_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_R_Beam_V_Y_Shift", settingPara->Front_R_Beam_V_Y_Shift);
    PLOGD << "Front_R_Beam_V_Y_Shift: " << settingPara->Front_R_Beam_V_Y_Shift;
}

void SettingWidget::on_lineEdit_Front_R_Beam_V_Z_Shift_editingFinished() {
    settingPara->Front_R_Beam_V_Z_Shift = ui->lineEdit_Front_R_Beam_V_Z_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_R_Beam_V_Z_Shift", settingPara->Front_R_Beam_V_Z_Shift);
    PLOGD << "Front_R_Beam_V_Z_Shift: " << settingPara->Front_R_Beam_V_Z_Shift;
}

void SettingWidget::on_lineEdit_Front_R_Beam_DH_X_Shift_editingFinished() {
    settingPara->Front_R_Beam_DH_X_Shift = ui->lineEdit_Front_R_Beam_DH_X_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_R_Beam_DH_X_Shift", settingPara->Front_R_Beam_DH_X_Shift);
    PLOGD << "Front_R_Beam_DH_X_Shift: " << settingPara->Front_R_Beam_DH_X_Shift;
}

void SettingWidget::on_lineEdit_Front_R_Beam_DH_Y_Shift_editingFinished() {
    settingPara->Front_R_Beam_DH_Y_Shift = ui->lineEdit_Front_R_Beam_DH_Y_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_R_Beam_DH_Y_Shift", settingPara->Front_R_Beam_DH_Y_Shift);
    PLOGD << "Front_R_Beam_DH_Y_Shift: " << settingPara->Front_R_Beam_DH_Y_Shift;
}

void SettingWidget::on_lineEdit_Front_R_Beam_DH_Z_Shift_editingFinished() {
    settingPara->Front_R_Beam_DH_Z_Shift = ui->lineEdit_Front_R_Beam_DH_Z_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_R_Beam_DH_Z_Shift", settingPara->Front_R_Beam_DH_Z_Shift);
    PLOGD << "Front_R_Beam_DH_Z_Shift: " << settingPara->Front_R_Beam_DH_Z_Shift;
}

void SettingWidget::on_lineEdit_Front_R_Beam_DV_X_Shift_editingFinished() {
    settingPara->Front_R_Beam_DV_X_Shift = ui->lineEdit_Front_R_Beam_DV_X_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_R_Beam_DV_X_Shift", settingPara->Front_R_Beam_DV_X_Shift);
    PLOGD << "Front_R_Beam_DV_X_Shift: " << settingPara->Front_R_Beam_DV_X_Shift;
}

void SettingWidget::on_lineEdit_Front_R_Beam_DV_Y_Shift_editingFinished() {
    settingPara->Front_R_Beam_DV_Y_Shift = ui->lineEdit_Front_R_Beam_DV_Y_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_R_Beam_DV_Y_Shift", settingPara->Front_R_Beam_DV_Y_Shift);
    PLOGD << "Front_R_Beam_DV_Y_Shift: " << settingPara->Front_R_Beam_DV_Y_Shift;
}

void SettingWidget::on_lineEdit_Front_R_Beam_DV_Z_Shift_editingFinished() {
    settingPara->Front_R_Beam_DV_Z_Shift = ui->lineEdit_Front_R_Beam_DV_Z_Shift->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/Front_R_Beam_DV_Z_Shift", settingPara->Front_R_Beam_DV_Z_Shift);
    PLOGD << "Front_R_Beam_DV_Z_Shift: " << settingPara->Front_R_Beam_DV_Z_Shift;
}

void SettingWidget::on_lineEdit_FrontVBeamLeft_ExtendStart_editingFinished() {
    settingPara->FrontVBeamLeft_ExtendStart = ui->lineEdit_FrontVBeamLeft_ExtendStart->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/FrontVBeamLeft_ExtendStart", settingPara->FrontVBeamLeft_ExtendStart);
    PLOGD << "FrontVBeamLeft_ExtendStart: " << settingPara->FrontVBeamLeft_ExtendStart;
}

void SettingWidget::on_lineEdit_FrontVBeamLeft_ExtendEnd_editingFinished() {
    settingPara->FrontVBeamLeft_ExtendEnd = ui->lineEdit_FrontVBeamLeft_ExtendEnd->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/FrontVBeamLeft_ExtendEnd", settingPara->FrontVBeamLeft_ExtendEnd);
    PLOGD << "FrontVBeamLeft_ExtendEnd: " << settingPara->FrontVBeamLeft_ExtendEnd;
}

void SettingWidget::on_lineEdit_FrontVBeamRight_ExtendStart_editingFinished() {
    settingPara->FrontVBeamRight_ExtendStart = ui->lineEdit_FrontVBeamRight_ExtendStart->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/FrontVBeamRight_ExtendStart", settingPara->FrontVBeamRight_ExtendStart);
    PLOGD << "FrontVBeamRight_ExtendStart: " << settingPara->FrontVBeamRight_ExtendStart;
}

void SettingWidget::on_lineEdit_FrontVBeamRight_ExtendEnd_editingFinished() {
    settingPara->FrontVBeamRight_ExtendEnd = ui->lineEdit_FrontVBeamRight_ExtendEnd->text().toDouble();
    settingPara->qSetting->setValue("SeamPosition/FrontVBeamRight_ExtendEnd", settingPara->FrontVBeamRight_ExtendEnd);
    PLOGD << "FrontVBeamRight_ExtendEnd: " << settingPara->FrontVBeamRight_ExtendEnd;
}

void SettingWidget::on_lineEdit_TSPFHStart_X_Shift_editingFinished() {
    settingPara->TubeSidePlatFilletStart_X = ui->lineEdit_TSPFHStart_X_Shift->text().toDouble();
    settingPara->qSetting->setValue("GFSeamPosition/TubeSidePlatFilletStart_X", settingPara->TubeSidePlatFilletStart_X);
    PLOGD << "TubeSidePlatFilletStart_X: " << settingPara->TubeSidePlatFilletStart_X;
}

void SettingWidget::on_lineEdit_TSPFHStart_Y_Shift_editingFinished() {
    settingPara->TubeSidePlatFilletStart_Y = ui->lineEdit_TSPFHStart_Y_Shift->text().toDouble();
    settingPara->qSetting->setValue("GFSeamPosition/TubeSidePlatFilletStart_Y", settingPara->TubeSidePlatFilletStart_Y);
    PLOGD << "TubeSidePlatFilletStart_Y: " << settingPara->TubeSidePlatFilletStart_Y;
}

void SettingWidget::on_lineEdit_TSPFHStart_Z_Shift_editingFinished() {
    settingPara->TubeSidePlatFilletStart_Z = ui->lineEdit_TSPFHStart_Z_Shift->text().toDouble();
    settingPara->qSetting->setValue("GFSeamPosition/TubeSidePlatFilletStart_Z", settingPara->TubeSidePlatFilletStart_Z);
    PLOGD << "TubeSidePlatFilletStart_Z: " << settingPara->TubeSidePlatFilletStart_Z;
}

void SettingWidget::on_lineEdit_TSPFHEnd_X_Shift_editingFinished() {
    settingPara->TubeSidePlatFilletEnd_X = ui->lineEdit_TSPFHEnd_X_Shift->text().toDouble();
    settingPara->qSetting->setValue("GFSeamPosition/TubeSidePlatFilletEnd_X", settingPara->TubeSidePlatFilletEnd_X);
    PLOGD << "TubeSidePlatFilletEnd_X: " << settingPara->TubeSidePlatFilletEnd_X;
}

void SettingWidget::on_lineEdit_TSPFHEnd_Y_Shift_editingFinished() {
    settingPara->TubeSidePlatFilletEnd_Y = ui->lineEdit_TSPFHEnd_Y_Shift->text().toDouble();
    settingPara->qSetting->setValue("GFSeamPosition/TubeSidePlatFilletEnd_Y", settingPara->TubeSidePlatFilletEnd_Y);
    PLOGD << "TubeSidePlatFilletEnd_Y: " << settingPara->TubeSidePlatFilletEnd_Y;
}

void SettingWidget::on_lineEdit_TSPFHEnd_Z_Shift_editingFinished() {
    settingPara->TubeSidePlatFilletEnd_Z = ui->lineEdit_TSPFHEnd_Z_Shift->text().toDouble();
    settingPara->qSetting->setValue("GFSeamPosition/TubeSidePlatFilletEnd_Z", settingPara->TubeSidePlatFilletEnd_Z);
    PLOGD << "TubeSidePlatFilletEnd_Z: " << settingPara->TubeSidePlatFilletEnd_Z;
}

void SettingWidget::on_lineEdit_TSPFHWeld_WithdrawDistance_editingFinished() {
    settingPara->TubeSidePlatFilletWithdrawDistance = ui->lineEdit_TSPFHWeld_WithdrawDistance->text().toDouble();
    settingPara->qSetting->setValue("GFSeamPosition/TubeSidePlatFilletWithdrawDistance", settingPara->TubeSidePlatFilletWithdrawDistance);
    PLOGD << "TubeSidePlatFilletWithdrawDistance: " << settingPara->TubeSidePlatFilletWithdrawDistance;
}

void SettingWidget::on_lineEdit_PPFHStart_X_Shift_editingFinished() {
    settingPara->PlatePlateFilletHorizontalStart_X = ui->lineEdit_PPFHStart_X_Shift->text().toDouble();
    settingPara->qSetting->setValue("GFSeamPosition/PlatePlateFilletHorizontalStart_X", settingPara->PlatePlateFilletHorizontalStart_X);
    PLOGD << "PlatePlateFilletHorizontalStart_X: " << settingPara->PlatePlateFilletHorizontalStart_X;
}

void SettingWidget::on_lineEdit_PPFHStart_Y_Shift_editingFinished() {
    settingPara->PlatePlateFilletHorizontalStart_Y = ui->lineEdit_PPFHStart_Y_Shift->text().toDouble();
    settingPara->qSetting->setValue("GFSeamPosition/PlatePlateFilletHorizontalStart_Y", settingPara->PlatePlateFilletHorizontalStart_Y);
    PLOGD << "PlatePlateFilletHorizontalStart_Y: " << settingPara->PlatePlateFilletHorizontalStart_Y;
}

void SettingWidget::on_lineEdit_PPFHStart_Z_Shift_editingFinished() {
    settingPara->PlatePlateFilletHorizontalStart_Z = ui->lineEdit_PPFHStart_Z_Shift->text().toDouble();
    settingPara->qSetting->setValue("GFSeamPosition/PlatePlateFilletHorizontalStart_Z", settingPara->PlatePlateFilletHorizontalStart_Z);
    PLOGD << "PlatePlateFilletHorizontalStart_Z: " << settingPara->PlatePlateFilletHorizontalStart_Z;
}

void SettingWidget::on_lineEdit_PPFHEnd_X_Shift_editingFinished() {
    settingPara->PlatePlateFilletHorizontalEnd_X = ui->lineEdit_PPFHEnd_X_Shift->text().toDouble();
    settingPara->qSetting->setValue("GFSeamPosition/PlatePlateFilletHorizontalEnd_X", settingPara->PlatePlateFilletHorizontalEnd_X);
    PLOGD << "PlatePlateFilletHorizontalEnd_X: " << settingPara->PlatePlateFilletHorizontalEnd_X;
}

void SettingWidget::on_lineEdit_PPFHEnd_Y_Shift_editingFinished() {
    settingPara->PlatePlateFilletHorizontalEnd_Y = ui->lineEdit_PPFHEnd_Y_Shift->text().toDouble();
    settingPara->qSetting->setValue("GFSeamPosition/PlatePlateFilletHorizontalEnd_Y", settingPara->PlatePlateFilletHorizontalEnd_Y);
    PLOGD << "PlatePlateFilletHorizontalEnd_Y: " << settingPara->PlatePlateFilletHorizontalEnd_Y;
}

void SettingWidget::on_lineEdit_PPFHEnd_Z_Shift_editingFinished() {
    settingPara->PlatePlateFilletHorizontalEnd_Z = ui->lineEdit_PPFHEnd_Z_Shift->text().toDouble();
    settingPara->qSetting->setValue("GFSeamPosition/PlatePlateFilletHorizontalEnd_Z", settingPara->PlatePlateFilletHorizontalEnd_Z);
    PLOGD << "PlatePlateFilletHorizontalEnd_Z: " << settingPara->PlatePlateFilletHorizontalEnd_Z;
}

void SettingWidget::on_lineEdit_PPFHWeld_WithdrawDistance_editingFinished() {
    settingPara->PlatePlateFilletHorizontalWithdrawDistance = ui->lineEdit_PPFHWeld_WithdrawDistance->text().toDouble();
    settingPara->qSetting->setValue("GFSeamPosition/PlatePlateFilletHorizontalWithdrawDistance",
                                    settingPara->PlatePlateFilletHorizontalWithdrawDistance);
    PLOGD << "PlatePlateFilletHorizontalWithdrawDistance: " << settingPara->PlatePlateFilletHorizontalWithdrawDistance;
}

void SettingWidget::on_lineEdit_PPFVStart_X_Shift_editingFinished() {
    settingPara->PlatePlateFilletVerticalStart_X = ui->lineEdit_PPFVStart_X_Shift->text().toDouble();
    settingPara->qSetting->setValue("GFSeamPosition/PlatePlateFilletVerticalStart_X", settingPara->PlatePlateFilletVerticalStart_X);
    PLOGD << "PlatePlateFilletVerticalStart_X: " << settingPara->PlatePlateFilletVerticalStart_X;
}

void SettingWidget::on_lineEdit_PPFVStart_Y_Shift_editingFinished() {
    settingPara->PlatePlateFilletVerticalStart_Y = ui->lineEdit_PPFVStart_Y_Shift->text().toDouble();
    settingPara->qSetting->setValue("GFSeamPosition/PlatePlateFilletVerticalStart_Y", settingPara->PlatePlateFilletVerticalStart_Y);
    PLOGD << "PlatePlateFilletVerticalStart_Y: " << settingPara->PlatePlateFilletVerticalStart_Y;
}

void SettingWidget::on_lineEdit_PPFVStart_Z_Shift_editingFinished() {
    settingPara->PlatePlateFilletVerticalStart_Z = ui->lineEdit_PPFVStart_Z_Shift->text().toDouble();
    settingPara->qSetting->setValue("GFSeamPosition/PlatePlateFilletVerticalStart_Z", settingPara->PlatePlateFilletVerticalStart_Z);
    PLOGD << "PlatePlateFilletVerticalStart_Z: " << settingPara->PlatePlateFilletVerticalStart_Z;
}

void SettingWidget::on_lineEdit_PPFVEnd_X_Shift_editingFinished() {
    settingPara->PlatePlateFilletVerticalEnd_X = ui->lineEdit_PPFVEnd_X_Shift->text().toDouble();
    settingPara->qSetting->setValue("GFSeamPosition/PlatePlateFilletVerticalEnd_X", settingPara->PlatePlateFilletVerticalEnd_X);
    PLOGD << "PlatePlateFilletVerticalEnd_X: " << settingPara->PlatePlateFilletVerticalEnd_X;
}

void SettingWidget::on_lineEdit_PPFVEnd_Y_Shift_editingFinished() {
    settingPara->PlatePlateFilletVerticalEnd_Y = ui->lineEdit_PPFVEnd_Y_Shift->text().toDouble();
    settingPara->qSetting->setValue("GFSeamPosition/PlatePlateFilletVerticalEnd_Y", settingPara->PlatePlateFilletVerticalEnd_Y);
    PLOGD << "PlatePlateFilletVerticalEnd_Y: " << settingPara->PlatePlateFilletVerticalEnd_Y;
}

void SettingWidget::on_lineEdit_PPFVEnd_Z_Shift_editingFinished() {
    settingPara->PlatePlateFilletVerticalEnd_Z = ui->lineEdit_PPFVEnd_Z_Shift->text().toDouble();
    settingPara->qSetting->setValue("GFSeamPosition/PlatePlateFilletVerticalEnd_Z", settingPara->PlatePlateFilletVerticalEnd_Z);
    PLOGD << "PlatePlateFilletVerticalEnd_Z: " << settingPara->PlatePlateFilletVerticalEnd_Z;
}

void SettingWidget::on_lineEdit_PPFVWeld_WithdrawDistance_editingFinished() {
    settingPara->PlatePlateFilletVerticalWithdrawDistance = ui->lineEdit_PPFVWeld_WithdrawDistance->text().toDouble();
    settingPara->qSetting->setValue("GFSeamPosition/PlatePlateFilletVerticalWithdrawDistance", settingPara->PlatePlateFilletVerticalWithdrawDistance);
    PLOGD << "PlatePlateFilletVerticalWithdrawDistance: " << settingPara->PlatePlateFilletVerticalWithdrawDistance;
}

void SettingWidget::on_lineEdit_TPFStartOffset_editingFinished() {
    settingPara->TubePlatFilletStartOffset = ui->lineEdit_TPFStartOffset->text().toDouble();
    settingPara->qSetting->setValue("GFSeamPosition/TubePlatFilletStartOffset", settingPara->TubePlatFilletStartOffset);
    PLOGD << "TubePlatFilletStartOffset: " << settingPara->TubePlatFilletStartOffset;
}

void SettingWidget::on_lineEdit_TPFEndOffset_editingFinished() {
    settingPara->TubePlatFilletEndOffset = ui->lineEdit_TPFEndOffset->text().toDouble();
    settingPara->qSetting->setValue("GFSeamPosition/TubePlatFilletEndOffset", settingPara->TubePlatFilletEndOffset);
    PLOGD << "TubePlatFilletEndOffset: " << settingPara->TubePlatFilletEndOffset;
}

void SettingWidget::on_lineEdit_TPFWeld_WithdrawDistance_editingFinished() {
    settingPara->TubePlatFilletWithdrawDistance = ui->lineEdit_TPFWeld_WithdrawDistance->text().toDouble();
    settingPara->qSetting->setValue("GFSeamPosition/TubePlatFilletWithdrawDistance", settingPara->TubePlatFilletWithdrawDistance);
    PLOGD << "TubePlatFilletWithdrawDistance: " << settingPara->TubePlatFilletWithdrawDistance;
}

void SettingWidget::on_lineEdit_WireCalibrationOffset_editingFinished() {
    settingPara->wireCalibrationOffset = ui->lineEdit_WireCalibrationOffset->text().toDouble();
    settingPara->qSetting->setValue("Welding/wireCalibrationOffset", settingPara->wireCalibrationOffset);
    PLOGD << "wireCalibrationOffset: " << settingPara->wireCalibrationOffset;
}

void SettingWidget::on_lineEdit_ToolRadius_editingFinished() {
    settingPara->toolRadius = ui->lineEdit_ToolRadius->text().toDouble();
    settingPara->qSetting->setValue("Welding/toolRadius", settingPara->toolRadius);
    PLOGD << "toolRadius: " << settingPara->toolRadius;
}
