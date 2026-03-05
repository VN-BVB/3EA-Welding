#include "RailWidget.h"

#include <QTextCodec>

#include "axis_manager.h"
#include "PLCCommunication.h"
#include "ui_RailWidget.h"
// #pragma execution_character_set("utf-8")

RailWidget::RailWidget(QWidget *parent) : QWidget(parent), ui(new Ui::RailWidget) {
    ui->setupUi(this);

    qRegisterMetaType<QString>("QString");
    qRegisterMetaType<QVector<bool>>("QVector<bool>");
    qRegisterMetaType<QVector<quint16>>("QVector<quint16>");
    // 可学习QMetaObject代替
    ui->label_X_CurrentPosition->setStyleSheet("background-color: white; border: 1px solid black;");
    ui->label_X_CurrentSpeed->setStyleSheet("background-color: white; border: 1px solid black;");
    // 初始化通信层
    m_communication_ = std::make_unique<PLCCommunication>();
    m_communication_->moveToThread(m_commThread);
    // 初始化三轴管理器
    m_xAxis = std::make_unique<AxisManager>(Axis::X, m_communication_.get());

    // 为每个轴分配独立线程

    m_xAxis->moveToThread(m_xAxisThread);
    // m_yAxis->moveToThread(m_yAxisThread);
    // m_zAxis->moveToThread(m_zAxisThread);
    m_commThread->start();
    m_xAxisThread->start();
    // m_yAxisThread->start();
    // m_zAxisThread->start();
    // 通信层
    connect(m_communication_.get(), &PLCCommunication::sendText, this, &RailWidget::whenAppendTextLog);
    connect(m_communication_.get(), &PLCCommunication::errorOccurred, this, &RailWidget::whenAppendErrorLog);
    connect(m_communication_.get(), &PLCCommunication::connectionStatusChanged, m_xAxis.get(), &AxisManager::enableServo);
    connect(this, &RailWidget::sendConnectToPLC, m_communication_.get(), &PLCCommunication::whenConnectToPLC);
    connect(this, &RailWidget::sendDisconnectToPLC, m_communication_.get(), &PLCCommunication::whenDisconnectFromPLC);
    // connect(m_communication, &PLCCommunication::sendPLCConnected, m_yAxis, &AxisManager::enableServo);
    // connect(m_communication, &PLCCommunication::sendPLCConnected, m_zAxis, &AxisManager::enableServo);
    // ---轴组---
    // X轴
    connect(m_xAxis.get(), &AxisManager::axisError, this, &RailWidget::whenAppendErrorLog);
    connect(m_xAxis.get(), &AxisManager::sendTextState, this, &RailWidget::whenUpdateAMState);
    connect(m_xAxis.get(), &AxisManager::sendText, this, &RailWidget::whenAppendTextLog);
    connect(m_xAxis.get(), &AxisManager::sendPositionAndSpeed, this, &RailWidget::whenUpdatePositionAndSpeed);
    connect(this, &RailWidget::sendMove2AbsPosition, m_xAxis.get(), &AxisManager::whenMove2AbsPosition);  // 绝对位置

    // connect(this, &RailWidget::sendWriteRegisters, rail, &Rail::writeRegisters);

    // connect(this, &RailWidget::sendForward, rail, &Rail::whenForward);
    // connect(this, &RailWidget::sendReverse, rail, &Rail::whenReverse);
}

RailWidget::~RailWidget() { this->disconnectRail(); }

// 设置绝对运动位置框
void RailWidget::setEditAbsPosition(QString position) { ui->edit_X_AbsPosition->setText(position); }

// 设置速度框
void RailWidget::setEditSpeed(QString speed) { ui->edit_X_AbsSpeed->setText(speed); }
// 获取当前位置
double RailWidget::getCurrentXPosition() const { return ui->label_X_CurrentPosition->text().toDouble(); }

// 在信息框推送信息
void RailWidget::whenAppendTextLog(const QString message) { ui->textEdit->append(message); }
void RailWidget::whenAppendErrorLog(const QString message) {
    ui->textEdit->setTextColor(Qt::red);
    ui->textEdit->append(message);
    ui->textEdit->setTextColor(Qt::black);  // 还原
}

// 更新轴和运动状态信息
void RailWidget::whenUpdateAMState(const QString messageAxis, const QString messageMotion) {
    // 检查 messageAxis 是否为空，若非空则更新 edit_AxisState
    if (!messageAxis.isEmpty()) {
        ui->edit_AxisState->clear();
        ui->edit_AxisState->append(messageAxis);
    }

    // 检查 messageMotion 是否为空，若非空则更新 edit_MotionState
    if (!messageMotion.isEmpty()) {
        ui->edit_MotionState->clear();
        ui->edit_MotionState->append(messageMotion);
    }
}

// 更新地轨当前位置和速度
void RailWidget::whenUpdatePositionAndSpeed(float position, float speed) {
    if (!std::isnan(position) && !std::isnan(speed)) {
        // 处理数据并更新 UI
        ui->label_X_CurrentPosition->setText(QString::number(position, 'f', 3));
        ui->label_X_CurrentSpeed->setText(QString::number(speed, 'f', 3));
    } else {
        ui->label_X_CurrentPosition->setText("Read failed!");
        ui->label_X_CurrentSpeed->setText("Read failed!");
    }
}

// --------------------------------------- 按钮调用 --------------------------------------------
// 连接PLC按钮点击事件（利用协议层的信号来触发轴组使能与定时器）
void RailWidget::connectRail() { emit sendConnectToPLC(ip, port); }
// 断开PLC连接按钮点击事件
void RailWidget::disconnectRail() {
    // 先禁用所有轴的伺服（安全措施）
    QMetaObject::invokeMethod(m_xAxis.get(), [=]() { m_xAxis->enableServo(false); }, Qt::BlockingQueuedConnection);
    // m_yAxis->enableServo(false);
    // m_zAxis->enableServo(false);
    // 断开PLC连接
    emit sendDisconnectToPLC();

    // 更新状态信息
    whenUpdateAMState(u8"断开连接", u8"断开连接");
}
// 地轨回归原点按钮点击事件
void RailWidget::on_btn_regressOrigin_clicked() {
    QMetaObject::invokeMethod(m_xAxis.get(), &AxisManager::setHome, Qt::QueuedConnection);
}

// 绝对位置运动按钮
void RailWidget::on_btn_X_AbsPositionCommand_clicked() {
    float v = ui->edit_X_AbsSpeed->text().toFloat();
    emit sendMove2AbsPosition(ui->edit_X_AbsPosition->text().toFloat(), ui->edit_X_AbsSpeed->text().toFloat());
}

// 运动绝对位置滑块
void RailWidget::on_horizontalSlider_X_AbsPosition_sliderMoved(int val) { ui->edit_X_AbsPosition->setText(QString::number(val)); }

// 运动速度滑块
void RailWidget::on_horizontalSlider_X_AbsSpeed_sliderMoved(int val) { ui->edit_X_AbsSpeed->setText(QString::number(val)); }

// 地轨停止按钮状态切换事件
void RailWidget::on_chk_Stop_toggled(bool checked) {
    QMetaObject::invokeMethod(m_xAxis.get(), [=]() { m_xAxis->stopSport(checked); }, Qt::QueuedConnection);
}

// 地轨重置按钮状态切换事件
void RailWidget::on_btn_chk_Rest_clicked() {
    QMetaObject::invokeMethod(m_xAxis.get(), &AxisManager::reset, Qt::QueuedConnection);
}

// 地轨紧急停止按钮状态切换事件
void RailWidget::on_chk_ImmediateStop_toggled(bool checked) {
    QMetaObject::invokeMethod(m_xAxis.get(), [=]() { m_xAxis->immediateStop(checked); }, Qt::QueuedConnection);
}

// 地轨正向点动按钮按下事件
void RailWidget::on_btn_X_JogForward_pressed() {
    float v = ui->edit_X_AbsSpeed->text().toFloat();
    QMetaObject::invokeMethod(m_xAxis.get(), [=]() { m_xAxis->moveForward(v, true); }, Qt::QueuedConnection);
}

// 地轨正向点动按钮释放事件
void RailWidget::on_btn_X_JogForward_released() {
    float v = ui->edit_X_AbsSpeed->text().toFloat();
    QMetaObject::invokeMethod(m_xAxis.get(), [=]() { m_xAxis->moveForward(v, false); }, Qt::QueuedConnection);
}

// 地轨反向点动按钮按下事件
void RailWidget::on_btn_X_JogReverse_pressed() {
    float v = ui->edit_X_AbsSpeed->text().toFloat();
    QMetaObject::invokeMethod(m_xAxis.get(), [=]() { m_xAxis->moveReverse(v, true); }, Qt::QueuedConnection);
}

// 地轨反向点动按钮释放事件
void RailWidget::on_btn_X_JogReverse_released() {
    float v = ui->edit_X_AbsSpeed->text().toFloat();
    QMetaObject::invokeMethod(m_xAxis.get(), [=]() { m_xAxis->moveReverse(v, false); }, Qt::QueuedConnection);
}
void RailWidget::on_btn_contectRail_clicked() { connectRail(); }

void RailWidget::on_btn_discontectRail_clicked() { disconnectRail(); }
