#include "RailWidget.h"

#include <QTextCodec>

#include "AbstractAxisFactory.h"
#include "PLCCommunication.h"
#include "concrete_axis/AxisManager.h"
#include "ui_RailWidget.h"
// #pragma execution_character_set("utf-8")

RailWidget::RailWidget(QWidget* parent) : QWidget(parent), ui(new Ui::RailWidget) {
    ui->setupUi(this);

    qRegisterMetaType<QString>("QString");
    qRegisterMetaType<Axis>("Axis");
    qRegisterMetaType<QVector<bool>>("QVector<bool>");
    qRegisterMetaType<QVector<quint16>>("QVector<quint16>");
    qRegisterMetaType<std::shared_ptr<AbstractAxis>>("std::shared_ptr<AbstractAxis>");
    // 可学习QMetaObject代替
    ui->label_X_CurrentPosition->setStyleSheet("background-color: white; border: 1px solid black;");
    ui->label_X_CurrentSpeed->setStyleSheet("background-color: white; border: 1px solid black;");
    ui->label_Y_CurrentPosition->setStyleSheet("background-color: white; border: 1px solid black;");
    ui->label_Y_CurrentSpeed->setStyleSheet("background-color: white; border: 1px solid black;");
    ui->label_Z_CurrentPosition->setStyleSheet("background-color: white; border: 1px solid black;");
    ui->label_Z_CurrentSpeed->setStyleSheet("background-color: white; border: 1px solid black;");
    ui->chk_ImmediateStop->setStyleSheet("QCheckBox { color: red; }");
    // 初始化通信层
    m_communication_ = std::make_unique<PLCCommunication>();
    m_communication_->moveToThread(m_commThread);
    for (Axis axis : axisList) {
        m_axes[axis] = AbstractAxisFactory::createAxis(axis, m_communication_.get());
    }
    // 为每个轴分配独立线程
    m_axes[Axis::X]->moveToThread(m_xAxisThread);
    m_axes[Axis::Y]->moveToThread(m_yAxisThread);
    m_axes[Axis::Z]->moveToThread(m_zAxisThread);
    m_commThread->start();
    m_xAxisThread->start();
    m_yAxisThread->start();
    m_zAxisThread->start();
    // 通信层
    connect(m_communication_.get(), &PLCCommunication::sendText, this, &RailWidget::whenAppendTextLog);
    connect(m_communication_.get(), &PLCCommunication::errorOccurred, this, &RailWidget::whenAppendErrorLog);
    for (const auto& pair : m_axes) {
        auto ptr = pair.second;
        connect(m_communication_.get(), &PLCCommunication::connectionStatusChanged, ptr.get(), &AbstractAxis::enableServo);
    }
    connect(this, &RailWidget::sendConnectToPLC, m_communication_.get(), &PLCCommunication::whenConnectToPLC);
    connect(this, &RailWidget::sendDisconnectToPLC, m_communication_.get(), &PLCCommunication::whenDisconnectFromPLC);
    // ---轴组---
    for (const auto& pair : m_axes) {
        auto axisType = pair.first;
        auto axisPtr = pair.second;

        connect(axisPtr.get(), &AbstractAxis::axisError, this, &RailWidget::whenAppendErrorLog);

        connect(axisPtr.get(), &AbstractAxis::sendTextState, this, [this, axisType](const QString& messageAxis, const QString& messageMotion) {
            whenUpdateAMState(messageAxis, messageMotion, axisType);
        });

        connect(axisPtr.get(), &AbstractAxis::sendText, this, &RailWidget::whenAppendTextLog);

        connect(axisPtr.get(), &AbstractAxis::sendPositionAndSpeed, this,
                [this, axisType](float position, float speed) { whenUpdatePositionAndSpeed(position, speed, axisType); });
    }
    connect(this, &RailWidget::sendMove2AbsPosition, m_axes[Axis::X].get(), &AbstractAxis::whenMove2AbsPosition);  // 绝对位置
}

RailWidget::~RailWidget() {
    QObject::disconnect(nullptr, nullptr, this, nullptr);
    QObject::disconnect(this, nullptr, nullptr, nullptr);

    if (m_communication_) {
        QObject::disconnect(m_communication_.get(), nullptr, nullptr, nullptr);
    }

    for (const auto& pair : m_axes) {
        const auto& ptr = pair.second;
        if (!ptr) {
            continue;
        }

        QObject::disconnect(ptr.get(), nullptr, nullptr, nullptr);

        QThread* axisThread = ptr->thread();
        if (axisThread && axisThread != QThread::currentThread() && axisThread->isRunning()) {
            QMetaObject::invokeMethod(ptr.get(), [ptr]() { ptr->enableServo(false); }, Qt::BlockingQueuedConnection);
        } else {
            ptr->enableServo(false);
        }
    }

    if (m_communication_) {
        QThread* commThread = m_communication_->thread();
        if (commThread && commThread != QThread::currentThread() && commThread->isRunning()) {
            QMetaObject::invokeMethod(
                m_communication_.get(), [communication = m_communication_.get()]() { communication->whenDisconnectFromPLC(); },
                Qt::BlockingQueuedConnection);
        } else {
            m_communication_->whenDisconnectFromPLC();
        }
    }
}

// 设置绝对运动位置框
PLCCommunication* RailWidget::communication() const { return m_communication_.get(); }

const std::unordered_map<Axis, std::shared_ptr<AbstractAxis>>& RailWidget::axes() const { return m_axes; }

void RailWidget::setEditAbsPosition(QString position, Axis axis) {
    switch (axis) {
        case Axis::X:
            ui->edit_X_AbsPosition->setText(position);
            break;
        case Axis::Y:
            ui->edit_Y_AbsPosition->setText(position);
            break;
        case Axis::Z:
            ui->edit_Z_AbsPosition->setText(position);
            break;
    }
}

// 设置速度框
void RailWidget::setEditSpeed(QString speed, Axis axis) {
    switch (axis) {
        case Axis::X:
            ui->edit_X_AbsSpeed->setText(speed);
            break;
        case Axis::Y:
            ui->edit_Y_AbsSpeed->setText(speed);
            break;
        case Axis::Z:
            ui->edit_Z_AbsSpeed->setText(speed);
            break;
    }
}

// 获取当前位置
double RailWidget::getCurrentPosition(Axis axis) const {
    switch (axis) {
        case Axis::X:
            return ui->label_X_CurrentPosition->text().toDouble();
        case Axis::Y:
            return ui->label_Y_CurrentPosition->text().toDouble();
        case Axis::Z:
            return ui->label_Z_CurrentPosition->text().toDouble();
        default:
            return 0.0;
    }
}

// 在信息框推送信息
void RailWidget::whenAppendTextLog(const QString message) { ui->textEdit->append(message); }
void RailWidget::whenAppendErrorLog(const QString message) {
    ui->textEdit->setTextColor(Qt::red);
    ui->textEdit->append(message);
    ui->textEdit->setTextColor(Qt::black);  // 还原
}
// 更新三轴状态的重载函数
void RailWidget::whenUpdateAMState(const QString messageAxis, const QString messageMotion, Axis axis) {
    // 检查 messageAxis 是否为空，若非空则更新对应轴的状态
    if (!messageAxis.isEmpty()) {
        switch (axis) {
            case Axis::X:
                ui->edit_X_AxisState->clear();
                ui->edit_X_AxisState->append(messageAxis);
                break;
            case Axis::Y:
                ui->edit_Y_AxisState->clear();
                ui->edit_Y_AxisState->append(messageAxis);
                break;
            case Axis::Z:
                ui->edit_Z_AxisState->clear();
                ui->edit_Z_AxisState->append(messageAxis);
                break;
        }
    }

    // 检查 messageMotion 是否为空，若非空则更新对应轴的运动状态
    if (!messageMotion.isEmpty()) {
        switch (axis) {
            case Axis::X:
                ui->edit_X_MotionState->clear();
                ui->edit_X_MotionState->append(messageMotion);
                break;
            case Axis::Y:
                ui->edit_Y_MotionState->clear();
                ui->edit_Y_MotionState->append(messageMotion);
                break;
            case Axis::Z:
                ui->edit_Z_MotionState->clear();
                ui->edit_Z_MotionState->append(messageMotion);
                break;
        }
    }
}
// 更新三轴位置和速度的重载函数
void RailWidget::whenUpdatePositionAndSpeed(float position, float speed, Axis axis) {
    if (!std::isnan(position) && !std::isnan(speed)) {
        switch (axis) {
            case Axis::X:
                ui->label_X_CurrentPosition->setText(QString::number(position, 'f', 3));
                ui->label_X_CurrentSpeed->setText(QString::number(speed, 'f', 3));
                break;
            case Axis::Y:
                ui->label_Y_CurrentPosition->setText(QString::number(position, 'f', 3));
                ui->label_Y_CurrentSpeed->setText(QString::number(speed, 'f', 3));
                break;
            case Axis::Z:
                ui->label_Z_CurrentPosition->setText(QString::number(position, 'f', 3));
                ui->label_Z_CurrentSpeed->setText(QString::number(speed, 'f', 3));
                break;
        }
    } else {
        switch (axis) {
            case Axis::X:
                ui->label_X_CurrentPosition->setText("Read failed!");
                ui->label_X_CurrentSpeed->setText("Read failed!");
                break;
            case Axis::Y:
                ui->label_Y_CurrentPosition->setText("Read failed!");
                ui->label_Y_CurrentSpeed->setText("Read failed!");
                break;
            case Axis::Z:
                ui->label_Z_CurrentPosition->setText("Read failed!");
                ui->label_Z_CurrentSpeed->setText("Read failed!");
                break;
        }
    }
}

// --------------------------------------- 按钮调用 --------------------------------------------
// 连接PLC按钮点击事件（利用协议层的信号来触发轴组使能与定时器）
void RailWidget::connectRail() { emit sendConnectToPLC(ip, port); }
// 断开PLC连接按钮点击事件
void RailWidget::disconnectRail() {
    // 先禁用所有轴的伺服（安全措施）
    for (const auto& pair : m_axes) {
        auto ptr = pair.second;
        QMetaObject::invokeMethod(ptr.get(), [=]() { ptr->enableServo(false); }, Qt::BlockingQueuedConnection);
    }

    // 断开PLC连接
    emit sendDisconnectToPLC();

    // 更新状态信息
    whenUpdateAMState(u8"断开连接", u8"断开连接", Axis::X);
    whenUpdateAMState(u8"断开连接", u8"断开连接", Axis::Y);
    whenUpdateAMState(u8"断开连接", u8"断开连接", Axis::Z);
}
void RailWidget::on_btn_contectRail_clicked() { connectRail(); }

void RailWidget::on_btn_discontectRail_clicked() { disconnectRail(); }
// 地轨紧急停止按钮状态切换事件
void RailWidget::on_chk_ImmediateStop_toggled(bool checked) {
    // 对所有轴执行急停操作
    for (const auto& pair : m_axes) {
        auto ptr = pair.second;
        QMetaObject::invokeMethod(ptr.get(), [=]() { ptr->immediateStop(checked); }, Qt::QueuedConnection);
    }
}
// 地轨重置按钮状态切换事件
void RailWidget::on_btn_chk_Rest_clicked() {
    for (const auto& pair : m_axes) {
        auto ptr = pair.second;
        QMetaObject::invokeMethod(ptr.get(), &AbstractAxis::reset, Qt::QueuedConnection);
    }
}

// 地轨回归原点按钮点击事件
void RailWidget::on_btn_X_regressOrigin_clicked() { QMetaObject::invokeMethod(m_axes[Axis::X].get(), &AbstractAxis::setHome, Qt::QueuedConnection); }
// 绝对位置运动按钮
void RailWidget::on_btn_X_AbsPositionCommand_clicked() {
    float v = ui->edit_X_AbsSpeed->text().toFloat();
    emit sendMove2AbsPosition(ui->edit_X_AbsSpeed->text().toFloat(), ui->edit_X_AbsPosition->text().toFloat());
}
// 运动绝对位置滑块
void RailWidget::on_horizontalSlider_X_AbsPosition_sliderMoved(int val) { ui->edit_X_AbsPosition->setText(QString::number(val)); }
// 运动速度滑块
void RailWidget::on_horizontalSlider_X_AbsSpeed_sliderMoved(int val) { ui->edit_X_AbsSpeed->setText(QString::number(val)); }
// 地轨停止按钮状态切换事件
void RailWidget::on_chk_X_Stop_toggled(bool checked) {
    QMetaObject::invokeMethod(m_axes[Axis::X].get(), [=]() { m_axes[Axis::X]->stopSport(checked); }, Qt::QueuedConnection);
}
// 地轨正向点动按钮按下事件
void RailWidget::on_btn_X_JogForward_pressed() {
    float v = ui->edit_X_AbsSpeed->text().toFloat();
    QMetaObject::invokeMethod(m_axes[Axis::X].get(), [=]() { m_axes[Axis::X]->moveForward(v, true); }, Qt::QueuedConnection);
}

// 地轨正向点动按钮释放事件
void RailWidget::on_btn_X_JogForward_released() {
    float v = ui->edit_X_AbsSpeed->text().toFloat();
    QMetaObject::invokeMethod(m_axes[Axis::X].get(), [=]() { m_axes[Axis::X]->moveForward(v, false); }, Qt::QueuedConnection);
}

// 地轨反向点动按钮按下事件
void RailWidget::on_btn_X_JogReverse_pressed() {
    float v = ui->edit_X_AbsSpeed->text().toFloat();
    QMetaObject::invokeMethod(m_axes[Axis::X].get(), [=]() { m_axes[Axis::X]->moveReverse(v, true); }, Qt::QueuedConnection);
}

// 地轨反向点动按钮释放事件
void RailWidget::on_btn_X_JogReverse_released() {
    float v = ui->edit_X_AbsSpeed->text().toFloat();
    QMetaObject::invokeMethod(m_axes[Axis::X].get(), [=]() { m_axes[Axis::X]->moveReverse(v, false); }, Qt::QueuedConnection);
}
// --------------------------------------- Y轴按钮事件 --------------------------------------------
// Y轴回归原点按钮点击事件
void RailWidget::on_btn_Y_regressOrigin_clicked() { QMetaObject::invokeMethod(m_axes[Axis::Y].get(), &AbstractAxis::setHome, Qt::QueuedConnection); }

// Y轴绝对位置运动按钮
void RailWidget::on_btn_Y_AbsPositionCommand_clicked() {
    float v = ui->edit_Y_AbsSpeed->text().toFloat();
    float p = ui->edit_Y_AbsPosition->text().toFloat();
    QMetaObject::invokeMethod(m_axes[Axis::Y].get(), [=]() { m_axes[Axis::Y]->whenMove2AbsPosition(v, p); }, Qt::QueuedConnection);
}

// Y轴运动绝对位置滑块
void RailWidget::on_horizontalSlider_Y_AbsPosition_sliderMoved(int val) { ui->edit_Y_AbsPosition->setText(QString::number(val)); }

// Y轴运动速度滑块
void RailWidget::on_horizontalSlider_Y_AbsSpeed_sliderMoved(int val) { ui->edit_Y_AbsSpeed->setText(QString::number(val)); }

// Y轴停止按钮状态切换事件
void RailWidget::on_chk_Y_Stop_toggled(bool checked) {
    QMetaObject::invokeMethod(m_axes[Axis::Y].get(), [=]() { m_axes[Axis::Y]->stopSport(checked); }, Qt::QueuedConnection);
}

// Y轴正向点动按钮按下事件
void RailWidget::on_btn_Y_JogForward_pressed() {
    float v = ui->edit_Y_AbsSpeed->text().toFloat();
    QMetaObject::invokeMethod(m_axes[Axis::Y].get(), [=]() { m_axes[Axis::Y]->moveForward(v, true); }, Qt::QueuedConnection);
}

// Y轴正向点动按钮释放事件
void RailWidget::on_btn_Y_JogForward_released() {
    float v = ui->edit_Y_AbsSpeed->text().toFloat();
    QMetaObject::invokeMethod(m_axes[Axis::Y].get(), [=]() { m_axes[Axis::Y]->moveForward(v, false); }, Qt::QueuedConnection);
}

// Y轴反向点动按钮按下事件
void RailWidget::on_btn_Y_JogReverse_pressed() {
    float v = ui->edit_Y_AbsSpeed->text().toFloat();
    QMetaObject::invokeMethod(m_axes[Axis::Y].get(), [=]() { m_axes[Axis::Y]->moveReverse(v, true); }, Qt::QueuedConnection);
}

// Y轴反向点动按钮释放事件
void RailWidget::on_btn_Y_JogReverse_released() {
    float v = ui->edit_Y_AbsSpeed->text().toFloat();
    QMetaObject::invokeMethod(m_axes[Axis::Y].get(), [=]() { m_axes[Axis::Y]->moveReverse(v, false); }, Qt::QueuedConnection);
}

// --------------------------------------- Z轴按钮事件 --------------------------------------------
// Z轴回归原点按钮点击事件
void RailWidget::on_btn_Z_regressOrigin_clicked() { QMetaObject::invokeMethod(m_axes[Axis::Z].get(), &AbstractAxis::setHome, Qt::QueuedConnection); }

// Z轴绝对位置运动按钮
void RailWidget::on_btn_Z_AbsPositionCommand_clicked() {
    float v = ui->edit_Z_AbsSpeed->text().toFloat();
    float p = ui->edit_Z_AbsPosition->text().toFloat();
    QMetaObject::invokeMethod(m_axes[Axis::Z].get(), [=]() { m_axes[Axis::Z]->whenMove2AbsPosition(v, p); }, Qt::QueuedConnection);
}

// Z轴运动绝对位置滑块
void RailWidget::on_horizontalSlider_Z_AbsPosition_sliderMoved(int val) { ui->edit_Z_AbsPosition->setText(QString::number(val)); }

// Z轴运动速度滑块
void RailWidget::on_horizontalSlider_Z_AbsSpeed_sliderMoved(int val) { ui->edit_Z_AbsSpeed->setText(QString::number(val)); }

// Z轴停止按钮状态切换事件
void RailWidget::on_chk_Z_Stop_toggled(bool checked) {
    QMetaObject::invokeMethod(m_axes[Axis::Z].get(), [=]() { m_axes[Axis::Z]->stopSport(checked); }, Qt::QueuedConnection);
}

// Z轴正向点动按钮按下事件
void RailWidget::on_btn_Z_JogForward_pressed() {
    float v = ui->edit_Z_AbsSpeed->text().toFloat();
    QMetaObject::invokeMethod(m_axes[Axis::Z].get(), [=]() { m_axes[Axis::Z]->moveForward(v, true); }, Qt::QueuedConnection);
}

// Z轴正向点动按钮释放事件
void RailWidget::on_btn_Z_JogForward_released() {
    float v = ui->edit_Z_AbsSpeed->text().toFloat();
    QMetaObject::invokeMethod(m_axes[Axis::Z].get(), [=]() { m_axes[Axis::Z]->moveForward(v, false); }, Qt::QueuedConnection);
}

// Z轴反向点动按钮按下事件
void RailWidget::on_btn_Z_JogReverse_pressed() {
    float v = ui->edit_Z_AbsSpeed->text().toFloat();
    QMetaObject::invokeMethod(m_axes[Axis::Z].get(), [=]() { m_axes[Axis::Z]->moveReverse(v, true); }, Qt::QueuedConnection);
}

// Z轴反向点动按钮释放事件
void RailWidget::on_btn_Z_JogReverse_released() {
    float v = ui->edit_Z_AbsSpeed->text().toFloat();
    QMetaObject::invokeMethod(m_axes[Axis::Z].get(), [=]() { m_axes[Axis::Z]->moveReverse(v, false); }, Qt::QueuedConnection);
}
