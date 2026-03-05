#include "axis_manager.h"

#include <QDebug>

#include "PLCCommunication.h"

AxisManager::AxisManager(Axis axis, PLCCommunication *comm, QObject *parent)
    : QObject(parent), m_axis(axis), m_communication(comm) {
    m_previousCoilStatuses = QVector<bool>(32, false);
    switch (m_axis) {
        case Axis::X:
            axisName = u8"X轴";
            break;
        case Axis::Y:
            axisName = u8"Y轴";
            break;
        case Axis::Z:
            axisName = u8"Z轴";
            break;
        default:
            axisName = u8"未知轴";
    }
}

AxisManager::~AxisManager() {
    m_communication->writeCoils(addr(m_axis, RegB::ServoEnable), {false});
    // qDebug() << "AxisManager destroyed at" << QTime::currentTime().toString() << "thread =" << QThread::currentThread();
}

void AxisManager::setTimer() {
    if (!m_realTimer) {
        m_realTimer = new QTimer(this);
        connect(m_realTimer, &QTimer::timeout, this, &AxisManager::onRealTimeout);
    }

    m_realTimer->start(100);
}
void AxisManager::setAxisVel(float vel, int address) {
    QVector<quint16> values = floatToQuint16(vel);
    m_communication->writeRegisters(address, values);
}

void AxisManager::whenMove2AbsPosition(float pos, float vel) {
    if (!m_isEnable) {
        emit sendText(QString(axisName + u8"未使能"));
        return;
    }
    m_previousCoilStatuses[16] = false;
    m_absoluteMoveDone = false;
    m_absoluteMoveStart = true;
    // 设置位置
    setAxisVel(pos, addr(m_axis, RegR::AbsPosition));
    // 设置速度
    setAxisVel(vel, addr(m_axis, RegR::AbsSpeed));
    // 发送命令,此处加入重复触发逻辑
    int cmdReg = addr(m_axis, RegB::AbsPositionCommand);
    m_communication->writeCoils(cmdReg, {false});
    QThread::msleep(100);  // 短暂延时
    m_communication->writeCoils(cmdReg, {true});
}

void AxisManager::moveForward(float vel, bool checked) {
    if (!m_isEnable) {
        emit sendText(QString(axisName + u8"未使能"));
        return;
    }
    if (checked) setAxisVel(vel, addr(m_axis, RegR::JogSpeed));
    int cmdReg = addr(m_axis, RegB::JogForward);
    m_communication->writeCoils(cmdReg, {checked});
}

void AxisManager::moveReverse(float vel, bool checked) {
    if (!m_isEnable) {
        emit sendText(QString(axisName + u8"未使能"));
        return;
    }
    if (checked) setAxisVel(vel, addr(m_axis, RegR::JogSpeed));
    int cmdReg = addr(m_axis, RegB::JogReverse);
    m_communication->writeCoils(cmdReg, {checked});
}

void AxisManager::enableServo(bool enable) {
    int reg = addr(m_axis, RegB::ServoEnable);
    m_communication->writeCoils(reg, {enable});
    if (enable) {
        reset();
        setTimer();
        m_isEnable = true;
    } else {
        m_isEnable = false;
        if (m_realTimer && m_realTimer->isActive()) m_realTimer->stop();
    }
}

void AxisManager::reset() {
    // if (!m_isEnable) {
    //     emit sendText(QString(axisName + u8"未使能"));
    //     return;
    // }
    int reg = addr(m_axis, RegB::Reset);
    m_communication->writeCoils(reg, {true});
}
void AxisManager::setHome() {
    if (!m_isEnable) {
        emit sendText(QString(axisName + u8"未使能"));
        return;
    }
    int reg = addr(m_axis, RegB::HomeCommand);
    m_communication->writeCoils(reg, {true});
}
void AxisManager::stopSport(bool checked) {
    if (!m_isEnable) {
        emit sendText(QString(axisName + u8"未使能"));
        return;
    }
    int reg = addr(m_axis, RegB::Stop);
    m_communication->writeCoils(reg, {checked});
}
void AxisManager::immediateStop(bool checked) {
    if (!m_isEnable) {
        emit sendText(QString(axisName + u8"未使能"));
        return;
    }
    int reg = addr(m_axis, RegB::ImmediateStop);
    m_communication->writeCoils(reg, {checked});
}
void AxisManager::onRealTimeout() {
    // 读取当前位置和速度
    int posReg = addr(m_axis, RegD::CurrentPosition);
    int speedReg = addr(m_axis, RegD::CurrentSpeed);

    QVector<quint16> results;
    if (m_communication->readRegisters(posReg, 2, results)) {
        m_currentPosition = quint16ToFloat(results);
    }

    if (m_communication->readRegisters(speedReg, 2, results)) {
        m_currentSpeed = quint16ToFloat(results);
    }
    emit sendPositionAndSpeed(m_currentPosition, m_currentSpeed);
    // 读取线圈状态
    int coilStart = addr(m_axis, RegS::PosLimitSignal);
    QVector<bool> coilStatus;
    if (m_communication->readCoils(coilStart, 32, coilStatus)) {
        processCoilStatus(coilStatus);
    }
}

void AxisManager::processCoilStatus(const QVector<bool> &status) {
    for (int i = 0; i < status.size() && i < m_previousCoilStatuses.size(); ++i) {
        if (status[i] && !m_previousCoilStatuses[i]) {
            sendAxisState(i);
            m_previousCoilStatuses[i] = status[i];
        } else if (!status[i]) {
            m_previousCoilStatuses[i] = status[i];
        }
    }
}

QVector<quint16> AxisManager::floatToQuint16(float value) {
    union {
        float f;
        quint16 u[2];
    } data;
    data.f = value;
    return QVector<quint16>{data.u[0], data.u[1]};
}

float AxisManager::quint16ToFloat(const QVector<quint16> &values) {
    if (values.size() < 2) return 0.0f;

    union {
        float f;
        quint16 u[2];
    } data;
    data.u[0] = values[0];
    data.u[1] = values[1];
    return data.f;
}

void AxisManager::sendAxisState(int coilIndex) {
    QString message;
    QString messageErro;
    QString messageAxis;
    QString messageMotion;

    // 根据线圈状态生成相应的消息
    switch (coilIndex) {
        // 轴状态 (Axis Status)
        case 0:
            messageAxis = axisName + u8"到达正极限位";
            break;
        case 2:
            messageAxis = axisName + u8"到达负极限位";
            break;
        case 12:
            messageAxis = axisName + u8"相对定位执行中";
            break;
        case 13:
            messageAxis = axisName + u8"相对定位完成保持";
            break;
        case 15:
            messageAxis = axisName + u8"绝对定位执行中";
            break;
        case 16:
            messageAxis = axisName + u8"绝对定位完成保持";
            if (m_absoluteMoveStart) {
                emit absoluteMoveFinished();
                m_absoluteMoveDone = true;
                m_absoluteMoveStart = false;
            }
            break;
        case 18:
            messageAxis = axisName + u8"回原执行中";
            break;
        case 19:
            messageAxis = axisName + u8"回原完成保持";
            break;
        case 21:
            messageAxis = axisName + u8"点动运行中";
            break;
        case 27:
            messageAxis = axisName + u8"去使能状态";
            break;
        case 28:
            messageAxis = axisName + u8"使能非运行";
            break;

            // 运动状态 (Motion Status)
        case 29:
            messageMotion = axisName + u8"恒速运动（或为零）";
            break;
        case 30:
            messageMotion = axisName + u8"加速运动";
            break;
        case 31:
            messageMotion = axisName + u8"减速运动";
            break;

            // 状态推送 (States Sender)
        case 4:
            message = axisName + u8"到达原点";
            break;
        case 5:
            message = axisName + u8"使能完成";
            break;
        case 6:
            message = axisName + u8"停止完成";
            break;
        case 7:
            message = axisName + u8"复位完成";
            m_communication->writeCoils(addr(m_axis, RegB::Reset), {false});
            break;
        case 8:
            message = axisName + u8"在当前绝对定位位置";
            break;
        case 14:
            message = axisName + u8"相对定位完成";
            break;
        case 17:
            message = axisName + u8"绝对定位完成";
            break;
        case 20:
            message = axisName + u8"回原完成";
            break;
        case 22:
            message = axisName + u8"急停完成";
            break;
            // 轴报警 (Axis Alarms)
        case 1:
            messageErro = axisName + u8"正限位报警";
            emit axisError(messageErro);
            break;
        case 3:
            messageErro = axisName + u8"负限位报警";
            emit axisError(messageErro);
            break;
        case 9:
            messageErro = axisName + u8"驱动器报警";
            emit axisError(messageErro);
            break;
        case 10:
            messageErro = axisName + u8"轴故障中";
            emit axisError(messageErro);
            break;
        case 23:
            messageErro = axisName + u8"急停错误";
            emit axisError(messageErro);
            break;
        case 24:
            message = axisName + u8"运动超调有效";
            break;
        case 25:
            message = axisName + u8"运动超调忙";
            break;
        case 26:
            messageErro = axisName + u8"运动超调故障";
            emit axisError(messageErro);
            break;
    }
    emit sendTextState(messageAxis, messageMotion);

    if (!message.isEmpty()) {
        emit sendText(message);
    }
}
