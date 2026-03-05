#ifndef AXIS_MANAGER_H
#define AXIS_MANAGER_H
#include <QObject>
#include <QTime>

#include "axis_register.h"

class PLCCommunication;

class AxisManager : public QObject {
    Q_OBJECT
public:
    explicit AxisManager(Axis axis, PLCCommunication *comm, QObject *parent = nullptr);
    ~AxisManager();

    // 运动控制
    void setAxisVel(float vel, int address);
    void whenMove2AbsPosition(float vel, float pos);
    void moveForward(float vel, bool checked);
    void moveReverse(float vel, bool checked);
    void enableServo(bool enable);
    void reset();
    void setHome();
    void stopSport(bool checked);
    void immediateStop(bool checked);
    // 获取状态
    void setTimer();
    float getCurrentPosition() const { return m_currentPosition; }
    float getCurrentSpeed() const { return m_currentSpeed; }
    Axis getAxis() const { return m_axis; }

signals:
    void sendText(const QString &state);
    void sendTextState(QString messageAxis, QString messageMotion);
    void axisError(const QString &errorMsg);
    void sendPositionAndSpeed(float position, float speed);
    void absoluteMoveFinished();

private slots:
    void onRealTimeout();
    void processCoilStatus(const QVector<bool> &status);

private:
    Axis m_axis;
    QString axisName;
    PLCCommunication *m_communication;
    QTimer *m_realTimer = nullptr;

    // 状态变量
    bool m_isEnable = false;
    float m_currentPosition = 0.0f;
    float m_currentSpeed = 0.0f;
    bool m_absoluteMoveDone = true;
    bool m_absoluteMoveStart = false;

    QVector<bool> m_previousCoilStatuses;

    // 辅助方法
    QVector<quint16> floatToQuint16(float value);
    float quint16ToFloat(const QVector<quint16> &values);
    void sendAxisState(int coilIndex);
};

#endif  // AXIS_MANAGER_H
