#ifndef AXISMANAGER_H
#define AXISMANAGER_H
#include <QObject>
#include <QTime>

#include "../AbstractAxis.h"
#include "axis_register.h"

class PLCCommunication;

class AxisManager : public AbstractAxis {
    Q_OBJECT
public:
    explicit AxisManager(Axis axis, PLCCommunication *comm, QObject *parent = nullptr);
    ~AxisManager() override;

    // 运动控制
    void setAxisVel(float vel, int address) override;
    void whenMove2AbsPosition(float vel, float pos) override;
    void moveForward(float vel, bool checked) override;
    void moveReverse(float vel, bool checked) override;
    void enableServo(bool enable) override;
    void reset() override;
    void setHome() override;
    void stopSport(bool checked) override;
    void immediateStop(bool checked) override;
    void startMonitoring(int intervalMs) override;
    void stopMonitoring() override;
    // 获取状态
    Axis getAxis() const { return m_axis; }
    QString getAxisName() const override { return axisName; }
    float getCurrentSpeed() const { return m_currentSpeed; }
    float getCurrentPosition() const { return m_currentPosition; }

private slots:
    void onRealTimeout();

private:
    Axis m_axis;
    QString axisName;
    QTimer *m_realTimer = nullptr;
    QVector<bool> m_previousCoilStatuses;

    // bool m_isEnable = false;
    // PLCCommunication *m_communication;

    // 状态变量
    float m_currentPosition = 0.0f;
    float m_currentSpeed = 0.0f;
    bool m_absoluteMoveDone = false;
    bool m_absoluteMoveStart = false;

    void sendAxisState(int coilIndex);
    void processCoilStatus(const QVector<bool> &status);
    QVector<quint16> floatToQuint16(float value);
    float quint16ToFloat(const QVector<quint16> &values);
};

#endif  // AXISMANAGER_H
