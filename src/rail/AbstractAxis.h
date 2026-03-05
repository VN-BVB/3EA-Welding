// abstract_axis.h
#ifndef ABSTRACT_AXIS_H
#define ABSTRACT_AXIS_H

#include <QObject>
#include <memory>

#include "axis_register.h"  // 包含 Axis 枚举等定义

class PLCCommunication;

class AbstractAxis : public QObject {
    Q_OBJECT
public:
    explicit AbstractAxis(Axis axisType, PLCCommunication* comm, QObject* parent = nullptr)
        : QObject(parent), m_axis(axisType), m_communication(comm) {}
    virtual ~AbstractAxis() = default;

    // 核心操作接口（纯虚函数）
    virtual void enableServo(bool enable) = 0;
    virtual void reset() = 0;
    virtual void setHome() = 0;
    virtual void stopSport(bool checked) = 0;
    virtual void immediateStop(bool checked) = 0;
    virtual void moveForward(float vel, bool checked) = 0;
    virtual void moveReverse(float vel, bool checked) = 0;
    virtual void moveToAbsPosition(float pos, float vel) = 0;
    virtual void startMonitoring(int intervalMs) = 0;
    virtual void stopMonitoring() = 0;

    // 属性访问
    Axis axisType() const { return m_axis; }
    QString axisName() const { return m_axisName; }
    bool isEnable() const { return m_isEnable; }

Q_SIGNALS:
    void axisError(const QString& msg);
    void sendText(const QString& msg);
    void sendTextState(const QString& axisState, const QString& motionState);
    void sendPositionAndSpeed(float pos, float speed);
    void absoluteMoveFinished();

protected:
    Axis m_axis;
    QString m_axisName;
    PLCCommunication* m_communication = nullptr;
    bool m_isEnable = false;
};
Q_DECLARE_METATYPE(std::shared_ptr<AbstractAxis>)

#endif  // ABSTRACT_AXIS_H
