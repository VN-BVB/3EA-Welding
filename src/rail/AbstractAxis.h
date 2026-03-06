// abstract_axis.h
#ifndef ABSTRACT_AXIS_H
#define ABSTRACT_AXIS_H

#include <QObject>
#include <memory>

class PLCCommunication;

class AbstractAxis : public QObject {
    Q_OBJECT
public:
    // 构造函数不再需要 Axis 参数，由具体实现决定
    explicit AbstractAxis(PLCCommunication* comm, QObject* parent = nullptr) : QObject(parent), m_communication(comm) {}
    virtual ~AbstractAxis() = default;

    // 核心操作接口（纯虚函数）

    virtual void enableServo(bool enable) = 0;
    virtual void reset() = 0;
    virtual void setHome() = 0;
    virtual void stopSport(bool checked) = 0;
    virtual void immediateStop(bool checked) = 0;
    virtual void moveForward(float vel, bool checked) = 0;
    virtual void moveReverse(float vel, bool checked) = 0;
    virtual void whenMove2AbsPosition(float vel, float pos) = 0;
    virtual void setAxisVel(float vel, int address) = 0;
    virtual void startMonitoring(int intervalMs) = 0;
    virtual void stopMonitoring() = 0;

    // 属性访问（不再返回轴的类型）
    virtual QString getAxisName() const = 0;  // 纯虚函数，由子类实现
    bool isEnable() const { return m_isEnable; }

signals:
    void axisError(const QString& msg);
    void sendText(const QString& msg);
    void sendTextState(const QString& axisState, const QString& motionState);
    void sendPositionAndSpeed(float pos, float speed);
    void absoluteMoveFinished();

protected:
    PLCCommunication* m_communication = nullptr;
    bool m_isEnable = false;
};
Q_DECLARE_METATYPE(std::shared_ptr<AbstractAxis>)

#endif  // ABSTRACT_AXIS_H
