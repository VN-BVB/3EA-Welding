#ifndef RAILWIDGET_H
#define RAILWIDGET_H

#include <QMetaType>
#include <QThread>
#include <QWidget>

#include "concrete_axis/axis_register.h"
QT_BEGIN_NAMESPACE
namespace Ui {
class RailWidget;
}
QT_END_NAMESPACE
class PLCCommunication;
class AbstractAxis;
class RailWidget : public QWidget {
    Q_OBJECT

public:
    RailWidget(QWidget *parent = nullptr);
    ~RailWidget();

    void updatePositionUI(float position);
    void updateSpeedUI(float speed);
    void setEditAbsPosition(QString position, Axis axis = Axis::X);  // 添加轴参数，默认为X轴
    void setEditSpeed(QString speed, Axis axis = Axis::X);           // 添加轴参数，默认为X轴
    double getCurrentPosition(Axis axis = Axis::X) const;            // 修改为通用函数，默认为X轴

signals:
    void sendConnectToPLC(QString ip, int port);
    void sendDisconnectToPLC();
    void sendWriteCoils(int address, const QVector<bool> &values);
    void sendWriteRegisters(int address, const QVector<quint16> &values);
    void sendMove2AbsPosition(float val, float pos);  // 地轨移动到指定位置
    void sendForward(float vel, bool checked);        // 正向点动
    void sendReverse(float vel, bool checked);        // 反向点动

public slots:
    void whenAppendErrorLog(QString message);
    void whenAppendTextLog(QString message);
    void whenUpdatePositionAndSpeed(float position, float speed, Axis axis);
    void whenUpdateAMState(const QString messageAxis, const QString messageMotion, Axis axis);

private slots:
    void connectRail();
    void disconnectRail();
    void on_btn_contectRail_clicked();
    void on_btn_discontectRail_clicked();
    void on_btn_chk_Rest_clicked();
    void on_chk_ImmediateStop_toggled(bool checked);

    void on_btn_X_AbsPositionCommand_clicked();
    void on_chk_X_Stop_toggled(bool checked);
    void on_btn_X_JogForward_pressed();
    void on_btn_X_JogForward_released();
    void on_btn_X_JogReverse_pressed();
    void on_btn_X_JogReverse_released();
    void on_btn_X_regressOrigin_clicked();
    void on_horizontalSlider_X_AbsPosition_sliderMoved(int position);
    void on_horizontalSlider_X_AbsSpeed_sliderMoved(int position);

    // Y轴槽函数
    void on_btn_Y_AbsPositionCommand_clicked();
    void on_chk_Y_Stop_toggled(bool checked);
    void on_btn_Y_JogForward_pressed();
    void on_btn_Y_JogForward_released();
    void on_btn_Y_JogReverse_pressed();
    void on_btn_Y_JogReverse_released();
    void on_btn_Y_regressOrigin_clicked();
    void on_horizontalSlider_Y_AbsPosition_sliderMoved(int position);
    void on_horizontalSlider_Y_AbsSpeed_sliderMoved(int position);

    // Z轴槽函数
    void on_btn_Z_AbsPositionCommand_clicked();
    void on_chk_Z_Stop_toggled(bool checked);
    void on_btn_Z_JogForward_pressed();
    void on_btn_Z_JogForward_released();
    void on_btn_Z_JogReverse_pressed();
    void on_btn_Z_JogReverse_released();
    void on_btn_Z_regressOrigin_clicked();
    void on_horizontalSlider_Z_AbsPosition_sliderMoved(int position);
    void on_horizontalSlider_Z_AbsSpeed_sliderMoved(int position);

private:
    Ui::RailWidget *ui;
    QString ip = "192.168.100.88";
    int port = 502;
    std::unique_ptr<PLCCommunication> m_communication_;  // 通信层独占指针

    // 三轴管理器
    std::shared_ptr<AbstractAxis> m_xAxis;  // X轴共享指针
    std::shared_ptr<AbstractAxis> m_yAxis;  // Y轴共享指针
    std::shared_ptr<AbstractAxis> m_zAxis;  // Z轴共享指针

    // 线程管理
    QThread *m_commThread = new QThread();
    QThread *m_xAxisThread = new QThread();
    QThread *m_yAxisThread = new QThread();
    QThread *m_zAxisThread = new QThread();
};
#endif  // RAILWIDGET_H
