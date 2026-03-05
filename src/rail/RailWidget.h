#ifndef RAILWIDGET_H
#define RAILWIDGET_H

#include <QMetaType>
#include <QThread>
#include <QWidget>
QT_BEGIN_NAMESPACE
namespace Ui {
class RailWidget;
}
QT_END_NAMESPACE
class PLCCommunication;
class AxisManager;
class RailWidget : public QWidget {
    Q_OBJECT

public:
    RailWidget(QWidget *parent = nullptr);
    ~RailWidget();

    void updatePositionUI(float position);
    void updateSpeedUI(float speed);
    void setEditAbsPosition(QString position);
    void setEditSpeed(QString speed);
    double getCurrentXPosition() const;

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
    void whenUpdatePositionAndSpeed(float position, float speed);
    void whenUpdateAMState(const QString messageAxis, const QString messageMotion);

private slots:
    void connectRail();
    void disconnectRail();
    void on_btn_X_AbsPositionCommand_clicked();
    void on_chk_Stop_toggled(bool checked);
    void on_chk_ImmediateStop_toggled(bool checked);
    void on_btn_X_JogForward_pressed();
    void on_btn_X_JogForward_released();
    void on_btn_X_JogReverse_pressed();
    void on_btn_X_JogReverse_released();
    void on_btn_regressOrigin_clicked();
    void on_horizontalSlider_X_AbsPosition_sliderMoved(int position);
    void on_horizontalSlider_X_AbsSpeed_sliderMoved(int position);
    void on_btn_chk_Rest_clicked();
    void on_btn_contectRail_clicked();

    void on_btn_discontectRail_clicked();

private:
    Ui::RailWidget *ui;
    QString ip = "192.168.100.88";
    int port = 502;
    std::unique_ptr<PLCCommunication> m_communication_;

    // 三轴管理器
    std::unique_ptr<AxisManager> m_xAxis;
    // AxisManager *m_yAxis;
    // AxisManager *m_zAxis;

    // 线程管理
    QThread *m_commThread = new QThread();
    QThread *m_xAxisThread = new QThread();
    // QThread *m_yAxisThread;
    // QThread *m_zAxisThread;
};
#endif  // RAILWIDGET_H
