#ifndef WELDINGMAINWINDOW_H
#define WELDINGMAINWINDOW_H

#include <QMainWindow>

#include "robotFactory/AbstractRobot.h"
#include "src/rail/concrete_axis/axis_register.h"
#include "src/stable.h"
#include "structLightCamera/StructLightCamera.h"
#include "workpieceCoarseLocalization/src/camera_control/basler/CoarsePositioningCamera.h"
QT_BEGIN_NAMESPACE
namespace Ui {
class WeldingMainWindow;
}
QT_END_NAMESPACE
enum COARES_LOC_CAMERA;
class WeldingMainWindow : public QMainWindow {
    Q_OBJECT

public:
    WeldingMainWindow(QWidget *parent = nullptr);
    ~WeldingMainWindow();
    void initIcon();                            // 初始化图标
    void initStatusLight();                     // 初始化指示灯
    void railDependencyInject();                // 地轨类依赖注入
    void initRailWeldingSystem();               // 初始化焊接系统
    void initVtkWindow();                       // 初始化点云显示页面
    void workpieceCoarseLocDependencyInject();  // 工件粗定位依赖注入

public slots:
    void whenStructLightStatusRenew(std::vector<DEVICE> device, std::vector<QString> color);  // 更新结构光指示灯
    void whenRobotStatusRenew(QString color);                                                 // 更新机器人指示灯
    void whenRailStatusRenew(QString color, Axis axis);                                       // 更新三轴指示灯
    void whenRailAMStateRenew(const QString messageAxis, const QString messageMotion, Axis axis);
    void whenRailPositionAndSpeedRenew(float position, float speed, Axis axis);
    // void whenCoarseLocCameraStatusRenew(std::vector<COARES_LOC_CAMERA> device,
    //                                     std::vector<QString> color);                   // 更新粗定位相机指示灯
    void whenGetWorkbenchPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);        // 获取到工作台点云
    void whenGetSeamInfo(std::vector<std::shared_ptr<WeldSeamInfo>> weldAreaInfo);     // 获取到检测完成的焊缝信息
    void whenGetMessage(QString message);                                              // 获取到需要显示的信息
    void whenGetImg2Ui(cv::Mat img);                                                   // 获取到需要显示的图像
    void whenGetRobotCurrentPose(robotPose p);                                         // 收到机器人当前位姿
    void whenGetRobotCurrentJointAngle(robotJointAngle j);                             // 收到机器人当前关节角
    void whenGetWorkpieceRailMap(cv::Mat res);                                         // 获得工件粗定位结果
    void whenGetWorkPieceCoord(int x, int y);                                          // 获得粗定位界面点击位置
    void whenGetWeldCoarseLocInfo(std::vector<std::vector<QTableWidgetItem *>> info);  // 获得粗定位信息用于显示在表格
    void whenGetCoarseCameraSerial(std::vector<std::string> serialNum);                // 获得粗定位相机序列号
    void whenGetCoarseLocWorkpieceNum(int num);                                        // 获得粗定位工件数量
signals:
    void connectStructLightCamera();                                                // 连接结构光相机
    void disconnectStructLightCamera();                                             // 断开结构光相机
    void sendGlobalReconstruct();                                                   // 发送全局点云重建信号
    void sendLocalReconstruct(double minU, double maxU, double minV, double maxV);  // 发送局部点云重建信号
    void sendUpdataWorkbench();                                                     // 发送更新工作台信号
    void sendScanWorkpiece();                                                       // 扫描工件信号
    void sendAutoWelding();                                                         // 自动焊接信号
    // void sendCoarseLoc(std::shared_ptr<workpieceBoxInWorld> res);                   // 发出工件粗定位信息
    void sendSaveRobotPose(robotPose p);   // 保存机器人位姿
    void sendRobotMoveJ2SouthWorkbench();  // 发送机器人运动到南工作台 (左侧)
    void sendRobotMoveJ2NorthWorkbench();  // 发送机器人运动到北工作台 (右侧)
private slots:
    void on_btnConnectStructLight_clicked();

    void on_btnDisConnectStructLight_clicked();

    void on_btnGlobalReconstruct_clicked();

    void on_btnLocalReconstruct_clicked();

    void on_ProScan_clicked();

    void on_btnConnectRobot_clicked();

    void on_btnDisconnectRobot_clicked();

    void on_pushButtonWelding_clicked();

    void on_btnRobotSavePose_clicked();

    void on_btnRobotMoveL_clicked();

    void on_comboBox_currentTextChanged(const QString &arg1);

    void on_combWorkpiece_currentTextChanged(const QString &arg1);

private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> pclVisualizer;  // 点云可视化界面
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_visual;
    cv::Mat ObjDetImg;  // 目标检测结果显示图

    std::shared_ptr<RailWeldingSystem> railWeldingSystem{nullptr};  // 地轨焊接系统控制类
    QThread *railWeldingSystemThread = new QThread;

    robotPose currRobotPose;
    double railPosition;
    Ui::WeldingMainWindow *ui;
};
#endif  // WELDINGMAINWINDOW_H
