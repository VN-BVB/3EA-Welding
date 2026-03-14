#ifndef RAILWELDINGSYSTEM_H
#define RAILWELDINGSYSTEM_H
#include "photoPlanner/PhotoPlanner.h"
#include "robotFactory/AbstractRobot.h"

class StructLightCamera;
class AbstractCameraFactory;
class AbstractProjectorFactory;
class WeldingMainWindow;
class SeamDetWithPointCloud;
class SeamDetWithSeg;
class RobotTrajectoryPlanning;
class WeldSeamInfo;
class ErrorSave;
class AbstractRobot;
class AbstractRobotFactory;
class Rail;
class WorkpieceCoarseLocalization;
struct workpieceBoxInWorld;
class robotPose;
class robotJointAngle;

enum WELD_MODE {  // 当前焊接模式
    AUTO_WELD,    // 自动焊接
    MANUAL_WELD   // 手动焊接
};

enum WELDING_STAGE {         // 当前焊接阶段
    PREPARE_STAGE,           // 准备阶段
    PRELIMINARY_PHOTO_AREA,  // 前期拍照区域
    LAST_PHOTO_AREA          // 最后拍照区域
};

class RailWeldingSystem : public QObject {
    Q_OBJECT
public:
    RailWeldingSystem(QObject *parent = nullptr);
    ~RailWeldingSystem();
    void initRobot();               // 初始化机器人类
    void initTrajectoryPlanning();  // 初始化轨迹规划类
    void initStructLightCamera();
    void initSeamDetWithPointCloud();
    void initSeamDetWithSeg();

    void connectRobot();
    void disconnectRobot();
    void welding();
    void move2SelectedWorkpiece(int tableRow);
public slots:
    void whenConnectingStructLight();                                                 // 连接结构光相机
    void whenDisconnectingStructLight();                                              // 断开结构光相机
    void whenGetFinalSeams(std::vector<std::shared_ptr<WeldSeamInfo>> weldAreaInfo);  // 获取到最终的焊缝
    void whenAutoWelding();                                                           // 自动焊接信号
    // void whenRailAbsActionFinished();                                          // 地轨绝对定位完成
    void whenTrajectoryPlanOver();                          // 机器人轨迹规划完成
    void whenRobotWeldOver();                               // 机器人焊接完成
    void whenRobotMoveOver();                               // 机器人运动完成
    void whenGetRobotMoveLData(robotPose p, double speed);  // 机器人直线运动到位姿
    void whenGetRobotCurrentPose(robotPose p);              // 收到机器人当前位姿
    void whenGetRobotCurrentJointAngle(robotJointAngle j);  // 收到机器人当前关节角
    void whenGetRobotPose2Save(robotPose p);                // 收到需要保存的机器人位姿
    // void whenGetCoarseLocalization(std::shared_ptr<workpieceBoxInWorld> res);  // 收到工件粗定位完成信息

signals:
    void sendMessage2Ui(QString message);                                          // 发送信息到UI界面
    void sendFinalSeams(std::vector<std::shared_ptr<WeldSeamInfo>> weldAreaInfo);  // 发送最终的焊缝
    void sendConnectRobot();                                                       // 连接机器人
    void sendDisconnectRobot();                                                    // 断开机器人
    void sendWelding();                                                            // 机器人焊接
    void sendRobotMoveLData(robotPose p, double speed);                            // 发出机器人直线运动到位姿
    void sendRobotCurrentPose(robotPose p);                                        // 发送机器人当前位姿
    void sendRobotCurrentJointAngle(robotJointAngle j);                            // 发送机器人当前关节角
    void sendUpdataWorkbench();                                                    // 发送扫描工作台

private:
    std::shared_ptr<StructLightCamera> structLightCamera{nullptr};  // 『结构光相机』
    QThread *structLightCameraThread = new QThread;                 // 结构光相机线程
    std::shared_ptr<AbstractCameraFactory> cameraFactory{nullptr};  // 相机工厂 (通过依赖注入的方式注入需要的类)
    std::shared_ptr<AbstractProjectorFactory> projectorFactory{nullptr};  // 投影仪工厂 (通过依赖注入的方式注入需要的类)

    std::shared_ptr<SeamDetWithPointCloud> seamDetWithPointCloud{nullptr};      // 『点云方法焊缝检测』
    QThread *seamDetWithPointCloudThread = new QThread;                         // 点云方法焊缝检测线程
    std::shared_ptr<SeamDetWithSeg> seamDetWithSeg{nullptr};                    // 『分割方法焊缝检测』
    QThread *seamDetWithSegThread = new QThread;                                // 分割方法焊缝检测线程
    std::shared_ptr<RobotTrajectoryPlanning> robotTrajectoryPlanning{nullptr};  // 『机器人轨迹规划』
    QThread *robotTrajectoryPlanningThread = new QThread;                       // 机器人轨迹规划线程

    std::shared_ptr<AbstractRobot> robot{nullptr};                // 『机器人』
    QThread *robotThread = new QThread;                           // 机器人线程
    std::shared_ptr<AbstractRobotFactory> robotFactory{nullptr};  // 机器人工厂

    // Rail *rail = nullptr;  // 『地轨』对象 (通过依赖注入初始化)

    // WorkpieceCoarseLocalization *workpieceCoarseLocalization = nullptr;  // 工件粗定位类

    const double photoRangeWidth = 500;
    const double photoRangeHeight = 280;
    std::shared_ptr<PhotoPlanner> photoPlanner =
        std::make_shared<PhotoPlanner>(photoRangeWidth, photoRangeHeight);  // 单一工件拍照位置规划类

    // 焊接数据
    std::vector<std::shared_ptr<WeldSeamInfo>> weldAreaInfo;  // 焊缝区域点云
    std::shared_ptr<workpieceBoxInWorld> coarseLocRes;        // 工件粗定位信息
    std::vector<float> workpiecePosition;                     // 模拟自动焊接时的几个工件位置

    std::vector<cv::Rect_<double>> rectOfPhotoPos;  // 当前拍照位置对应的目标检测框
    bool rectOfPhotoPosFlag = false;                // 当前拍照位置对应的目标检测框是否存储信息标志位

    // 焊接模式和阶段
    WELD_MODE weldMode = WELD_MODE::MANUAL_WELD;                // 当前焊接模式 (默认手动焊接)
    WELDING_STAGE weldingStage = WELDING_STAGE::PREPARE_STAGE;  // 当前焊接阶段 (默认准备阶段)

    double photoPosOffsetX = 15;  // 拍照位置偏移
    double photoPosOffsetY = -320;
    double photoPosOffsetZ = 285;
    double photoPosOffsetExtraX = 120;  // 单一粗定位区域拍照位置进一步偏移
    float railScaleWith5000 = 1;        // 当前地轨相比于5米地轨的尺度
    int savedPos = 0;                   // 保存机器人位姿序号
    robotPose currentRobotPose;         // 机器人当前位姿
    int currTableRow = 0;               // 当前表格行号
    float railPosition = 0;             // 当前地轨位置

    bool railAbsActionFinishedFlag = false;  // 地轨绝对运动完成标志位
    bool robotMoveLFinishedFlag = false;     // 机器人绝对运动完成标志位

    friend class WeldingMainWindow;
};

#endif  // RAILWELDINGSYSTEM_H
