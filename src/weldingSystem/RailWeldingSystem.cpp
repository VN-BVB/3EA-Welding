#include "weldingSystem/RailWeldingSystem.h"
// clang-format off
#include "cameraFactory/basler/BaslerCameraFactory.h"
#include "errorSave/ErrorSave.h"
#include "projectFactory/tengJu/TengJuProjectorFactory.h"
#include "rail/RailWidget.h"
#include "robotFactory/an_chaun/AnChuanRobotFactory.h"
#include "robotFactory/bao_yuan/BaoYuanRobotFactory.h"
#include "robotTrajectoryPlanning/RobotTrajectoryPlanning.h"
#include "robotTrajectoryPlanning/LargeWorkpieceTrajectoryPlanning.h"
#include "robotTrajectoryPlanning/config/TrajectoryPlanningConfig.h"
#include "seamDetWithPointCloud/SeamDetWithPointCloud.h"
#include "seamDetWithSeg/SeamDetWithSeg.h"
#include "structLightCamera/StructLightCamera.h"
#include "structLightCamera/config/StructLightConfig.h"
#include "utils/common/WeldSeamInfo.h"
// #include "workpieceCoarseLocalization/WorkpieceCoarseLocalization.h"
#include "settingPara/SettingPara.h"
// #include "photoPlanner/PhotoPlanner.h"
// clang-format on
RailWeldingSystem::RailWeldingSystem(QObject* parent)
    : cameraFactory(std::make_shared<BaslerCameraFactory>(nullptr)),
      projectorFactory(std::make_shared<TengJuProjectorFactory>(nullptr)),
      seamDetWithPointCloud(std::make_shared<SeamDetWithPointCloud>(nullptr)) {
    this->initStructLightCamera();
    this->initSeamDetWithPointCloud();
    this->initSeamDetWithSeg();
    this->initTrajectoryPlanning();
    // this->initErrorSave();
    this->initRobot();
}

RailWeldingSystem::~RailWeldingSystem() {}

// 初始化结构光相机
void RailWeldingSystem::initStructLightCamera() {
    // 实例化结构光相机对象 (通过注入『对应品牌的相机和投影仪工厂』依赖的方式)
    structLightCamera = std::make_shared<StructLightCamera>(cameraFactory, projectorFactory);
    if (structLightCamera) {
        structLightCamera->moveToThread(structLightCameraThread);
        structLightCameraThread->start();

        connect(this, &RailWeldingSystem::sendUpdataWorkbench, structLightCamera.get(), &StructLightCamera::whenScanWorkpiece);
        PLOGD << "结构光相机类初始化成功";
    } else {
        PLOGE << "结构光相机类初始化失败";
    }
}

// 初始化点云方法检测焊缝类
void RailWeldingSystem::initSeamDetWithPointCloud() {
    if (seamDetWithPointCloud) {
        seamDetWithPointCloud->moveToThread(seamDetWithPointCloudThread);
        seamDetWithPointCloudThread->start();
        // 相机重建出最初的焊缝区域点云, 发送到点云方法检测焊缝线程
        connect(structLightCamera.get(), &StructLightCamera::sendWeldAreaInfo, seamDetWithPointCloud.get(),
                &SeamDetWithPointCloud::whenDetSeamWithPointCloud);
        connect(structLightCamera.get(), &StructLightCamera::sendWeldAreaInfoLW, seamDetWithPointCloud.get(),
                &SeamDetWithPointCloud::whenDetSeamWithPointCloudLW);

        PLOGD << "点云方法焊缝检测类初始化成功";
    } else {
        PLOGE << "点云方法焊缝检测类初始化失败";
    }
}
// 初始化分割方法检测焊缝类
void RailWeldingSystem::initSeamDetWithSeg() {
    seamDetWithSeg = std::make_shared<SeamDetWithSeg>(nullptr);

    if (seamDetWithSeg) {
        seamDetWithSeg->moveToThread(seamDetWithSegThread);
        seamDetWithSegThread->start();
        // 点云方法检测焊缝类计算出焊缝, 发送到分割方法检测焊缝线程
        connect(seamDetWithPointCloud.get(), &SeamDetWithPointCloud::sendDetSeamWithPointCloud, seamDetWithSeg.get(),
                &SeamDetWithSeg::whenDetSeamWithSeg);

        PLOGD << "分割方法焊缝检测类初始化成功";
    } else {
        PLOGE << "分割方法焊缝检测类初始化失败";
    }
}
// 初始化轨迹规划类
void RailWeldingSystem::initTrajectoryPlanning() {
    robotTrajectoryPlanning = std::make_shared<LargeWorkpieceTrajectoryPlanning>(nullptr);

    if (robotTrajectoryPlanning) {
        robotTrajectoryPlanning->moveToThread(robotTrajectoryPlanningThread);
        robotTrajectoryPlanningThread->start();
        // // 分割方法检测焊缝类计算出焊缝, 发送到轨迹规划线程. 同时发送到本类暂存, 以便未来保存错误数据以及显示.
        connect(seamDetWithSeg.get(), &SeamDetWithSeg::sendDetSeamWithSeg, robotTrajectoryPlanning.get(),
                &AbstractTrajectoryPlanning::whenPlanningTrajectory);
        connect(robotTrajectoryPlanning.get(), &AbstractTrajectoryPlanning::sendPlannedSeams, this,
                &RailWeldingSystem::whenGetFinalSeams);

        // 轨迹规划完成后, 发送到本类以便自动模式直接开始焊接
        connect(robotTrajectoryPlanning.get(), &AbstractTrajectoryPlanning::sendTrajectoryPlanOver, this,
                &RailWeldingSystem::whenTrajectoryPlanOver);

        PLOGD << "轨迹规划类初始化成功";
    } else {
        PLOGE << "轨迹规划类初始化失败";
    }
}
void RailWeldingSystem::switchTrajectoryPlanning(WORKPIECE_TYPE workpieceType) {
    if (robotTrajectoryPlanning) {
        // 断开旧的信号连接
        disconnect(seamDetWithSeg.get(), &SeamDetWithSeg::sendDetSeamWithSeg, robotTrajectoryPlanning.get(),
                   &AbstractTrajectoryPlanning::whenPlanningTrajectory);
        disconnect(robotTrajectoryPlanning.get(), &AbstractTrajectoryPlanning::sendPlannedSeams, this,
                   &RailWeldingSystem::whenGetFinalSeams);
        disconnect(robotTrajectoryPlanning.get(), &AbstractTrajectoryPlanning::sendTrajectoryPlanOver, this,
                   &RailWeldingSystem::whenTrajectoryPlanOver);

        // 旧对象会被智能指针自动释放（如果不再被引用）
        robotTrajectoryPlanning = nullptr;
    }

    // 根据工件类型创建新的轨迹规划对象
    if (workpieceType == WORKPIECE_TYPE::LARGE_WORKPIECE) {
        robotTrajectoryPlanning = std::make_shared<LargeWorkpieceTrajectoryPlanning>(nullptr);
        PLOGD << "创建大型工件轨迹规划类";
    } else {
        robotTrajectoryPlanning = std::make_shared<RobotTrajectoryPlanning>(nullptr);
        PLOGD << "创建小型工件轨迹规划类";
    }

    if (robotTrajectoryPlanning) {
        // 将新对象移动到现有的轨迹规划线程
        // 这一步是必须的，因为新对象默认在主线程中
        robotTrajectoryPlanning->moveToThread(robotTrajectoryPlanningThread);

        // 确保线程正在运行
        if (!robotTrajectoryPlanningThread->isRunning()) {
            robotTrajectoryPlanningThread->start();
        }

        // 重新建立信号连接
        connect(seamDetWithSeg.get(), &SeamDetWithSeg::sendDetSeamWithSeg, robotTrajectoryPlanning.get(),
                &AbstractTrajectoryPlanning::whenPlanningTrajectory);
        connect(robotTrajectoryPlanning.get(), &AbstractTrajectoryPlanning::sendPlannedSeams, this,
                &RailWeldingSystem::whenGetFinalSeams);
        connect(robotTrajectoryPlanning.get(), &AbstractTrajectoryPlanning::sendTrajectoryPlanOver, this,
                &RailWeldingSystem::whenTrajectoryPlanOver);

        PLOGD << "轨迹规划类切换成功";
    } else {
        PLOGE << "轨迹规划类切换失败";
    }
}
void RailWeldingSystem::initRobot() {
    if (robotTrajectoryPlanning) {
        if (robotTrajectoryPlanning->trajectoryConfig.robotType == MyToolFunc::getRobotTypeString(ROBOT_TYPE::BAO_YUAN)) {
            robotFactory = std::make_shared<BaoYuanRobotFactory>(nullptr);
        } else if (robotTrajectoryPlanning->trajectoryConfig.robotType == MyToolFunc::getRobotTypeString(ROBOT_TYPE::AN_CHUAN)) {
            robotFactory = std::make_shared<AnChuanRobotFactory>(nullptr);
        }
    }

    if (robotFactory) {
        robot = robotFactory->createRobot();

        if (robot) {
            robot->moveToThread(robotThread);
            robotThread->start();
            // UI点击了连接断开机器人以及焊接时, 发送到机器人类进行相关操作
            connect(this, &RailWeldingSystem::sendConnectRobot, robot.get(), &AbstractRobot::connectRobot);
            connect(this, &RailWeldingSystem::sendDisconnectRobot, robot.get(), &AbstractRobot::disconnectRobot);
            connect(this, &RailWeldingSystem::sendWelding, robot.get(), &AbstractRobot::welding);
            connect(this, &RailWeldingSystem::sendRobotMoveLData, robot.get(), &AbstractRobot::moveL);
            connect(robot.get(), &AbstractRobot::sendRobotCurrentPose, this, &RailWeldingSystem::whenGetRobotCurrentPose);
            connect(robot.get(), &AbstractRobot::sendRobotCurrentJointAngle, this,
                    &RailWeldingSystem::whenGetRobotCurrentJointAngle);

            // 机器人焊接完成后, 发送信号通知此线程
            connect(robot.get(), &AbstractRobot::sendRobotWeldOver, this, &RailWeldingSystem::whenRobotWeldOver);
            connect(robot.get(), &AbstractRobot::sendRobotMoveOver, this, &RailWeldingSystem::whenRobotMoveOver);

            PLOGD << "机器人类初始化成功";
        } else {
            PLOGE << "机器人类初始化失败";
        }
    }
}
// 连接结构光相机
void RailWeldingSystem::whenConnectingStructLight() {
    if (structLightCamera) {
        PLOGD << "连接结构光相机...";
        uint8_t status = structLightCamera->open();

        if (status & DEVICE::PRIMARY_CAMERA) {
            PLOGD << "主相机连接成功";
        } else {
            PLOGE << "主相机连接失败";
        }

        if (StructLightConfig::getInstance().getSecondaryCameraSerialNum() != "00000000") {
            if (status & DEVICE::SECONDARY_CAMERA) {
                PLOGD << "次相机连接成功";
            } else {
                PLOGE << "次相机连接失败";
            }
        }

        if (status & DEVICE::PROJECTOR) {
            PLOGD << "投影仪连接成功";
        } else {
            PLOGE << "投影仪连接失败";
        }
    } else {
        PLOGE << "结构光相机类未初始化";
    }
}

// 断开结构光相机
void RailWeldingSystem::whenDisconnectingStructLight() {
    if (structLightCamera) {
        PLOGD << "断开结构光相机...";
        structLightCamera->close();
    } else {
        PLOGE << "结构光相机类未初始化";
    }
}
// 连接机器人
void RailWeldingSystem::connectRobot() { emit sendConnectRobot(); }

// 断开机器人
void RailWeldingSystem::disconnectRobot() { emit sendDisconnectRobot(); }

// 机器人焊接
void RailWeldingSystem::welding() { emit sendWelding(); }

// 移动到选中的工件
void RailWeldingSystem::move2SelectedWorkpiece(int tableRow) {}
void RailWeldingSystem::whenGetRobotMoveLData(robotPose p, double speed) {
    // PLOGD << "收到机器人直线运动数据: " << p.x_ << " " << p.y_ << " " << p.z_ << " " << p.a_ << " " << p.b_ << " " << p.c_;
    emit sendRobotMoveLData(p, speed * 10);
}

void RailWeldingSystem::whenGetRobotCurrentPose(robotPose p) {
    // PLOGD << "当前机器人位姿: " << p.x_ << " " << p.y_ << " " << p.z_ << " " << p.a_ << " " << p.b_ << " " << p.c_;
    robotTrajectoryPlanning->trajectoryConfig.currentRobotPose = p;
    // 计算末端(工具)到基坐标系的转换矩阵
    robotTrajectoryPlanning->trajectoryConfig.matrixEnd2Base =
        MyToolFunc::createTransformationMatrixZYX(p.x_, p.y_, p.z_, p.a_, p.b_, p.c_);

    emit sendRobotCurrentPose(p);
}

void RailWeldingSystem::whenGetRobotCurrentJointAngle(robotJointAngle j) {
    // PLOGD << "当前机器人角度: " << j.joint1 << " " << j.joint2 << " " << j.joint3 << " " << j.joint4 << " " << j.joint5 << " "
    // << j.joint6;
    emit sendRobotCurrentJointAngle(j);
}
// 收到需要保存的机器人位姿
void RailWeldingSystem::whenGetRobotPose2Save(robotPose p) {
    // 创建并打开文件
    std::string filePath;
    if (savedPos < 10) {
        filePath = "./data/robotPose/robotpos0";
    } else {
        filePath = "./data/robotPose/robotpos";
    }
    std::ofstream outFile(filePath + std::to_string(savedPos++) + ".xml");
    if (!outFile.is_open()) {
        PLOGE << "无法创建文件";
        return;
    }
    // 写入XML内容
    outFile << "<?xml version=\"1.0\"?>\n"
            << "<opencv_storage>\n"
            << "<Position0>" << p.x_ / 1000 << "</Position0>\n"
            << "<Position1>" << p.y_ / 1000 << "</Position1>\n"
            << "<Position2>" << p.z_ / 1000 << "</Position2>\n"
            << "<Position3>" << p.a_ << "</Position3>\n"
            << "<Position4>" << p.b_ << "</Position4>\n"
            << "<Position5>" << p.c_ << "</Position5>\n"
            << "</opencv_storage>\n";
    // 关闭文件
    outFile.close();

    PLOGD << "文件保存成功";
    emit sendMessage2Ui(u8"文件保存成功");
}

// 机器人焊接完成
void RailWeldingSystem::whenRobotWeldOver() {
    PLOGD << "机器人焊接完成";
    emit sendMessage2Ui(u8"机器人焊接完成");

    // if (weldMode == WELD_MODE::AUTO_WELD && coarseLocRes && coarseLocRes->workpieceInfoInWorld.size() > currTableRow &&
    //     coarseLocRes->workpieceInfoInWorld[currTableRow].photoPos.size() > 0) {
    //     PLOGD << "下一拍照位置";
    //     emit sendMessage2Ui(u8"下一拍照位置");

    //     emit sendMove2NextWorkpiece();
    // } else if (weldMode == WELD_MODE::AUTO_WELD && coarseLocRes && coarseLocRes->workpieceInfoInWorld.size() > currTableRow + 1
    // &&
    //            coarseLocRes->workpieceInfoInWorld[currTableRow + 1].photoPos.size() > 0) {
    //     PLOGD << "下一工件";
    //     emit sendMessage2Ui(u8"下一工件");

    //     emit sendMove2NextWorkpiece();
    // } else if (weldMode == WELD_MODE::AUTO_WELD) {  // 不存在待焊工件, 则回到手动模式
    //     weldMode = WELD_MODE::MANUAL_WELD;
    //     currTableRow = 0;

    //     PLOGD << "自动焊接完成";
    //     emit sendMessage2Ui(u8"自动焊接完成");
    // }
}

// 机器人运动完成
void RailWeldingSystem::whenRobotMoveOver() {
    PLOGD << "机器人运动完成";
    emit sendMessage2Ui(u8"机器人运动完成");

    // // 如果是自动焊接模式, 直接开始扫描工作台
    // if (weldMode == WELD_MODE::AUTO_WELD) {
    //     robotMoveLFinishedFlag = true;

    //     if (railAbsActionFinishedFlag == true && robotMoveLFinishedFlag == true) {
    //         emit sendUpdataWorkbench();

    //         railAbsActionFinishedFlag = false;
    //         robotMoveLFinishedFlag = false;
    //     }
    // }
}
// 自动焊接信号
void RailWeldingSystem::whenAutoWelding() {
    // PLOGD << "自动焊接... ...";
    // emit sendMessage2Ui(u8"自动焊接...");
    // weldMode = WELD_MODE::AUTO_WELD;  // 切换焊接模式为自动焊接

    // if (coarseLocRes && coarseLocRes->workpieceInfoInWorld.size() > currTableRow &&
    //     coarseLocRes->workpieceInfoInWorld[currTableRow].photoPos.size() > 0) {
    //     emit sendMove2NextWorkpiece();
    // } else {  // 不存在待焊工件, 则回到手动模式
    //     weldMode = WELD_MODE::MANUAL_WELD;
    //     currTableRow = 0;

    //     PLOGD << "自动焊接完成";
    //     emit sendMessage2Ui(u8"自动焊接完成");
}
// 获取到最终的焊缝
void RailWeldingSystem::whenGetFinalSeams(std::vector<std::shared_ptr<WeldSeamInfo>> weldAreaInfo) {
    this->weldAreaInfo = weldAreaInfo;

    // 焊缝信息写入文件
    this->robotTrajectoryPlanning->write2File(this->weldAreaInfo, this->robotTrajectoryPlanning->endOfLeftSeamSerial);

    emit sendFinalSeams(this->weldAreaInfo);  // 发送最终的焊缝

    // 发送焊缝检测信息到主UI
    int succNum = 0;  // 统计检测到多少焊缝
    for (auto& info : this->weldAreaInfo) {
        if (info->detectSuccFlag == true) {
            succNum++;
        }
    }
    QString message = QString(QStringLiteral("检测到 %1 条焊缝")).arg(QString::number(succNum));
    emit sendMessage2Ui(message);
}
// 机器人轨迹规划完成
void RailWeldingSystem::whenTrajectoryPlanOver() {
    PLOGD << "机器人轨迹规划完成";
    emit sendMessage2Ui(u8"机器人轨迹规划完成");

    // 如果是自动焊接模式, 直接开始焊接
    if (weldMode == WELD_MODE::AUTO_WELD) {
        emit sendWelding();
    }
}
