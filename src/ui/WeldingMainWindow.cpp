#include "WeldingMainWindow.h"

#include "ui_WeldingMainWindow.h"

// clang-format off
#include "cameraFactory/AbstractCamera.h"
#include "robotFactory/AbstractRobot.h"
#include "robotTrajectoryPlanning/config/TrajectoryPlanningConfig.h"
#include "settingPara/SettingPara.h"
#include "structLightCamera/StructLightCamera.h"
#include "structLightCamera/reconstruction/PointCloudReconstruction.h"
#include "ui/SystemMirrorWidget.h"
#include "utils/common/WeldSeamInfo.h"
#include "utils/stateLight/StateLight.h"
#include "weldingSystem/RailWeldingSystem.h"
#include "ui/common.hpp"
// clang-format on
WeldingMainWindow::WeldingMainWindow(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::WeldingMainWindow), railWeldingSystem(std::make_shared<RailWeldingSystem>(nullptr)) {
    ui->setupUi(this);
    setWindowState(Qt::WindowMaximized);         // 设置全屏
    this->initIcon();                            // 初始化图标
    this->initVtkWindow();                       // 初始化点云显示页面
    this->initStatusLight();                     // 初始化指示灯
    this->workpieceCoarseLocDependencyInject();  // 工件粗定位依赖注入
    this->initRailWeldingSystem();               // 初始化地轨焊接系统

    PLOGD << "三轴焊接系统软件启动成功";
}

WeldingMainWindow::~WeldingMainWindow() { delete ui; }

void WeldingMainWindow::initIcon() {}
void WeldingMainWindow::initStatusLight() {
    std::vector<DEVICE> device;
    std::vector<QString> color;

    device.push_back(DEVICE::PRIMARY_CAMERA);
    device.push_back(DEVICE::SECONDARY_CAMERA);
    device.push_back(DEVICE::PROJECTOR);
    color.push_back(MY_COLOR::GRAY);
    color.push_back(MY_COLOR::GRAY);
    color.push_back(MY_COLOR::GRAY);

    this->whenStructLightStatusRenew(device, color);  // 初始化指示灯状态
}
// 初始化点云显示页面
void WeldingMainWindow::initVtkWindow() {
    cloud_visual.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pclVisualizer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    pclVisualizer->addPointCloud(cloud_visual, "cloud");
    ui->qvtkWidget->SetRenderWindow(pclVisualizer->getRenderWindow());
    pclVisualizer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
}
// 工件粗定位依赖注入
void WeldingMainWindow::workpieceCoarseLocDependencyInject() {
    this->railWeldingSystem->workpieceCoarseLocalization = ui->workpieceCoarseLocWidget;
    PLOGD << "工件粗定位对象注入完成";

    // 完成工件粗定位类的初始化
    railWeldingSystem->initWorkpieceCoarseLoc();
    connect(ui->workpieceCoarseLocWidget->fittingWorkpieceCoordinate, &FittingWorkpieceCoordinate::sendWorkpieceResultToMainWindow, this,
            &WeldingMainWindow::whenGetWorkpieceRailMap);
    connect(ui->graphicsViewCoarseLoc, &ScalableGraphicsView::senderSignalPixelCoordinates,
            this->railWeldingSystem->workpieceCoarseLocalization->fittingWorkpieceCoordinate, &FittingWorkpieceCoordinate::handleClickEvent);
    connect(ui->graphicsViewCoarseLoc, &ScalableGraphicsView::senderSignalPixelCoordinates, this, &WeldingMainWindow::whenGetWorkPieceCoord);

    connect(ui->workpieceCoarseLocWidget, &WorkpieceCoarseLocalization::sendCameraComboBox, this, &WeldingMainWindow::whenGetCoarseCameraSerial);
    connect(ui->workpieceCoarseLocWidget->baslerControl, &CoarsePositioningCamera::sendImageToView, this, &WeldingMainWindow::whenGetImg2Ui);
    connect(ui->workpieceCoarseLocWidget, &WorkpieceCoarseLocalization::sendCoarseLocWorkpieceNum, this,
            &WeldingMainWindow::whenGetCoarseLocWorkpieceNum);
    connect(ui->workpieceCoarseLocWidget, &WorkpieceCoarseLocalization::sendImg2MainWindow, this, &WeldingMainWindow::whenGetImg2Ui);
    connect(ui->workpieceCoarseLocWidget, &WorkpieceCoarseLocalization::sendMessage2MainWindow, this, &WeldingMainWindow::whenGetMessage);

    ui->tabWidget_CoarseLoc->setCurrentIndex(0);
    // ui->tabWidget->removeTab(ui->tabWidget->indexOf(ui->tab_coarseLoc));  // 移除工件粗定位页面
}
// 初始化地轨焊接系统
void WeldingMainWindow::initRailWeldingSystem() {
    if (railWeldingSystem) {
        railWeldingSystem->moveToThread(railWeldingSystemThread);
        railWeldingSystemThread->start();

        // 依赖注入
        ui->settingWidget->structLightCamera = this->railWeldingSystem->structLightCamera;
        ui->settingWidget->initSetting();

        // clang-format off
        // ************************************ 与本页面有关的信号槽链接 ************************************
        // 连接和断开结构光相机信号槽
        connect(this, &WeldingMainWindow::connectStructLightCamera, railWeldingSystem.get(), &RailWeldingSystem::whenConnectingStructLight);
        connect(this, &WeldingMainWindow::disconnectStructLightCamera, railWeldingSystem.get(), &RailWeldingSystem::whenDisconnectingStructLight);

        // 重建『焊缝区域或全局/局部点云』信号槽
        connect(this, &WeldingMainWindow::sendScanWorkpiece, railWeldingSystem->structLightCamera.get(), &StructLightCamera::whenScanWorkpiece);
        connect(this, &WeldingMainWindow::sendGlobalReconstruct, railWeldingSystem->structLightCamera.get(), &StructLightCamera::whenGlobalReconstruct);
        connect(this, &WeldingMainWindow::sendLocalReconstruct, railWeldingSystem->structLightCamera.get(), &StructLightCamera::whenLocalReconstruct);

        // 更新『硬件状态指示灯』信号槽
        connect(railWeldingSystem->structLightCamera.get(), &StructLightCamera::sendStructLightStatus, this, &WeldingMainWindow::whenStructLightStatusRenew);
        connect(railWeldingSystem->robot.get(), &AbstractRobot::sendRobotStatus, this, &WeldingMainWindow::whenRobotStatusRenew);
        // connect(railWeldingSystem->rail, &Rail::sendRailStatus, this, &WeldingMainWindow::whenRailStatusRenew);
        // connect(ui->workpieceCoarseLocWidget->baslerControl, &CoarsePositioningCamera::sendCameraStatus, this, &WeldingMainWindow::whenCoarseLocCameraStatusRenew);

        // 获取到『点云或图像』信号槽
        connect(railWeldingSystem->structLightCamera.get(), &StructLightCamera::sendPointCloud, this, &WeldingMainWindow::whenGetWorkbenchPointCloud);
        connect(railWeldingSystem->structLightCamera.get(), &StructLightCamera::sendImage, this, &WeldingMainWindow::whenGetImg2Ui);
        connect(railWeldingSystem->structLightCamera->primaryCamera.get(), &AbstractCamera::sendImage, this, &WeldingMainWindow::whenGetImg2Ui);
        connect(railWeldingSystem.get(), &RailWeldingSystem::sendFinalSeams, this, &WeldingMainWindow::whenGetSeamInfo);

        // 获取到需要显示的『Message』的信号槽
        connect(railWeldingSystem->structLightCamera.get(), &StructLightCamera::sendMessage2Ui, this, &WeldingMainWindow::whenGetMessage);
        connect(railWeldingSystem.get(), &RailWeldingSystem::sendMessage2Ui, this, &WeldingMainWindow::whenGetMessage);

        // 『自动焊接』信号槽
        connect(this, &WeldingMainWindow::sendAutoWelding, railWeldingSystem.get(), &RailWeldingSystem::whenAutoWelding);

        // // 『工件粗定位』信号槽
        // connect(railWeldingSystem.get(), &RailWeldingSystem::sendWeldCoarseLocInfo, this, &WeldingMainWindow::whenGetWeldCoarseLocInfo);

        // 机器人信息信号槽
        connect(railWeldingSystem.get(), &RailWeldingSystem::sendRobotCurrentPose, this, &WeldingMainWindow::whenGetRobotCurrentPose);
        connect(railWeldingSystem.get(), &RailWeldingSystem::sendRobotCurrentJointAngle, this, &WeldingMainWindow::whenGetRobotCurrentJointAngle);
        connect(this, &WeldingMainWindow::sendSaveRobotPose, railWeldingSystem.get(), &RailWeldingSystem::whenGetRobotPose2Save);
        // connect(this, &WeldingMainWindow::sendRobotMoveJ2SouthWorkbench, railWeldingSystem->robot.get(), &AbstractRobot::whenRobotMoveJ2SouthWorkbench);
        // connect(this, &WeldingMainWindow::sendRobotMoveJ2NorthWorkbench, railWeldingSystem->robot.get(), &AbstractRobot::whenRobotMoveJ2NorthWorkbench);

        // // 粗定位表格信号槽
        // connect(ui->tableWidgetCoarseLoc, &QTableWidget::currentCellChanged,this, &WeldingMainWindow::on_tableWidgetCoarseLoc_currentCellChanged);
        // connect(ui->comboBoxCoarseLocInfo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &WeldingMainWindow::on_comboBoxCoarseLocInfo_currentIndexChanged);
        // connect(railWeldingSystem.get(), &RailWeldingSystem::sendWorkpieceResidualPhotoPos, this, &WeldingMainWindow::whenWorkpieceResidualPhotoPos);
        // connect(railWeldingSystem.get(), &RailWeldingSystem::sendRenewTableRow, this, &WeldingMainWindow::whenTableRowRenew);
        // connect(railWeldingSystem.get(), &RailWeldingSystem::sendMove2NextWorkpiece, this, &WeldingMainWindow::on_btn_nextWorkpiece_clicked);
        // connect(ui->settingWidget, &SettingWidget::sendCoarseCameraExposure, ui->workpieceCoarseLocWidget->baslerControl,
        //         &CoarsePositioningCamera::whenGetCameraExposure);
        // clang-format on

        PLOGD << "地轨焊接系统类初始化成功";
    } else {
        PLOGE << "地轨焊接系统类初始化失败";
    }
}
// 更新结构光指示灯
void WeldingMainWindow::whenStructLightStatusRenew(std::vector<DEVICE> device, std::vector<QString> color) {
    for (int i = 0; i < device.size(); ++i) {
        if (device[i] == DEVICE::PRIMARY_CAMERA) {
            ui->labelPrimaryCameraStatusLight->setStyleSheet(color[i]);
        } else if (device[i] == DEVICE::SECONDARY_CAMERA) {
            ui->labelSecondaryCameraStatusLight->setStyleSheet(color[i]);
        } else if (device[i] == DEVICE::PROJECTOR) {
            ui->labelProjectorStatusLight->setStyleSheet(color[i]);
        }
    }
}

// 更新机器人指示灯
void WeldingMainWindow::whenRobotStatusRenew(QString color) { ui->labelRobotStatusLight->setStyleSheet(color); }

// 更新地轨指示灯
void WeldingMainWindow::whenRailStatusRenew(QString color, Axis axis) {
    switch (axis) {
        case Axis::X:
            ui->labelConnectStatusLight_X->setStyleSheet(color);
            break;
        case Axis::Y:
            ui->labelConnectStatusLight_Y->setStyleSheet(color);
            break;
        case Axis::Z:
            ui->labelConnectStatusLight_Z->setStyleSheet(color);
            break;
        default:
            ui->labelRailStatusLight->setStyleSheet(color);
            break;
    }
}

// // 更新粗定位相机指示灯
// void WeldingMainWindow::whenCoarseLocCameraStatusRenew(std::vector<COARES_LOC_CAMERA> device, std::vector<QString> color) {
//     for (int i = 0; i < device.size(); ++i) {
//         if (device[i] == COARES_LOC_CAMERA::CAMERA_1 && color.size() > i) {
//             ui->labelCoarseLocCamera1->setStyleSheet(color[i]);
//         } else if (device[i] == COARES_LOC_CAMERA::CAMERA_2 && color.size() > i) {
//             ui->labelCoarseLocCamera2->setStyleSheet(color[i]);
//         } else if (device[i] == COARES_LOC_CAMERA::CAMERA_3 && color.size() > i) {
//             ui->labelCoarseLocCamera3->setStyleSheet(color[i]);
//         } else if (device[i] == COARES_LOC_CAMERA::CAMERA_4 && color.size() > i) {
//             ui->labelCoarseLocCamera4->setStyleSheet(color[i]);
//         } else if (device[i] == COARES_LOC_CAMERA::CAMERA_5 && color.size() > i) {
//             ui->labelCoarseLocCamera5->setStyleSheet(color[i]);
//         } else if (device[i] == COARES_LOC_CAMERA::CAMERA_6 && color.size() > i) {
//             ui->labelCoarseLocCamera6->setStyleSheet(color[i]);
//         } else if (device[i] == COARES_LOC_CAMERA::CAMERA_UNCONNECTED) {
//             ui->labelCoarseLocCamera1->setStyleSheet(MY_COLOR::RED);
//             ui->labelCoarseLocCamera2->setStyleSheet(MY_COLOR::RED);
//             ui->labelCoarseLocCamera3->setStyleSheet(MY_COLOR::RED);
//             ui->labelCoarseLocCamera4->setStyleSheet(MY_COLOR::RED);
//             ui->labelCoarseLocCamera5->setStyleSheet(MY_COLOR::RED);
//             ui->labelCoarseLocCamera6->setStyleSheet(MY_COLOR::RED);
//         }
//     }
// }

// 获取到工作台点云
void WeldingMainWindow::whenGetWorkbenchPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    if (cloud) {
        // VTK界面点云显示
        pclVisualizer->removeAllShapes();       // 清空上次的shape显示
        pclVisualizer->removeAllPointClouds();  // 清空点云
        pclVisualizer->addPointCloud(cloud, "cloud");

        // 获取点云边界框大小
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*cloud, minPt, maxPt);
        Eigen::Vector3f center((maxPt.x + minPt.x) / 2, (maxPt.y + minPt.y) / 2, (maxPt.z + minPt.z) / 2);
        pclVisualizer->setCameraPosition(center(0), center(1), center(2) - 0.1, center(0), center(1), center(2), 0, -1, 0);
        pclVisualizer->resetCamera();
        ui->qvtkWidget->update();
    }
}

// 获取到检测完成的焊缝信息
void WeldingMainWindow::whenGetSeamInfo(std::vector<std::shared_ptr<WeldSeamInfo>> weldAreaInfo) {
    PLOGD << L"显示焊缝... ...";

    handleWeldAreaInfo2Display(weldAreaInfo, pclVisualizer, ObjDetImg);
    if (!ObjDetImg.empty()) {
        this->whenGetImg2Ui(ObjDetImg);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr visualCloud(new pcl::PointCloud<pcl::PointXYZ>);  // 用于显示的点云
    std::set<int> seamAreaPointCloudNum;

    for (auto& info : weldAreaInfo) {
        if (info->detectSuccFlag == true && info->weldAreaPointCloudInRobot && !info->weldAreaPointCloudInRobot->empty()) {
            if (seamAreaPointCloudNum.find(info->areaNum) == seamAreaPointCloudNum.end()) {  // 当前区域点云还未显示
                seamAreaPointCloudNum.insert(info->areaNum);
                *visualCloud = *visualCloud + *(info->weldAreaPointCloudInRobot);
            }

            if (info->detectSuccFlag == true && info->weldEndPointsInRobot != nullptr && info->weldEndPointsInRobot->size() >= 2) {
                ui->systemMirrorWidget->displayLines(info->weldEndPointsInRobot, {1.0, 0.0, 0.0});  // 在系统镜像中显示焊缝
            }
        }
    }

    ui->systemMirrorWidget->displayPointCloud(visualCloud, {0.0, 1.0, 0.0});  // 在系统镜像中显示点云
    ui->qvtkWidget->update();
}

// 获取到需要显示的信息
void WeldingMainWindow::whenGetMessage(QString message) { ui->textBrowser->append(message); }

// 获取到需要显示的图像
void WeldingMainWindow::whenGetImg2Ui(cv::Mat img) { ui->imageWidget->setOpenCVImage(img); }
// 收到机器人当前位姿
void WeldingMainWindow::whenGetRobotCurrentPose(robotPose p) {
    currRobotPose = p;

    ui->labelRobotCurrentX->setText(QString::number(p.x_));
    ui->labelRobotCurrentY->setText(QString::number(p.y_));
    ui->labelRobotCurrentZ->setText(QString::number(p.z_));
    ui->labelRobotCurrentA->setText(QString::number(p.a_));
    ui->labelRobotCurrentB->setText(QString::number(p.b_));
    ui->labelRobotCurrentC->setText(QString::number(p.c_));
}

// 收到机器人当前关节角
void WeldingMainWindow::whenGetRobotCurrentJointAngle(robotJointAngle j) {
    ui->labelRobotCurrentJoint1->setText(QString::number(j.joint1));
    ui->labelRobotCurrentJoint2->setText(QString::number(j.joint2));
    ui->labelRobotCurrentJoint3->setText(QString::number(j.joint3));
    ui->labelRobotCurrentJoint4->setText(QString::number(j.joint4));
    ui->labelRobotCurrentJoint5->setText(QString::number(j.joint5));
    ui->labelRobotCurrentJoint6->setText(QString::number(j.joint6));

    ui->systemMirrorWidget->setRobotJointAngle(j);
}
//======================粗定位=================
// 获得工件粗定位结果图
void WeldingMainWindow::whenGetWorkpieceRailMap(cv::Mat res) {
    // cv::imwrite("./data/paper/cImg.bmp", res);

    QImage img = QImage(res.data, res.cols, res.rows, res.step, QImage::Format_RGB888);

    QGraphicsScene* scene = ui->graphicsViewCoarseLoc->scene();                          // 获取 graphicsViewCoarseLoc 的场景
    QGraphicsPixmapItem* pixmapItem = new QGraphicsPixmapItem(QPixmap::fromImage(img));  // 创建 QGraphicsPixmapItem 并将图像添加到场景中

    // 清空场景并添加新的图像项到场景
    scene->clear();              // 清空场景上的所有项
    scene->addItem(pixmapItem);  // 添加图像项到场景
    ui->graphicsViewCoarseLoc->setOriginalImageInfo(res.cols, res.rows, railMapRotationAngle);

    // 更新视图 (如果没有立即显示，尝试刷新视图)
    ui->graphicsViewCoarseLoc->setScene(scene);  // 确保场景设置正确
}
// 获得粗定位界面点击位置
void WeldingMainWindow::whenGetWorkPieceCoord(int x, int y) { ui->label_coordinateLabel->setText(QString("X: %1, Y: %2").arg(x).arg(y)); }
// 获得粗定位相机序列号
void WeldingMainWindow::whenGetCoarseCameraSerial(std::vector<std::string> serialNum) {
    ui->comboBoxCoarseLocCamera->clear();
    for (auto& num : serialNum) {
        ui->comboBoxCoarseLocCamera->addItem(QString::fromStdString(num));
    }
}
void WeldingMainWindow::whenGetCoarseLocWorkpieceNum(int num) {
    ui->comboBoxCoarseLocInfo->clear();
    for (int i = 1; i <= num; ++i) {
        ui->comboBoxCoarseLocInfo->addItem(QString::number(i));
    }
}
// 获得粗定位信息用于显示在表格
void WeldingMainWindow::whenGetWeldCoarseLocInfo(std::vector<std::vector<QTableWidgetItem*>> info) {
    ui->tableWidgetCoarseLoc->clearContents();

    PLOGD << "收到粗定位表格信息";
    for (int i = 0; i < info.size(); ++i) {
        for (int j = 0; j < info[i].size(); ++j) {
            ui->tableWidgetCoarseLoc->setItem(i, j, info[i][j]);
        }
    }
    ui->tableWidgetCoarseLoc->resizeColumnsToContents();  // 自适应宽度
    ui->tableWidgetCoarseLoc->setCurrentCell(0, 0);
}
//----按钮----
void WeldingMainWindow::on_btnConnectStructLight_clicked() { emit connectStructLightCamera(); }

void WeldingMainWindow::on_btnDisConnectStructLight_clicked() { emit disconnectStructLightCamera(); }

void WeldingMainWindow::on_btnGlobalReconstruct_clicked() { emit sendGlobalReconstruct(); }

void WeldingMainWindow::on_btnLocalReconstruct_clicked() {
    emit sendLocalReconstruct(ui->lineEditReconstructMinU->text().toDouble(), ui->lineEditReconstructMaxU->text().toDouble(),
                              ui->lineEditReconstructMinV->text().toDouble(), ui->lineEditReconstructMaxV->text().toDouble());
}

void WeldingMainWindow::on_ProScan_clicked() { emit sendScanWorkpiece(); }

void WeldingMainWindow::on_btnConnectRobot_clicked() { this->railWeldingSystem->connectRobot(); }

void WeldingMainWindow::on_btnDisconnectRobot_clicked() { this->railWeldingSystem->disconnectRobot(); }

void WeldingMainWindow::on_btnRobotSavePose_clicked() {
    emit sendSaveRobotPose(robotPose(ui->labelRobotCurrentX->text().toDouble(), ui->labelRobotCurrentY->text().toDouble(),
                                     ui->labelRobotCurrentZ->text().toDouble(), ui->labelRobotCurrentA->text().toDouble(),
                                     ui->labelRobotCurrentB->text().toDouble(), ui->labelRobotCurrentC->text().toDouble()));
}

void WeldingMainWindow::on_pushButtonWelding_clicked() { this->railWeldingSystem->welding(); }

void WeldingMainWindow::on_comboBox_currentTextChanged(const QString& arg1) {
    if (arg1 == u8"模拟模式") {
        this->railWeldingSystem->robot->robotWorkMode = ROBOT_WORK_MODE::SIMULATION_MODE;
    } else if (arg1 == u8"焊接模式") {
        this->railWeldingSystem->robot->robotWorkMode = ROBOT_WORK_MODE::WELDING_MODE;
    }
}
void WeldingMainWindow::on_btnRobotMoveL_clicked() {
    robotPose p(ui->lineEditRobotTargetX->text().toDouble(), ui->lineEditRobotTargetY->text().toDouble(), ui->lineEditRobotTargetZ->text().toDouble(),
                ui->lineEditRobotTargetA->text().toDouble(), ui->lineEditRobotTargetB->text().toDouble(),
                ui->lineEditRobotTargetC->text().toDouble());

    this->railWeldingSystem->whenGetRobotMoveLData(p, SettingPara::getInstance().Value_MoveSpeed);
}

void WeldingMainWindow::on_combWorkpiece_currentTextChanged(const QString& arg1) {
    if (arg1 == u8"角钢") {
        this->railWeldingSystem->structLightCamera->workpieceType = WORKPIECE_TYPE::STEEL_ANGLE;
        this->railWeldingSystem->switchTrajectoryPlanning(WORKPIECE_TYPE::STEEL_ANGLE);
    } else if (arg1 == u8"龙门支架") {
        this->railWeldingSystem->structLightCamera->workpieceType = WORKPIECE_TYPE::GANTRAY_FRAME;
        this->railWeldingSystem->switchTrajectoryPlanning(WORKPIECE_TYPE::GANTRAY_FRAME);
    } else {
        this->railWeldingSystem->structLightCamera->workpieceType = WORKPIECE_TYPE::GANTRAY_FRAME;
        this->railWeldingSystem->switchTrajectoryPlanning(WORKPIECE_TYPE::GANTRAY_FRAME);
    }
}
