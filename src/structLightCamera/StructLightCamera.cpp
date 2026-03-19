#include "StructLightCamera.h"

#include "cameraFactory/AbstractCamera.h"
#include "cameraFactory/AbstractCameraFactory.h"
#include "projectFactory/AbstractProjector.h"
#include "projectFactory/AbstractProjectorFactory.h"
#include "settingPara/SettingPara.h"
#include "structLightCamera/config/StructLightConfig.h"
#include "structLightCamera/reconstruction/PointCloudReconstruction.h"
#include "utils/stateLight/StateLight.h"

StructLightCamera::StructLightCamera(QObject *parent) { (void)parent; }

// 带抽象工厂参数的构造函数
StructLightCamera::StructLightCamera(std::shared_ptr<AbstractCameraFactory> camFac,
                                     std::shared_ptr<AbstractProjectorFactory> projFac)
    : cameraFactory(camFac), projectorFactory(projFac) {
    this->initConfig();                    // 初始化配置信息 (此处各个模块初始化的顺序不能变)
    this->initCamerasAndProjector();       // 初始化相机和投影仪
    this->initPointCloudReconstruction();  // 初始化点云重建类
}

StructLightCamera::~StructLightCamera() {}

// 初始化配置信息
void StructLightCamera::initConfig() {
    // this->writeConfig();  // 写配置文件
    this->readConfig();  // 读配置文件
    // this->printConfig();  // 打印配置文件

    StructLightConfig::getInstance().myDataStructure2LibDataStructure();  // 自定义数据类型转换为库数据类型
}

// 写配置文件
void StructLightCamera::writeConfig() {
    {  // 写
        std::ofstream os("./data/config/struct_light_config.json");
        cereal::JSONOutputArchive jsonOutputArchive(os);
        jsonOutputArchive(cereal::make_nvp("config about structLight camera", StructLightConfig::getInstance()));
    }
}

// 读配置文件
void StructLightCamera::readConfig() {
    {  // 读
        std::ifstream is("./data/config/struct_light_config.json");
        cereal::JSONInputArchive inputArchive(is);
        inputArchive(StructLightConfig::getInstance());
        PLOGD << "结构光相机配置文件读取成功";
    }
}

// 打印配置文件
void StructLightCamera::printConfig() {
    {  // 打印
        cereal::JSONOutputArchive jsonOutputArchive(std::cout);
        jsonOutputArchive(cereal::make_nvp("config about structLight camera", StructLightConfig::getInstance()));
        std::cout << std::endl;
    }
}

// 初始化相机和投影仪
void StructLightCamera::initCamerasAndProjector() {
    if (cameraFactory) {
        if (StructLightConfig::getInstance().getPrimaryCameraSerialNum() != "00000000") {
            primaryCamera = cameraFactory->createCamera();  // 工厂创建相机
            primaryCamera->moveToThread(primaryCameraThread);
            primaryCameraThread->start();  // 线程启动
            connect(this, &StructLightCamera::cameraStartGrabbing, primaryCamera.get(), &AbstractCamera::start);
            connect(primaryCamera.get(), &AbstractCamera::sendImage, this, &StructLightCamera::whenGetPrimaryCameraImage);

            PLOGD << "主相机初始化成功";
        }
        if (StructLightConfig::getInstance().getSecondaryCameraSerialNum() != "00000000") {
            secondaryCamera = cameraFactory->createCamera();  // 工厂创建相机
            secondaryCamera->moveToThread(secondaryCameraThread);
            secondaryCameraThread->start();  // 线程启动
            connect(this, &StructLightCamera::cameraStartGrabbing, secondaryCamera.get(), &AbstractCamera::start);
            connect(secondaryCamera.get(), &AbstractCamera::sendImage, this, &StructLightCamera::whenGetSecondaryCameraImage);

            PLOGD << "次相机初始化成功";
        }
    } else {
        PLOGE << "相机工厂未初始化, 无法初始化相机";
    }

    if (projectorFactory) {
        projector = projectorFactory->createProjector();  // 工厂创建投影仪
        projector->moveToThread(projectorThread);
        projectorThread->start();  // 线程启动
        connect(this, &StructLightCamera::sendTriggerProj, projector.get(), &AbstractProjector::triggerProj);

        PLOGD << "投影仪初始化成功";
    } else {
        PLOGE << "投影仪工厂未初始化, 无法初始化投影仪";
    }
}

// 初始化点云重建类
void StructLightCamera::initPointCloudReconstruction() {
    pointCloudReconstruction = std::make_shared<PointCloudReconstruction>();
    pointCloudReconstruction->primaryCameraCapturedImg = this->primaryCameraCapturedImg;
    pointCloudReconstruction->secondaryCameraCapturedImg = this->secondaryCameraCapturedImg;
}

// 打开相机和投影仪
uint8_t StructLightCamera::open() {
    deviceStatus = 0;            // 三个硬件设备的状态
    std::vector<DEVICE> device;  // 用于修改UI页面设备指示灯的两个容器
    std::vector<QString> color;

    if (primaryCamera) {
        if (primaryCamera->open(StructLightConfig::getInstance().getPrimaryCameraSerialNum().data())) {
            primaryCamera->setWorkMode(CAMERA_WORK_MODE::HARDWARE_TRIGGER);  // 设置工作模式硬件触发
            whenGetCameraExposure(SettingPara::getInstance().camera_exposure);
            deviceStatus |= DEVICE::PRIMARY_CAMERA;

            device.push_back(DEVICE::PRIMARY_CAMERA);
            color.push_back(MY_COLOR::GREEN);
            emit sendMessage2Ui(u8"主相机连接成功");
        } else {
            device.push_back(DEVICE::PRIMARY_CAMERA);
            color.push_back(MY_COLOR::RED);
            emit sendMessage2Ui(u8"主相机连接失败");
        }
    } else {
        device.push_back(DEVICE::PRIMARY_CAMERA);
        color.push_back(MY_COLOR::RED);
        PLOGE << "结构光主相机未初始化";  // 主相机必须有
    }
    emit sendStructLightStatus(device, color);  // 发送设备状态

    if (StructLightConfig::getInstance().getSecondaryCameraSerialNum() != "00000000") {  // 没有配置则不需要连
        if (secondaryCamera) {
            if (secondaryCamera->open(StructLightConfig::getInstance().getSecondaryCameraSerialNum().data())) {
                secondaryCamera->setWorkMode(CAMERA_WORK_MODE::HARDWARE_TRIGGER);  // 设置工作模式硬件触发
                whenGetCameraExposure(SettingPara::getInstance().camera_exposure);
                deviceStatus |= DEVICE::SECONDARY_CAMERA;

                device.push_back(DEVICE::SECONDARY_CAMERA);
                color.push_back(MY_COLOR::GREEN);
                emit sendMessage2Ui(u8"次相机连接成功");
            } else {
                device.push_back(DEVICE::SECONDARY_CAMERA);
                color.push_back(MY_COLOR::RED);
                emit sendMessage2Ui(u8"次相机连接失败");
            }
        } else {
            device.push_back(DEVICE::SECONDARY_CAMERA);
            color.push_back(MY_COLOR::RED);
            PLOGW << "结构光次相机未初始化";  // 次相机非必须
        }
    } else {
        device.push_back(DEVICE::SECONDARY_CAMERA);
        color.push_back(MY_COLOR::GRAY);
    }
    emit sendStructLightStatus(device, color);  // 发送设备状态

    if (projector) {
        if (projector->open()) {
            deviceStatus |= DEVICE::PROJECTOR;

            device.push_back(DEVICE::PROJECTOR);
            color.push_back(MY_COLOR::GREEN);
            emit sendMessage2Ui(u8"投影仪连接成功");
        } else {
            device.push_back(DEVICE::PROJECTOR);
            color.push_back(MY_COLOR::RED);
            emit sendMessage2Ui(u8"投影仪连接失败");
        }
    } else {
        device.push_back(DEVICE::PROJECTOR);
        color.push_back(MY_COLOR::RED);
        PLOGE << "结构光投影仪未初始化";  // 投影仪必须有
    }
    emit sendStructLightStatus(device, color);  // 发送设备状态

    emit cameraStartGrabbing();  // 发射信号, 开始采图

    return deviceStatus;
}

// 关闭相机和投影仪
void StructLightCamera::close() {
    std::vector<DEVICE> device;
    std::vector<QString> color;

    if (primaryCamera) {
        primaryCamera->close();
        device.push_back(DEVICE::PRIMARY_CAMERA);
        color.push_back(MY_COLOR::RED);
        emit sendMessage2Ui(u8"主相机断开连接");
    } else {
        PLOGE << "结构光主相机未初始化";
    }

    if (projector) {
        projector->close();
        device.push_back(DEVICE::PROJECTOR);
        color.push_back(MY_COLOR::RED);
        emit sendMessage2Ui(u8"投影仪断开连接");
    } else {
        PLOGE << "结构光次相机未初始化";
    }

    if (StructLightConfig::getInstance().getSecondaryCameraSerialNum() != "00000000") {  // 没有配置则不需要关
        if (secondaryCamera) {
            secondaryCamera->close();
            device.push_back(DEVICE::SECONDARY_CAMERA);
            color.push_back(MY_COLOR::RED);
            emit sendMessage2Ui(u8"次相机断开连接");
        } else {
            PLOGE << "结构光投影仪未初始化";
        }
    }

    emit sendStructLightStatus(device, color);  // 发送设备状态
}

// 从主相机获取到图像
void StructLightCamera::whenGetPrimaryCameraImage(cv::Mat img, CAMERA_WORK_MODE workMode) {
    if (workMode == CAMERA_WORK_MODE::HARDWARE_TRIGGER) {  // 硬件模式才存图到容器重建
        std::cout << "主相机获取到图像: " << primaryCameraCapturedImg->size() + 1 << std::endl;
        primaryCameraCapturedImg->push_back(img.clone());
        if (primaryCameraCapturedImg->size() == this->projector->projNum) {  // 采集到目标数量
            cameraImagCapturedStatus |= DEVICE::PRIMARY_CAMERA;              // 主相机采集到对应数量图像
            if (deviceStatus & DEVICE::SECONDARY_CAMERA) {                   // 还存在次相机
                if (cameraImagCapturedStatus & DEVICE::SECONDARY_CAMERA) {   // 次相机也采集到对应数量图像图像
                    projector->closeLed();                                   // 关闭投影仪LED灯
                    PLOGD << "采集到两组图像, 进行重建...";
                    // 发出采集到的两组图像进行重建
                    if (reconstructionMode == RECONSTRUCTION_MODE::WORKPIECE) {
                    } else if (reconstructionMode == RECONSTRUCTION_MODE::COMMON) {
                    }
                }
            } else {                    // 没有次相机
                projector->closeLed();  // 关闭投影仪LED灯
                PLOGD << "采集到一组图像, 进行重建...";
                if (reconstructionMode == RECONSTRUCTION_MODE::WORKPIECE) {
                    if (workpieceType == WORKPIECE_TYPE::STEEL_ANGLE) {
                        PLOGD << "重建角钢工件";
                        weldAreaInfo = pointCloudReconstruction->weldAreaReconstructToSA();   // 焊缝区域点云重建
                        workbenchPointCloud = pointCloudReconstruction->workbenchPointCloud;  // 获取工作台平面点云

                        QString message = QString(QStringLiteral("拟合平面的参数为: %1, %2, %3, %4"))
                                              .arg(QString::number(pointCloudReconstruction->workbenchCoeff[0], 'f', 6),
                                                   QString::number(pointCloudReconstruction->workbenchCoeff[1], 'f', 6),
                                                   QString::number(pointCloudReconstruction->workbenchCoeff[2], 'f', 6),
                                                   QString::number(pointCloudReconstruction->workbenchCoeff[3], 'f', 6));
                        emit sendMessage2Ui(message);              // 发送信息到UI界面
                        emit sendPointCloud(workbenchPointCloud);  // 发送工作台点云
                        emit sendWeldAreaInfo(weldAreaInfo);       // 发送焊缝区域点云
                    } else if (workpieceType == WORKPIECE_TYPE::LARGE_WORKPIECE) {
                        PLOGD << "重建三轴工件";
                        weldAreaInfo = pointCloudReconstruction->weldAreaReconstructToLW();   // 焊缝区域点云重建
                        workbenchPointCloud = pointCloudReconstruction->workbenchPointCloud;  // 获取工作台平面点云

                        QString message = QString(QStringLiteral("拟合平面的参数为: %1, %2, %3, %4"))
                                              .arg(QString::number(pointCloudReconstruction->workbenchCoeff[0], 'f', 6),
                                                   QString::number(pointCloudReconstruction->workbenchCoeff[1], 'f', 6),
                                                   QString::number(pointCloudReconstruction->workbenchCoeff[2], 'f', 6),
                                                   QString::number(pointCloudReconstruction->workbenchCoeff[3], 'f', 6));
                        emit sendMessage2Ui(message);              // 发送信息到UI界面
                        emit sendPointCloud(workbenchPointCloud);  // 发送工作台点云
                        emit sendWeldAreaInfoLW(weldAreaInfo);     // 发送焊缝区域点云
                    }

                } else if (reconstructionMode == RECONSTRUCTION_MODE::COMMON) {
                    pointCloud = pointCloudReconstruction->localReconstruct(
                        normMinU * pointCloudReconstruction->cameraWidth, normMaxU * pointCloudReconstruction->cameraWidth,
                        normMinV * pointCloudReconstruction->cameraHeight, normMaxV * pointCloudReconstruction->cameraHeight);

                    emit sendPointCloud(pointCloud);  // 发送普通点云
                }
            }
        }
    } else if (workMode == CAMERA_WORK_MODE::SOFTWARE_TRIGGER) {  // 软件模式不存图到容器
        // std::cout << "主相机获取到图像" << std::endl;
    }
}

// 从次相机获取到图像
void StructLightCamera::whenGetSecondaryCameraImage(cv::Mat img, CAMERA_WORK_MODE workMode) {
    if (workMode == CAMERA_WORK_MODE::HARDWARE_TRIGGER) {  // 硬件模式才存图到容器重建
        std::cout << "次相机获取到图像: " << secondaryCameraCapturedImg->size() + 1 << std::endl;
        secondaryCameraCapturedImg->push_back(img.clone());
        if (secondaryCameraCapturedImg->size() == this->projector->projNum) {  // 采集到目标数量
            cameraImagCapturedStatus |= DEVICE::SECONDARY_CAMERA;              // 次相机采集到对应数量图像
            if (cameraImagCapturedStatus & DEVICE::PRIMARY_CAMERA) {           // 主相机也采集到对应数量图像图像
                projector->closeLed();                                         // 关闭投影仪LED灯
                PLOGD << "采集到两组图像, 进行重建...";
                // 发出采集到的两组图像进行重建
                if (reconstructionMode == RECONSTRUCTION_MODE::WORKPIECE) {
                } else if (reconstructionMode == RECONSTRUCTION_MODE::COMMON) {
                }
            }
        }
    } else if (workMode == CAMERA_WORK_MODE::SOFTWARE_TRIGGER) {  // 软件模式不存图到容器
        // std::cout << "次相机获取到图像" << std::endl;
    }
}

// 相机曝光改变
void StructLightCamera::whenGetCameraExposure(int exposure) {
    if (primaryCamera) {
        primaryCamera->setPara("ExposureTimeRaw", (int64_t)exposure);
    } else {
        PLOGE << "结构光主相机未初始化";
    }

    if (StructLightConfig::getInstance().getSecondaryCameraSerialNum() != "00000000") {  // 没有配置则不需要关
        if (secondaryCamera) {
            secondaryCamera->setPara("ExposureTimeRaw", (int64_t)exposure);
        } else {
            PLOGE << "结构光次相机未初始化";
        }
    }
}

// 相机增益改变
void StructLightCamera::whenGetCameraGain(int gain) {
    if (primaryCamera) {
        primaryCamera->setPara("GainRaw", (int64_t)gain);
    } else {
        PLOGE << "结构光主相机未初始化";
    }

    if (StructLightConfig::getInstance().getSecondaryCameraSerialNum() != "00000000") {  // 没有配置则不需要关
        if (secondaryCamera) {
            secondaryCamera->setPara("GainRaw", (int64_t)gain);
        } else {
            PLOGE << "结构光次相机未初始化";
        }
    }
}

// 相机工作模式改变
void StructLightCamera::whenGetCameraWorkMode(CAMERA_WORK_MODE workMode) {
    if (primaryCamera) {
        primaryCamera->stop();
        primaryCamera->setWorkMode(workMode);
    } else {
        PLOGE << "结构光主相机未初始化";
    }

    if (StructLightConfig::getInstance().getSecondaryCameraSerialNum() != "00000000") {  // 没有配置则不需要关
        if (secondaryCamera) {
            secondaryCamera->stop();
            secondaryCamera->setWorkMode(workMode);
        } else {
            PLOGE << "结构光次相机未初始化";
        }
    }

    emit cameraStartGrabbing();  // 发射信号, 开始采图
}

// 投影仪亮度改变
void StructLightCamera::whenGetProjectorBrightness(int brightness) {
    if (projector) {
        projector->setPara("brightness", brightness);
    } else {
        PLOGE << "结构光投影仪未初始化";
    }
}

// 投影仪帧率改变
void StructLightCamera::whenGetProjectorFps(int fps) {
    if (projector) {
        projector->setPara("fps", fps);
    } else {
        PLOGE << "结构光投影仪未初始化";
    }
}

// 投影仪图片数量改变
void StructLightCamera::whenGetProjectorNum(int num) {
    if (projector) {
        projector->setPara("projNum", num);
    } else {
        PLOGE << "结构光投影仪未初始化";
    }
}

// 全局点云重建
void StructLightCamera::whenGlobalReconstruct() { whenLocalReconstruct(0, 1, 0, 1); }

// 局部点云重建
void StructLightCamera::whenLocalReconstruct(double minU, double maxU, double minV, double maxV) {
    primaryCameraCapturedImg->clear();  // 清空图像采集容器
    secondaryCameraCapturedImg->clear();
    cameraImagCapturedStatus = 0x00;  // 清空采图状态标志量

    this->normMinU = minU;
    this->normMaxU = maxU;
    this->normMinV = minV;
    this->normMaxV = maxV;

    reconstructionMode = RECONSTRUCTION_MODE::COMMON;  // 设置重建模式为普通重建
    emit sendTriggerProj();
}

// 扫描工件逻辑
void StructLightCamera::whenScanWorkpiece() {
    primaryCameraCapturedImg->clear();  // 清空图像采集容器
    secondaryCameraCapturedImg->clear();
    cameraImagCapturedStatus = 0x00;  // 清空采图状态标志量

    reconstructionMode = RECONSTRUCTION_MODE::WORKPIECE;  // 设置重建模式为工件
    emit sendTriggerProj();
}
