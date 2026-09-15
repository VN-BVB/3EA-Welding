#include "CoarsePositioningCamera.h"

#include "utils/stateLight/StateLight.h"
#pragma execution_character_set("utf-8")
int cameraIndex = 0;

CoarsePositioningCamera::CoarsePositioningCamera() : cameraFactory(std::make_shared<BaslerCameraFactory>(nullptr)) {
    this->loadCalibConfigFromFile(configFilePath);
}

CoarsePositioningCamera::~CoarsePositioningCamera() { this->closeCamera(); }

void CoarsePositioningCamera::openCamera() {
    if (!cameras.empty()) {
        closeCamera();
        cameras.clear();
    }

    S_Ns.clear();
    calibCameraIndex = 0;
    S_Ns.push_back("未选择相机");

    std::vector<COARES_LOC_CAMERA> coaresLocCamera(serialNum.size());
    std::vector<QString> color(serialNum.size());

    PLOGD << L"正在连接粗定位Basler相机... ...";
    emit appendCameraLog(QString(u8"正在连接Basler相机... ..."));

    for (size_t i = 0; i < serialNum.size(); ++i) {
        std::string devSerial = serialNum[i];
        if (devSerial.empty()) continue;

        uint8_t index = 0x01 << i;
        switch (index) {
        case COARES_LOC_CAMERA::CAMERA_1:
            coaresLocCamera[i] = COARES_LOC_CAMERA::CAMERA_1;
            break;
        case COARES_LOC_CAMERA::CAMERA_2:
            coaresLocCamera[i] = COARES_LOC_CAMERA::CAMERA_2;
            break;
        default:
            break;
        }

        std::shared_ptr<AbstractCamera> cam = cameraFactory->createCamera();
        cam->moveToThread(sharedCameraThread);

        if (cam->open(devSerial.c_str())) {
            cam->setWorkMode(CAMERA_WORK_MODE::SOFTWARE_TRIGGER);
            cam->setPara("ExposureTimeRaw", (int64_t)exposure);
            connect(cam.get(), &AbstractCamera::sendImage, this, &CoarsePositioningCamera::whenGetCameraImage);
            cameras.push_back(cam);
            S_Ns.push_back(devSerial);
            color[i] = MY_COLOR::GREEN;
            emit appendCameraLog(QString(u8"相机 %1 连接成功").arg(QString::fromStdString(devSerial)));
        } else {
            color[i] = MY_COLOR::RED;
            emit appendCameraLog(QString(u8"相机 %1 连接失败").arg(QString::fromStdString(devSerial)));
        }
    }

    emit sendCameraStatus(coaresLocCamera, color);
    emit sendSerialNumber(S_Ns);

    currentS_N = S_Ns[0];
    savedImages = nullptr;
    sharedCameraThread->start();
}

void CoarsePositioningCamera::whenCameraImageInfer() {
    QEventLoop imageWaitLoop;
    connect(this, &CoarsePositioningCamera::imageReady, &imageWaitLoop, &QEventLoop::quit);

    if (cameras.empty() || currentS_N == "未选择相机") {
        PLOGE << L"推理失败，相机未连接或未选择。";
        emit appendCameraLog(QString(u8"推理失败，相机未连接或未选择。"));
        return;
    }

    // 找到当前选中的相机
    auto it = std::find(S_Ns.begin(), S_Ns.end(), currentS_N);
    if (it == S_Ns.end()) {
        PLOGE << L"未找到选中的相机。";
        return;
    }
    int idx = it - S_Ns.begin() - 1;  // 减1是因为 S_Ns[0] 是"未选择相机"

    imagesReceived.clear();
    receivedImages = 0;

    // 停止旧连接
    disconnect(this, &CoarsePositioningCamera::cameraStartGrabbing, cameras[idx].get(), &AbstractCamera::start);
    cameras[idx]->stop();

    // 只对选中的相机采一张图
    imageSaverToInfer = 1;
    connect(this, &CoarsePositioningCamera::cameraStartGrabbing, cameras[idx].get(), &AbstractCamera::start);
    emit cameraStartGrabbing();
    imageWaitLoop.exec();
    disconnect(this, &CoarsePositioningCamera::cameraStartGrabbing, cameras[idx].get(), &AbstractCamera::start);
    cameras[idx]->stop();

    if (saveEveryImg) {
        std::time_t now = std::time(nullptr);
        std::tm* localTime = std::localtime(&now);
        std::ostringstream dateTimeStream;
        dateTimeStream << std::put_time(localTime, "%Y%m%d_%H%M%S");
        for (size_t i = 0; i < imagesReceived.size(); ++i) {
            std::string filename = "./data/workpieceCoaLoc/infer/camera" + std::to_string(i) + "_" + dateTimeStream.str() +
                                   "_ori.bmp";
            cv::imwrite(filename, imagesReceived[i]);
            PLOGD << L"camera" << (i + 1) << L" Save image in workpieceCoaLoc succ.";
        }
    }
    emit sendCvImagesToInfer(imagesReceived);
}

void CoarsePositioningCamera::whenChooseCameraToShow() {
    for (int i = 0; i < cameras.size(); ++i) {
        disconnect(this, &CoarsePositioningCamera::cameraStartGrabbing, cameras[i].get(), &AbstractCamera::start);
        cameras[i]->stop();
    }
    int cameraIndexPtr = std::find(S_Ns.begin(), S_Ns.end(), currentS_N) - S_Ns.begin();
    PLOGD << L"choose cameraIndexPtr: " << cameraIndexPtr;
    if (cameraIndex != cameraIndexPtr) {
        cameraIndex = cameraIndexPtr;
    }
    if (cameraIndex > 0 && cameraIndex <= static_cast<int>(cameras.size())) {
        int selectedIndex = cameraIndex - 1;  // S_Ns 比 cameras 多一个 "未选择相机"
        if (cameras[selectedIndex]) {
            connect(this, &CoarsePositioningCamera::cameraStartGrabbing, cameras[selectedIndex].get(), &AbstractCamera::start);
            emit cameraStartGrabbing();
            PLOGD << L"Started camera index: " << selectedIndex;
        }
    } else {
        cv::Mat blackImage = cv::Mat::zeros(480, 640, CV_8UC3);
        emit sendImageToView(blackImage);
    }
}

void CoarsePositioningCamera::closeCamera() {
    S_Ns.clear();
    std::vector<COARES_LOC_CAMERA> coaresLocCamera(cameras.size(), COARES_LOC_CAMERA::CAMERA_UNCONNECTED);
    std::vector<QString> color(cameras.size(), MY_COLOR::RED);
    for (size_t i = 0; i < cameras.size(); ++i) {
        if (cameras[i]) {
            cameras[i]->stop();
            cameras[i]->close();
        }
    }

    emit sendCameraStatus(coaresLocCamera, color);
    cameras.clear();
    PLOGD << L"Basler相机断开连接";
    emit appendCameraLog(QString(u8"Basler相机断开连接"));
}

void CoarsePositioningCamera::whenGetCameraImage(cv::Mat img, CAMERA_WORK_MODE workMode) {
    if (workMode == CAMERA_WORK_MODE::HARDWARE_TRIGGER) {         // 硬件模式才存图到容器重建
    } else if (workMode == CAMERA_WORK_MODE::SOFTWARE_TRIGGER) {  // 软件模式不存图到容器
        emit sendImageToView(img.clone());
        if (imageSaverToInfer > 0) {
            cv::Mat rgbImg;
            if (img.channels() == 1) {
                cv::cvtColor(img, rgbImg, cv::COLOR_GRAY2BGR);  // 灰度图转 COLOR_GRAY2BGR
            }

            imagesReceived.push_back(rgbImg);
            imageSaverToInfer--;
            emit imageReady();  // 发出当前图片采集完成信号
        }
        if (imageNumberToSaveInCalibration > 0) {
            std::string saveTypePath;
            switch (saveTypeEnable) {
            case 0:
                saveTypePath = "img";
                savedImages = &savedCalibImages;
                break;
            case 1:
                saveTypePath = "plane";
                savedImages = &savedPlaneImages;
                break;
            case 2:
                saveTypePath = "x_axis";
                savedImages = &savedTrackImages;
                break;
            case 3:
                saveTypePath = "y_axis";
                savedImages = &savedTrackImages;
                break;
            case 4:
                saveTypePath = "z_axis";
                savedImages = &savedTrackImages;
                break;
            default:
                saveTypePath = "normal";
                savedImages = &savednNormalImages;
                break;
            }
            if ((*savedImages) < 10) {
                cv::imwrite("./data/workpieceCoaLoc/saveImg/camera" + std::to_string(cameraIndex) + "/" + saveTypePath + "/image0" +
                                std::to_string((*savedImages)) + ".bmp",
                            img.clone());
            } else {
                cv::imwrite("./data/workpieceCoaLoc/saveImg/camera" + std::to_string(cameraIndex) + "/" + saveTypePath + "/image" +
                                std::to_string((*savedImages)) + ".bmp",
                            img.clone());
            }
            QString logMessage = QString("Camera %1: Saved image %2 successfully at path: %3")
                                     .arg(cameraIndex)
                                     .arg((*savedImages))
                                     .arg(QString::fromStdString(saveTypePath));
            appendCameraLog(logMessage);
            PLOGE << L"图片保存成功";
            (*savedImages)++;
            imageNumberToSaveInCalibration--;
        }
    }
}

void CoarsePositioningCamera::loadCalibConfigFromFile(const std::string& filename) {
    serialNum.clear();

    //读粗定位相机序列号
    {
        std::ifstream is(filename);
        if (!is.is_open()) {
            std::cerr << "Failed to open file: " << filename << std::endl;
            throw std::runtime_error("无法打开配置文件：" + filename);
        }
        cereal::JSONInputArchive archive(is);

        CoarseLocalizationMatrix calibResult;
        archive(cereal::make_nvp("CalibrationResult", calibResult));

        if (!calibResult.CameraSerialNum.empty()) {
            serialNum.push_back(calibResult.CameraSerialNum);
        }
    }

    //读精定位相机序列号
    {
        std::ifstream is("./data/config/struct_light_config.json");
        if (is.is_open()) {
            cereal::JSONInputArchive archive(is);
            CalibConfig config;
            archive(config);
            if (!config.primaryCameraSerialNum.empty()) {
                serialNum.push_back(config.primaryCameraSerialNum);
            }
        }
    }
}

// 相机曝光改变
void CoarsePositioningCamera::whenGetCameraExposure(int inputexposure) {
    for (auto& cam : cameras) {
        if (cam) {
            cam->setPara("ExposureTimeRaw", (int64_t)inputexposure);
        } else {
            PLOGE << "相机未初始化";
        }
    }
    // exposure = inputexposure;
}
