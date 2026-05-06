#include "BaslerControl.h"

#include <QObject>

#include "utils/stateLight/StateLight.h"

#pragma execution_character_set("utf-8")

int cameraIndex = 0;  // 相机索引
BaslerControl::BaslerControl() { this->loadCalibConfigFromFile(configFilePath); }

BaslerControl::~BaslerControl() { this->closeCamera(); }

void BaslerControl::openCamera() {
    S_Ns.clear();
    calibCameraIndex = 0;
    S_Ns.push_back("未选择相机");

    PLOGD << L"正在连接Basler相机... ...";
    emit appendCameraLog(QString(u8"正在连接Basler相机... ..."));
    Pylon::PylonInitialize();  // 初始化Pylon对象
    std::vector<COARES_LOC_CAMERA> coaresLocCamera;
    std::vector<QString> color;
    coaresLocCamera.push_back(COARES_LOC_CAMERA::CAMERA_1);
    coaresLocCamera.push_back(COARES_LOC_CAMERA::CAMERA_2);
    coaresLocCamera.push_back(COARES_LOC_CAMERA::CAMERA_3);

    // 判断相机连接情况
    if (!Pylon::CTlFactory::GetInstance().EnumerateDevices(device)) {
        PLOGE << L"未找到Basler相机，请检查相机连接情况";
        emit appendCameraLog(QString(u8"未找到Basler相机，请检查相机连接情况"));
        color.push_back(MY_COLOR::RED);
        color.push_back(MY_COLOR::RED);
        color.push_back(MY_COLOR::RED);
        emit sendCameraStatus(coaresLocCamera, color);  // 发送粗定位相机状态
        return;
    }

    for (size_t i = 0; i < device.size(); ++i) {
        std::string devSerial = device[i].GetSerialNumber();

        // 当前相机是配置文件中的第几个相机
        auto it = std::find(serialNum.begin(), serialNum.end(), devSerial);
        uint8_t index;
        if (it != serialNum.end()) {
            index = std::distance(serialNum.begin(), it);  // 计算索引位置
            if (index >= 0) {
                int skipNum = index - color.size();
                for (; skipNum > 0; skipNum--) {
                    color.push_back(MY_COLOR::RED);
                    emit sendCameraStatus(coaresLocCamera, color);  // 发送粗定位相机状态
                }
            }
        }

        if (std::find(serialNum.begin(), serialNum.end(), devSerial) != serialNum.end()) {
            PLOGD << L"尝试连接第" << i + 1 << L"台相机...";
            cameras[calibCameraIndex].Attach(Pylon::CTlFactory::GetInstance().CreateDevice(device[i]));  // 创建并连接相机

            try {
                PLOGD << L"尝试打开第" << i + 1 << L"台Basler相机...";
                emit appendCameraLog(QString(u8"尝试打开第%1台Basler相机...").arg(i + 1));
                cameras[i].Open();  // 打开相机
                PLOGD << L"已经打开第" << i + 1 << L"台Basler相机";
                emit appendCameraLog(QString(u8"已经打开第%1台Basler相机").arg(i + 1));

                color.push_back(MY_COLOR::GREEN);
                emit sendCameraStatus(coaresLocCamera, color);  // 发送粗定位相机状态
            } catch (...) {
                PLOGE << L"Basler相机连接失败, 可能存在其它程序正在使用相机";
                emit appendCameraLog(QString(u8"Basler相机连接失败, 可能存在其它程序正在使用相机"));

                color.push_back(MY_COLOR::RED);
                emit sendCameraStatus(coaresLocCamera, color);  // 发送粗定位相机状态
                continue;
            }
            // 设置相机曝光时间
            GenApi::INodeMap& cameraNodeMap = cameras[i].GetNodeMap();
            const GenApi::CFloatPtr exposureTime = cameraNodeMap.GetNode("ExposureTimeAbs");
            exposureTime->SetValue(exposure);

            cameras[i].StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);  // 启动抓取模式
            formatConverter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
            PLOGD << L"第" << i + 1 << L"台Basler相机连接成功";
            emit appendCameraLog(QString(u8"第%1台Basler相机连接成功").arg(i + 1));
            // std::string S_N = cameras[calibCameraIndex].GetDeviceInfo().GetSerialNumber();
            S_Ns.push_back(devSerial);
            calibCameraIndex++;
        }
    }
    emit sendSerialNumber(S_Ns);

    currentS_N = S_Ns[0];  // 初始化为未采图。
    int* savedImages = nullptr;

    while (cameraFlag) {
        cv::Mat cvImage = cv::Mat(1, 1, CV_8UC3, cv::Scalar(0, 0, 0));

        if (imageSaverToInfer > 0) {
            std::vector<cv::Mat> cvImages;
            for (int j = 0; j < calibCameraIndex; j++) {
                cameras[j].RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
                if (ptrGrabResult->GrabSucceeded()) {
                    formatConverter.Convert(pylonImage, ptrGrabResult);  // 将抓取的缓冲数据转化成pylonImage
                    cv::Mat cvImage =
                        cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)pylonImage.GetBuffer());

                    // 在inference统一保存原图与分割结果
                    // 获取当前时间
                    std::time_t now = std::time(nullptr);
                    std::tm* localTime = std::localtime(&now);
                    // 创建字符串流
                    std::ostringstream dateTimeStream;
                    // 格式化年月日时分秒，添加前导零
                    dateTimeStream << std::put_time(localTime, "%Y%m%d_%H%M%S");
                    cv::imwrite("./data/workpieceCoaLoc/infer/camera" + std::to_string(j) + "_" + dateTimeStream.str() + ".bmp",
                                cvImage);
                    std::cout << "camera" + std::to_string(j + 1) + " Save image in workpieceCoaLoc succ." << std::endl;
                    cvImages.push_back(cvImage.clone());
                }
            }
            emit sendCvImagesToInfer(cvImages);
            // PLOGE << L"图片保存成功";
            // savedImages++;
            imageSaverToInfer--;
        }
        if (currentS_N != u8"未选择相机") {
            int cameraIndexPtr = std::find(S_Ns.begin(), S_Ns.end(), currentS_N) - S_Ns.begin();
            PLOGD << "cameraIndexPtr: " << cameraIndexPtr;
            if (cameraIndex != cameraIndexPtr) {
                cameraIndex = cameraIndexPtr;
            }
            try {
                cameras[cameraIndex - 1].RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
                if (ptrGrabResult->GrabSucceeded()) {
                    formatConverter.Convert(pylonImage, ptrGrabResult);  // 将抓取的缓冲数据转化成pylonImage
                    cvImage =
                        cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)pylonImage.GetBuffer());
                    emit sendImageToView(cvImage);
                }

                if (imageNumberToSaveInCalibration > 0) {
                    std::string saveTypePath;
                    // cv::Size boardSize = cv::Size(BOARD_HEIGHT, BOARD_WIDTH);
                    std::vector<cv::Point2f> imagePointsBuf;
                    cv::Mat a = cvImage.clone();
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
                            saveTypePath = "compare";
                            savedImages = &savedTrackImages;
                            break;
                        default:
                            saveTypePath = "normal";
                            savedImages = &savednNormalImages;
                            break;
                    }

                    if ((*savedImages) < 10) {
                        cv::imwrite("./data/calib/camera" + std::to_string(cameraIndex) + "/" + saveTypePath + "/image0" +
                                        std::to_string((*savedImages)) + ".bmp",
                                    cvImage);
                    } else {
                        cv::imwrite("./data/calib/camera" + std::to_string(cameraIndex) + "/" + saveTypePath + "/image" +
                                        std::to_string((*savedImages)) + ".bmp",
                                    cvImage);
                    }

                    // std::cout << "camera"+std::to_string(cameraIndex)+"Save image" + std::to_string(savedImages) + " succ." <<
                    // std::endl;
                    QString logMessage = QString("Camera %1: Saved image %2 successfully at path: %3")
                                             .arg(cameraIndex)
                                             .arg((*savedImages))
                                             .arg(QString::fromStdString(saveTypePath));
                    appendCameraLog(logMessage);
                    PLOGE << L"图片保存成功";
                    (*savedImages)++;
                    imageNumberToSaveInCalibration--;
                }

                cv::waitKey(5);  // 防止采图卡顿

            } catch (...) {
                PLOGE << L"采图失败，退出采图程序。";
                break;
            }
        }
    }
}

void BaslerControl::closeCamera() {
    // std::thread::id this_id = std::this_thread::get_id();
    // std::cout << "Current thread id: " << this_id << std::endl;
    for (auto& camera : cameras) {
        camera.Close();
        camera.DetachDevice();
    }
    Pylon::PylonTerminate();

    PLOGD << L"Basler相机断开连接";
    emit appendCameraLog(QString(u8"Basler相机断开连接"));

    std::vector<COARES_LOC_CAMERA> coaresLocCamera;
    std::vector<QString> color;

    coaresLocCamera.push_back(COARES_LOC_CAMERA::CAMERA_1);
    color.push_back(MY_COLOR::RED);
    coaresLocCamera.push_back(COARES_LOC_CAMERA::CAMERA_2);
    color.push_back(MY_COLOR::RED);
    coaresLocCamera.push_back(COARES_LOC_CAMERA::CAMERA_3);
    color.push_back(MY_COLOR::RED);
    emit sendCameraStatus(coaresLocCamera, color);  // 发送粗定位相机状态
}

void BaslerControl::loadCalibConfigFromFile(const std::string& filename) {
    std::map<std::string, CoarseLocalizationMatrix> cameraConfigMap;

    std::ifstream is(filename);
    if (!is.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        throw std::runtime_error("无法打开配置文件：" + filename);
    }

    cereal::JSONInputArchive archive(is);
    archive(cereal::make_nvp("Cameras", cameraConfigMap));

    serialNum.clear();  // 假设是 std::vector<std::string> serialNum;

    // 分别保存三个相机序列号，安全起见先初始化为空字符串
    std::string serial1, serial2, serial3;

    if (cameraConfigMap.count("Camera1") && !cameraConfigMap["Camera1"].CameraSerialNum.empty())
        serial1 = cameraConfigMap["Camera1"].CameraSerialNum;
    if (cameraConfigMap.count("Camera2") && !cameraConfigMap["Camera2"].CameraSerialNum.empty())
        serial2 = cameraConfigMap["Camera2"].CameraSerialNum;
    if (cameraConfigMap.count("Camera3") && !cameraConfigMap["Camera3"].CameraSerialNum.empty())
        serial3 = cameraConfigMap["Camera3"].CameraSerialNum;

    serialNum.push_back(serial1);
    serialNum.push_back(serial2);
    serialNum.push_back(serial3);

    // 打印确认
    // std::cout << "Camera1 SerialNum: " << serial1 << std::endl;
    // std::cout << "Camera2 SerialNum: " << serial2 << std::endl;
    // std::cout << "Camera3 SerialNum: " << serial3 << std::endl;
}
