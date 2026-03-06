#include "BaslerCamera.h"

BaslerCamera::BaslerCamera(QObject *parent) { (void)parent; }

// 设置工作模式
bool BaslerCamera::setWorkMode(CAMERA_WORK_MODE mode) {
    if (mode == CAMERA_WORK_MODE::HARDWARE_TRIGGER) {
        this->setPara("TriggerSelector", "FrameStart");
        this->setPara("TriggerMode", "On");
        this->setPara("TriggerSource", "Line1");
        this->workMode = CAMERA_WORK_MODE::HARDWARE_TRIGGER;

        PLOGD << "设置Basler相机为硬件触发模式";
    } else if (mode == CAMERA_WORK_MODE::SOFTWARE_TRIGGER) {
        this->setPara("TriggerSelector", "FrameStart");
        this->setPara("TriggerMode", "Off");
        this->workMode = CAMERA_WORK_MODE::SOFTWARE_TRIGGER;

        PLOGD << "设置Basler相机为软件触发模式";
    } else {
        PLOGE << "设置Basler相机工作模式失败, 不支持的工作模式";
        return false;
    }

    return true;
}

bool BaslerCamera::open() {
    PLOGD << "打开Basler相机";
    return true;
}

bool BaslerCamera::open(const char *serialNum) {
    if (this->opening == false) {  // 当前相机还没有连接
        Pylon::PylonInitialize();  // 初始化Pylon运行时
        formatConverter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
        try {
            if (!Pylon::CTlFactory::GetInstance().EnumerateDevices(device)) {  // 判断相机硬件连接情况
                PLOGE << "未找到任何Basler相机, 请检查相机连接情况";
                return false;
            }
            for (int i = 0; i < device.size(); ++i) {  // 找到目标序列号相机
                if (device[i].GetSerialNumber() == serialNum) {
                    mCamera = new Pylon::CInstantCamera(Pylon::CTlFactory::GetInstance().CreateDevice(device[i]));  // 关联相机
                    if (mCamera != nullptr) {
                        nodemap = &mCamera->GetNodeMap();  // 获取节点映射
                        mCamera->Open();                   // 打开相机

                        opening = true;
                        Pylon::CIntegerParameter(nodemap, "GevSCPSPacketSize").SetValue(7000);  // 调整网络传输packet size
                        Pylon::CIntegerParameter(nodemap, "GevSCPD").SetValue(1000);            // 调整网络传输延迟
                        PLOGD << "已连接Basler相机, 序列号: " << serialNum;
                        break;
                    } else {
                        PLOGE << "创建Basler相机对象失败";
                        return false;
                    }
                }
                if (i == device.size() - 1) {
                    PLOGE << "未找到序列号为" << serialNum << "的Basler相机";
                    return false;
                }
            }
        } catch (const Pylon::GenericException &e) {
            PLOGE << "Pylon相关接口执行失败: " << e.GetDescription();
            std::cin.ignore(std::cin.rdbuf()->in_avail());
            opening = false;
            return false;
        }
    } else {
        PLOGD << "Basler相机已经连接, 无需再连, 序列号: " << serialNum;
        return true;
    }

    return true;
}

bool BaslerCamera::start() {
    if (opening == true && running == false && mCamera != nullptr && mCamera->CanWaitForFrameTriggerReady()) {
        mCamera->StartGrabbing(Pylon::GrabStrategy_OneByOne, Pylon::GrabLoop_ProvidedByUser);  // 开始相机采图
        running = true;
        PLOGD << "Basler相机开始采图";

        // 开始循环采图
        while (running == true && mCamera->IsGrabbing()) {
            try {
                // 获取图像, 超时时间设置为900000000ms, 超时则返回并往下继续执行, 但仍然会抛出异常
                mCamera->RetrieveResult(900000000, ptrGrabResult, Pylon::TimeoutHandling_Return);
                if (ptrGrabResult->GrabSucceeded()) {  // 采图成功
                    const uint8_t *pImageBuffer = (uint8_t *)ptrGrabResult->GetBuffer();
                    cvImg = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8U, (void *)pImageBuffer);
                    // formatConverter.Convert(pylonImage, ptrGrabResult);  // 将抓取的缓冲数据转化成pylonImage
                    // cv::Mat cvImg = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t
                    // *)pylonImage.GetBuffer());

                    emit sendImage(cvImg.clone(), this->workMode);  // 发出图像
                }
            } catch (const Pylon::GenericException &e) {
                using namespace std::chrono;
                auto now = steady_clock::now();  // 获取当前时间

                // 如果是第一次异常 或 距离上次记录超过1秒
                if (firstGrabbingException || (now - lastGrabbingExceptionTime) >= 1s) {
                    PLOGE << "Basler相机采图异常: " << e.GetDescription();
                    lastGrabbingExceptionTime = now;  // 更新记录时间
                    firstGrabbingException = false;   // 标记已处理过首次异常
                }
            }
        }

        return true;
    } else {
        PLOGE << "Basler相机无法开始采图, 可能未初始化, 或未连接, 或已在采图中";
        return false;
    }
}

void BaslerCamera::stop() {
    if (mCamera != nullptr && running == true) {
        running = false;
        mCamera->StopGrabbing();  // 停止相机采图
    }
    PLOGD << "Basler相机终止采图";
}

void BaslerCamera::close() {
    if (mCamera != nullptr) {
        mCamera->Close();   // 关闭相机
        delete mCamera;     // 删除相机对象
        mCamera = nullptr;  // 清空相机指针
        nodemap = nullptr;  // 清空节点映射指针
        running = false;
        opening = false;
    }
    PLOGD << "已关闭Basler相机";
}

bool BaslerCamera::getPara(const char *nameNode, double &para) {
    if (nodemap) {
        GenApi::CFloatPtr tmp(nodemap->GetNode(nameNode));
        para = tmp->GetMin();
        PLOGD << "Basler相机参数: " << nameNode << " = " << para << " 获取成功";
        return true;
    }

    PLOGE << "Basler相机节点映射指针未初始化";
    return false;
}

bool BaslerCamera::getPara(const char *nameNode, int64_t &para) {
    if (nodemap) {
        GenApi::CIntegerPtr tmp(nodemap->GetNode(nameNode));
        para = tmp->GetValue();
        PLOGD << "Basler相机参数: " << nameNode << " = " << para << " 获取成功";
        return true;
    }

    PLOGE << "Basler相机节点映射指针未初始化";
    return false;
}

bool BaslerCamera::getPara(const char *nameNode, std::string &para) {
    if (nodemap) {
        GenApi::CEnumerationPtr tmp(nodemap->GetNode(nameNode));
        para = tmp->ToString();
        PLOGD << "Basler相机参数: " << nameNode << " = " << para << " 获取成功";
        return true;
    }

    PLOGE << "Basler相机节点映射指针未初始化";
    return false;
}

bool BaslerCamera::setPara(const char *nameNode, double para) {
    if (nodemap) {
        GenApi::CFloatPtr tmp(nodemap->GetNode(nameNode));

        if (IsWritable(tmp)) {
            tmp->SetValue(para);
            PLOGD << "Basler相机参数: " << nameNode << " = " << para << " 设置成功";
            return true;
        } else {
            PLOGE << "Basler相机参数: " << nameNode << " = " << para << " 设置失败";
            return false;
        }
    }

    PLOGE << "Basler相机节点映射指针未初始化";
    return false;
}

bool BaslerCamera::setPara(const char *nameNode, int64_t para) {
    if (nodemap) {
        GenApi::CIntegerPtr tmp(nodemap->GetNode(nameNode));

        if (IsWritable(tmp)) {
            tmp->SetValue(para);
            PLOGD << "Basler相机参数: " << nameNode << " = " << para << " 设置成功";
            return true;
        } else {
            PLOGE << "Basler相机参数: " << nameNode << " = " << para << " 设置失败";
            return false;
        }
    }

    PLOGE << "Basler相机节点映射指针未初始化";
    return false;
}

bool BaslerCamera::setPara(const char *nameNode, std::string para) {
    Pylon::String_t p = para.data();
    if (nodemap) {
        GenApi::CEnumerationPtr tmp(nodemap->GetNode(nameNode));

        if (IsWritable(tmp) && IsAvailable(tmp->GetEntryByName(p))) {
            tmp->FromString(p);
            PLOGD << "Basler相机参数: " << nameNode << " = " << para << " 设置成功";
            return true;
        } else {
            PLOGE << "Basler相机参数: " << nameNode << " = " << para << " 设置失败";
            return false;
        }
    }

    PLOGE << "Basler相机节点映射指针未初始化";
    return false;
}
