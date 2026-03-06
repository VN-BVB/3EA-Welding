#ifndef BASLERCAMERA_H
#define BASLERCAMERA_H

#include <pylon/PylonIncludes.h>

#include <QImage>

#include "cameraFactory/AbstractCamera.h"

class BaslerCamera : public AbstractCamera {
public:
    BaslerCamera(QObject *parent = nullptr);

    bool setWorkMode(CAMERA_WORK_MODE mode) override;  // 设置工作模式

signals:

public slots:
    bool open() override;                       // 打开相机
    bool open(const char *serialNum) override;  // 打开指定序列号相机
    bool start() override;                      // 开始采图
    void stop() override;                       // 停止采图
    void close() override;                      // 关闭相机

    bool getPara(const char *nameNode, double &para) override;
    bool getPara(const char *nameNode, int64_t &para) override;
    bool getPara(const char *nameNode, std::string &para) override;

    bool setPara(const char *nameNode, double para) override;
    bool setPara(const char *nameNode, int64_t para) override;
    bool setPara(const char *nameNode, std::string para) override;

private:
    Pylon::DeviceInfoList_t device;                // pylon相机设备
    Pylon::CInstantCamera *mCamera = nullptr;      // pylon相机对象
    Pylon::CImageFormatConverter formatConverter;  // 图像格式转换器
    Pylon::CPylonImage pylonImage;                 // pylon图像对象
    Pylon::CGrabResultPtr ptrGrabResult;           // 抓取结果指针
    GenApi::INodeMap *nodemap = nullptr;           // 节点映射指针
    QImage img;
    cv::Mat cvImg;

    std::chrono::steady_clock::time_point lastGrabbingExceptionTime;  // 上次记录异常的时间
    bool firstGrabbingException = true;                               // 首次异常标志
};

#endif  // BASLERCAMERA_H
