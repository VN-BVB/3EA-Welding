#ifndef ABSTRACTCAMERA_H
#define ABSTRACTCAMERA_H

#include <plog/Log.h>

#include <QImage>
#include <QObject>
#include <iostream>
#include <opencv2/core.hpp>
// #include <opencv2/highgui.hpp>

enum CAMERA_WORK_MODE {  // 触发方式
    HARDWARE_TRIGGER,    // 硬件触发
    SOFTWARE_TRIGGER     // 软件触发
};

class AbstractCamera : public QObject {
    Q_OBJECT
public:
    AbstractCamera(QObject* parent = nullptr);
    ~AbstractCamera();

    virtual bool setWorkMode(CAMERA_WORK_MODE mode) = 0;  // 设置工作模式

signals:
    void sendImage(cv::Mat img, CAMERA_WORK_MODE workMode);  // 发出图像的信号

public slots:
    virtual bool open() = 0;                       // 打开相机
    virtual bool open(const char* serialNum) = 0;  // 打开指定序列号相机
    virtual bool start() = 0;                      // 开始采图
    virtual void stop() = 0;                       // 停止采图
    virtual void close() = 0;                      // 关闭相机

    virtual bool getPara(const char* nameNode, double& para) = 0;
    virtual bool getPara(const char* nameNode, int64_t& para) = 0;
    virtual bool getPara(const char* nameNode, std::string& para) = 0;

    virtual bool setPara(const char* nameNode, double para) = 0;
    virtual bool setPara(const char* nameNode, int64_t para) = 0;
    virtual bool setPara(const char* nameNode, std::string para) = 0;

protected:
    bool opening = false;  // 是否打开相机标志位
    bool running = false;  // 是否正在采图标志位

    int exposure = 20000;  // 曝光
    int gain = 5;          // 增益

    CAMERA_WORK_MODE workMode = HARDWARE_TRIGGER;  // 默认硬件触发工作模式

private:
    friend class SettingWidget;
    friend class StructLightCamera;
};

#endif  // ABSTRACTCAMERA_H
