#ifndef ABSTRACTPROJECTOR_H
#define ABSTRACTPROJECTOR_H

#include <plog/Log.h>

#include <QObject>

enum PROJECT_COLOR {  // 投影颜色
    RED = 0,
    GREEN = 1,
    BLUE = 2,
    WHITE = 3
};

class AbstractProjector : public QObject {
    Q_OBJECT
public:
    AbstractProjector(QObject *parent = nullptr);

signals:

public slots:
    virtual bool open() = 0;   // 打开投影仪
    virtual void close() = 0;  // 关闭投影仪

    virtual bool openLed() = 0;   // 打开LED
    virtual bool closeLed() = 0;  // 关闭LED

    virtual bool triggerProj() = 0;  // 触发投影

    virtual bool getPara(const char *nameNode, int &para) = 0;  // 获得参数
    virtual bool setPara(const char *nameNode, int para) = 0;   // 设置参数
    virtual bool sendCmd(std::string cmd) = 0;                  // 发送命令

protected:
    bool opening = false;              // 是否打开投影仪标志位
    int projNum = 21;                  // 投影图片数量
    int fps = 20;                      // 投影仪帧率
    int brightness = 100;              // 投影仪亮度
    int color = PROJECT_COLOR::WHITE;  // 投影颜色

private:
    friend class SettingWidget;
    friend class StructLightCamera;
};

#endif  // ABSTRACTPROJECTOR_H
