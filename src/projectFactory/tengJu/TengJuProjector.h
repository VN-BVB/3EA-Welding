#ifndef TENGJUPROJECTOR_H
#define TENGJUPROJECTOR_H

#include <TJSTProjectorApi.h>

#include "projectFactory/AbstractProjector.h"

typedef struct PrjDev {
    bool bOpen;
    TJSTPRJ devID;
    TJSTPrjInfo_t *prjInfo;
    PrjDev() {
        bOpen = false;
        devID = 0;
        prjInfo = 0;
    }
} PrjDev_t;

class TengJuProjector : public AbstractProjector {
public:
    TengJuProjector(QObject *parent = nullptr);

signals:

public slots:
    bool open() override;   // 打开投影仪
    void close() override;  // 关闭投影仪

    bool openLed() override;   // 打开LED
    bool closeLed() override;  // 关闭LED

    bool triggerProj() override;  // 触发投影

    bool getPara(const char *nameNode, int &para) override;  // 获得参数
    bool setPara(const char *nameNode, int para) override;   // 设置参数
    bool sendCmd(std::string cmd) override;                  // 发送命令

private:
    PrjDev_t m_PrjDev;  // 控制投影仪结构体
};

#endif  // TENGJUPROJECTOR_H
