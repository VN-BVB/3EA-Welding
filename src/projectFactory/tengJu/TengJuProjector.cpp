#include "TengJuProjector.h"

TengJuProjector::TengJuProjector(QObject *parent) {
    m_PrjDev.prjInfo = new TJSTPrjInfo_t;
    m_PrjDev.prjInfo->prjType = PRJ_TYPE_NET;
    m_PrjDev.prjInfo->prjInfo.prjIP[0] = 192;
    m_PrjDev.prjInfo->prjInfo.prjIP[1] = 168;
    m_PrjDev.prjInfo->prjInfo.prjIP[2] = 100;
    m_PrjDev.prjInfo->prjInfo.prjIP[3] = 100;
    (void)parent;
}

bool TengJuProjector::open() {
    TJSTPRJ prj = TJSTPrjOpen(m_PrjDev.prjInfo);
    if (prj != 0) {
        m_PrjDev.bOpen = true;
        m_PrjDev.devID = prj;
        PLOGD << "已连接腾聚投影仪";

        // 打开后完成基本的参数配置
        this->setPara("fps", fps);                // 设置帧率
        this->setPara("projNum", projNum);        // 设置投影数量
        this->setPara("color", color);            // 设置投影颜色
        this->setPara("brightness", brightness);  // 设置亮度
        return true;
    } else {
        PLOGE << "无法连接腾聚投影仪";
        return false;
    }
}

void TengJuProjector::close() {
    if (m_PrjDev.bOpen) {
        TJSTPrjClose(m_PrjDev.devID);
        m_PrjDev.bOpen = false;
        m_PrjDev.devID = 0;
        PLOGD << "已关闭腾聚投影仪";
    } else {
        PLOGW << "腾聚投影仪未打开, 无需关闭";
    }
}

bool TengJuProjector::openLed() {
    if (!m_PrjDev.bOpen) {
        PLOGW << "腾聚投影仪未打开, 无法打开LED";
        return false;
    }

    if (TJSTPrjLedOn(m_PrjDev.devID)) {  // 打开LED灯指令
        PLOGD << "腾聚投影仪LED灯打开成功";
        return true;
    } else {
        PLOGE << "腾聚投影仪LED灯打开失败";
        return false;
    }
}

bool TengJuProjector::closeLed() {
    if (!m_PrjDev.bOpen) {
        PLOGW << "腾聚投影仪未打开, 无法打开LED";
        return false;
    }

    if (TJSTPrjLedOff(m_PrjDev.devID)) {  // 关闭LED灯指令
        PLOGD << "腾聚投影仪LED灯关闭成功";
        return true;
    } else {
        PLOGE << "腾聚投影仪LED灯关闭失败";
        return false;
    }
}

bool TengJuProjector::triggerProj() {
    this->openLed();
    if (!m_PrjDev.bOpen) {
        PLOGW << "腾聚投影仪未打开, 无法投影";
        return false;
    }

    if (TJSTPrjTriggerOnce(m_PrjDev.devID, 255)) {
        PLOGD << "腾聚投影仪连续投影成功";
        return true;
    } else {
        PLOGE << "腾聚投影仪连续投影失败";
        return false;
    }
}

// 包含了投影数量, 帧率, 亮度的获取
bool TengJuProjector::getPara(const char *nameNode, int &para) {
    if (m_PrjDev.bOpen) {
        if (std::string(nameNode) == "fps") {
            para = fps;
        } else if (std::string(nameNode) == "projNum") {
            para = projNum;
        } else if (std::string(nameNode) == "brightness") {
            para = brightness;
        } else if (std::string(nameNode) == "color") {
            para = color;
        } else {
            PLOGE << "腾聚投影仪无此参数: " << nameNode;
            return false;
        }
    } else {
        PLOGW << "腾聚投影仪未打开, 无法获取参数";
        return false;
    }
    PLOGD << "腾聚投影仪参数: " << nameNode << " = " << para << " 获取成功";
    return true;
}

// 包含了投影数量, 帧率, 亮度的设置
bool TengJuProjector::setPara(const char *nameNode, int para) {
    if (m_PrjDev.bOpen) {
        bool succ = false;
        if (std::string(nameNode) == "fps") {
            fps = para;
            int FPSR = int(60. / float(fps) - 1);
            succ = sendCmd("MA " + std::to_string(FPSR) + " " + std::to_string(projNum - 1) + " 1 0");
            succ = sendCmd("MS");
            if (succ == false) {
                PLOGE << "腾聚投影仪帧率设置失败";
                return false;
            }
        } else if (std::string(nameNode) == "projNum") {
            projNum = para;
            int FPSR = int(60.0 / float(fps) - 1);
            succ = sendCmd("MA " + std::to_string(FPSR) + " " + std::to_string(projNum - 1) + " 1 0");
            succ = sendCmd("MS");
            if (succ == false) {
                PLOGE << "腾聚投影仪投影数量设置失败";
                return false;
            }
        } else if (std::string(nameNode) == "brightness") {
            brightness = para;
            succ = TJSTPrjSetLight(m_PrjDev.devID, brightness);
            if (succ == false) {
                PLOGE << "腾聚投影仪亮度设置失败";
                return false;
            }
        } else if (std::string(nameNode) == "color") {
            color = para;
            succ = TJSTPrjSetColor(m_PrjDev.devID, color);
            if (succ == false) {
                PLOGE << "腾聚投影仪光源颜色设置失败";
                return false;
            }
        } else {
            PLOGE << "腾聚投影仪无此参数: " << nameNode;
            return false;
        }
    } else {
        PLOGW << "腾聚投影仪未打开, 无法设置参数";
        return false;
    }
    PLOGD << "腾聚投影仪参数: " << nameNode << " = " << para << " 设置成功";
    return true;
}

bool TengJuProjector::sendCmd(std::string cmd) {
    if (!m_PrjDev.bOpen) {
        PLOGW << "腾聚投影仪未打开, 无法发送指令";
        return false;
    }

    cmd = cmd + "\r\n";
    const char *sendcmd = cmd.c_str();
    if (TJSTPrjWrite(m_PrjDev.devID, sendcmd, strlen(sendcmd))) {
        PLOGD << "腾聚投影仪命令发送成功";
        // PLOGD << "腾聚投影仪命令: " << cmd.c_str() << " 发送成功";
        return true;
    } else {
        PLOGE << "腾聚投影仪命令发送失败";
        // PLOGE << "腾聚投影仪命令: " << cmd.c_str() << " 发送失败";
        return false;
    }
}
