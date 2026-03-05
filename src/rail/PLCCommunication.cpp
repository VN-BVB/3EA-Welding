#include "PLCCommunication.h"

#include <errno.h>

#include <QDebug>
#include <QString>

PLCCommunication::PLCCommunication(QObject *parent) : QObject(parent) {}

PLCCommunication::~PLCCommunication() { whenDisconnectFromPLC(); }

void PLCCommunication::whenConnectToPLC(const QString &ip, int port) {
    if (modbusTcp) {
        modbus_close(modbusTcp);
        modbus_free(modbusTcp);
        modbusTcp = nullptr;
    }

    modbusTcp = modbus_new_tcp(ip.toStdString().c_str(), port);
    if (!modbusTcp) {
        emit errorOccurred(QString(u8"创建Modbus TCP对象失败"));
        return;
    }

    if (modbus_set_response_timeout(modbusTcp, 0, 200 * 1000) == -1) {
        emit errorOccurred(QString(u8"设置超时时间失败"));
        modbus_free(modbusTcp);
        modbusTcp = nullptr;
        return;
    }

    if (modbus_connect(modbusTcp) == -1) {
        QString errorStr = QString::fromLocal8Bit(modbus_strerror(errno));
        emit errorOccurred(QString(u8"连接失败：%1").arg(errorStr));
        modbus_free(modbusTcp);
        modbusTcp = nullptr;
        return;
    }

    m_isConnected = true;
    emit sendText(QString(u8"Modbus 状态:协议连接成功，等待使能完成。"));
    emit connectionStatusChanged(true);
}

void PLCCommunication::whenDisconnectFromPLC() {
    if (modbusTcp) {
        modbus_close(modbusTcp);
        modbus_free(modbusTcp);
        modbusTcp = nullptr;
    }
    m_isConnected = false;
    emit sendText(QString(u8"Modbus 状态: 断开连接"));
}

bool PLCCommunication::isConnected() const { return m_isConnected && modbusTcp && modbus_get_socket(modbusTcp) >= 0; }
bool PLCCommunication::writeCoils(int address, const QVector<bool> &values) {
    if (!isConnected()) {
        emit errorOccurred(u8"Modbus未连接，无法写入线圈");
        return false;
    }

    std::vector<uint8_t> bits(values.size());
    for (int i = 0; i < values.size(); ++i) {
        bits[i] = values[i] ? 1 : 0;
    }

    int ret = modbus_write_bits(modbusTcp, address, values.size(), bits.data());
    if (ret == -1) {
        QString errorStr = QString::fromLocal8Bit(modbus_strerror(errno));
        emit errorOccurred(QString(u8"写入线圈失败：%1").arg(errorStr));
        whenDisconnectFromPLC();
        return false;
    }
    return true;
}
bool PLCCommunication::writeRegisters(int address, const QVector<quint16> &values) {
    if (!isConnected()) {
        emit errorOccurred(u8"Modbus未连接，无法写入寄存器");
        return false;
    }

    int size = values.size();
    int writeNum = (size <= MAX_WRITE_REGISTERS) ? size : MAX_WRITE_REGISTERS;
    // libmodbus要求uint16_t*，QVector<quint16>底层类型兼容，直接取data()即可
    int ret = modbus_write_registers(modbusTcp, address, writeNum, reinterpret_cast<const uint16_t *>(values.data()));
    if (ret == -1) {
        QString errorStr = QString::fromLocal8Bit(modbus_strerror(errno));
        emit errorOccurred(QString(u8"写入寄存器失败：%1").arg(errorStr));
        whenDisconnectFromPLC();
        return false;
    }
    return true;
}
bool PLCCommunication::readCoils(int address, int num, QVector<bool> &results) {
    if (!isConnected()) {
        emit errorOccurred(u8"Modbus未连接，无法读取线圈");
        return false;
    }

    std::vector<uint8_t> bits(num);
    int ret = modbus_read_bits(modbusTcp, address, num, bits.data());
    if (ret == -1) {
        QString errorStr = QString::fromLocal8Bit(modbus_strerror(errno));
        emit errorOccurred(QString(u8"读取线圈失败：%1").arg(errorStr));
        whenDisconnectFromPLC();
        return false;
    }

    results.resize(num);
    for (int i = 0; i < num; ++i) {
        results[i] = bits[i] != 0;
    }
    return true;
}

bool PLCCommunication::readRegisters(int address, int num, QVector<quint16> &results) {
    if (!isConnected()) {
        emit errorOccurred(u8"Modbus未连接，无法读取寄存器");
        return false;
    }

    std::vector<uint16_t> regs(num);
    int ret = modbus_read_registers(modbusTcp, address, num, regs.data());
    if (ret == -1) {
        QString errorStr = QString::fromLocal8Bit(modbus_strerror(errno));
        emit errorOccurred(QString(u8"读取寄存器失败：%1").arg(errorStr));
        whenDisconnectFromPLC();
        return false;
    }

    results.resize(num);
    for (int i = 0; i < num; ++i) {
        results[i] = regs[i];
    }
    return true;
}
