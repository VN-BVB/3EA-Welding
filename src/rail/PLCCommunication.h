#ifndef PLCCOMMUNICATION_H
#define PLCCOMMUNICATION_H

#include <QElapsedTimer>
#include <QObject>
#include <QVector>
// libmodbus
#include "modbus-tcp.h"
#include "modbus.h"

class PLCCommunication : public QObject {
    Q_OBJECT
public:
    explicit PLCCommunication(QObject *parent = nullptr);
    ~PLCCommunication();

    void whenConnectToPLC(const QString &ip, int port);
    void whenDisconnectFromPLC();
    bool isConnected() const;

    // 写入操作
    bool writeCoils(int address, const QVector<bool> &values);
    bool writeRegisters(int address, const QVector<quint16> &values);

    // 读取操作
    bool readCoils(int address, int num, QVector<bool> &results);
    bool readRegisters(int address, int num, QVector<quint16> &results);

signals:
    void sendText(QString state);
    void connectionStatusChanged(bool state);
    void errorOccurred(QString errorMsg);

private:
    modbus_t *modbusTcp = nullptr;
    bool m_isConnected = false;
    const int MAX_WRITE_REGISTERS = 123;  // Modbus协议规定 单次写入寄存器的数量最多为123个。
    std::mutex m_modbusMutex;
};

#endif  // PLCCOMMUNICATION_H
