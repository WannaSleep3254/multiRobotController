#ifndef MODBUSCLIENT_H
#define MODBUSCLIENT_H

#include <QObject>

class QModbusClient;
class QTimer;

class ModbusClient : public QObject
{
    Q_OBJECT
public:
    explicit ModbusClient(QObject *parent = nullptr);
    ~ModbusClient();

    bool connectTo(const QString& host, int port);
    void disconnectFrom();

    // Async read/write helpers
    void readCoils(int start, int count);
    void readHolding(int start, int count);
    void readInputs(int start, int count);
    void readDiscreteInputs(int start, int count);

    void writeCoil(int addr, bool value);
    void writeHolding(int addr, quint16 value);
    void writeHoldingBlock(int start, const QVector<quint16>& values);


signals:
    void connected();
    void disconnected();
    void error(const QString& msg);
    void log(const QString& line);
    void log2(const QString& line, int level);      // 0=Debug, 1=Info, 2=Warn, 3=Error
    void heartbeat(bool ok);

    void coilsRead(int start, QVector<bool> data);
    void holdingRead(int start, QVector<quint16> data);
    void inputRead(int start, QVector<quint16> data);
    void discreteInputsRead(int start, QVector<bool> data);


private slots:
    void onStateChanged(int s);
    void onTimeoutPing();

private:
    QModbusClient* m_client; // QModbusTcpClient under the hood
    QTimer* m_ping;
};

#endif // MODBUSCLIENT_H
