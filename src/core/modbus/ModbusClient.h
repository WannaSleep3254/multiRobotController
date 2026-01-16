#ifndef MODBUSCLIENT_H
#define MODBUSCLIENT_H

#include <QObject>
#include <QUuid>
#include <QMap>
#include <QQueue>
#include <QTimer>
#include <QHash>

#include "LogLevel.h"

class QModbusClient;
class QTimer;

struct MbOp {
    enum class Kind {
//        ReadCoils, ReadHolding, ReadInputs, ReadDiscreteInputs,
//        WriteCoil, WriteHolding, WriteHoldingBlock
        ReadCoils, ReadHolding, ReadInputs, ReadDiscreteInputs,
        WriteCoil, WriteCoilBlock, WriteHolding, WriteHoldingBlock,
        DelayMs
    } kind;

    QUuid id;
    int start = 0;
    int count = 0;

    // write용
    bool coilValue = false;
    quint16 holdingValue = 0;
    QVector<quint16> blockValues;

    // coalescing 키(폴링 중복 제거용)
    int delayMs = 0; // DelayMs용
    QString key; // 예: "poll:DI:310:32"
};

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
    void writeCoilBlock(int start, const QVector<quint16> &values);
    void writeHolding(int addr, quint16 value);
    void writeHoldingBlock(int start, const QVector<quint16>& values);

    bool isConnected() const;

signals:
    void connected();
    void disconnected();
    void error(const QString& msg);

    void log(const QString& line, Common::LogLevel level);      // 0=Debug, 1=Info, 2=Warn, 3=Error
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

public:
    // 기존 API는 유지하되, 내부에서 enqueue로 보내도록 변경 권장
    void enqueue(const MbOp& op);

signals:
    void opFinished(QUuid id, bool ok, QString err);
    void opDropped(QString key, QString reason);

private:
    void pump();
    void startOp(const MbOp& op);

    QQueue<MbOp> m_q;
    bool m_inFlight = false;
    QTimer m_pumpTimer;

    // 폴링 중복 제거(선택)
    QHash<QString, int> m_pendingByKey; // key->count 같은 용도(간단하게만)
    int m_maxQueue = 200;
};

#endif // MODBUSCLIENT_H
