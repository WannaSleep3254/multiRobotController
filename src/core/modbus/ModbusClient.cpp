#include "ModbusClient.h"
#include <QModbusTcpClient>
#include <QModbusReply>
#include <QTimer>
#include <QVariant>
#include <QModbusDataUnit>

ModbusClient::ModbusClient(QObject *parent)
    : QObject{parent}
    , m_client(new QModbusTcpClient(this))
    , m_ping(new QTimer(this))
{
    connect(m_client, &QModbusClient::stateChanged, this, &ModbusClient::onStateChanged);
    connect(m_client, &QModbusClient::errorOccurred, this, [this](QModbusDevice::Error e){
        emit error(QString("Modbus error %1: %2").arg(e).arg(m_client->errorString()));
        emit log(QString("%1").arg(m_client->errorString()), Common::LogLevel::Error); // Error
    });
//    connect(m_ping, &QTimer::timeout, this, &ModbusClient::onTimeoutPing);
//    m_ping->setInterval(1000);

    m_pumpTimer.setSingleShot(true);
    connect(&m_pumpTimer, &QTimer::timeout, this, &ModbusClient::pump);
}

ModbusClient::~ModbusClient()
{
    disconnectFrom();
}

bool ModbusClient::connectTo(const QString& host, int port)
{
    if(!m_client)
        return false;

    if(host.isEmpty() || port <= 0 || port > 65535) {
        emit error("Invalid host or port");
        return false;
    }
    if (m_client->state() == QModbusDevice::ConnectedState)
        return true;

    m_client->setConnectionParameter(QModbusDevice::NetworkPortParameter, port);
    m_client->setConnectionParameter(QModbusDevice::NetworkAddressParameter, host);
    m_client->setTimeout(200);
    m_client->setNumberOfRetries(1);
    const bool ok = m_client->connectDevice();
    if (ok){
        m_ping->start();
    }
    return ok;
}


void ModbusClient::disconnectFrom()
{
    m_ping->stop();
    if (m_client->state() == QModbusDevice::ConnectedState)
        m_client->disconnectDevice();
}

void ModbusClient::onStateChanged(int s)
{
    if (s == QModbusDevice::ConnectedState) {
        emit connected();
        emit heartbeat(true);
        emit log("[OK] Connected", Common::LogLevel::Info);   // Info
    }
    else if (s == QModbusDevice::UnconnectedState) {
        emit disconnected();
        emit log("[OK] Disconnected", Common::LogLevel::Warn); // Warn(연결 끊김 표시)
        emit heartbeat(false);
    }
}

void ModbusClient::onTimeoutPing()
{
    // Light-touch ping by reading 1 holding register at 0
    auto *reply = m_client->sendReadRequest(QModbusDataUnit(QModbusDataUnit::HoldingRegisters, 0, 1), 1);
    if (!reply) { emit heartbeat(false); return; }
    connect(reply, &QModbusReply::finished, this, [this, reply]{
        bool ok = (reply->error() == QModbusDevice::NoError);
        emit heartbeat(ok);
        reply->deleteLater();
    });
}

void ModbusClient::readCoils(int start, int count)
{
#if false
    auto *reply = m_client->sendReadRequest(QModbusDataUnit(QModbusDataUnit::Coils, start, count), 1);
    if(!reply) return;
    connect(reply, &QModbusReply::finished, this, [this, reply, start]{
        QVector<bool> data;
        if (reply->error() == QModbusDevice::NoError) {
            auto u = reply->result();
            data.reserve(u.valueCount());

            for(uint i=0;i<u.valueCount();++i)
                data.push_back(u.value(i));
        }
        emit coilsRead(start, data);
        reply->deleteLater();
    });
#else
    MbOp op;
    op.kind  = MbOp::Kind::ReadCoils;
    op.start = start;
    op.count = count;
    op.key   = QString("poll:CO:%1:%2").arg(start).arg(count); // 폴링이면 coalescing
    enqueue(op);
#endif
}

void ModbusClient::readHolding(int start, int count)
{
#if false
    auto *reply = m_client->sendReadRequest(QModbusDataUnit(QModbusDataUnit::HoldingRegisters, start, count), 1);
    if(!reply) {
        return;
    }

    connect(reply, &QModbusReply::finished, this, [this, reply, start]{
        QVector<quint16> data;
        if (reply->error() == QModbusDevice::NoError) {
            auto u = reply->result();
            data.reserve(u.valueCount());

            for(uint i=0;i<u.valueCount();++i)
                data.push_back(u.value(i));
        }
        emit holdingRead(start, data);
        reply->deleteLater();
    });
#else
    MbOp op;
    op.kind  = MbOp::Kind::ReadHolding;
    op.start = start;
    op.count = count;
    op.key   = QString("poll:DI:%1:%2").arg(start).arg(count);
    enqueue(op);
#endif
}

void ModbusClient::readInputs(int start, int count)
{
#if false
    auto *reply = m_client->sendReadRequest(
        QModbusDataUnit(QModbusDataUnit::InputRegisters, start, count), 1);
    if (!reply)
        return;

    connect(reply, &QModbusReply::finished, this, [this, reply, start]{
        QVector<quint16> data;
        if (reply->error() == QModbusDevice::NoError) {
            auto u = reply->result();
            data.reserve(u.valueCount());

            for (uint i=0; i<u.valueCount(); ++i)
                data.push_back(u.value(i));
        }
//        qDebug()<<"[MC] readInputs result start="<<start<<"data="<<data;
        emit inputRead(start, data);
        reply->deleteLater();
    });
#else
    MbOp op;
    op.kind  = MbOp::Kind::ReadInputs;
    op.start = start;
    op.count = count;
    op.key   = QString("poll:DI:%1:%2").arg(start).arg(count);
    enqueue(op);
#endif
}

void ModbusClient:: readDiscreteInputs(int start, int count)
{
#if false
//    qDebug()<<start<<count;
    auto *reply = m_client->sendReadRequest(
        QModbusDataUnit(QModbusDataUnit::DiscreteInputs, start, count), 1);
    if(!reply)
        return;

    connect(reply, &QModbusReply::finished, this, [this, reply, start]{
        QVector<bool> data;
        if (reply->error() == QModbusDevice::NoError) {
            auto u = reply->result();
            data.reserve(u.valueCount());

            for(uint i=0;i<u.valueCount();++i)
                data.push_back(u.value(i));
        }
        emit discreteInputsRead(start, data);
        reply->deleteLater();
    });

#else
    MbOp op;
    op.kind  = MbOp::Kind::ReadDiscreteInputs;
    op.start = start;
    op.count = count;
    op.key   = QString("poll:DI:%1:%2").arg(start).arg(count);
    enqueue(op);
#endif
}

void ModbusClient::writeCoil(int addr, bool value)
{
#if false
    auto unit = QModbusDataUnit(QModbusDataUnit::Coils, addr, 1);
    unit.setValue(0, value);
    auto *reply = m_client->sendWriteRequest(unit, 1);
    if (reply) {
        connect(reply, &QModbusReply::finished, reply, &QObject::deleteLater);
    }
#else
    MbOp op;
    op.kind      = MbOp::Kind::WriteCoil;
    op.start     = addr;
    op.coilValue = value;
    enqueue(op);
#endif
}

void ModbusClient::writeCoilBlock(int start, const QVector<quint16>& values)
{
#if false
    auto unit = QModbusDataUnit(QModbusDataUnit::Coils, start, values.size());
    for (int i=0;i<values.size();++i) {
        unit.setValue(i, values[i]);
    }

    auto *reply = m_client->sendWriteRequest(unit, 1);
    if (reply) {
        connect(reply, &QModbusReply::finished, reply, &QObject::deleteLater);
    }
#else
    MbOp op;
    op.kind        = MbOp::Kind::WriteCoilBlock;
    op.start       = start;
    op.blockValues = values;
    enqueue(op);
#endif
}

void ModbusClient::writeHolding(int addr, quint16 value)
{
#if false
    auto unit = QModbusDataUnit(QModbusDataUnit::HoldingRegisters, addr, 1);
    unit.setValue(0, value);

    auto *reply = m_client->sendWriteRequest(unit, 1);
    if (reply) {
        connect(reply, &QModbusReply::finished, reply, &QObject::deleteLater);
    }
#else
    MbOp op;
    op.kind         = MbOp::Kind::WriteHolding;
    op.start        = addr;
    op.holdingValue = value;
    enqueue(op);
#endif
}

void ModbusClient::writeHoldingBlock(int start, const QVector<quint16>& values)
{
#if false
    auto unit = QModbusDataUnit(QModbusDataUnit::HoldingRegisters, start, values.size());
    for (int i=0;i<values.size();++i) {
        unit.setValue(i, values[i]);
    }

    auto *reply = m_client->sendWriteRequest(unit, 1);
    if (reply) {
        connect(reply, &QModbusReply::finished, reply, &QObject::deleteLater);
    }
#else
    MbOp op;
    op.kind        = MbOp::Kind::WriteHoldingBlock;
    op.start       = start;
    op.blockValues = values;
    enqueue(op);
#endif
}

bool ModbusClient::isConnected() const {
    return m_client && m_client->state() == QModbusDevice::ConnectedState;
}

//////////////////////////////////////////////////////
void ModbusClient::enqueue(const MbOp& in)
{
    if (!isConnected()) return;

    MbOp op = in;
    op.id = op.id.isNull() ? QUuid::createUuid() : op.id;

    // (선택) 폴링 중복(coalescing): 같은 key가 이미 대기중이면 추가 안 함
    if (!op.key.isEmpty()) {
        if (m_pendingByKey.contains(op.key)) {
            emit opDropped(op.key, "coalesced");
            return;
        }
        m_pendingByKey.insert(op.key, 1);
    }

    // 큐 폭주 방지
    if (m_q.size() >= m_maxQueue) {
        if (!op.key.isEmpty()) m_pendingByKey.remove(op.key);
        emit opDropped(op.key, "queue overflow");
        return;
    }

    m_q.enqueue(op);
    if (!m_inFlight && !m_pumpTimer.isActive())
        m_pumpTimer.start(0);
}

void ModbusClient::pump()
{
    if (m_inFlight) return;
    if (m_q.isEmpty()) return;

    const MbOp op = m_q.dequeue();
    m_inFlight = true;
    startOp(op);
}

void ModbusClient::startOp(const MbOp& op)
{
    auto clearKey = [this, op](){
        if (!op.key.isEmpty()) m_pendingByKey.remove(op.key);
    };

    if (op.kind == MbOp::Kind::DelayMs) {
        QTimer::singleShot(qMax(0, op.delayMs), this, [this, op, clearKey](){
            clearKey();
            m_inFlight = false;
            emit opFinished(op.id, true, "");
            if (!m_pumpTimer.isActive()) m_pumpTimer.start(0);
        });
        return;
    }

    QModbusReply* reply = nullptr;

    switch (op.kind) {
    case MbOp::Kind::ReadCoils:
        reply = m_client->sendReadRequest(
            QModbusDataUnit(QModbusDataUnit::Coils, op.start, op.count), 1);
        break;

    case MbOp::Kind::ReadDiscreteInputs:
        reply = m_client->sendReadRequest(
            QModbusDataUnit(QModbusDataUnit::DiscreteInputs, op.start, op.count), 1);
        break;

    case MbOp::Kind::ReadInputs:
        reply = m_client->sendReadRequest(
            QModbusDataUnit(QModbusDataUnit::InputRegisters, op.start, op.count), 1);
        break;

    case MbOp::Kind::ReadHolding:
        reply = m_client->sendReadRequest(
            QModbusDataUnit(QModbusDataUnit::HoldingRegisters, op.start, op.count), 1);
        break;

    case MbOp::Kind::WriteCoil: {
        QModbusDataUnit u(QModbusDataUnit::Coils, op.start, 1);
        u.setValue(0, op.coilValue);
        reply = m_client->sendWriteRequest(u, 1);
        break;
    }
    case MbOp::Kind::WriteCoilBlock: {
        QModbusDataUnit u(QModbusDataUnit::Coils, op.start, op.blockValues.size());
        for (int i=0;i<op.blockValues.size();++i) u.setValue(i, op.blockValues[i]);
        reply = m_client->sendWriteRequest(u, 1);
        break;
    }
    case MbOp::Kind::WriteHolding: {
        QModbusDataUnit u(QModbusDataUnit::HoldingRegisters, op.start, 1);
        u.setValue(0, op.holdingValue);
        reply = m_client->sendWriteRequest(u, 1);
        break;
    }
    case MbOp::Kind::WriteHoldingBlock: {
        QModbusDataUnit u(QModbusDataUnit::HoldingRegisters, op.start, op.blockValues.size());
        for (int i=0;i<op.blockValues.size();++i) u.setValue(i, op.blockValues[i]);
        reply = m_client->sendWriteRequest(u, 1);
        break;
    }

    // 나머지도 동일하게 케이스 추가 (ReadHolding/WriteHoldingBlock 등)
    default:
        break;
    }

    if (!reply) {
        clearKey();
        m_inFlight = false;
        emit opFinished(op.id, false, "sendRequest failed");
        if (!m_pumpTimer.isActive()) m_pumpTimer.start(0);
        return;
    }

    connect(reply, &QModbusReply::finished, this, [this, reply, op, clearKey](){
        const bool ok = (reply->error() == QModbusDevice::NoError);
        const QString err = ok ? "" : reply->errorString();

        if (ok) {
            const auto u = reply->result();
            if (op.kind == MbOp::Kind::ReadCoils) {
                QVector<bool> data; data.reserve(u.valueCount());
                for (uint i=0;i<u.valueCount();++i) data.push_back(u.value(i));
                emit coilsRead(op.start, data);
            }
            else if (op.kind == MbOp::Kind::ReadDiscreteInputs) {
                QVector<bool> data; data.reserve(u.valueCount());
                for (uint i=0;i<u.valueCount();++i) data.push_back(u.value(i));
                emit discreteInputsRead(op.start, data);
            } else if (op.kind == MbOp::Kind::ReadInputs) {
                QVector<quint16> data; data.reserve(u.valueCount());
                for (uint i=0;i<u.valueCount();++i) data.push_back(u.value(i));
                emit inputRead(op.start, data);
            } else if (op.kind == MbOp::Kind::ReadHolding) {
                QVector<quint16> data; data.reserve(u.valueCount());
                for (uint i=0;i<u.valueCount();++i) data.push_back(u.value(i));
                emit holdingRead(op.start, data);
            }
        }

        reply->deleteLater();
        clearKey();

        m_inFlight = false;
        emit opFinished(op.id, ok, err);

        if (!m_pumpTimer.isActive())
            m_pumpTimer.start(0);
    });
}
