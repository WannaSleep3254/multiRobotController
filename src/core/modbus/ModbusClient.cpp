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
        emit log(QString("[ERR] %1").arg(m_client->errorString()));
        emit log2(QString("[ERR] %1").arg(m_client->errorString()), 3); // Error
    });
    connect(m_ping, &QTimer::timeout, this, &ModbusClient::onTimeoutPing);

    m_ping->setInterval(1000);
}

ModbusClient::~ModbusClient()
{
    disconnectFrom();
}

bool ModbusClient::connectTo(const QString& host, int port)
{
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
        emit log("[OK] Connected");
        emit log2("[OK] Connected", 1);   // Info
    }
    else if (s == QModbusDevice::UnconnectedState) {
        emit disconnected();
        emit log("[OK] Disconnected");
        emit log2("[OK] Disconnected", 2); // Warn(연결 끊김 표시)
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
}

void ModbusClient::readHolding(int start, int count)
{
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
}

void ModbusClient::readInputs(int start, int count)
{
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
        emit inputRead(start, data);
        reply->deleteLater();
    });
}

void ModbusClient:: readDiscreteInputs(int start, int count)
{
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
}

void ModbusClient::writeCoil(int addr, bool value)
{
    auto unit = QModbusDataUnit(QModbusDataUnit::Coils, addr, 1);
    unit.setValue(0, value);
    auto *reply = m_client->sendWriteRequest(unit, 1);
    if (reply) {
        connect(reply, &QModbusReply::finished, reply, &QObject::deleteLater);
    }
}

void ModbusClient::writeHolding(int addr, quint16 value)
{
    auto unit = QModbusDataUnit(QModbusDataUnit::HoldingRegisters, addr, 1);
    unit.setValue(0, value);

    auto *reply = m_client->sendWriteRequest(unit, 1);
    if (reply) {
        connect(reply, &QModbusReply::finished, reply, &QObject::deleteLater);
    }
}

void ModbusClient::writeHoldingBlock(int start, const QVector<quint16>& values)
{
    auto unit = QModbusDataUnit(QModbusDataUnit::HoldingRegisters, start, values.size());
    for (int i=0;i<values.size();++i) {
        unit.setValue(i, values[i]);
    }

    auto *reply = m_client->sendWriteRequest(unit, 1);
    if (reply) {
        connect(reply, &QModbusReply::finished, reply, &QObject::deleteLater);
    }
}
