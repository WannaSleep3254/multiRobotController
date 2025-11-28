#ifndef MODBUS_H
#define MODBUS_H

#include <QObject>
#include <QModbusDevice>

#include "serialport.h"

QT_BEGIN_NAMESPACE
class QModbusDataUnit;
class QModbusClient;
QT_END_NAMESPACE

namespace Com {
    class Modbus : public QObject
    {
        Q_OBJECT
    public:
        explicit Modbus(QObject *parent = nullptr);
        ~Modbus();

        void doConnect();
        void doDisConnect();

        void writeModbus(const QModbusDataUnit &writeUnit, const int &serverAddress);
        void readModbus(const QModbusDataUnit &readUnit, const int &serverAddress);

        Serial::Settings portSettings() const;
        void setPortSettings(const Serial::Settings &settings);

    signals:
        void comState(const int&);
        void errorState(const int&);
        void readData(const int&, const int&, QVector<quint16>);

    private slots:
        void onModbusStateChanged(const QModbusDevice::State &state);
        void onModbusErrorOccurred(const QModbusDevice::Error &error);
        void onReadReady();

    private:
        QModbusClient *modbusDevice_ = nullptr;
        Serial::Settings m_settings;
        bool isConnect = false;
    };
}

#endif // MODBUS_H
