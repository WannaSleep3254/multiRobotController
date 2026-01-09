#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <QObject>
#include <QSerialPort>
#include <QSerialPortInfo>

namespace Serial
{
    struct Settings
    {
        QString portName;
        int parityBits = QSerialPort::NoParity;
        int baudRate = QSerialPort::Baud19200;
        int dataBits = QSerialPort::Data8;
        int stopBits = QSerialPort::OneStop;
        int responseTime = 100;//1000;
        int numberOfRetries = 1;//3;
    };

    class SerialPort : public QObject
    {
        Q_OBJECT
    public:
        explicit SerialPort(QObject *parent = nullptr);

        void setPortName(const QString &);
        void setBaud(const QString &);

        QSerialPortInfo getPort(const int &);
        QList<QSerialPortInfo> getPortLists();

        Settings settings();
        void onUpdate();

    signals:
    private:
        Settings m_settings;
        QList<QSerialPortInfo> serialPortInfos_;
    };
}

#endif // SERIALPORT_H
