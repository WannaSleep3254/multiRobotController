#include "serialport.h"

#include <QDebug>
#define port_debug false
namespace Serial
{
    SerialPort::SerialPort(QObject *parent)
        : QObject{parent}
    {
    }

    void SerialPort::setPortName(const QString &portName)
    {
        m_settings.portName = portName;
    }

    void SerialPort::setBaud(const QString &strBaud)
    {
        const int baud = strBaud.toInt();
        switch(baud)
        {
        case 9600:
            m_settings.baudRate=QSerialPort::Baud9600;
            break;
        case 19200:
            m_settings.baudRate=QSerialPort::Baud19200;
            break;
        case 38400:
            m_settings.baudRate=QSerialPort::Baud38400;
            break;
        case 57600:
            m_settings.baudRate=QSerialPort::Baud57600;
            break;
        case 115200:
            m_settings.baudRate=QSerialPort::Baud115200;
            break;
        }
    }

    void SerialPort::onUpdate()
    {
        const auto serialPortInfos = QSerialPortInfo::availablePorts();
#if port_debug
        for (const QSerialPortInfo &portInfo : serialPortInfos)
        {

            qDebug() << "\n"
                     << "Port:" << portInfo.portName() << "\n"
                     << "Location:" << portInfo.systemLocation() << "\n"
                     << "Description:" << portInfo.description() << "\n"
                     << "Manufacturer:" << portInfo.manufacturer() << "\n"
                     << "Serial number:" << portInfo.serialNumber() << "\n"
                     << "Vendor Identifier:"
                     << (portInfo.hasVendorIdentifier()
                             ? QByteArray::number(portInfo.vendorIdentifier(), 16)
                             : QByteArray()) << "\n"
                     << "Product Identifier:"
                     << (portInfo.hasProductIdentifier()
                             ? QByteArray::number(portInfo.productIdentifier(), 16)
                             : QByteArray());
        }
#endif
        serialPortInfos_ = serialPortInfos;
    }

    QSerialPortInfo SerialPort::getPort(const int &count)
    {
        return serialPortInfos_.at(count);
    }

    Settings SerialPort::settings()
    {
        return m_settings;
    }

    QList<QSerialPortInfo> SerialPort::getPortLists()
    {
        return serialPortInfos_;
    }
}
