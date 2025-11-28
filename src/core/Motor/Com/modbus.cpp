#include "modbus.h"
#include "qvariant.h"
#include <QModbusClient>
//#include <QModbusRtuSerialMaster>
#include <QModbusRtuSerialClient>
#include <QDebug>
namespace Com
{
    Modbus::Modbus(QObject *parent)
        : QObject{parent}
    {
        /* modbus */
        //modbusDevice_ = new QModbusRtuSerialMaster(this);
        modbusDevice_ = new QModbusRtuSerialClient(this);
        if (!modbusDevice_)
        {
            qDebug()<<QString("Could not create Modbus master.");
        }
        else
        {
            qDebug()<<QString("Create Modbus master.");
            qDebug()<<"state: "<<modbusDevice_->state();


            connect(modbusDevice_, &QModbusClient::errorOccurred, this, &Modbus::onModbusErrorOccurred);
            connect(modbusDevice_, &QModbusClient::stateChanged, this, &Modbus::onModbusStateChanged);
//            doConnect();
        }
    }

    Modbus::~Modbus()
    {
        if (modbusDevice_)
        {
            modbusDevice_->disconnectDevice();
            modbusDevice_->deleteLater();
        }
    }

    void Modbus::doConnect()
    {
        /* Connect */
        modbusDevice_->setConnectionParameter(QModbusDevice::SerialPortNameParameter, m_settings.portName);
        modbusDevice_->setConnectionParameter(QModbusDevice::SerialParityParameter,   m_settings.parityBits);
        modbusDevice_->setConnectionParameter(QModbusDevice::SerialBaudRateParameter, m_settings.baudRate);
        modbusDevice_->setConnectionParameter(QModbusDevice::SerialDataBitsParameter, m_settings.dataBits);
        modbusDevice_->setConnectionParameter(QModbusDevice::SerialStopBitsParameter, m_settings.stopBits);

        modbusDevice_->setTimeout(m_settings.responseTime);
        modbusDevice_->setNumberOfRetries(m_settings.numberOfRetries);

        if (!modbusDevice_->connectDevice())
        {
            qDebug()<<"connect: fail!!";
        }
        else
        {
            qDebug()<<"connect: succed!!";
        }
    }

    void Modbus::doDisConnect()
    {
        modbusDevice_->disconnectDevice();
    }

    void Modbus::writeModbus(const QModbusDataUnit &writeUnit, const int &serverAddress)
    {
        if (!modbusDevice_)
            return;

        if (auto *reply = modbusDevice_->sendWriteRequest(writeUnit, serverAddress))
        {
            if (!reply->isFinished())
            {
                connect(reply, &QModbusReply::finished, this, [this, reply]() {
                    if (reply->error() == QModbusDevice::ProtocolError)
                    {
                        qDebug()<<(tr("Write response error: %1 (Mobus exception: 0x%2)").arg(reply->errorString()).arg(reply->rawResult().exceptionCode(), -1, 16));
                    }
                    else if (reply->error() != QModbusDevice::NoError)
                    {
                        qDebug()<<(tr("Write response error: %1 (code: 0x%2)").arg(reply->errorString()).arg(reply->error(), -1, 16));
                    }
                    reply->deleteLater();
                });
            }
            else
            {
                // broadcast replies return immediately
                reply->deleteLater();
            }
        } else {
            qDebug()<<(QString("Write error: ")+ modbusDevice_->errorString());
        }
    }

    void Modbus::readModbus(const QModbusDataUnit &readUnit, const int &serverAddress)
    {
        if (!modbusDevice_)
            return;

        if (auto *reply = modbusDevice_->sendReadRequest(readUnit, serverAddress))
        {
            if (!reply->isFinished())
            {
                connect(reply, &QModbusReply::finished, this, &Modbus::onReadReady);
            }
            else
            {   // 이미 완료된 경우
                reply->deleteLater();
            }
        } else
        {
            qDebug() << "읽기 요청 실패:" << modbusDevice_->errorString();
        }
    }

    Serial::Settings Modbus::portSettings() const
    {
        return m_settings;
    }

    void Modbus::setPortSettings(const Serial::Settings &settings)
    {
        m_settings = settings;
    }

    void Modbus::onModbusErrorOccurred(const QModbusDevice::Error &error)
    {
        /**
        QModbusDevice::NoError              0	No errors have occurred.
        QModbusDevice::ReadError            1	An error occurred during a read operation.
        QModbusDevice::WriteError           2	An error occurred during a write operation.
        QModbusDevice::ConnectionError      3	An error occurred when attempting to open the backend.
        QModbusDevice::ConfigurationError	4	An error occurred when attempting to set a configuration parameter.
        QModbusDevice::TimeoutError         5	A timeout occurred during I/O. An I/O operation did not finish within a given time frame.
        QModbusDevice::ProtocolError        6	A Modbus specific protocol error occurred.
        QModbusDevice::ReplyAbortedError	7	The reply was aborted due to a disconnection of the device.
        QModbusDevice::UnknownError         8	An unknown error occurred.
        **/

        qDebug()<<modbusDevice_->errorString();
        emit errorState(error);
#if false
        switch(error)
        {
        case QModbusDevice::NoError:
            break;
        case QModbusDevice::ReadError:
            break;
        case QModbusDevice::WriteError:
            break;
        case QModbusDevice::ConnectionError:
            break;
        case QModbusDevice::ConfigurationError:
            break;
        case QModbusDevice::TimeoutError:
            break;
        case QModbusDevice::ProtocolError:
            break;
        case QModbusDevice::ReplyAbortedError:
            break;
        case QModbusDevice::UnknownError:
            break;
        }
#endif
    }

    void Modbus::onModbusStateChanged(const QModbusDevice::State &state)
    {
        /**
        QModbusDevice::UnconnectedState	0	The device is disconnected.
        QModbusDevice::ConnectingState	1	The device is being connected.
        QModbusDevice::ConnectedState	2	The device is connected to the Modbus network.
        QModbusDevice::ClosingState	    3	The device is being closed.
        **/
        isConnect = (state != QModbusDevice::UnconnectedState);
        qDebug()<<"comState: "<<state;

        emit comState(state);
    }
    void Modbus::onReadReady()
    {
        auto reply = qobject_cast<QModbusReply *>(sender());

        if (!reply)
            return;

        if (reply->error() == QModbusDevice::NoError)
        {
            const QModbusDataUnit unit = reply->result();
            emit readData(reply->serverAddress(), unit.startAddress(), unit.values());
        }
        else if (reply->error() == QModbusDevice::ProtocolError)
        {
            qDebug()<<(tr("Read response error: %1 (Mobus exception: 0x%2)").arg(reply->errorString()).arg(reply->rawResult().exceptionCode(), -1, 16));
        }
        else
        {
            qDebug()<<(tr("Read response error: %1 (code: 0x%2)").arg(reply->errorString()).arg(reply->error(), -1, 16));
        }
        reply->deleteLater();
    }
}
