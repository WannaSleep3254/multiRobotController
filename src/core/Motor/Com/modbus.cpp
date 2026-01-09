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

        modbusDevice_->setTimeout(m_settings.responseTime); // 80ms
        modbusDevice_->setNumberOfRetries(m_settings.numberOfRetries);  // 1

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
        // 연결 끊기면 큐/인플라이트 정리
        inflight_ = false;
        queue_.clear();
        currentReply_.clear();
        modbusDevice_->disconnectDevice();
    }

    void Modbus::writeModbus(const QModbusDataUnit &writeUnit, const int &serverAddress)
    {
        if (!modbusDevice_)
            return;
#if false
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
#else
        enqueueWrite(writeUnit, serverAddress);
#endif
    }

    void Modbus::readModbus(const QModbusDataUnit &readUnit, const int &serverAddress)
    {
        if (!modbusDevice_)
            return;
#if false
        if (auto *reply = modbusDevice_->sendReadRequest(readUnit, serverAddress))
        {
            if (!reply->isFinished())
            {
//                connect(reply, &QModbusReply::finished, this, &Modbus::onReadReady);
            }
            else
            {   // 이미 완료된 경우
                reply->deleteLater();
            }
        } else
        {
            qDebug() << "읽기 요청 실패:" << modbusDevice_->errorString();
        }
#else
        enqueueRead(readUnit, serverAddress);
#endif
    }

    Serial::Settings Modbus::portSettings() const
    {
        return m_settings;
    }

    void Modbus::setPortSettings(const Serial::Settings &settings)
    {
        m_settings = settings;
    }

    void Modbus::enqueueRead(const QModbusDataUnit& unit, int serverAddress)
    {
        queue_.enqueue(pendingRequest{ReqType::Read, unit, serverAddress});
        kick();
    }

    void Modbus::enqueueWrite(const QModbusDataUnit& unit, int serverAddress)
    {
        queue_.enqueue(pendingRequest{ReqType::Write, unit, serverAddress});
        kick();
    }

    void Modbus::kick()
    {
        if (!modbusDevice_) return;
        if (modbusDevice_->state() != QModbusDevice::ConnectedState) return;
        if (inflight_) return;
        if (queue_.isEmpty()) return;

        currentReq_ = queue_.dequeue();

        QModbusReply* reply = nullptr;

        // ✅ RTT 측정 시작
        rttTimer_.restart();

        if (currentReq_.type == ReqType::Read) {
            reply = modbusDevice_->sendReadRequest(currentReq_.unit, currentReq_.serverAddress);
        } else {
            reply = modbusDevice_->sendWriteRequest(currentReq_.unit, currentReq_.serverAddress);
        }

        if (!reply) {
            qDebug() << "Modbus request failed:" << modbusDevice_->errorString();
            // 실패해도 다음 요청 진행
            inflight_ = false;
            kick();
            return;
        }

        inflight_ = true;
        currentReply_ = reply;

        if (reply->isFinished()) {
            // broadcast 등 즉시 종료 케이스
            onReplyFinished();
        } else {
            connect(reply, &QModbusReply::finished, this, &Modbus::onReplyFinished);
        }
    }

    void Modbus::onReplyFinished()
    {
        QModbusReply* reply = qobject_cast<QModbusReply*>(sender());
        // sender()가 nullptr인 경우(즉시 종료) 대비
        if (!reply) reply = currentReply_.data();

        const qint64 rttMs = rttTimer_.elapsed();
        if (reply) {
            if (reply->error() == QModbusDevice::NoError) {
                // ✅ RTT 로그
                qDebug().noquote()
                    << QString("[RTT] %1 addr=0x%2 id=%3 rtt=%4 ms")
                           .arg(currentReq_.type == ReqType::Read ? "READ " : "WRITE")
                           .arg(currentReq_.unit.startAddress(), 4, 16, QLatin1Char('0'))
                           .arg(currentReq_.serverAddress)
                           .arg(rttMs);
                qDebug()<<queue_.size()<<"requests pending.";

                // Read면 result()가 의미 있고, Write도 unit 정보는 currentReq_로 알 수 있음
                if (currentReq_.type == ReqType::Read) {
                    const QModbusDataUnit unit = reply->result();
                    emit readData(currentReq_.serverAddress, unit.startAddress(), unit.values());
                }
            } else if (reply->error() == QModbusDevice::ProtocolError) {
                qDebug() << "Modbus ProtocolError:" << reply->errorString()
                << " exception:" << Qt::hex << reply->rawResult().exceptionCode();
            } else {
                qDebug() << "Modbus error:" << reply->error() << reply->errorString();
            }

            reply->deleteLater();
        }

        currentReply_.clear();
        inflight_ = false;

        // 다음 요청 실행
        kick();
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

        // 연결이 끊기면 큐/인플라이트 정리
        if (state == QModbusDevice::UnconnectedState || state == QModbusDevice::ClosingState) {
            inflight_ = false;
            queue_.clear();
            currentReply_.clear();
        } else if (state == QModbusDevice::ConnectedState) {
            // 연결되자마자 대기중인 큐가 있으면 진행
            kick();
        }
        emit comState(state);
    }
/*
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
*/
}
