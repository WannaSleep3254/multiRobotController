#ifndef MODBUS_H
#define MODBUS_H

#include <QObject>
#include <QModbusDevice>
#include <QModbusDataUnit>
#include <QQueue>
#include <QPointer>
#include <QElapsedTimer>
#include "serialport.h"

QT_BEGIN_NAMESPACE
//class QModbusDataUnit;
class QModbusClient;
class QModbusReply;
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
//        void onReadReady();
    private:
        enum class ReqType
        {
            Read,
            Write
        };

        struct pendingRequest
        {
            ReqType type;
            QModbusDataUnit unit;
            int serverAddress = 1;
        };

    private:
        void enqueueRead (const QModbusDataUnit& unit, int serverAddress);
        void enqueueWrite(const QModbusDataUnit& unit, int serverAddress);
        void kick();                 // 큐에서 다음 요청 실행
        void onReplyFinished();      // 현재 reply finished 처리

    private:
        QModbusClient *modbusDevice_ = nullptr;
        Serial::Settings m_settings;
        bool isConnect = false;

        // in-flight + queue
        bool inflight_ = false;
        QPointer<QModbusReply> currentReply_;
        pendingRequest currentReq_;
        QQueue<pendingRequest> queue_;

        QElapsedTimer rttTimer_;
    };
}

#endif // MODBUS_H
