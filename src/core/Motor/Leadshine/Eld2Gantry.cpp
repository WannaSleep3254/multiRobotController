#include "Eld2Gantry.h"

#include <QModbusDataUnit>
#include <QTimer>
#include <QPoint>
#include <QDebug>

#include "modbus.h"
#include "serialport.h"
#include "common/convert.h"

namespace Leadshine
{
    Eld2Gantry::Eld2Gantry(QObject *parent)
        : QObject{parent}
    {
        // 0번은 dummy / 기본값
        config_.append(Config (1, 10, 100, 100)); // index 0: default
        // 1~4번: 실제 4축 (원하는 lead/속도 값으로 조정)
        config_.append(Config (1, 5, 750, 2500));   //  Gantry X 1500
        config_.append(Config (1, 5, 700, 1000));   //  Gantry Z  500
        config_.append(Config (15, 1, 50, 300));    //  Gantry Picker 200
        config_.append(Config (30, 1, 2500, 2500)); //  AxisC / Conveyor

        runtime_.resize(config_.size());

        gantry_.bus  = new Com::Modbus(this);
        gantry_.timer = new QTimer(this);
        gantry_.axes = {Axis::GantryX, Axis::GantryZ, Axis::GantryPicker};
        gantry_.tickMs = 20;

        QObject::connect(gantry_.bus, &Com::Modbus::comState, this, [=](int state){
            /**
            QModbusDevice::UnconnectedState	0	The device is disconnected.
            QModbusDevice::ConnectingState	1	The device is being connected.
            QModbusDevice::ConnectedState	2	The device is connected to the Modbus network.
            QModbusDevice::ClosingState	    3	The device is being closed.
            **/
            switch(state)
            {
            case QModbusDevice::UnconnectedState:
                //                qDebug()<<"state: "<<state;
                break;
            case QModbusDevice::ConnectingState:
                //                qDebug()<<"state: "<<state;
                break;
            case QModbusDevice::ConnectedState:
                gantry_.timer->start(25);
                break;
            case QModbusDevice::ClosingState:
                gantry_.timer->stop();
                for (int axis = Axis::AxisX; axis <= Axis::AxisC; ++axis)
                    config_[axis].isConnect = false;
                break;
            }
            emit comState(state);
        });

        QObject::connect(gantry_.bus, &Com::Modbus::errorState, this, [=](int error){
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
            emit errorState(error);
        });

        QObject::connect(gantry_.bus, &Com::Modbus::readData, this, &Eld2Gantry::readData);

        pollAxis_ = Axis::AxisX;
        QObject::connect(gantry_.timer, &QTimer::timeout, this, [=](){
            int axis = pollAxis_;

            if (config_[axis].isConnect)
            {
                // 통신되는 축이면 상태 폴링
                if (pollKind_ == PollKind::PrPos)
                {
                    reqReadEncoder(axis);   // 0x602C
                    pollKind_ = PollKind::Status;
                }
                else
                {
                    reqReadError(axis);     // 0x0B03
                    pollKind_ = PollKind::PrPos;
                }
            }
            else
            {
                // 아직 버전 못 읽은 축 → 버전 읽으면서 존재 확인
                reqReadVersion(axis);
            }
            ++pollAxis_;
            if (pollAxis_ > Axis::AxisZ)    //if (pollAxis_ > Axis::AxisC)
                pollAxis_ = Axis::AxisX;

        });
    }

    Eld2Gantry::~Eld2Gantry()
    {

    }

    void Eld2Gantry::setPort(const QString& name, const QString& baud)
    {
        Serial::SerialPort port;
        port.setPortName(name);
        port.setBaud(baud);

        gantry_.bus->setPortSettings(port.settings());
    }

    void Eld2Gantry::doConnect()
    {
        gantry_.bus->doConnect();
    }

    void Eld2Gantry::doDisConnect()
    {
        gantry_.bus->doDisConnect();
    }

    void Eld2Gantry::doConnect(bool &connect)
    {
        if(connect)
        {
            gantry_.bus->doConnect();
        }
        else
        {
            gantry_.bus->doDisConnect();
        }
    }

    void Eld2Gantry::reqReadEncoder(const int &id)
    {
        QModbusDataUnit encoderData(QModbusDataUnit::HoldingRegisters, 0x602C, 2);
        gantry_.bus->readModbus(encoderData, id);
    }

    void Eld2Gantry::reqReadVersion(const int &id)
    {
        QModbusDataUnit encoderData(QModbusDataUnit::HoldingRegisters, 0x0B00, 3);
        gantry_.bus->readModbus(encoderData, id);
    }

    void Eld2Gantry::reqReadError(const int &id)
    {
        QModbusDataUnit encoderData(QModbusDataUnit::HoldingRegisters, 0x0B03, 7);
        gantry_.bus->readModbus(encoderData, id);
    }

    void Eld2Gantry::reqWriteServo(const int &id, const bool &servo)
    {
        QModbusDataUnit servoData(QModbusDataUnit::HoldingRegisters, 0x0405, 1);
        if(servo)
        {
            servoData.setValue(0,0x83);
        }
        else
        {
            servoData.setValue(0,0x03);
        }
        gantry_.bus->writeModbus(servoData, id);
    }

    void Eld2Gantry::reqWriteJog(const int &id, const int &dir)
    {
        uint16_t dirJog = 0;
        switch(dir)
        {
        case Dir::Stop:
            dirJog = 0;
            break;
        case Dir::Backward:
            dirJog = -1;
            break;
        case Dir::Forward:
            dirJog = 1;
            break;
        }

        uint16_t velJog =   config_.at(id).velJog;//100;
        uint16_t accelJog = 50;//config_.at(id).accelJog;
        uint16_t decelJog = 50;//config_.at(id).decelJog;
        uint16_t send_buffer[8] = {0, };
        send_buffer[0] = 0x02;  // 속도제어
        send_buffer[1] = 0;  // 위치값_H  -
        send_buffer[2] = 0;  // 위치값_L  -
        send_buffer[3] = velJog*dirJog;//0x05DC;  // 속도값    - 0 ~ 6000  [rpm]
        send_buffer[4] = accelJog;//0x01F4;  // 가속도    - 1 ~ 32767 [ms/1000rpm], default: 100 (0x64)
        send_buffer[5] = decelJog;//0x01F4;  // 감속도    - 1 ~ 32767 [ms/1000rpm], default: 100 (0x64)
        send_buffer[6] = 0x0000;  // 대기시간  - 0 ~ 32767[ms]
        send_buffer[7] = 0x0010;  // 패스지정  - PR0: 0x10, PR3: 0x13

        QModbusDataUnit jogData(QModbusDataUnit::HoldingRegisters, 0x06200, 8);
        jogData.setValues(QVector<uint16_t>(send_buffer,send_buffer+8));
        gantry_.bus->writeModbus(jogData, id);
    }

    void Eld2Gantry::reqWritePos(const int &id, const int32_t &pos)
    {
        QByteArray ba = Common::intTo2Byte(pos);
        uint16_t goal_h = static_cast<uint16_t>(ba.mid(0,2).toHex().toUShort(nullptr,16));
        uint16_t goal_l = static_cast<uint16_t>(ba.mid(2,2).toHex().toUShort(nullptr,16));
        uint16_t velMove = config_.at(id).velMove;//100;
        uint16_t accelMove = 50;//config_.at(id).accelMove;
        uint16_t decelMove = 50;//config_.at(id).decelMove;
        uint16_t send_buffer[8] = {0, };
        send_buffer[0] = 0x01;  // 절대위치  - 0x01, 상대위치:0x41
        send_buffer[1] = goal_h;  // 위치값_H  -
        send_buffer[2] = goal_l;  // 위치값_L  -
        send_buffer[3] = velMove; //0x05DC;  // 속도값    - 0 ~ 6000  [rpm]
        send_buffer[4] = accelMove;//0x01F4;  // 가속도    - 1 ~ 32767 [ms/1000rpm], default: 100 (0x64)
        send_buffer[5] = decelMove;//0x01F4;  // 감속도    - 1 ~ 32767 [ms/1000rpm], default: 100 (0x64)
        send_buffer[6] = 0x0000;  // 대기시간  - 0 ~ 32767[ms]
        send_buffer[7] = 0x0010;  // 패스지정  - PR0: 0x10, PR3: 0x13

        // 1) runtime에 목표 입력 + moving 플래그 ON
        runtime_[id].targetPos = pos;
        runtime_[id].moving    = true;

        QModbusDataUnit posData(QModbusDataUnit::HoldingRegisters, 0x6200, 8);
        posData.setValues(QVector<uint16_t>(send_buffer,send_buffer+8));
        gantry_.bus->writeModbus(posData, id);
    }

    void Eld2Gantry::reqWriteShift(const int &id, const int32_t &pos)
    {
        QByteArray ba = Common::intTo2Byte(pos);
        uint16_t goal_h = static_cast<uint16_t>(ba.mid(0,2).toHex().toUShort(nullptr,16));
        uint16_t goal_l = static_cast<uint16_t>(ba.mid(2,2).toHex().toUShort(nullptr,16));
        uint16_t velMove = config_.at(id).velMove;
        uint16_t accelMove = 50;//config_.at(id).accelMove;
        uint16_t decelMove = 50;//config_.at(id).decelMove;
        uint16_t send_buffer[8] = {0, };
        send_buffer[0] = 0x41;     // 절대위치  - 0x01, 상대위치:0x41
        send_buffer[1] = goal_h;  // 위치값_H  -
        send_buffer[2] = goal_l;  // 위치값_L  -
        send_buffer[3] = velMove; // 속도값    - 0 ~ 6000  [rpm]
        send_buffer[4] = accelMove;     //0x01F4;  // 가속도    - 1 ~ 32767 [ms/1000rpm], default: 100 (0x64)
        send_buffer[5] = decelMove;     //0x01F4;  // 감속도    - 1 ~ 32767 [ms/1000rpm], default: 100 (0x64)
        send_buffer[6] = 0x0000;  // 대기시간  - 0 ~ 32767[ms]
        send_buffer[7] = 0x0010;  // 패스지정  - PR0: 0x10, PR3: 0x13

        // 1) runtime에 목표 입력 + moving 플래그 ON

        runtime_[id].targetPos = runtime_[id].lastPos+pos;
        runtime_[id].moving    = true;

        QModbusDataUnit posData(QModbusDataUnit::HoldingRegisters, 0x6200, 8);
        posData.setValues(QVector<uint16_t>(send_buffer,send_buffer+8));
        gantry_.bus->writeModbus(posData, id);
    }

    void Eld2Gantry::reqWriteLimit(const int &id, const int &dir)
    {
        qDebug()<<id<<dir;
//        QModbusDataUnit limitData(QModbusDataUnit::HoldingRegisters, 0x0405, 1);
    }
/*
    void ELD2::reqMovePosition(const float & goal_x, const float & goal_y)
    {
        int32_t pulse_x = convertPosToPulse(Axis::AxisX, goal_x);
        int32_t pulse_y = convertPosToPulse(Axis::AxisY, goal_y);

        reqWritePos(Axis::AxisX, pulse_x);
        reqWritePos(Axis::AxisY, pulse_y);
    }
*/
    void Eld2Gantry::reqMoveStop(const int &id)
    {
        uint16_t velMove = config_.at(id).velMove;
        uint16_t accelMove = config_.at(id).accelMove;
        uint16_t decelMove = config_.at(id).decelMove;

        uint16_t send_buffer[8] = {0, };
        send_buffer[0] = 0x04;     // 노멀: 0x04, 인터럽트: 0x14, 오버랩: 0x24
        send_buffer[1] = 0;        // 위치값_H  -
        send_buffer[2] = 0;        // 위치값_L  -
        send_buffer[3] = velMove;  // 속도값    - 0 ~ 6000  [rpm]
        send_buffer[4] = accelMove;     //0x01F4;  // 가속도    - 1 ~ 32767 [ms/1000rpm], default: 100 (0x64)
        send_buffer[5] = decelMove;     //0x01F4;  // 감속도    - 1 ~ 32767 [ms/1000rpm], default: 100 (0x64)
        send_buffer[6] = 0x0000;   // 대기시간  - 0 ~ 32767[ms]
        send_buffer[7] = 0x0010;   // 패스지정  - PR0: 0x10, PR3: 0x13

        QModbusDataUnit posData(QModbusDataUnit::HoldingRegisters, 0x6200, 8);
        posData.setValues(QVector<uint16_t>(send_buffer,send_buffer+8));
        gantry_.bus->writeModbus(posData, id);
    }

    void Eld2Gantry::reqMoveStop()
    {
        for (int axis = Axis::AxisX; axis <= Axis::AxisC; ++axis)
            reqMoveStop(axis);
    }

    void Eld2Gantry::readData(const int &id, const int &addr, const QVector<quint16>&val)
    {
        switch(addr)
        {
        case 0x0405: {// 1029: 서보 On/Off
            emit readServo(id , (val.at(0)==0x83));
            break;
        }
        case 0x602C: { // 24620: 엔코더
            int32_t encoderVal = (val.at(0)<<16) + val.at(1);
            float pos = convertPulseToPos(id, encoderVal);
            runtime_[id].lastPos = encoderVal;
            emit readEncoder(id, encoderVal, pos);
            break;
        }
        case 0x0B00: {
            config_[id].isConnect = true;
            emit readVersion(id, QString("%1.%2%3").arg(val.at(0)).arg(val.at(1)).arg(val.at(2)));
            break;
        }

        case 0x0B03 : { //reqReadError 1
            emit readError(id, val.at(0));

            qint16 state = static_cast<int16_t>(val.at(2));
            qint16 vel = static_cast<int16_t>(val.at(6));
            runtime_[id].lastVel = vel;
//            emit readVelocity(id, vel);
            bool rdy = state & (1<<0);
            bool run = state & (1<<1);
            emit readReady(id , rdy);
            emit readServo(id , run);
            bool inPosition = state & (1 << 4);   // Bit 4
            if(inPosition)
            {
                if (isMotionDone(id)) {
                    runtime_[id].moving = false;
                    emit motionFinished(id, runtime_[id].targetPos);   // 새 signal
                }
            }
            break;
        }
        }

    }

    bool Eld2Gantry::isMotionDone(int id) const
    {
        const auto &rt = runtime_.at(id);
        if (!rt.moving)
            return false;

        // 예: 위치 오차 ±0.01mm, 속도 0 근처
        const float posTol = 5.0f;
        const int   velTol = 3;  // 1pulse/s or 1rpm 등

        if (qAbs(rt.lastPos-rt.targetPos) < posTol &&
            qAbs(rt.lastVel) <= velTol)
        {
            return true;
        }
        return false;
    }

    float Eld2Gantry::convertPulseToPos(const int &axis, const int32_t &encoder)
    {
        int lead = config_.at(axis).Lead;
        float pulsePerRound = 10000.0;

        return float(encoder / pulsePerRound * lead);
    }

    qint32 Eld2Gantry::convertPosToPulse(const int &axis, const float &position)
    {
        int lead = config_.at(axis).Lead;
        float pulsePerRound = 10000.0;

        return qint32(position * pulsePerRound / lead);
    }
}
