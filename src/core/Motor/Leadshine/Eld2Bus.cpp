#include "Eld2Bus.h"

#include <QModbusDataUnit>
#include <QTimer>
#include <QPoint>
#include <QDebug>

#include "modbus.h"
#include "serialport.h"

QByteArray intTo2Byte(int32_t data)
{
    QByteArray qba(reinterpret_cast<const char *>(&data), sizeof(int));

    QByteArray buffer;
    buffer.resize(4);
    buffer[3]=qba[0];
    buffer[2]=qba[1];
    buffer[1]=qba[2];
    buffer[0]=qba[3];

    return buffer;
}

namespace Leadshine
{
    Eld2Bus::Eld2Bus(QObject *parent)
        : QObject{parent}
    {

    }

    Eld2Bus::~Eld2Bus()
    {
    }
}
