#pragma once
#include <QString>

namespace Common {

inline QByteArray intTo2Byte(int32_t data)
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

} // namespace Common
