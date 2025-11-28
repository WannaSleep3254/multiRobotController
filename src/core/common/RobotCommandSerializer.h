#ifndef ROBOTCOMMANDSERIALIZER_H
#define ROBOTCOMMANDSERIALIZER_H

#include <QJsonObject>
#include "RobotCommand.h"

class RobotCommandSerializer
{
public:
    static QJsonObject toJson(const RobotCommand& cmd);
};

#endif // ROBOTCOMMANDSERIALIZER_H
