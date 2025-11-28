#ifndef ROBOTCOMMANDPARSER_H
#define ROBOTCOMMANDPARSER_H

#include <QJsonObject>
#include "RobotCommand.h"

class RobotCommandParser
{
public:
    static bool parse(const QJsonObject& obj, RobotCommand& out);
};

#endif // ROBOTCOMMANDPARSER_H
