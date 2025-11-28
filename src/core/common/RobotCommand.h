#ifndef ROBOTCOMMAND_H
#define ROBOTCOMMAND_H

#include <QString>
#include <QJsonObject>
#include "Pose6D.h"

enum class RobotId {
    A,
    B,
    Unknown
};

enum class CmdType {
    Sorting,
    Conveyor,
    Align,
    Unknown
};

enum class CmdKind {
    Tool,
    Ready,
    Pick,
    Place,
    Clamp,
    Init,
    Assy,
    Forward,
    Unknown
};

enum class CmdState {
    Idle,
    Sending,
    Waiting,
    Done,
    Error,
    Timeout
};

struct RobotCommand
{
    RobotId  robot{RobotId::Unknown};
    CmdType  type{CmdType::Unknown};
    CmdKind  kind{CmdKind::Unknown};

    quint32  seq{0};
    int      dir{0};

    // 옵션 필드
    bool     flip{false};
    int      offset{0};         // A-sorting place 에서 쓰는 offset
    QString  clamp;             // "open"/"close"

    bool     hasPick{false};
    bool     hasPlace{false};
    Pose6D   pick{};
    Pose6D   place{};

    QJsonObject raw;            // 디버깅용 원본 JSON
};

#endif // ROBOTCOMMAND_H
