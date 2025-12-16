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
    Tool,
    Bulk,
    Sorting,
    Conveyor,
    Align,
    Unknown
};

enum class CmdKind {
    Ready,
    Pick,
    Place,
    Clamp,
    Init,
    Assy,
    Forward,
    Tool_Mount,
    Tool_UnMount,
    Tool_Change,
    Scrap,
    Arrange,
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

struct ToolCommand {
    QString  toolName;          // mount/unmount 용
    QString  toolFrom;          // change 용
    QString  toolTo;            // change 용
};

struct sortingOffset {
    int height;
    int thickness;
    int shift;
};

struct arrangeCommnad {
    Pose6D poseOrig;
    Pose6D poseDest;
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

    bool    isOffset{false};   // offset 필드 사용 여부
    sortingOffset sortOffset{}; // A-sorting place 에서 쓰는 offset 상세

    bool    isArrange{false};    // arrange 명령 여부
    arrangeCommnad arrangeCmd{};

    QString  clamp;             // "open"/"close"

    bool     hasPick{false};
    bool     hasPlace{false};
    Pose6D   pick{};
    Pose6D   place{};

    bool     isTool{false};
    ToolCommand toolCmd{};

    QString mode;               // 벌크 모드: "dual" or "single"

    QJsonObject raw;            // 디버깅용 원본 JSON
};

#endif // ROBOTCOMMAND_H
