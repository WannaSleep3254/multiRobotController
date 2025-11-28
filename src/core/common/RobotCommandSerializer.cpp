#include "RobotCommandSerializer.h"

static QString robotIdToString(RobotId id)
{
    switch (id) {
    case RobotId::A: return "a";
    case RobotId::B: return "b";
    default: return "";
    }
}

static QString cmdTypeToString(CmdType t)
{
    switch (t) {
    case CmdType::Sorting:  return "sorting";
    case CmdType::Conveyor: return "conveyor";
    case CmdType::Align:    return "align";
    default: return "";
    }
}

static QString cmdKindToString(CmdKind k)
{
    switch (k) {
    case CmdKind::Tool:    return "tool";
    case CmdKind::Ready:   return "ready";
    case CmdKind::Pick:    return "pick";
    case CmdKind::Place:   return "place";
    case CmdKind::Clamp:   return "clamp";
    case CmdKind::Init:    return "init";
    case CmdKind::Assy:    return "assy";
    case CmdKind::Forward: return "forward";
    default: return "";
    }
}

static QJsonObject poseToJson(const Pose6D& p)
{
    return QJsonObject{
        {"x",  p.x},
        {"y",  p.y},
        {"z",  p.z},
        {"rx", p.rx},
        {"ry", p.ry},
        {"rz", p.rz}
    };
}

QJsonObject RobotCommandSerializer::toJson(const RobotCommand& cmd)
{
    QJsonObject o;
    o["robot"] = robotIdToString(cmd.robot);
    o["type"]  = cmdTypeToString(cmd.type);
    o["kind"]  = cmdKindToString(cmd.kind);

    o["seq"] = int(cmd.seq);
    o["dir"] = cmd.dir;
    o["flip"] = cmd.flip;

    if (cmd.offset != 0)
        o["offset"] = cmd.offset;

    if (cmd.kind == CmdKind::Clamp)
        o["clamp"] = cmd.clamp;

    if (cmd.hasPick)
        o["pick"] = poseToJson(cmd.pick);

    if (cmd.hasPlace)
        o["place"] = poseToJson(cmd.place);

    return o;
}
