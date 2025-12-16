#include "RobotCommandParser.h"

static RobotId parseRobotId(const QString& s)
{
    const auto lower = s.toLower();
    if (lower == "a") return RobotId::A;
    if (lower == "b") return RobotId::B;
    return RobotId::Unknown;
}

static CmdType parseCmdType(const QString& s)
{
    if (s == "bulk")   return CmdType::Bulk;
    if (s == "sorting")   return CmdType::Sorting;
    if (s == "conveyor")  return CmdType::Conveyor;
    if (s == "align")     return CmdType::Align;
    if (s == "tool")     return CmdType::Tool;
    return CmdType::Unknown;
}

static CmdKind parseCmdKind(const QString& s)
{
    //if (s == "ready")   return CmdKind::Ready;
    if (s == "standby")   return CmdKind::Ready;
    if (s == "pick")    return CmdKind::Pick;
    if (s == "place")   return CmdKind::Place;
    if (s == "clamp")   return CmdKind::Clamp;
    if (s == "init")    return CmdKind::Init;
    if (s == "assy")    return CmdKind::Assy;
    if (s == "forward") return CmdKind::Forward;
    if (s == "scrap") return CmdKind::Scrap;
    if (s == "mount") return CmdKind::Tool_Mount;
    if (s == "unmount") return CmdKind::Tool_UnMount;
    if (s == "change") return CmdKind::Tool_Change;
    if (s == "arrange") return CmdKind::Arrange;
    return CmdKind::Unknown;
}

static bool parsePoseObj(const QJsonObject& o, Pose6D &out)
{
    if (!o.contains("x") || !o.contains("y") || !o.contains("z") ||
        !o.contains("rx") || !o.contains("ry") || !o.contains("rz"))
        return false;

    out.x  = o.value("x").toDouble();
    out.y  = o.value("y").toDouble();
    out.z  = o.value("z").toDouble();
    out.rx = o.value("rx").toDouble();
    out.ry = o.value("ry").toDouble();
    out.rz = o.value("rz").toDouble();
    return true;
}

static bool parseToolObj(const QJsonObject& o, const CmdKind kind, ToolCommand &out)
{
    if ( (!o.contains("name"))&&(kind==CmdKind::Tool_Mount || kind==CmdKind::Tool_UnMount) )
        return false;

    if ( (!o.contains("from") || !o.contains("to")) && (kind==CmdKind::Tool_Change) )
        return false;


    out.toolName = o.value("name").toString();
    out.toolFrom = o.value("from").toString();
    out.toolTo = o.value("to").toString();

    return true;
}

static bool parseOffsetObj(const QJsonObject& o, const CmdKind kind, sortingOffset &out)
{
//    if ( !o.contains("height") || !o.contains("thickness")|| !o.contains("shfit") )
    if ( !o.contains("height") || !o.contains("thickness"))
        return false;

    out.height = o.value("height").toInt(0);
    out.thickness = o.value("thickness").toInt(0);
//    out.shift = o.value("shift").toInt(0);

    return true;
}

static bool parseArrangeObj(const QJsonObject& o, arrangeCommnad &out)
{
    if ( !o.contains("start") || !o.contains("dest") )
        return false;

    const auto poseObj = o["start"].toObject();
    if (!parsePoseObj(poseObj, out.poseOrig))
        return false;

    const auto poseObj2 = o["dest"].toObject();
    if (!parsePoseObj(poseObj2, out.poseDest))
        return false;

    return true;
}

bool RobotCommandParser::parse(const QJsonObject& obj, RobotCommand& out)
{
    // 공통 필드
    const auto robotStr = obj.value("robot").toString("B");
    const auto typeStr  = obj.value("type").toString();
    const auto kindStr  = obj.value("kind").toString("");

    out.robot = parseRobotId(robotStr);
    out.type  = parseCmdType(typeStr);
    out.kind  = parseCmdKind(kindStr);

    out.seq   = obj.value("seq").toInt(0);
    out.dir   = obj.value("dir").toInt(0);

    out.flip   = obj.value("flip").toBool(false);
    out.offset = obj.value("offset").toInt(0);
    out.clamp  = obj.value("clamp").toString();

    out.mode = obj.value("mode").toString("dual");

    // 1) 새로운 포맷: pose 하나만 있는 경우
    if (obj.contains("pose") && obj["pose"].isObject()) {
        const auto poseObj = obj["pose"].toObject();
        switch (out.kind) {
        case CmdKind::Pick:
            out.hasPick = parsePoseObj(poseObj, out.pick);
            break;
        case CmdKind::Place:
            out.hasPlace = parsePoseObj(poseObj, out.place);
            break;
        default:
            // kind가 pick/place가 아니면, 필요에 맞게 처리
            // 여기서는 pick에 그냥 넣어두거나, 무시하거나 선택 가능
            out.hasPick = parsePoseObj(poseObj, out.pick);
            break;
        }
    }
    if(obj.contains("tool") && obj["tool"].isObject()) {
        const auto toolObj = obj["tool"].toObject();
        out.isTool=parseToolObj(toolObj, out.kind, out.toolCmd);
        //out.kind
    }
    if(obj.contains("offset") && obj["offset"].isObject()) {
        const auto offsetObj = obj["offset"].toObject();
        out.isOffset=parseOffsetObj(offsetObj, out.kind, out.sortOffset);
    }
    if(obj.contains("arrange") && obj["arrange"].isObject()) {
        const auto arrangeObj = obj["arrange"].toObject();
        out.isArrange=parseArrangeObj(arrangeObj, out.arrangeCmd);
    }

    // 2) 구버전 포맷: pick/place 개별 필드도 계속 지원 (하위호환)
    if (obj.contains("pick") && obj["pick"].isObject()) {
        out.hasPick = parsePoseObj(obj["pick"].toObject(), out.pick);
    }
    if (obj.contains("place") && obj["place"].isObject()) {
        out.hasPlace = parsePoseObj(obj["place"].toObject(), out.place);
    }
    out.raw = obj;

    // dir!=1 은 상위에서 거를 수 있도록 그대로 반환하거나, 여기서 false 처리할지 선택
    return true;
}
