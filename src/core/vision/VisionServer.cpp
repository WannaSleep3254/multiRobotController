#include "VisionServer.h"
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDateTime>

VisionServer::VisionServer(QObject* parent)
    : QObject(parent)
    , m_srv(new Server(this))
{
    // Server 이벤트를 그대로 포워딩
    connect(m_srv, &Server::log,               this, &VisionServer::log);
    connect(m_srv, &Server::started,           this, &VisionServer::started);
    connect(m_srv, &Server::stopped,           this, &VisionServer::stopped);
    connect(m_srv, &Server::peerCountChanged,  this, &VisionServer::peerCountChanged);

    // 핵심: 한 줄(line) 단위 JSON 파싱
    connect(m_srv, &Server::lineReceived, this, &VisionServer::onLine);
}

bool VisionServer::start(quint16 port)
{
    return start(QHostAddress::Any, port);
}

bool VisionServer::start(const QHostAddress& bindAddr, quint16 port)
{
    return m_srv->start(bindAddr, port);
}

void VisionServer::stop()
{
    m_srv->stop();
}

void VisionServer::onLine(QTcpSocket* from, const QByteArray& line)
{
    QJsonParseError pe{};
    const auto doc = QJsonDocument::fromJson(line, &pe);
    if (pe.error != QJsonParseError::NoError || !doc.isObject()) {
        emit log(QString("[ERR] Vision JSON parse failed: %1, line=%2")
                     .arg(pe.errorString(), QString::fromUtf8(line)));
        sendAck(from, 0, "error", "invalid_json");
        return;
    }

    const auto obj = doc.object();
    const auto type = obj.value("type").toString();
    const auto robot = obj.value("robot").toString(); // "A","B","C"... (없으면 빈 문자열)
    const quint32 seq = obj.value("seq").toInt(0);

    // 옵션 토큰 체크
    if (!m_token.isEmpty()) {
        const auto tok = obj.value("token").toString();
        if (tok != m_token) {
            emit log("[WARN] auth failed (token mismatch)");
            sendAck(from, seq, "error", "auth_failed");
            return;
        }
    }

    const auto extras = collectExtras(obj);

    if (type == "pose") {
        PickPose p{};
        if (!parsePoseObj(obj, p)) {
            sendAck(from, seq, "error", "missing_fields");
            return;
        }
        emit poseReceived(robot, p, seq, extras);
        sendAck(from, seq, "ok");
        return;
    }

    if (type == "poses") {
        QList<PickPose> list;
        const auto arr = obj.value("items").toArray();
        list.reserve(arr.size());
        for (const auto& v : arr) {
            const auto o = v.toObject();
            PickPose p{};
            if (!parsePoseObj(o, p)) continue; // 일부만 건너뜀
            list.push_back(p);
        }
        if (list.isEmpty()) {
            sendAck(from, seq, "error", "empty_items");
            return;
        }
        emit posesReceived(robot, list, seq, extras);
        sendAck(from, seq, "ok");
        return;
    }

    // 알 수 없는 타입
    emit log(QString("[ERR] Vision unknown type: %1").arg(type));
    sendAck(from, seq, "error", "unknown_type");
}

bool VisionServer::parsePoseObj(const QJsonObject& o, PickPose& out) const
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

QVariantMap VisionServer::collectExtras(const QJsonObject& o) const
{
    QVariantMap ex;
    // 자유 확장 필드들(있을 때만)
    if (o.contains("speed_pct")) ex["speed_pct"] = o.value("speed_pct").toInt();
    if (o.contains("tool_id"))   ex["tool_id"]   = o.value("tool_id").toInt();
    if (o.contains("frame_id"))  ex["frame_id"]  = o.value("frame_id").toInt();
    if (o.contains("tags"))      ex["tags"]      = o.value("tags").toVariant();

    // timestamp를 클라이언트가 보냈다면 전달 (아니면 서버 수신 시각 부여 가능)
    if (o.contains("ts"))        ex["ts"]        = o.value("ts").toString();
    else                         ex["ts"]        = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
    return ex;
}

void VisionServer::sendAck(QTcpSocket* to, quint32 seq, const QString& status, const QString& message)
{
    if (!to) return;
    QJsonObject o{
        {"type",   "ack"},
        {"seq",    static_cast<int>(seq)},
        {"status", status},
        {"ts",     QDateTime::currentDateTimeUtc().toString(Qt::ISODate)}
    };
    if (!message.isEmpty()) o["message"] = message;

    const auto bytes = QJsonDocument(o).toJson(QJsonDocument::Compact) + '\n';
    m_srv->writeTo(to, bytes);
}
