#include "VisionClient.h"

#include <QJsonDocument>
#include <QDateTime>
#include <QHostAddress>

#include "RobotCommandParser.h"

VisionClient::VisionClient(QObject* parent)
    : QObject(parent)
    , m_sock(new QTcpSocket(this))
{
    connect(m_sock, &QTcpSocket::connected,    this, &VisionClient::onConnected);
    connect(m_sock, &QTcpSocket::disconnected, this, &VisionClient::onDisconnected);
    connect(m_sock, &QTcpSocket::readyRead,    this, &VisionClient::onReadyRead);
}

VisionClient::~VisionClient()
{
    disconnectFrom();
}

bool VisionClient::connectTo(const QString& host, quint16 port)
{
    if (m_sock->state() == QAbstractSocket::ConnectedState)
        return true;

    m_sock->connectToHost(host, port);
    if (!m_sock->waitForConnected(2000)) {
        emit log(QString("[ERR] Connect failed: %1").arg(m_sock->errorString()));
        return false;
    }

    emit log(QString("[OK] Connected to VisionServer %1:%2").arg(host).arg(port));
    return true;
}

void VisionClient::disconnectFrom()
{
    if (m_sock->state() == QAbstractSocket::ConnectedState)
        m_sock->disconnectFromHost();
}

void VisionClient::sendPose(const PickPose& p, const QString& kind, const int dir, const int id, quint32 seq, int speed_pct)
{
    QJsonObject o{
        {"type", "pose"},
        {"kind", kind},
        {"robot", id},  // 1: 로봇1, 2: 로봇2
        {"seq", static_cast<int>(seq)},
        {"x", p.x}, {"y", p.y}, {"z", p.z},
        {"rx", p.rx}, {"ry", p.ry}, {"rz", p.rz},
        {"dir", dir} ,
        {"speed_pct", speed_pct}
    };
    sendJson(o);
}

void VisionClient::sendAck(quint32 seq, const QString& status, const QString& msg)
{
    QJsonObject o{
        {"type","ack"},
        {"seq", static_cast<int>(seq)},
        {"ts",  QDateTime::currentDateTimeUtc().toString(Qt::ISODate)},
        {"status", status}
    };
    if (!msg.isEmpty()) o["message"] = msg;

    const QByteArray json = QJsonDocument(o).toJson(QJsonDocument::Compact) + '\n';
//    enqueueJson(json);
}

void VisionClient::sendFeedbackPose(const QString& robot, const QString& from, const QString& to, quint32 seq)
{
    if (robot.isEmpty())
        return;

    QJsonObject o{
        {"robot", robot.toLower()},
        {"type", "tool"},
        {"kind","moving"},
        {"tool", QJsonObject {
            {"from", from},
            {"to", to}
        }},
        {"seq", static_cast<int>(seq)},
        {"dir", 11}
    };

    const QByteArray json = QJsonDocument(o).toJson(QJsonDocument::Compact) + '\n';
    qDebug()<<QDateTime::currentDateTime()<<QString("VisionClient::sendFeedbackPose json: %1").arg(QString::fromUtf8(json).trimmed());
    enqueueJson(json);
}

void VisionClient::sendWorkComplete(const QString& robot, const QString& type, const QString& kind, quint32 seq, bool clampState)
{
    if (robot.isEmpty() || kind.isEmpty())
        return;

    QJsonObject o{
        {"robot", robot.toLower()},
        {"type", type},
        {"kind", kind},
        {"success", true},
        {"error_code", 0},
        {"seq", static_cast<int>(seq)},
        {"dir",  11}
    };
    if (robot.toLower()=="b" && type == "align"&& kind == "clamp") {
        o["clamp"] = clampState ? "close" : "open";
        qDebug()<<QDateTime::currentDateTime()<<QString("VisionClient::sendWorkComplete clamp state: %1").arg(clampState?"close":"open");
    }
    const QByteArray json = QJsonDocument(o).toJson(QJsonDocument::Compact) + '\n';
    qDebug()<<QDateTime::currentDateTime()<<QString("VisionClient::sendWorkComplete json: %1").arg(QString::fromUtf8(json).trimmed());

    enqueueJson(json);
}

void VisionClient::sendToolComplete(const QString& robot, quint32 seq, bool state)
{
    if (robot.isEmpty())
        return;

    if (m_lastCmdKind == CmdKind::Unknown)
        return;

    QString kind,name,from,to;
    QJsonObject toolObj;
    if (m_lastCmdKind == CmdKind::Tool_Mount){
        kind = "mount";
        name = m_lastToolCmd.toolName;
        toolObj["name"] = name;
    }else if (m_lastCmdKind == CmdKind::Tool_UnMount){
        kind = "unmount";
        name = m_lastToolCmd.toolName;
        toolObj["name"] = name;
    }else if (m_lastCmdKind == CmdKind::Tool_Change){
        kind = "change";
        from = m_lastToolCmd.toolFrom;
        to = m_lastToolCmd.toolTo;
        toolObj["from"] = from;
        toolObj["to"] = to;
    }

    QJsonObject o{
        {"robot", robot.toLower()},
        {"type", "tool"},
        {"kind", kind},
        {"tool", toolObj},
        {"success", true},
        {"error_code", 0},
        {"seq", static_cast<int>(seq)},
        {"dir",  11}
    };
    const QByteArray json = QJsonDocument(o).toJson(QJsonDocument::Compact) + '\n';
    enqueueJson(json);
}

void VisionClient::enqueueJson(const QByteArray& json)
{
    if (json.isEmpty())
        return;

    m_jsonQueue.enqueue(json);

    // 타이머가 안 돌고 있으면 시작
    if (!m_jsonTimer.isActive()) {
        m_jsonTimer.setTimerType(Qt::PreciseTimer);

        connect(&m_jsonTimer, &QTimer::timeout, this, &VisionClient::onTxJson, Qt::UniqueConnection);
        m_jsonTimer.start(m_jsonIntervalMs);
    }
}

void VisionClient::onTxJson()
{
    if (!m_sock || m_sock->state() != QAbstractSocket::ConnectedState)
    {
        m_jsonTimer.stop();
        m_jsonQueue.clear();

        return;
    }

    if (m_jsonQueue.isEmpty()) {
        m_jsonTimer.stop();
        return;
    }

    const QByteArray byte_json = m_jsonQueue.dequeue();
    quint64 len =m_sock->write(byte_json);
    qDebug()<<QDateTime::currentDateTime()<<QString("VisionClient::onTxJson sent %1 bytes: %2").arg(len).arg(QString::fromUtf8(byte_json).trimmed());
//    m_sock->flush();
/*
    // 다 비면 타이머 정지
    if (m_jsonQueue.isEmpty())
        m_jsonTimer.stop();
*/
}

void VisionClient::sendJson(const QJsonObject& obj)
{
    if (!m_sock || m_sock->state() != QAbstractSocket::ConnectedState) {
        emit log("[ERR] Socket not connected");
        return;
    }

    const QByteArray json = QJsonDocument(obj).toJson(QJsonDocument::Compact) + '\n';
    enqueueJson(json);
    emit log(QString("[TX] %1").arg(QString::fromUtf8(json).trimmed()));
}

void VisionClient::onConnected()
{
    emit connected();
    emit log("[NET] VisionClient connected");
}

void VisionClient::onDisconnected()
{
    emit disconnected();
    emit log("[NET] VisionClient disconnected");
}

void VisionClient::onReadyRead()
{
    m_inbuf.append(m_sock->readAll());

    int idx = -1;
    while ((idx = m_inbuf.indexOf('\n')) >= 0) {
        QByteArray line = m_inbuf.left(idx);
        m_inbuf.remove(0, idx + 1);

        if (!line.isEmpty() && line.endsWith('\r'))
            line.chop(1);

        line = line.trimmed();
        if (line.isEmpty())
            continue;

        const QString s = QString::fromUtf8(line);
        emit lineReceived(s);   // 원하면 유지

        // ── JSON 파싱
        QJsonParseError pe{};
        const auto doc = QJsonDocument::fromJson(line, &pe);
        if (pe.error != QJsonParseError::NoError || !doc.isObject())
            continue;

        const auto obj  = doc.object();
        const auto type = obj.value("type").toString();
        const auto dir = obj.value("dir").toInt(0);

        if (type == "ack") {
            const auto seq    = obj.value("seq").toInt(0);
            const auto status = obj.value("status").toString();
            const auto msg    = obj.value("message").toString();
            emit ackReceived(seq, status, msg);
        }

        if (dir != 1)
            continue;   // DIR 1 아닌 CMD는 그냥 무시

        RobotCommand cmd;
        if (RobotCommandParser::parse(obj, cmd)) {
            const QByteArray compact = QJsonDocument(obj).toJson(QJsonDocument::Compact);
            qDebug()<<QDateTime::currentDateTime()<<compact.trimmed();

            if (cmd.type == CmdType::Tool) {
                m_lastCmdKind = cmd.kind;
                m_lastToolCmd = cmd.toolCmd;
            }
            emit commandReceived(cmd);
        }
    }
}
