#include "VisionServer.h"
#include <QJsonDocument>
#include <QJsonArray>
#include <QDateTime>
#include <QElapsedTimer>

static inline qint64 nowMs() {
#if QT_VERSION >= QT_VERSION_CHECK(5, 8, 0)
    return QElapsedTimer::clockType() == QElapsedTimer::MonotonicClock
               ? QElapsedTimer().msecsSinceReference()
               : QDateTime::currentMSecsSinceEpoch();
#else
    return QDateTime::currentMSecsSinceEpoch();
#endif
}

VisionServer::VisionServer(QObject* parent)
    : QObject(parent)
    , m_srv(new Server(this))
{
    // Server 이벤트 포워딩
    connect(m_srv, &Server::log,               this, &VisionServer::log);
    connect(m_srv, &Server::started,           this, &VisionServer::started);
    connect(m_srv, &Server::stopped,           this, &VisionServer::stopped);
    connect(m_srv, &Server::peerCountChanged,  this, &VisionServer::peerCountChanged);
    connect(m_srv, &Server::lineReceived,      this, &VisionServer::onLine);

    // ★ 추가: 접속 즉시 화이트리스트 검사
    connect(m_srv, &Server::clientConnected,   this, &VisionServer::onClientConnected);
}

bool VisionServer::start(quint16 port)
{
    return start(QHostAddress::Any, port);
}

bool VisionServer::start(const QHostAddress& a, quint16 p)
{
    return m_srv->start(a, p);
}

void VisionServer::stop()
{
    m_srv->stop();
}

QString VisionServer::peerIp(QTcpSocket* s) const
{
    return s ? s->peerAddress().toString() : QString();
}

bool VisionServer::allowedByWhitelist(QTcpSocket* s) const
{
    if (!m_opt.enforceWhitelist) return true;
    const QString ip = peerIp(s);
    return m_opt.whitelistIPs.contains(ip);
}

bool VisionServer::passRateLimit(QTcpSocket* s)
{
    if (!s) return false;
    auto& st = m_stats[s];
    const qint64 t = nowMs();

    // 초기화
    if (st.lastRefillMs == 0) {
        st.lastRefillMs = t;
        st.tokens = m_opt.rateBurst;
    }

    // 리필
    if (m_opt.ratePerSec > 0) {
        const qint64 elapsedMs = t - st.lastRefillMs;
        if (elapsedMs > 0) {
            const qint64 refill = (elapsedMs * m_opt.ratePerSec) / 1000;
            if (refill > 0) {
                st.tokens = qMin(m_opt.rateBurst, st.tokens + int(refill));
                st.lastRefillMs = t;
            }
        }
    } else {
        st.tokens = m_opt.rateBurst;
    }

    // 소비
    if (st.tokens > 0) {
        --st.tokens;
        return true;
    }
    return false;
}

void VisionServer::onLine(QTcpSocket* from, const QByteArray& line)
{
    auto& st = m_stats[from];
    ++st.linesTotal; ++m_global.linesTotal;

    qDebug()<<"[VS] Line received from"<<peerIp(from)<<":"<<QString::fromUtf8(line);
#if false
    // ── 화이트리스트 검사
    if (!allowedByWhitelist(from)) {
        ++st.whitelistBlock; ++m_global.whitelistBlock;
        if (m_opt.metricsEnabled)
            emit log(QString("[SEC] whitelist block from %1").arg(peerIp(from)));
        // 필요 시 연결 종료
        if (m_opt.closeOnFlood) {
            emit log("[SEC] closing unwhitelisted connection");
            from->disconnect(this);
            from->close();
        }
        return;
    }

    qDebug()<<"[VS] Whitelist passed for"<<peerIp(from);
    // ── 레이트리밋
    if (!passRateLimit(from)) {
        ++st.rateLimitBlock; ++m_global.rateLimitBlock;
        if (m_opt.metricsEnabled)
            emit log(QString("[RL] rate-limit drop from %1").arg(peerIp(from)));
        if (m_opt.dropOnLimit) {
            ++st.linesDropped; ++m_global.linesDropped;
            if (m_opt.closeOnFlood && (st.rateLimitBlock % 100 == 0)) {
                // 과도한 버스트 반복 시 선택적으로 차단
                from->disconnect(this);
                from->close();
            }
            return; // 조용히 드롭(또는 아래에서 ack:error를 보낼 수도 있음)
        } else {
            // 강제 에러 응답(옵션—현재 미사용)
            sendAck(from, 0, "error", "rate_limit");
            ++st.ackErr; ++m_global.ackErr;
            return;
        }
    }
#endif
    // ── JSON 파싱
    QJsonParseError pe{};
    const auto doc = QJsonDocument::fromJson(line, &pe);
    if (pe.error != QJsonParseError::NoError || !doc.isObject()) {
        ++st.jsonErr; ++m_global.jsonErr;
        emit log(QString("[ERR] Vision JSON parse failed: %1, line=%2")
                     .arg(pe.errorString(), QString::fromUtf8(line)));
        sendAck(from, 0, "error", "invalid_json");
        ++st.ackErr; ++m_global.ackErr;
        return;
    }

    const auto obj   = doc.object();
    const auto type  = obj.value("type").toString();
    const auto robot = obj.value("robot").toString("B"); // "A","B","C"... (없으면 빈 문자열)
    const auto kind = obj.value("kind").toString(""); // "pick", "place" 등 (없으면 빈 문자열)
    const quint32 seq = obj.value("seq").toInt(0);
    const auto dir = obj.value("dir").toInt(0);

    qDebug()<<"[VS] Parsed JSON type="<<type<<"robot="<<robot<<"seq="<<seq;
#if false
    // ── 인증 토큰
    if (!m_token.isEmpty()) {
        const auto tok = obj.value("token").toString();
        if (tok != m_token) {
            ++st.authFail; ++m_global.authFail;
            emit log(QString("[WARN] auth failed from %1").arg(peerIp(from)));
            sendAck(from, seq, "error", "auth_failed");
            ++st.ackErr; ++m_global.ackErr;
            return;
        }
    }
#endif
    const auto extras = collectExtras(obj);

    if (type == "pose") {
        if(robot=="A" && kind=="bulk" && dir==1)
        {
            // bulk pick/place 좌표
            Pose6D pick{}, place{};
            // pick
            if(obj.contains("pick")&&obj["pick"].isObject())
            {
                parsePoseObj(obj["pick"].toObject(), pick);
                //qDebug()<<"[VS] Pick Pose parsed:"<<pick.x<<pick.y<<pick.z<<pick.rx<<pick.ry<<pick.rz;
            }
            else
            {
                qDebug()<<"[VS] Pick Pose parsing failed";
                ++st.jsonErr; ++m_global.jsonErr;
                sendAck(from, seq, "error", "missing_fields [pick]");
                ++st.ackErr; ++m_global.ackErr;
                return;
            }
            // place
            if(obj.contains("place")&&obj["place"].isObject())
            {
                parsePoseObj(obj["place"].toObject(), place);
                //qDebug()<<"[VS] place Pose parsed:"<<place.x<<place.y<<place.z<<place.rx<<place.ry<<place.rz;
            }
            else
            {
                qDebug()<<"[VS] PlacePose parsing failed";
                ++st.jsonErr; ++m_global.jsonErr;
                sendAck(from, seq, "error", "missing_fields [place]");
                ++st.ackErr; ++m_global.ackErr;
                return;
            }
//            qDebug()<<"[VS] Bulk Pose parsed:";
            emit poseBulkReceived(robot, pick, place, seq, extras);
            sendAck(from, seq, "ok");
            ++st.jsonOk; ++m_global.jsonOk;
            ++st.ackOk;  ++m_global.ackOk;
            return;
        }
        else if(robot=="B")
        {
            Pose6D p{};
            if (!parsePoseObj(obj, p)) {
                qDebug()<<"[VS] Pose parsing failed";
                ++st.jsonErr; ++m_global.jsonErr;
                sendAck(from, seq, "error", "missing_fields [pose]");
                ++st.ackErr; ++m_global.ackErr;
                return;
            }
    //        qDebug()<<"[VS] Pose parsed:"<<p.x<<p.y<<p.z<<p.rx<<p.ry<<p.rz<<dir;

            if(kind.isEmpty())
            {
                //TODO: kind 이 없으면 에러 처리
                qDebug()<<"[VS] Pose kind missing";
                ++st.jsonErr; ++m_global.jsonErr;
                sendAck(from, seq, "error", "missing_kind");
                ++st.ackErr; ++m_global.ackErr;
                return;
            }

            emit poseReceived(robot, kind, p, seq, extras);
            sendAck(from, seq, "ok");
            ++st.jsonOk; ++m_global.jsonOk;
            ++st.ackOk;  ++m_global.ackOk;
            return;
        }
    }
    else if(type == "stop")
    {
        if(robot=="A" && kind=="bulk" && dir==1)
        {
            qDebug() << "[VS] Bulk Stop command received for robot"<<robot;
            emit commandReceived(robot, kind, seq);
        }
    }
    else if(type == "status")
    {
        // 1) 요청에서 로봇 식별자/ID 가져오기 (없으면 기본 "A")
        const QString rid = obj.value("robot").toString().isEmpty()
                                ? QStringLiteral("A")
                                : obj.value("robot").toString();
        const int reqId = obj.value("id").toInt(0); // 요청에 id가 있으면 우선

        // 2) 최신 캐시 찾기
        const auto it = m_latest.constFind(rid);
        if (it == m_latest.cend() || !it->valid) {
            // 데이터가 아직 없으면 0으로 채워서 동일 형식으로 보냄
            PickPose  pose{0,0,0,0,0,0};
            JointPose joint{0,0,0,0,0,0};
            const int id = reqId ? reqId : 1; // robot "A"=1, "B"=2 등 규칙 쓰고 싶으면 아래 헬퍼 사용
            sendStatus(pose, joint, id);
            return ;
        }

        // 3) 캐시 → PickPose / JointPose 로 변환 (brace-init)
        const auto& s = *it;
        PickPose  pose{
            float(s.tcp.x), float(s.tcp.y), float(s.tcp.z),
            float(s.tcp.rx), float(s.tcp.ry), float(s.tcp.rz)
        };
        JointPose joint{
            float(s.joints.x), float(s.joints.y), float(s.joints.z),
            float(s.joints.rx), float(s.joints.ry), float(s.joints.rz)
        };

        // 4) id 결정: 요청에 없으면 로봇명으로부터 매핑
        auto robotStrToId = [](const QString& r){
            if (r.size()==1 && r[0]>='A' && r[0]<='Z') return (r[0].unicode()-'A')+1; // A->1, B->2 ...
            bool ok=false; int n=r.toInt(&ok); return ok? n : 1;
        };
        const int id = reqId ? reqId : robotStrToId(rid);

        // 5) 원하는 형식 그대로 전송
        sendStatus(pose, joint, id);
        return;
        /////////////////////////////
    }
#if false
    if (type == "poses") {
        QList<Pose6D> list;
        const auto arr = obj.value("items").toArray();
        list.reserve(arr.size());
        for (const auto& v : arr) {
            const auto o = v.toObject();
            Pose6D p{};
            if (!parsePoseObj(o, p)) continue;
            list.push_back(p);
        }
        if (list.isEmpty()) {
            ++st.jsonErr; ++m_global.jsonErr;
            sendAck(from, seq, "error", "empty_items");
            ++st.ackErr; ++m_global.ackErr;
            return;
        }
        emit posesReceived(robot, list, seq, extras);
        sendAck(from, seq, "ok");
        ++st.jsonOk; ++m_global.jsonOk;
        ++st.ackOk;  ++m_global.ackOk;
        return;
    }
#endif
    // 알 수 없는 타입
    ++st.jsonErr;
    ++m_global.jsonErr;
    emit log(QString("[ERR] Vision unknown type: %1").arg(type));
    sendAck(from, seq, "error", "unknown_type");
    ++st.ackErr;
    ++m_global.ackErr;
}

bool VisionServer::parsePoseObj(const QJsonObject& o, Pose6D &out) const
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

bool VisionServer::parsePickPoseObj(const QJsonObject& o, Pose6D &out) const
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

bool VisionServer::parsePlacePoseObj(const QJsonObject& o, Pose6D &out) const
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
    if (o.contains("speed_pct")) ex["speed_pct"] = o.value("speed_pct").toInt();
    if (o.contains("tool_id"))   ex["tool_id"]   = o.value("tool_id").toInt();
    if (o.contains("frame_id"))  ex["frame_id"]  = o.value("frame_id").toInt();
    if (o.contains("tags"))      ex["tags"]      = o.value("tags").toVariant();
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
    if (status == "ok") { ++m_global.ackOk; } else { ++m_global.ackErr; }
    m_srv->writeTo(to, bytes);
}

QVariantMap VisionServer::metrics() const
{
    QVariantMap m;
    if (!m_opt.metricsEnabled) return m;
    m["lines_total"]      = static_cast<qulonglong>(m_global.linesTotal);
    m["lines_dropped"]    = static_cast<qulonglong>(m_global.linesDropped);
    m["json_ok"]          = static_cast<qulonglong>(m_global.jsonOk);
    m["json_err"]         = static_cast<qulonglong>(m_global.jsonErr);
    m["ack_ok"]           = static_cast<qulonglong>(m_global.ackOk);
    m["ack_err"]          = static_cast<qulonglong>(m_global.ackErr);
    m["auth_fail"]        = static_cast<qulonglong>(m_global.authFail);
    m["whitelist_block"]  = static_cast<qulonglong>(m_global.whitelistBlock);
    m["ratelimit_block"]  = static_cast<qulonglong>(m_global.rateLimitBlock);
    return m;
}

QVariantMap VisionServer::metricsFor(const QTcpSocket* s) const
{
    QVariantMap m;
    if (!m_opt.metricsEnabled || !s) return m;
    const auto it = m_stats.constFind(const_cast<QTcpSocket*>(s));
    if (it == m_stats.cend()) return m;

    const ClientStat& st = it.value();
    m["lines_total"]     = static_cast<qulonglong>(st.linesTotal);
    m["lines_dropped"]   = static_cast<qulonglong>(st.linesDropped);
    m["json_ok"]         = static_cast<qulonglong>(st.jsonOk);
    m["json_err"]        = static_cast<qulonglong>(st.jsonErr);
    m["ack_ok"]          = static_cast<qulonglong>(st.ackOk);
    m["ack_err"]         = static_cast<qulonglong>(st.ackErr);
    m["auth_fail"]       = static_cast<qulonglong>(st.authFail);
    m["whitelist_block"] = static_cast<qulonglong>(st.whitelistBlock);
    m["ratelimit_block"] = static_cast<qulonglong>(st.rateLimitBlock);
    m["tokens"]          = st.tokens;
    return m;
}

void VisionServer::onClientConnected(QTcpSocket* s)
{
    if (!s || !m_ef.enforceWhitelist) return;

    const QString ip = peerIp(s);
    const bool allowed = m_ef.whitelistIPs.contains(ip);

    if (!allowed) {
        if (m_ef.logReject)
            emit log(QString("[SEC] reject (early whitelist): %1").arg(ip));

        if (m_ef.closeOnReject) {
            // 접속 즉시 차단
            s->disconnect(this);
            s->close();
        }
        // 필요하면 여기서 '거부 사유'를 소켓에 한 줄 써줄 수도 있지만,
        // 보안/공격자에게 힌트가 될 수 있어 보통은 바로 끊는 걸 권장합니다.
    }
}

// 공통: JSON 한 줄 생성 + 브로드캐스트
void VisionServer::requestCaptureKind(const char* kind, quint32 seq, int speed_pct)
{
    const auto it = m_latest.constFind("B"); // 기본 로봇 "B"");
    PickPose  pose{};
    if (it != m_latest.cend() && it->valid) {
        const auto& s = *it;
        pose  = PickPose { float(s.tcp.x),    float(s.tcp.y),    float(s.tcp.z),
                           float(s.tcp.rx),   float(s.tcp.ry),   float(s.tcp.rz) };
    } else {
        // 최신 데이터가 없으면 0으로 채우거나, 여기서 return으로 보내지 않도록 선택하세요.
        // return;  // ← 이렇게 막아도 됨
    }

    QJsonObject o{
        {"type", "request_pose"},
        {"kind", QString::fromUtf8(kind)},
        {"robot", 2},  // 1: 로봇1, 2: 로봇2
        {"x", pose.x}, {"y", pose.y}, {"z", pose.z}, {"rx", pose.rx}, {"ry", pose.ry}, {"rz", pose.rz},
        {"seq",  int(seq)},
        {"ts",   QDateTime::currentDateTimeUtc().toString(Qt::ISODate)}
    };
    if (speed_pct >= 0) o["speed_pct"] = speed_pct;

    const QByteArray line = QJsonDocument(o).toJson(QJsonDocument::Compact) + '\n';
    qDebug()<<"[VS] Broadcasting request_pose:"<<QString::fromUtf8(line);

    if (m_srv) m_srv->broadcast(line);

    emit log(QString("[NET] request_pose(kind=%1, seq=%2, speed=%3) broadcast")
                 .arg(kind).arg(seq).arg(speed_pct));
}

void VisionServer::requestPoseBulk(const char* kind, quint32 seq)
{
    QJsonObject o{
        {"type", "ready"},
        {"kind", QString::fromUtf8(kind)},
        {"robot", "A"},  // 1: 로봇1, 2: 로봇2},
        {"dir", 2},  //0: none, 1: 비전->로봇(명령), 2: 로봇->비전(상태)},
        {"seq",  int(seq)},
        {"ts",   QDateTime::currentDateTimeUtc().toString(Qt::ISODate)}
    };

    const QByteArray line = QJsonDocument(o).toJson(QJsonDocument::Compact) + '\n';
    qDebug()<<"[VS] Broadcasting request_pose:"<<QString::fromUtf8(line);
    if (m_srv) m_srv->broadcast(line);
    emit log(QString("[NET] request_pose(kind=%1, seq=%2 broadcast")
                 .arg(kind).arg(seq));
}

void VisionServer::requestPoseKind(const char* kind, quint32 seq, int speed_pct)
{
    QJsonObject o{
        {"type", "request_pose"},
        {"kind", QString::fromUtf8(kind)},
        {"seq",  int(seq)},
        {"ts",   QDateTime::currentDateTimeUtc().toString(Qt::ISODate)}
    };
    if (speed_pct >= 0) o["speed_pct"] = speed_pct;

    const QByteArray line = QJsonDocument(o).toJson(QJsonDocument::Compact) + '\n';
    qDebug()<<"[VS] Broadcasting request_pose:"<<QString::fromUtf8(line);
    if (m_srv) m_srv->broadcast(line);
    emit log(QString("[NET] request_pose(kind=%1, seq=%2, speed=%3) broadcast")
                 .arg(kind).arg(seq).arg(speed_pct));
}

// 특정 타깃으로 전송
void VisionServer::requestPoseKindTo(const QString& targetId, const char* kind, quint32 seq, int speed_pct)
{
    QJsonObject o{
        {"type", "request_pose"},
        {"kind", QString::fromUtf8(kind)},
        {"seq",  int(seq)},
        {"ts",   QDateTime::currentDateTimeUtc().toString(Qt::ISODate)}
    };
    if (speed_pct >= 0) o["speed_pct"] = speed_pct;

    const QByteArray line = QJsonDocument(o).toJson(QJsonDocument::Compact) + '\n';
    if (m_srv) m_srv->writeToId(targetId, line);
    emit log(QString("[NET] request_pose(kind=%1, seq=%2, speed=%3) -> %4")
                 .arg(kind).arg(seq).arg(speed_pct).arg(targetId));
}

void VisionServer::requestCapture(quint32 seq, QString type)
{
    requestCaptureKind(type.toUtf8().constData(), seq, -1);
}

void VisionServer::requestTestPose(quint32 seq, int speed_pct)       { requestPoseKind("test",    seq, speed_pct); }
void VisionServer::requestPickPose(quint32 seq, int speed_pct)       { requestPoseKind("pick",    seq, speed_pct); }
void VisionServer::requestInspectPose(quint32 seq, int speed_pct)    { requestPoseKind("inspect", seq, speed_pct); }

void VisionServer::requestPickPoseTo(const QString& id, quint32 s, int v)    { requestPoseKindTo(id, "pick",    s, v); }
void VisionServer::requestInspectPoseTo(const QString& id, quint32 s, int v) { requestPoseKindTo(id, "inspect", s, v); }

void VisionServer::sendStatus(const PickPose& p, const JointPose& j, const int id)
{
    QJsonObject o{
                  {"type", "status"},
                  {"ready", 1}, //1:Ready , 0: Busy
                  {"robot", id},  // 1: 로봇1, 2: 로봇2
                  {"x", p.x}, {"y", p.y}, {"z", p.z}, {"rx", p.rx}, {"ry", p.ry}, {"rz", p.rz},
                  {"j1", j.j1}, {"j2", j.j2}, {"j3", j.j3}, {"j4", j.j4}, {"j5", j.j5}, {"j6", j.j6},

                  };
    sendJson(o);
}

void VisionServer::sendJson(const QJsonObject& obj)
{

    const QByteArray json = QJsonDocument(obj).toJson(QJsonDocument::Compact) + '\n';
    //const QByteArray line = QJsonDocument(o).toJson(QJsonDocument::Compact) + '\n';
    qDebug()<<"[VS] Broadcasting JSON:"<<QString::fromUtf8(json);
    m_srv->broadcast(json);
}

void VisionServer::updateRobotState(const QString& id, const Pose6D& tcp, const Pose6D& joints, qint64 tsMs)
{
    auto &s = m_latest[id];
    s.tcp = tcp;
    s.joints = joints;
    s.tsMs = tsMs;
    s.valid = true;
#if false
    emit log(QString("[VS] state cached (%1) tcp=[%2,%3,%4,%5,%6,%7]")
                 .arg(id)
                 .arg(tcp.x, 0, 'f', 1)
                 .arg(tcp.y, 0, 'f', 1)
                 .arg(tcp.z, 0, 'f', 1)
                 .arg(tcp.rx, 0, 'f', 1)
                 .arg(tcp.ry, 0, 'f', 1)
                 .arg(tcp.rz, 0, 'f', 1));
#endif
}
