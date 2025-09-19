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
    const auto robot = obj.value("robot").toString(); // "A","B","C"... (없으면 빈 문자열)
    const quint32 seq = obj.value("seq").toInt(0);

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

    const auto extras = collectExtras(obj);

    if (type == "pose") {
        PickPose p{};
        if (!parsePoseObj(obj, p)) {
            ++st.jsonErr; ++m_global.jsonErr;
            sendAck(from, seq, "error", "missing_fields");
            ++st.ackErr; ++m_global.ackErr;
            return;
        }
        emit poseReceived(robot, p, seq, extras);
        sendAck(from, seq, "ok");
        ++st.jsonOk; ++m_global.jsonOk;
        ++st.ackOk;  ++m_global.ackOk;
        return;
    }

    if (type == "poses") {
        QList<PickPose> list;
        const auto arr = obj.value("items").toArray();
        list.reserve(arr.size());
        for (const auto& v : arr) {
            const auto o = v.toObject();
            PickPose p{};
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

    // 알 수 없는 타입
    ++st.jsonErr; ++m_global.jsonErr;
    emit log(QString("[ERR] Vision unknown type: %1").arg(type));
    sendAck(from, seq, "error", "unknown_type");
    ++st.ackErr; ++m_global.ackErr;
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
