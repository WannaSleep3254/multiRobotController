#ifndef VISIONSERVER_H
#define VISIONSERVER_H

#include <QObject>
#include <QVariantMap>
#include <QPointer>
#include <QTcpSocket>
#include <QSet>
#include <QHash>
#include <QJsonObject>

#include "Server.h"        // core/network/Server.h
#include "Pose6D.h"        // core/models/Pose6D.h
/*
VisionServer* vs = new VisionServer(this);

// 접속 즉시 화이트리스트 검사 활성화
VisionServer::EarlyFilterOptions ef;
ef.enforceWhitelist = true;
ef.whitelistIPs     = {"192.168.0.50", "192.168.0.60"}; // 허용 IP
ef.closeOnReject    = true;   // 미허용 즉시 차단
ef.logReject        = true;   // 로그 남김
vs->setEarlyFilterOptions(ef);

// 서버 시작
vs->start(QHostAddress::Any, 50000);
 */
struct PickPose {
    double x, y, z, rx, ry, rz;
};
struct JointPose {
    double j1, j2, j3, j4, j5, j6;
};

struct VisionRobotState {
    Pose6D tcp;
    Pose6D joints;
    qint64 tsMs{0};
    bool   valid{false};
};

class VisionServer : public QObject
{
    Q_OBJECT
public:
    struct Options {
        // ── 보안/접속 제한
        bool     enforceWhitelist = false;      // true면 화이트리스트에 없는 IP는 거부
        QSet<QString> whitelistIPs;             // "192.168.0.10", "10.0.0.5" 형식(IP만)
        int      maxConnections    = 128;       // (선택) 허용할 최대 동시 접속 수

        // ── rate-limit (토큰 버킷, 클라이언트별)
        int      ratePerSec        = 100;       // 초당 토큰 보충 개수 (라인/초)
        int      rateBurst         = 200;       // 버스트 용량 (버킷 최대치)
        bool     dropOnLimit       = true;      // true면 초과 라인은 드롭(권장)
        bool     closeOnFlood      = false;     // true면 반복 초과시 연결 종료

        // ── 메트릭
        bool     metricsEnabled    = true;      // 카운터 수집/스냅샷 허용
    };

    explicit VisionServer(QObject* parent=nullptr);

    // Server 제어 프록시
    bool start(quint16 port);
    bool start(const QHostAddress& bindAddr, quint16 port);
    void stop();

    // 인증 토큰(없으면 미사용)
    void setAuthToken(const QString& t) { m_token = t; }
    QString authToken() const { return m_token; }

    // 옵션 설정/조회
    void setOptions(const Options& o) { m_opt = o; }
    Options options() const { return m_opt; }

    // 메트릭 스냅샷(표시/로그/테스트용)
    QVariantMap metrics() const;              // 전체
    QVariantMap metricsFor(const QTcpSocket* s) const; // 개별 클라이언트

    void sendJson(const QJsonObject& obj);
    void sendStatus(const PickPose& p, const JointPose& j, const int id);
// KJW: 2025-10-28
    void updateRobotState(const QString& id, const Pose6D& tcp, const Pose6D& joints, qint64 tsMs);

signals:
    // 단건/다건 좌표 수신
    void poseReceived(const QString& robotId, const QString& kind, const Pose6D& pose,
                      quint32 seq, const QVariantMap& extras);


    void posesReceived(const QString& robotId, const QList<Pose6D>& poses,
                       quint32 seq, const QVariantMap& extras);

    void poseBulkReceived(const QString& robotId, const Pose6D& pick, const Pose6D& place,
                       quint32 seq, const QVariantMap& extras);

    void commandReceived(const QString& robotId, const QString& type, quint32 seq);

    // 상태/로그
    void log(const QString& line);
    void started(const QHostAddress& addr, quint16 port);
    void stopped();
    void peerCountChanged(int count);

private slots:
    void onLine(QTcpSocket* from, const QByteArray& line);

private:
    // 내부 헬퍼
    bool parsePoseObj(const QJsonObject& o, Pose6D& out) const;
    bool parsePickPoseObj(const QJsonObject& o, Pose6D& out) const;
    bool parsePlacePoseObj(const QJsonObject& o, Pose6D& out) const;

    QVariantMap collectExtras(const QJsonObject& o) const;
    void sendAck(QTcpSocket* to, quint32 seq, const QString& status,
                 const QString& message = QString());

    // 화이트리스트/레이트리밋
    bool allowedByWhitelist(QTcpSocket* s) const;
    bool passRateLimit(QTcpSocket* s);
    QString peerIp(QTcpSocket* s) const;

private:
    QPointer<Server> m_srv;    // 네트워크 레이어
    QString          m_token;  // 옵션 인증 토큰
    Options          m_opt;

    // ── 메트릭/레이트리밋 상태(클라이언트별)
    struct ClientStat {
        // token bucket
        int      tokens = 0;           // 현재 토큰
        qint64   lastRefillMs = 0;     // 마지막 리필 시각(ms, monotonic)

        // counters
        quint64  linesTotal = 0;
        quint64  linesDropped = 0;
        quint64  jsonOk = 0;
        quint64  jsonErr = 0;
        quint64  ackOk = 0;
        quint64  ackErr = 0;
        quint64  authFail = 0;
        quint64  whitelistBlock = 0;
        quint64  rateLimitBlock = 0;
    };
    QHash<QTcpSocket*, ClientStat> m_stats;

    // 글로벌 카운터
    struct GlobalStat {
        quint64 linesTotal = 0;
        quint64 linesDropped = 0;
        quint64 jsonOk = 0;
        quint64 jsonErr = 0;
        quint64 ackOk = 0;
        quint64 ackErr = 0;
        quint64 authFail = 0;
        quint64 whitelistBlock = 0;
        quint64 rateLimitBlock = 0;
    } m_global;

public:
    struct EarlyFilterOptions {
        bool            enforceWhitelist = false;     // true면 접속 즉시 화이트리스트 검사
        QSet<QString>   whitelistIPs;                 // 허용 IP 목록 ("192.168.0.50" 등)
        bool            closeOnReject   = true;       // 미허용이면 바로 close()
        bool            logReject       = true;       // 로그 남길지 여부
    };
    void setEarlyFilterOptions(const EarlyFilterOptions& o) { m_ef = o; }
    EarlyFilterOptions earlyFilterOptions() const { return m_ef; }
private slots:
    void onClientConnected(QTcpSocket* s);            // ★ 접속 즉시 검사 슬롯
private:
    EarlyFilterOptions m_ef;                          // ★ 옵션 보관

public:
    // 모든 클라이언트에 pick/inspect 좌표 요청
    void requestCapture(quint32 seq = 0, QString type = "3-A");

    void requestTestPose(quint32 seq = 0, int speed_pct = -1);
    void requestPickPose(quint32 seq = 0, int speed_pct = -1);
    void requestInspectPose(quint32 seq = 0, int speed_pct = -1);

    // 특정 클라이언트(targetId="ip:port")로만 요청
    void requestPickPoseTo(const QString& targetId, quint32 seq = 0, int speed_pct = -1);
    void requestInspectPoseTo(const QString& targetId, quint32 seq = 0, int speed_pct = -1);

    void requestPoseBulk(const char* kind, quint32 seq);

private:
    void requestCaptureKind(const char* kind, quint32 seq, int speed_pct);
    void requestPoseKind(const char* kind, quint32 seq, int speed_pct);
    void requestPoseKindTo(const QString& targetId, const char* kind, quint32 seq, int speed_pct);


    QHash<QString, VisionRobotState> m_latest;  // robotId → 최신 상태 캐시

};
#endif // VISIONSERVER_H
