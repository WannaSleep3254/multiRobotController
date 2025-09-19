#ifndef VISIONSERVER_H
#define VISIONSERVER_H

#include <QObject>
#include <QVariantMap>
#include <QPointer>
#include <QTcpSocket>
#include <QSet>
#include <QHash>

#include "Server.h"        // core/network/Server.h
#include "PickListModel.h" // PickPose 선언용

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

signals:
    // 단건/다건 좌표 수신
    void poseReceived(const QString& robotId, const PickPose& pose,
                      quint32 seq, const QVariantMap& extras);
    void posesReceived(const QString& robotId, const QList<PickPose>& poses,
                       quint32 seq, const QVariantMap& extras);

    // 상태/로그
    void log(const QString& line);
    void started(const QHostAddress& addr, quint16 port);
    void stopped();
    void peerCountChanged(int count);

private slots:
    void onLine(QTcpSocket* from, const QByteArray& line);

private:
    // 내부 헬퍼
    bool parsePoseObj(const QJsonObject& o, PickPose& out) const;
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

};
#endif // VISIONSERVER_H
