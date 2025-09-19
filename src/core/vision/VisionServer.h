#ifndef VISIONSERVER_H
#define VISIONSERVER_H

#include <QObject>
#include <QVariantMap>
#include <QPointer>
#include <QTcpSocket>

#include "Server.h"        // core/network/Server.h
#include "PickListModel.h" // PickPose 선언용

class VisionServer : public QObject
{
    Q_OBJECT
public:
    explicit VisionServer(QObject* parent=nullptr);

    // Server 제어 프록시
    bool start(quint16 port);
    bool start(const QHostAddress& bindAddr, quint16 port);
    void stop();

    // 선택: 간단 인증 토큰(없으면 무시)
    void setAuthToken(const QString& t) { m_token = t; }
    QString authToken() const { return m_token; }

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
    bool parsePoseObj(const QJsonObject& o, PickPose& out) const;
    QVariantMap collectExtras(const QJsonObject& o) const;
    void sendAck(QTcpSocket* to, quint32 seq, const QString& status,
                 const QString& message = QString());

private:
    QPointer<Server> m_srv;    // 네트워크 레이어
    QString m_token;           // 옵션 인증 토큰
};

#endif // VISIONSERVER_H
