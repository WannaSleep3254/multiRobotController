#ifndef VISIONCLIENT_H
#define VISIONCLIENT_H

#include <QObject>
#include <QTcpSocket>
#include <QJsonObject>

#include "RobotCommand.h"

class VisionClient : public QObject
{
    Q_OBJECT
public:
    explicit VisionClient(QObject *parent = nullptr);

    ~VisionClient();

    bool connectTo(const QString& host, quint16 port);
    void disconnectFrom();
    // 단일 Pose 전송
    void sendPose(const PickPose& p, const QString& kind, const int dir, const int id, quint32 seq = 0, int speed_pct = 50);
    void sendAck(quint32 seq, const QString& status, const QString& msg = QString());
    void sendWorkComplete(const QString& robot, const QString& type, const QString& kind, quint32 seq, bool clampState = false);

signals:
    void connected();
    void disconnected();
    void ackReceived(quint32 seq, const QString& status, const QString& msg);
    void log(const QString& line);
    void lineReceived(const QString& line);
    void commandReceived(const RobotCommand& cmd);

private slots:
    void onConnected();
    void onDisconnected();
    void onReadyRead();

private:
    void sendJson(const QJsonObject& obj);

private:
    QTcpSocket* m_sock = nullptr;
    QByteArray m_inbuf;
};

#endif // VISIONCLIENT_H
