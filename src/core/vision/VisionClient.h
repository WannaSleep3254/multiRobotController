#ifndef VISIONCLIENT_H
#define VISIONCLIENT_H

#include <QObject>
#include <QTcpSocket>
#include <QJsonObject>

#include <QQueue>
#include <QTimer>

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
    void sendFeedbackPose(const QString& robot, const QString& from, const QString& to, quint32 seq);
    void sendWorkComplete(const QString& robot, const QString& type, const QString& kind, quint32 seq, bool clampState = false);
    void sendToolComplete(const QString& robot, quint32 seq, bool state);

    void sendError(const QString& robot, QString error, int code1=0, int code2=0);
    void enqueueJson(const QByteArray& json);

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

private slots:
    void onTxJson();

private:
    void sendJson(const QJsonObject& obj);

private:
    QTcpSocket* m_sock = nullptr;
    QByteArray m_inbuf;

    CmdKind m_lastCmdKind {CmdKind::Unknown};
    ToolCommand m_lastToolCmd;

    QQueue<QByteArray> m_jsonQueue;
    QTimer m_jsonTimer;
    int m_jsonIntervalMs{1};

private:
    RobotCommand lasCmd_[2];
    bool isMotion_[2];
};

#endif // VISIONCLIENT_H
