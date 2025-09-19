#ifndef SERVER_H
#define SERVER_H
#include <QTcpServer>
#include <QTcpSocket>
#include <QSet>
#include <QHash>

class Server : public QTcpServer
{
    Q_OBJECT
public:
    explicit Server(QObject* parent=nullptr);

    // listen 제어
    bool start(const QHostAddress& bindAddr, quint16 port);
    void stop();

    quint16 port() const { return serverPort(); }
    QSet<QTcpSocket*> clients() const { return m_clients; }

    // 송신 API
    int  broadcast(const QByteArray& bytes);
    bool writeTo(QTcpSocket* s, const QByteArray& bytes);

    // 새로 추가
    QString clientId(QTcpSocket* s) const;               // "ip:port" 문자열 반환
    bool writeToId(const QString& id, const QByteArray& bytes); // ID로 전송

signals:
    void log(const QString& line);
    void started(const QHostAddress& addr, quint16 port);
    void stopped();
    void peerCountChanged(int count);
    void lineReceived(QTcpSocket* from, const QByteArray& line);

private slots:
    void onNewConnection();
    void onReadyRead();
    void onDisconnected();
    void onErrorOccurred(QAbstractSocket::SocketError);

protected:
    // QTcpServer
    // void incomingConnection(qintptr) override; // 필요 시

private:
    static constexpr int kMaxLine = 1<<20; // 1MB 가드
    QSet<QTcpSocket*> m_clients;
    QHash<QTcpSocket*, QByteArray> m_inbuf; // 라인 프레이밍 버퍼
};

#endif // SERVER_H
