#include "Server.h"
#include <QNetworkInterface>
#include <QDebug>

Server::Server(QObject* parent) : QTcpServer(parent)
{
    connect(this, &QTcpServer::newConnection, this, &Server::onNewConnection);
}

bool Server::start(const QHostAddress& bindAddr, quint16 port)
{
    if (isListening()) stop();
    if (!listen(bindAddr, port)) {
        emit log(QString("[ERR] listen failed: %1").arg(errorString()));
        return false;
    }

    emit started(serverAddress(), serverPort());
    emit log(QString("[OK] listening on %1:%2").arg(serverAddress().toString()).arg(serverPort()));
    return true;
}

void Server::stop()
{
    for (auto s : std::as_const(m_clients)) {
        if (!s) continue;
        s->disconnect(this);
        s->close();
        s->deleteLater();
    }
    m_clients.clear();
    m_inbuf.clear();

    if (isListening()) close();
    emit stopped();
}

void Server::onNewConnection()
{
    while (hasPendingConnections()) {
        QTcpSocket* s = nextPendingConnection();
        m_clients.insert(s);
        emit peerCountChanged(m_clients.size());

        connect(s, &QTcpSocket::readyRead,      this, &Server::onReadyRead);
        connect(s, &QTcpSocket::disconnected,   this, &Server::onDisconnected);
        connect(s, &QTcpSocket::errorOccurred,  this, &Server::onErrorOccurred);

        emit log(QString("[NET] +conn %1:%2 (count=%3)")
                     .arg(s->peerAddress().toString()).arg(s->peerPort()).arg(m_clients.size()));

        emit clientConnected(s);  // ★ 추가: 여기서 즉시 알림
    }
}

void Server::onDisconnected()
{
    auto* s = qobject_cast<QTcpSocket*>(sender());
    if (!s) return;
    m_clients.remove(s);
    m_inbuf.remove(s);
    s->deleteLater();
    emit peerCountChanged(m_clients.size());
    emit log(QString("[NET] -conn %1:%2 (count=%3)")
                 .arg(s->peerAddress().toString()).arg(s->peerPort()).arg(m_clients.size()));
}

void Server::onErrorOccurred(QAbstractSocket::SocketError)
{
    auto* s = qobject_cast<QTcpSocket*>(sender());
    if (!s) return;
    emit log(QString("[NET] error %1:%2 -> %3")
                 .arg(s->peerAddress().toString()).arg(s->peerPort()).arg(s->errorString()));
}

void Server::onReadyRead()
{
    auto* s = qobject_cast<QTcpSocket*>(sender());
    if (!s) return;

    QByteArray& buf = m_inbuf[s];
    buf.append(s->readAll());

    // 라인 프레이밍
    while (true) {
        int idx = buf.indexOf('\n');
        if (idx < 0) break;
        QByteArray line = buf.left(idx);
        buf.remove(0, idx + 1);

        if (line.size() > 0) emit lineReceived(s, line);
    }

    // DoS 가드
    if (buf.size() > kMaxLine) {
        emit log(QString("[WARN] line too long from %1:%2, closing")
                     .arg(s->peerAddress().toString()).arg(s->peerPort()));
        s->disconnect(this);
        s->close();
    }
}

int Server::broadcast(const QByteArray& bytes)
{
    int sent = 0;
    for (auto* s : std::as_const(m_clients)) {
        if (s && s->state() == QAbstractSocket::ConnectedState) {
            qint64 w = s->write(bytes);
            if (w >= 0) ++sent;
        }
    }
    return sent;
}

bool Server::writeTo(QTcpSocket* s, const QByteArray& bytes)
{
    if (!s || s->state() != QAbstractSocket::ConnectedState) return false;
    return s->write(bytes) >= 0;
}

QString Server::clientId(QTcpSocket* s) const
{
    if (!s) return {};
    return QString("%1:%2").arg(s->peerAddress().toString()).arg(s->peerPort());
}

bool Server::writeToId(const QString& id, const QByteArray& bytes)
{
    for (QTcpSocket* s : std::as_const(m_clients)) {
        if (!s) continue;
        const QString cid = clientId(s);
        if (cid == id && s->state() == QAbstractSocket::ConnectedState) {
            return s->write(bytes) >= 0;
        }
    }
    return false; // 해당 ID 클라이언트 없음
}
