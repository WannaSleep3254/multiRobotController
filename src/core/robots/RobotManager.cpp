#include "RobotManager.h"
#include "PickListModel.h"
#include "ModbusClient.h"
#include "Orchestrator.h"

#include <QTimer>
RobotManager::RobotManager(QObject* parent) : QObject(parent)
{

}

void RobotManager::addRobot(const QString& id, const QString& host, int port,
                            const QVariantMap& addr, QObject* owner) {
    RobotContext ctx;
    ctx.id    = id;
    ctx.model = new PickListModel(owner);
    ctx.bus   = new ModbusClient(owner);
    ctx.orch  = new Orchestrator(ctx.bus, ctx.model, owner);
    ctx.addr  = addr;
    ctx.orch->applyAddressMap(addr);

    qDebug()<<host<<port;
    m_ctx.insert(id, ctx);

    m_busToId.insert(ctx.bus, id);
    connect(ctx.bus, &ModbusClient::heartbeat,   this, &RobotManager::onBusHeartbeat,   Qt::UniqueConnection);
    connect(ctx.bus, &ModbusClient::connected,   this, &RobotManager::onBusConnected,   Qt::UniqueConnection);
    connect(ctx.bus, &ModbusClient::disconnected,this, &RobotManager::onBusDisconnected,Qt::UniqueConnection);

    connect(ctx.orch, &Orchestrator::stateChanged, this,
            [this, id](int state, const QString& name){
                emit stateChanged(id, state, name);
            });
    connect(ctx.orch, &Orchestrator::currentRowChanged, this,
            [this, id](int row){
                emit currentRowChanged(id, row);
            });

    connect(ctx.orch, SIGNAL(log(QString,Common::LogLevel)), this, SIGNAL(log(QString,Common::LogLevel)));
    connect(ctx.bus,  SIGNAL(log(QString,Common::LogLevel)), this, SIGNAL(log(QString,Common::LogLevel)));

    QTimer::singleShot(0, this, [this, id, host, port]{
        if (!m_ctx.contains(id) || !m_ctx[id].bus) return;
        m_ctx[id].bus->connectTo(host, port);
    });
}

void RobotManager::enqueuePose(const QString& id, const Pose6D &p) {
    if (m_ctx.contains(id))
        m_ctx[id].model->add(p);
}

void RobotManager::applyExtras(const QString& id, const QVariantMap& extras) {
    if (!m_ctx.contains(id)) return;
    auto& c = m_ctx[id];
    if (extras.contains("speed_pct")) {
        int addrSpeed = c.addr.value("holding").toMap().value("SPEED_PCT").toInt();
        if (addrSpeed >= 0) {
            quint16 v = quint16(extras.value("speed_pct").toInt());
            c.bus->writeHolding(addrSpeed, v);
        }
    }
}

void RobotManager::start(const QString& id) {
    if (m_ctx.contains(id)) m_ctx[id].orch->start();
}

void RobotManager::stop(const QString& id) {
    if (m_ctx.contains(id)) m_ctx[id].orch->stop();
}

void RobotManager::startAll() {
    for (auto& c : m_ctx) c.orch->start();
}

void RobotManager::stopAll() {
    for (auto& c : m_ctx) c.orch->stop();
}

QAbstractItemModel* RobotManager::model(const QString& id) const {
    return m_ctx.contains(id) ? m_ctx[id].model : nullptr;
}

void RobotManager::connectTo(const QString& id, const QString& host, int port)
{
    if(!m_ctx.contains(id))
        return;
    if(m_ctx[id].bus) {
        m_ctx[id].bus->connectTo(host, port);
    }
}

void RobotManager::disconnect(const QString& id)
{
    if(!m_ctx.contains(id))
        return;

    if(m_ctx[id].bus) {
        m_ctx[id].bus->disconnectFrom();
    }
}

void RobotManager::setRepeat(const QString&id, bool on)
{
    if(!m_ctx.contains(id))
        return;

    if(m_ctx[id].orch) {
        m_ctx[id].orch->setRepeat(on);
    }
}

bool RobotManager::hasRobot(const QString& id) const
{
    return m_ctx.contains(id);
}

void RobotManager::addOrConnect(const QString& id, const QString& host, int port,
                                const QVariantMap& addr, QObject* owner)
{
    if (!hasRobot(id)) {
        RobotContext ctx;
        ctx.id    = id;
        ctx.model = new PickListModel(owner);
        ctx.bus   = new ModbusClient(owner);
        ctx.orch  = new Orchestrator(ctx.bus, ctx.model, owner);
        ctx.addr  = addr;
        ctx.orch->applyAddressMap(addr);
        if (!ctx.orch->isAddressMapValid())
        {
            qWarning()<<"[RM] invalid addr_map"<<id; return;
        }
        m_ctx.insert(id, ctx);
    } else {
        // 주소맵 변경이 필요하면 여기서 재적용
        m_ctx[id].addr = addr;
        if (m_ctx[id].orch)
        {
            m_ctx[id].orch->applyAddressMap(addr);
        }
    }

    // 연결은 다음 틱에 (즉시 시그널로 인한 UAF/레이스 방지)
    QTimer::singleShot(0, this, [this, id, host, port]{
        if (!m_ctx.contains(id) || !m_ctx[id].bus) return;
        m_ctx[id].bus->connectTo(host, port);
    });
}

void RobotManager::reconnect(const QString& id, const QString& host, int port)
{
    if (!m_ctx.contains(id) || !m_ctx[id].bus) return;
    QTimer::singleShot(0, this, [this, id, host, port]{
        m_ctx[id].bus->connectTo(host, port);
    });
}

bool RobotManager::isConnected(const QString& id) const
{
    if (!m_ctx.contains(id) || !m_ctx[id].bus) return false;
    return m_ctx[id].bus->isConnected();  // 아래 ModbusClient 보강 참고
}

void RobotManager::onBusHeartbeat(bool ok)
{
    auto* bus = qobject_cast<QObject*>(sender());
    const QString id = m_busToId.value(bus);
    if (!id.isEmpty()) emit heartbeat(id, ok);
}

void RobotManager::onBusConnected()
{
    auto* bus = qobject_cast<QObject*>(sender());
    const QString id = m_busToId.value(bus);
    if (!id.isEmpty()) emit connectionChanged(id, true);
}

void RobotManager::onBusDisconnected()
{
    auto* bus = qobject_cast<QObject*>(sender());
    const QString id = m_busToId.value(bus);
    if (!id.isEmpty()) emit connectionChanged(id, false);
}
