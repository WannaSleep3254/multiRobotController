#include "RobotManager.h"
#include "PickListModel.h"
#include "ModbusClient.h"
#include "Orchestrator.h"

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
    ctx.bus->connectTo(host, port);
    m_ctx.insert(id, ctx);

    connect(ctx.bus, &ModbusClient::heartbeat, this, &RobotManager::heartbeat);
    connect(ctx.orch, &Orchestrator::stateChanged, this,
            [this, id](int state, const QString& name){
                emit stateChanged(id, state, name);
            });
    connect(ctx.orch, &Orchestrator::currentRowChanged, this,
            [this, id](int row){
                emit currentRowChanged(id, row);
            });
/*
    connect(ctx.orch, SIGNAL(log(QString,Orchestrator::LogLevel)),
            this, SIGNAL(log(QString,Orchestrator::LogLevel)));

    connect(ctx.bus, &ModbusClient::log, this, [this](const QString& line){
        emit log(line, 1); // Info);
    });

    connect(ctx.bus, &ModbusClient::log2, this, [this](const QString& line, int level){
        emit log(line, level);
    });
*/
    connect(ctx.orch, SIGNAL(log(QString,Common::LogLevel)), this, SIGNAL(log(QString,Common::LogLevel)));
    connect(ctx.bus,  SIGNAL(log(QString,Common::LogLevel)), this, SIGNAL(log(QString,Common::LogLevel)));
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
