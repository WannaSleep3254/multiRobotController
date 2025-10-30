#include "RobotManager.h"
#include "PickListModel.h"
#include "ModbusClient.h"
#include "Orchestrator.h"

#include "vision/VisionServer.h"

#include <QTimer>
#include <QFile>
#include <QTextStream>
#include <QVector>

RobotManager::RobotManager(QObject* parent) : QObject(parent)
{

}
#if false // legacy - 미사용
void RobotManager::addRobot(const QString& id, const QString& host, int port,
                            const QVariantMap& addr, QObject* owner)
{
#if false
    RobotContext ctx;
    ctx.id    = id;
    ctx.model = new PickListModel(owner);
    ctx.bus   = new ModbusClient(owner);
    ctx.orch  = new Orchestrator(ctx.bus, ctx.model, owner);
    ctx.addr  = addr;
    ctx.orch->applyAddressMap(addr);

    m_ctx.insert(id, ctx);

    hookSignals(id, ctx.bus, ctx.orch);
#else
    // 기존 컨텍스트가 있으면 정리
    if (m_ctx.contains(id))
    {
        auto c = m_ctx.value(id);
        if (c.orch)  { c.orch->stop(); c.orch->deleteLater(); }
        if (c.bus)   { c.bus->deleteLater(); }
        if (c.model) { c.model->deleteLater(); }
        m_ctx.remove(id);
    }

    RobotContext ctx;
    ctx.id    = id;
    ctx.model = new PickListModel(owner);                // ★ 새 모델
    ctx.bus   = new ModbusClient(owner);
    ctx.orch  = new Orchestrator(ctx.bus, ctx.model, owner);
    ctx.addr  = addr;
    ctx.orch->applyAddressMap(addr);
    ctx.orch->setRobotId(id);

    m_ctx.insert(id, ctx);
    hookSignals(id, ctx.bus, ctx.orch);
#endif
    QTimer::singleShot(0, this, [this, id, host, port]{
        if (!m_ctx.contains(id) || !m_ctx[id].bus) return;
        m_ctx[id].bus->connectTo(host, port);
    });
}
#endif
void RobotManager::enqueuePose(const QString& id, const Pose6D &p) {
    if (m_ctx.contains(id))
        m_ctx[id].model->add(p);
}

void RobotManager::publishPoseNow(const QString& id, const Pose6D& p, int speedPct)
{
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->orch) {
        emit log(QString("[RM] publishPoseNow failed: no orch for %1").arg(id),
                 Common::LogLevel::Warn);
        return;
    }
    const QVector<double> pose{ p.x, p.y, p.z, p.rx, p.ry, p.rz };
    it->orch->publishPoseToRobot1(pose, speedPct); // 기존 함수 재사용
    emit log(QString("[RM] publishPoseNow(%1) sent (speed=%2)")
                 .arg(id).arg(speedPct),
             Common::LogLevel::Info);
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

PickListModel* RobotManager::pickModel(const QString& id) const {
    return m_ctx.contains(id) ? m_ctx.value(id).model : nullptr;
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
#if false
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
        hookSignals(id, ctx.bus, ctx.orch);
    } else {
        // 주소맵 변경이 필요하면 여기서 재적용
        m_ctx[id].addr = addr;
        if (m_ctx[id].orch)
        {
            m_ctx[id].orch->applyAddressMap(addr);
        }
    }
#else
    if (!m_ctx.contains(id)) {
        RobotContext ctx;
        ctx.id    = id;
        ctx.model = new PickListModel(owner ? owner : this); // 모델은 항상 보유
        m_ctx.insert(id, ctx);
    }
    auto& c = m_ctx[id];
    c.addr = addr;
    if (!c.bus)  c.bus  = new ModbusClient(owner ? owner : this);
    if (!c.orch) c.orch = new Orchestrator(c.bus, c.model, owner ? owner : this);
    c.orch->applyAddressMap(addr);

    if (!c.orch->isAddressMapValid()) {
        qWarning() << "[RM] invalid addr_map" << id;
        return;
    }

    c.orch->setRobotId(id);
    // 시그널은 한 번만
    static QSet<Orchestrator*> hooked;
    if (!hooked.contains(c.orch)) {
        hookSignals(id, c.bus, c.orch);
        hooked.insert(c.orch);
    }
#endif
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

void RobotManager::setPoseList(const QString& id, const QVector<Pose6D>& list)
{
    if (!m_ctx.contains(id) || !m_ctx[id].model) return;
    m_ctx[id].model->setAll(list);
}

void RobotManager::clearPoseList(const QString& id)
{
    if (!m_ctx.contains(id) || !m_ctx[id].model) return;
    m_ctx[id].model->clear();
}

static bool parseLineToPose(const QString& line, Pose6D& out)
{
    QString s = line.trimmed();
    if (s.isEmpty() || s.startsWith('#')) return false;
    // 콤마/탭 모두 허용
    const QChar sep = s.contains('\t') ? '\t' : ',';
    const auto parts = s.split(sep, Qt::SkipEmptyParts);
    if (parts.size() < 6) return false;
    bool ok[6]{};
    double vals[6]{};
    for (int i=0;i<6;i++) {
        vals[i] = parts[i].trimmed().toDouble(&ok[i]);
        if (!ok[i]) return false;
    }
    out = Pose6D{vals[0],vals[1],vals[2],vals[3],vals[4],vals[5]};
    return true;
}

bool RobotManager::loadCsvToModel(const QString& id, const QString& filePath, QString* errMsg, QObject* owner)
{
    // 컨텍스트가 없으면 '모델만' 갖춘 컨텍스트를 먼저 만든다
    if (!m_ctx.contains(id)) {
        RobotContext ctx;
        ctx.id    = id;
        ctx.model = new PickListModel(owner ? owner : this);
        m_ctx.insert(id, ctx);
    } else if (!m_ctx[id].model) {
        m_ctx[id].model = new PickListModel(owner ? owner : this);
    }

    QFile f(filePath);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (errMsg) *errMsg = QString("open failed: %1").arg(filePath);
        return false;
    }
    QTextStream ts(&f);
    QVector<Pose6D> list;
    int lineNo = 0, okCnt = 0;
    while (!ts.atEnd()) {
        const QString line = ts.readLine();
        ++lineNo;
        Pose6D p;
        if (parseLineToPose(line, p)) {
            list.push_back(p);
            ++okCnt;
        } else {
            // 첫 줄이 헤더일 수도 있으니 조용히 스킵
        }
    }
    if (okCnt == 0) {
        if (errMsg) *errMsg = "no valid rows";
        return false;
    }
    m_ctx[id].model->setAll(list);
    emit log(QString("[OK] Loaded %1 poses into %2").arg(okCnt).arg(id));
    return true;
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
    if (!id.isEmpty())
        emit connectionChanged(id, true);
}

void RobotManager::onBusDisconnected()
{
    auto* bus = qobject_cast<QObject*>(sender());
    const QString id = m_busToId.value(bus);
    if (!id.isEmpty())
        emit connectionChanged(id, false);
}

void RobotManager::hookSignals(const QString& id, ModbusClient* bus, Orchestrator* orch)
{
    if (!bus || !orch) return;
    // 역참조용 맵에 등록
    m_busToId.insert(bus, id);
    // ModbusClient 시그널
    connect(bus, &ModbusClient::heartbeat,   this, &RobotManager::onBusHeartbeat);
    connect(bus, &ModbusClient::connected,   this, &RobotManager::onBusConnected);
    connect(bus, &ModbusClient::disconnected,this, &RobotManager::onBusDisconnected);
    connect(bus, &ModbusClient::log, this,[this, id](const QString& line, Common::LogLevel lv) {
        emit logByRobot(id, line, lv);   // ★ 패널용
        emit log(QString("%1 (%2)").arg(line,id), lv);              // ★ 전체용
    });
/*
    connect(bus, &ModbusClient::holdingRead, this, [this, id](int start, QVector<quint16> data){
        qDebug()<<"[RM] holdingRead from"<<id<<"start="<<start<<"data="<<data;
        // --- ⬇ float 변환 구간 추가 ---
        QVector<float> floats;
        for (int i = 0; i + 1 < data.size(); i += 2) {
            quint32 combined = (static_cast<quint32>(data[i]) << 16) | data[i+1];
            float value;
            std::memcpy(&value, &combined, sizeof(float));
            floats.push_back(value);
        }

        qDebug() << "[MC] readInputs result start=" << start
                 //<< "words=" << data
                 << "floats=" << floats;
    });
*/
/*
    connect(bus, &ModbusClient::inputRead, this, [this, id](int start, QVector<quint16> data){
        // --- float 변환 구간 추가 ---
        QVector<float> floats;
        for (int i = 0; i + 1 < data.size(); i += 2) {
            quint32 combined = (static_cast<quint32>(data[i]) << 16) | data[i+1];
            float value;
            std::memcpy(&value, &combined, sizeof(float));
            floats.push_back(value);
        }
        if(start==340) // Joint
        {
        }
        else if(start==388) // Tcp
        {
        }
    });
*/
    // Orchestrator 시그널
    connect(orch, &Orchestrator::kinematicsUpdated, m_vsrv,
            [this](const QString& rid, const Orchestrator::RobotState& state){
                m_vsrv->updateRobotState(rid, state.tcp, state.joints, state.tsMs);
    });

    connect(orch, &Orchestrator::stateChanged, this,
            [this, id](int state, const QString& name){
                emit stateChanged(id, state, name);
    });
    connect(orch, &Orchestrator::currentRowChanged, this,
            [this, id](int row){
                emit currentRowChanged(id, row);
//
    });    
    connect(orch, &Orchestrator::log, this, [this, id](const QString& line, Common::LogLevel lv) {
        emit logByRobot(id, line, lv);   // ★ 패널용
        emit log(QString("%1 (%2)").arg(line,id), lv);              // ★ 전체용
    });

    connect(orch, &Orchestrator::processPulse, this, [this, id](const QString& rid, int idx){
        if (!m_vsrv) return;

        qDebug()<<"[RM] processPulse from"<<rid<<"idx="<<idx;

        static quint32 seq = 1;
        if      (idx==0)    m_vsrv->requestCapture(seq++, "A");
        else if (idx==1)    m_vsrv->requestCapture(seq++, "B");
        else if (idx==2)    m_vsrv->requestCapture(seq++, "C");
    });
}

// 2025-10-21
void RobotManager::setVisionMode(const QString& id, bool on) {
    m_visionMode[id] = on;
    emit log(QString("[RM] VisionMode(%1)=%2").arg(id).arg(on), Common::LogLevel::Info);
}

bool RobotManager::visionMode(const QString& id) const {
    return m_visionMode.value(id, false);
}

void RobotManager::processVisionPose(const QString& id, const QString &kind, const Pose6D& p, const QVariantMap& extras)
{
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->orch) {
        emit log(QString("[RM] no orchestrator for %1").arg(id)); return;
    }
    const int speed = extras.value("speed_pct", 50).toInt();
    const QVector<double> v{ p.x,p.y,p.z,p.rx,p.ry,p.rz };

    // ✔ 테스트 체크박스(비전 모드)가 있다면: 켜짐=즉시 발행, 꺼짐=큐 적재 (선택)
    if (visionMode(id)) {        // ← 이미 있는 함수면 그대로 사용
        it->orch->publishPoseWithKind(v, speed, kind);
        if (it->model) it->model->add(p); // 필요 시 큐에 쌓고 나중에 실행
    } else {
        if (it->model) it->model->add(p); // 필요 시 큐에 쌓고 나중에 실행
    }

    /*
    if (!m_ctx.contains(id)) return;

    auto& c = m_ctx[id];
    const int speed = extras.value("speed_pct", 50).toInt();
    applyExtras(id, extras); // SPEED_PCT 등 등록

    if (visionMode(id)) {
        // ✅ 테스트(비전) 모드: enqueue → 즉시 전송 → 곧바로 삭제
        if (c.model) {
            c.model->add(p);
            if (c.orch) {
                QVector<double> pose{ p.x, p.y, p.z, p.rx, p.ry, p.rz };
                c.orch->publishPoseToRobot1(pose, speed);
            }
//            if (last >= 0) c.model->removeRow(last);
        }
        qDebug()<<"[RM] Vision mode: sent pose for"<<id
               <<"("<<p.x<<p.y<<p.z<<p.rx<<p.ry<<p.rz<<")"
              <<"speed_pct="<<speed;
    } else {
        // ✅ CSV 모드(기존): 큐에 쌓아서 FSM이 순서대로 처리
        qDebug()<<"[RM] Enqueue vision pose for"<<id
               <<"("<<p.x<<p.y<<p.z<<p.rx<<p.ry<<p.rz<<")"
              <<"speed_pct="<<speed;
        if (c.model) c.model->add(p);
    }
    */
}


    void RobotManager::triggerByKey(const QString& id, const QString& coilKey, int pulseMs)
    {
        auto it = m_ctx.find(id);
        if (it == m_ctx.end() || !it->bus) {
            emit log(QString("[RM] trigger: no bus for %1").arg(id));
            return;
        }

        const auto coils   = it.value().addr.value("coils").toMap();
       int addr = coils.value(coilKey).toInt();

        if (addr <= 0) {
            emit log(QString("[RM] trigger: invalid key %1 for %2").arg(coilKey, id));
            return;
        }

        // true → (pulseMs 후) false 로 펄스
        it->bus->writeCoil(addr, true);
        QTimer::singleShot(pulseMs, this, [bus=it->bus, addr]{
            bus->writeCoil(addr, false);
        });

        emit log(QString("[RM] trigger %1(%2) pulsed %3ms").arg(coilKey).arg(addr).arg(pulseMs));
    }

void RobotManager::triggerProcessA(const QString& id, int pulseMs)
{   // A_DI2
    triggerByKey(id, "DI2", pulseMs);
}

void RobotManager::triggerProcessB(const QString& id, int pulseMs)
{   // A_DI3
    triggerByKey(id, "DI3", pulseMs);
}

void RobotManager::triggerProcessC(const QString& id, int pulseMs)
{   // A_DI4
    triggerByKey(id, "DI4", pulseMs);
}
