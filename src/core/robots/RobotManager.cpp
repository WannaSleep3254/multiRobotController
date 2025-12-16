#include "RobotManager.h"
#include "PickListModel.h"
#include "ModbusClient.h"
#include "Orchestrator.h"
#include "vision/VisionClient.h"

#include <QTimer>
#include <QFile>
#include <QTextStream>
#include <QVector>

namespace {
    using Step = RobotCommandQueue::Step;

    Step stepWriteCoil(QObject* owner, ModbusClient* bus, int addr, bool value)
    {
        return [=](std::function<void(bool)> done){
            MbOp op;
            op.kind = MbOp::Kind::WriteCoil;
            op.start = addr;
            op.coilValue = value;
            op.id = QUuid::createUuid();

            QMetaObject::Connection c;
            c = QObject::connect(bus, &ModbusClient::opFinished, owner,
                                 [=, &c](QUuid id, bool ok, const QString&){
                                     if (id != op.id) return;
                                     QObject::disconnect(c);
                                     done(ok);
                                 });

            bus->enqueue(op);
        };
    }

    Step stepDelay(QObject* owner, int ms)
    {
        return [=](std::function<void(bool)> done){
            QTimer::singleShot(ms, owner, [=]{ done(true); });
        };
    }

    // ON → (pulseMs 후) OFF
    Step stepPulseCoil(QObject* owner, ModbusClient* bus, int addr, int pulseMs)
    {
        return [=](std::function<void(bool)> done){
            QList<Step> seq;
            seq << stepWriteCoil(owner, bus, addr, true);
            seq << stepDelay(owner, pulseMs);
            seq << stepWriteCoil(owner, bus, addr, false);

            // seq를 즉석에서 실행
            auto* runner = new RobotCommandQueue(owner);
            runner->enqueue(seq);
            QObject::connect(runner, &QObject::destroyed, owner, []{});
            // 마지막 step이 끝나면 done을 호출하게 하려면 runner를 조금 확장하는 게 정석인데,
            // 간단히 pulse는 마지막 write 완료 시 done 되게 아래처럼 구현하는 방식이 더 깔끔함(아래 3번에서)
            done(true);
        };
    }

    Step stepWriteHoldingBlock(QObject* owner,
                               ModbusClient* bus,
                               int start,
                               const QVector<quint16>& vals)
    {
        return [=](std::function<void(bool)> done){
            MbOp op;
            op.kind = MbOp::Kind::WriteHoldingBlock;
            op.start = start;
            op.blockValues = vals;
            op.id = QUuid::createUuid();

            QMetaObject::Connection c;
            c = QObject::connect(bus, &ModbusClient::opFinished, owner,
                                 [=, &c](QUuid id, bool ok, const QString&){
                                     if (id != op.id) return;
                                     QObject::disconnect(c);
                                     done(ok);
                                 });

            bus->enqueue(op);
        };
    }
}

RobotManager::RobotManager(QObject* parent) : QObject(parent)
{

}

void RobotManager::enqueuePose(const QString& id, const Pose6D &p) {
    if (m_ctx.contains(id))
        m_ctx[id].model->add(p);
}

void RobotManager::applyExtras(const QString& id, const QVariantMap& extras) {
    if (!m_ctx.contains(id)) return;
    auto& c = m_ctx[id];
    if (extras.contains("speed_pct")) {
        int addrSpeed = c.addr_.value("holding").toMap().value("SPEED_PCT").toInt();
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
    if (!m_ctx.contains(id)) {
        RobotContext ctx;
        ctx.id    = id;
        ctx.model = new PickListModel(owner ? owner : this); // 모델은 항상 보유
        if (!ctx.cmdq) ctx.cmdq = new RobotCommandQueue(owner ? owner : this);
        m_ctx.insert(id, ctx);
    }
    auto& c = m_ctx[id];
    c.addr_ = addr;
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
    // Orchestrator 시그널
/*
    connect(orch, &Orchestrator::kinematicsUpdated, m_vsrv,
            [this](const QString& rid, const Orchestrator::RobotState& state){
//                m_vsrv->updateRobotState(rid, state.tcp, state.joints, state.tsMs);
    });
*/
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
        emit logByRobot(id, QString("[RM] processPulse from %1 idx=%2").arg(rid).arg(idx), Common::LogLevel::Info);
        if (rid == "A") {
            switch(idx)
            {
            case 0: // 프로그램 시작
                break;

            case 1: // 동작 수행
                qInfo()<<QDateTime::currentDateTime()<<": "<<"command Start";
                break;

            case 2: // none
                qInfo()<<QDateTime::currentDateTime()<<": "<<"command Complete";
                break;

            case 3: // 툴 교체 완료
            {
                QString key = QString("%1_tool").arg(rid);
                if (m_workCompleteSent.value(key, false)) {// 이미 전송된 상태 → 무시
                    return;
                }
                m_workCompleteSent[key] = true;
                QTimer::singleShot(10, this, [=]() {
                    m_vsrv->sendToolComplete(id, 0, true);
                });
                QTimer::singleShot(50, this, [=]() {
                    m_workCompleteSent[key] = false;
                });
                qInfo()<<QDateTime::currentDateTime()<<": "<<"Tool Complete";
            }   break;
            case 4: // 대기자세 완료
            {
                QString key = QString("%1_ready").arg(rid);
                if (m_workCompleteSent.value(key, false)) {// 이미 전송된 상태 → 무시
                    return;
                }
                m_workCompleteSent[key] = true;
                QTimer::singleShot(10, this, [=]() {
                    m_vsrv->sendWorkComplete(id, "sorting", "standby", 0);
                });
                QTimer::singleShot(50, this, [=]() {
                    m_workCompleteSent[key] = false;
                });
                qInfo()<<QDateTime::currentDateTime()<<": "<<"Standby Complete";
            }   break;
            case 5: // none
            {
                QString key = QString("%1_pick_nonFlip").arg(rid);
                qInfo()<<"RobotManager::processPulse PICK nonflip key="<<key;
                if (m_workCompleteSent.value(key, false)) {// 이미 전송된 상태 → 무시
                    return;
                }
                m_workCompleteSent[key] = true;
                QTimer::singleShot(10, this, [=]() {
                    m_vsrv->sendWorkComplete(id, "sorting", "pick", 0);
                });
                QTimer::singleShot(50, this, [=]() {
                    m_workCompleteSent[key] = false;
                    qDebug()<<"RobotManager::processPulse PICK complete key="<<key;
                });
                qInfo()<<QDateTime::currentDateTime()<<": "<<"Sort Pick_non-Flip Complete";

            }   break;
            case 6: // 소팅 PICK
            {
                QString key = QString("%1_pick").arg(rid);
                qDebug()<<"RobotManager::processPulse PICK key="<<key;
                if (m_workCompleteSent.value(key, false)) {// 이미 전송된 상태 → 무시
                    return;
                }
                m_workCompleteSent[key] = true;
                QTimer::singleShot(10, this, [=]() {
                    m_vsrv->sendWorkComplete(id, "sorting", "pick", 0);
                });
                QTimer::singleShot(50, this, [=]() {
                    m_workCompleteSent[key] = false;
                    qDebug()<<"RobotManager::processPulse PICK complete key="<<key;
                });
                qInfo()<<QDateTime::currentDateTime()<<": "<<"Sort Pick_Flip Complete";
            }   break;
            case 7: // 소팅 PLACE
            {
                QString key = QString("%1_place").arg(rid);
                if (m_workCompleteSent.value(key, false)) {// 이미 전송된 상태 → 무시
                    return;
                }
                m_workCompleteSent[key] = true;
                QTimer::singleShot(10, this, [=]() {
                    m_vsrv->sendWorkComplete(id, "sorting", "place", 0);
                });
                QTimer::singleShot(20, this, [=]() {
                    m_vsrv->sendWorkComplete(id, "sorting", "idle", 0);
                });
                QTimer::singleShot(50, this, [=]() {
                    m_workCompleteSent[key] = false;
                });
                qInfo()<<QDateTime::currentDateTime()<<": "<<"Sort Place Complete";
                emit sortProcessFinished(id);
            }   break;
            case 8: // 갠트리 툴
            {
//                m_vsrv->sendWorkComplete(id, "sorting", "place", 0);
//                emit sortProcessFinished(id);
//                emit reqGentryReady();
            }   break;
            case 9: // 벌크 픽
            {
                QString key = QString("%1_bulk_pick").arg(rid);
                if (m_workCompleteSent.value(key, false)) {// 이미 전송된 상태 → 무시
                    return;
                }
                m_workCompleteSent[key] = true;
                QTimer::singleShot(10, this, [=]() {
                    m_vsrv->sendWorkComplete(rid, "bulk", "pick", 0);
                });
                QTimer::singleShot(50, this, [=]() {
                    m_workCompleteSent[key] = false;
                });
                qInfo()<<QDateTime::currentDateTime()<<": "<<"Bulk Pick Complete";
            }   break;
            case 10:    // 벌크 플레이스
            {
                QString key = QString("%1_bulk_place").arg(rid);
                if (m_workCompleteSent.value(key, false)) {// 이미 전송된 상태 → 무시
                    return;
                }
                m_workCompleteSent[key] = true;
                QTimer::singleShot(10, this, [=]() {
                    m_vsrv->sendWorkComplete(id, "bulk", "place", 0);
                });
                QTimer::singleShot(50, this, [=]() {
                    m_workCompleteSent[key] = false;
                });
                qInfo()<<QDateTime::currentDateTime()<<": "<<"Bulk Place Complete";
            }   break;
            case 11:    // arrange
            {
                QString key = QString("%1_arrange").arg(rid);
                if (m_workCompleteSent.value(key, false)) {// 이미 전송된 상태 → 무시
                    return;
                }
                m_workCompleteSent[key] = true;
                QTimer::singleShot(10, this, [=]() {
                    m_vsrv->sendWorkComplete(id, "sorting", "arrange", 0);
                });
                QTimer::singleShot(50, this, [=]() {
                    m_workCompleteSent[key] = false;
                });
                qInfo()<<QDateTime::currentDateTime()<<": "<<"Arrange Complete";
            }   break;

            case 12:    // IDLE: 소팅 완료 후 대기 상태
            {
                QString key = QString("%1_idle").arg(rid);
                if (m_workCompleteSent.value(key, false)) {// 이미 전송된 상태 → 무시
                    return;
                }
                m_workCompleteSent[key] = true;
                QTimer::singleShot(10, this, [=]() {
                    m_vsrv->sendWorkComplete(id, "sorting", "idle", 0);
                });
                QTimer::singleShot(100, this, [=]() {
                    m_workCompleteSent[key] = false;
                });
                qInfo()<<QDateTime::currentDateTime()<<": "<<"IDLE  Complete";
            }   break;
            case 13:    // 툴 체인지 중 촬상가능위치: Bulk to Sorting
            {
                QString key = QString("%1_capture_1").arg(rid);
                if (m_workCompleteSent.value(key, false)) {// 이미 전송된 상태 → 무시
                    return;
                }
                m_workCompleteSent[key] = true;
                QTimer::singleShot(500, this, [=]() {
                    m_vsrv->sendFeedbackPose(id,"bulk","sorting",0);
                });
                QTimer::singleShot(100, this, [=]() {
                    m_workCompleteSent[key] = false;
                });
                qInfo()<<QDateTime::currentDateTime()<<": "<<"CAPTURE 1 Complete";
            }   break;
            case 14:    // 툴 체인지 중 촬상가능위치: Sorting to Bulk
            {
                QString key = QString("%1_capture_2").arg(rid);
                if (m_workCompleteSent.value(key, false)) {// 이미 전송된 상태 → 무시
                    return;
                }
                m_workCompleteSent[key] = true;
                QTimer::singleShot(500, this, [=]() {
                    m_vsrv->sendFeedbackPose(id,"sorting","bulk", 0);
                });
                QTimer::singleShot(100, this, [=]() {
                    m_workCompleteSent[key] = false;
                });
                qInfo()<<QDateTime::currentDateTime()<<": "<<"CAPTURE 2 Complete";
            }   break;
            }
        }
        else if (rid == "B") {
            switch(idx)
            {
            case 0:  // 프로그램 시작
            case 1:  // 동작
                break;
            case 2: // INIT
            {
                QString key = QString("%1_init").arg(rid);
                if (m_workCompleteSent.value(key, false)) {// 이미 전송된 상태 → 무시
                    return;
                }
                m_workCompleteSent[key] = true;
                QTimer::singleShot(10, this, [=]() {
                    m_vsrv->sendWorkComplete(id, "align", "init", 0);
                });
                QTimer::singleShot(100, this, [=]() {
                    m_workCompleteSent[key] = false;
                });
            }   break;
            case 3:  // none
                break;
            case 4: // READY
            {
                QString key = QString("%1_ready").arg(rid);
                if (m_workCompleteSent.value(key, false)) {// 이미 전송된 상태 → 무시
                    return;
                }
                m_workCompleteSent[key] = true;
                QTimer::singleShot(10, this, [=]() {
                    m_vsrv->sendWorkComplete(id, "align", "standby", 0);
                });
                QTimer::singleShot(100, this, [=]() {
                    m_workCompleteSent[key] = false;
                });
            }   break;
            case 5: //ASSY
            {
                QString key = QString("%1_assy").arg(rid);
                if (m_workCompleteSent.value(key, false)) {// 이미 전송된 상태 → 무시
                    return;
                }
                m_workCompleteSent[key] = true;
                QTimer::singleShot(10, this, [=]() {
                    m_vsrv->sendWorkComplete(id, "align", "assy", 0);
                });
                QTimer::singleShot(100, this, [=]() {
                    m_workCompleteSent[key] = false;
                });
            }   break;
            case 6: // PICK
            {
                QString key = QString("%1_pick").arg(rid);
                if (m_workCompleteSent.value(key, false)) {// 이미 전송된 상태 → 무시
                    return;
                }
                m_workCompleteSent[key] = true;
                QTimer::singleShot(10, this, [=]() {
                    m_vsrv->sendWorkComplete(id, "align", "pick", 0);
                });
                QTimer::singleShot(100, this, [=]() {
                    m_workCompleteSent[key] = false;
                });
            }   break;
            case 7: // PLACE
            {
                QString key = QString("%1_pick").arg(rid);
                if (m_workCompleteSent.value(key, false)) {// 이미 전송된 상태 → 무시
                    return;
                }
                m_workCompleteSent[key] = true;
                QTimer::singleShot(10, this, [=]() {
                    m_vsrv->sendWorkComplete(id, "align", "place", 0);
                });
                QTimer::singleShot(100, this, [=]() {
                    m_workCompleteSent[key] = false;
                });
            }   break;
            case 8: // CLAMP close
            {
                QString key = QString("%1_clamp_1").arg(rid);
                if (m_workCompleteSent.value(key, false)) {// 이미 전송된 상태 → 무시
                    return;
                }
                m_workCompleteSent[key] = true;
                QTimer::singleShot(10, this, [=]() {
                    m_vsrv->sendWorkComplete(id, "align", "clamp", 0, true);
                });
                QTimer::singleShot(100, this, [=]() {
                    m_workCompleteSent[key] = false;
                });
            }   break;
            case 9: // CLAMP open
            {
                QString key = QString("%1_clamp_2").arg(rid);
                if (m_workCompleteSent.value(key, false)) {// 이미 전송된 상태 → 무시
                    return;
                }
                m_workCompleteSent[key] = true;
                QTimer::singleShot(10, this, [=]() {
                    m_vsrv->sendWorkComplete(id, "align", "clamp", 0, false);
                });
                QTimer::singleShot(100, this, [=]() {
                    m_workCompleteSent[key] = false;
                });
            }   break;
            case 10:  // none
                break;
            case 11: // scrap
            {
                QString key = QString("%1_scrap").arg(rid);
                if (m_workCompleteSent.value(key, false)) {// 이미 전송된 상태 → 무시
                    return;
                }
                m_workCompleteSent[key] = true;
                QTimer::singleShot(10, this, [=]() {
                    m_vsrv->sendWorkComplete(id, "align", "scrap", 0, false);
                });
                QTimer::singleShot(100, this, [=]() {
                    m_workCompleteSent[key] = false;
                });
            }   break;
            case 12:  //
                break;
            }
        }
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

void RobotManager::triggerByKey(const QString& id, const QString& coilKey, int pulseMs)
{
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {

        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }

    const auto coils   = it.value().addr_.value("coils").toMap();
    int addr = coils.value(coilKey).toInt();
    if (addr <= 0) {
        emit log(QString("[RM] trigger: invalid key %1 for %2").arg(coilKey, id));
        return;
    }
/*
    auto* bus = it->bus.data();
    auto* owner = this;

    QList<RobotCommandQueue::Step> steps;
    steps << stepWriteCoil(owner, bus, addr, true);
    steps << stepDelay(owner, pulseMs);
    steps << stepWriteCoil(owner, bus, addr, false);
    it->cmdq->enqueue(steps);
*/

    QPointer<ModbusClient> busPtr = it->bus;
    busPtr->writeCoil(addr, true);
    QTimer::singleShot(pulseMs, this, [busPtr, addr]{
        if (!busPtr) return;
        busPtr->writeCoil(addr, false);
    });

    emit log(QString("[RM] trigger %1(%2) pulsed %3ms for %4").arg(coilKey).arg(addr).arg(pulseMs).arg(id));
}

////////////////////////////////////////////////////////////////////////////////////////
/// 2025-11-21: VisionClient의 제어 API
/// /////////////////////////////////////////////////////////////////////////////////////
/* Bulk */
void RobotManager::cmdBulk_AttachTool()
{
    QString id("A");
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {
        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }
    QVector<quint16> regs;// AI0:1, AI1:1
    regs.reserve(2);
    regs << 1 << 1;
    it->orch->publishToolComnad(regs);

    triggerByKey(id, "DI2", 500);
    emit logByRobot(id, QString("[RM] cmdBulk_AttachTool triggered for %1").arg(id), Common::LogLevel::Info);
}

void RobotManager::cmdBulk_DettachTool()
{
    QString id("A");
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {
        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }
    QVector<quint16> regs;// AI0:2, AI1:1
    regs.reserve(2);
    regs << 2 << 1;
    it->orch->publishToolComnad(regs);

    triggerByKey(id, "DI2", 500);
    emit logByRobot(id, QString("[RM] cmdBulk_DettachTool triggered for %1").arg(id), Common::LogLevel::Info);
}

void RobotManager::cmdBulk_ChangeTool()
{
    QString id("A");
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {
        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }
    QVector<quint16> regs;  // AI0:3, AI1:1
    regs.reserve(2);
    regs << 3 << 1;
    it->orch->publishToolComnad(regs);
    // Bulk to Sorting
    QTimer::singleShot(100, this, [this, id]() {
        triggerByKey(id, "DI2", 500);
    });

    emit logByRobot(id, QString("[RM] cmdBulk_ChangeTool triggered for %1").arg(id), Common::LogLevel::Info);
}

/* Bulk */
void RobotManager::cmdBulk_DoPickup(const Pose6D& pose, const int& mode)
{
    QString id("A");
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {
        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }
    qDebug()<<"[RM] cmdBulk_DoPickup pose:"<<pose.x<<pose.y<<pose.z<<pose.rx<<pose.ry<<pose.rz;
    const QVector<double> v{ pose.x,pose.y,pose.z,pose.rx,pose.ry,pose.rz };
    /////////////////////////////////////////////////////////////////////
    // ✔ 테스트 체크박스(비전 모드)가 있다면: 켜짐=즉시 발행, 꺼짐=큐 적재 (선택)
    if (visionMode(id)) {        // ← 이미 있는 함수면 그대로 사용
        it->orch->publishBulkMode(mode);
        it->orch->publishBulkPoseWithKind(v, "pick");
        triggerByKey(id, "DI8", 500);
        emit logByRobot(id, QString("[RM] cmdBulk_DoPickup triggered for %1").arg(id), Common::LogLevel::Info);
        if (it->model) it->model->add(pose); // 필요 시 큐에 쌓고 나중에 실행
    } else {
        if (it->model) it->model->add(pose); // 필요 시 큐에 쌓고 나중에 실행
    }
    /// ///////////////////////////////////////////////////////////////////
}

void RobotManager::cmdBulk_DoPlace(const Pose6D& pose, const int &mode)
{
    QString id("A");
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {
        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }
    qDebug()<<"[RM] cmdBulk_DoPlace pose:"<<pose.x<<pose.y<<pose.z<<pose.rx<<pose.ry<<pose.rz;
    const QVector<double> v{ pose.x,pose.y,pose.z,pose.rx,pose.ry,pose.rz };
    /////////////////////////////////////////////////////////////////////
    // ✔ 테스트 체크박스(비전 모드)가 있다면: 켜짐=즉시 발행, 꺼짐=큐 적재 (선택)
    if (visionMode(id)) {        // ← 이미 있는 함수면 그대로 사용
        it->orch->publishBulkMode(mode);
        it->orch->publishBulkPoseWithKind(v, "place");
        triggerByKey(id, "DI9", 500);
        emit logByRobot(id, QString("[RM] cmdBulk_DoPlace triggered for %1").arg(id), Common::LogLevel::Info);
        if (it->model) it->model->add(pose); // 필요 시 큐에 쌓고 나중에 실행
    } else {
        if (it->model) it->model->add(pose); // 필요 시 큐에 쌓고 나중에 실행
    }
    /// ///////////////////////////////////////////////////////////////////

}

/* Sorting */
// 1. 소팅 툴 장착
void RobotManager::cmdSort_AttachTool()
{
    QString id("A");
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {
        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }
    QVector<quint16> regs;  // AI0:1, AI1:2
    regs.reserve(2);
    regs << 1 << 2;
    it->orch->publishToolComnad(regs);

    triggerByKey(id, "DI2", 500);
    emit logByRobot(id, QString("[RM] cmdSort_AttachTool triggered for %1").arg(id), Common::LogLevel::Info);
}
void RobotManager::cmdSort_DettachTool()
{
    QString id("A");
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {
        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }
    QVector<quint16> regs;  // AI0:2, AI1:2
    regs.reserve(2);
    regs << 2 << 2;
    it->orch->publishToolComnad(regs);
    QTimer::singleShot(100, this, [this, id]() {
        triggerByKey(id, "DI2", 500);
    });

    emit logByRobot(id, QString("[RM] cmdSort_DettachTool triggered for %1").arg(id), Common::LogLevel::Info);
}

void RobotManager::cmdSort_ChangeTool()
{
    QString id("A");
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {
        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }
    QVector<quint16> regs;  // AI0:3, AI1:2
    regs.reserve(2);
    regs << 3 << 2;
    it->orch->publishToolComnad(regs);
    QTimer::singleShot(100, this, [this, id]() {
        triggerByKey(id, "DI2", 500);
    });

    emit logByRobot(id, QString("[RM] cmdSort_ChangeTool triggered for %1").arg(id), Common::LogLevel::Info);
}
// 2. 피킹 촬상위치로 이동
void RobotManager::cmdSort_MoveToPickupReady()
{
    QString id("A");
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {
        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }
    triggerByKey(id, "DI4", 500);
    emit logByRobot(id, QString("[RM] cmdSort_MoveToPickupReady triggered for %1").arg(id), Common::LogLevel::Info);
}
// 3. 피킹 동작 수행
void RobotManager::cmdSort_DoPickup(const Pose6D& pose, bool flip, int offset, int thick)
{
    QString id("A");
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {
        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }
    /////////////////////////////////////////////////////////////////////
    const QVector<double> v{ pose.x,pose.y,pose.z,pose.rx,pose.ry,pose.rz };

    if( 30 < v[5] && v[5]<= 90 )
    {
        m_yawOffset=+60;
    }
else if(90 <= v[5] && v[5] <150)
    {
        m_yawOffset=-60;
    }
    else
    {
        m_yawOffset=0;
    }
    /////////////////////////////////////////////////////////////////////
    // ✔ 테스트 체크박스(비전 모드)가 있다면: 켜짐=즉시 발행, 꺼짐=큐 적재 (선택)
    if (!visionMode(id)) {
        if (it->model) it->model->add(pose);
        return;
    }
/*
    // ✅ 1) 레지스터 생성 (통신 없음)
    const QVector<quint16> regs =
        Orchestrator::makeSortPickRegs(v, flip, offset, m_yawOffset, thick);
    // ✅ 2) 주소 resolve (addr_.json 기준)
    const auto holding = it->addr_.value("holding").toMap();
    const auto coils   = it->addr_.value("coils").toMap();

    const int basePick = holding.value("TARGET_POSE_PICK").toInt();
    const int coilPick = coils.value("DI5").toInt();   // 기존 PICK 트리거
    if (basePick <= 0 || coilPick <= 0) {
        emit log("[RM] cmdSort_DoPickup: invalid addr", Common::LogLevel::Error);
        return;
    }
    // ✅ 3) cmdq에 “write → pulse”를 하나의 시퀀스로
    QList<RobotCommandQueue::Step> steps;
    steps << stepWriteHoldingBlock(this, it->bus, basePick, regs);
    steps << stepWriteCoil(this, it->bus, coilPick, true);
    steps << stepDelay(this, 80);   // 80~150ms 권장
    steps << stepWriteCoil(this, it->bus, coilPick, false);

    it->cmdq->enqueue(steps);

    emit logByRobot(id,
                    QString("[RM] cmdSort_DoPickup queued (base=%1, coil=%2)")
                        .arg(basePick).arg(coilPick),
                    Common::LogLevel::Info);
*/
    triggerByKey(id, "DI5", 500);
    it->orch->publishSortPick(v,flip, offset, m_yawOffset, thick);

    emit logByRobot(id, QString("[RM] cmdSort_DoPickup triggered for %1").arg(id), Common::LogLevel::Info);
    if (it->model) it->model->add(pose); // 필요 시 큐에 쌓고 나중에 실행

    /// ///////////////////////////////////////////////////////////////////
}
// 4. 컨베이어 이동
void RobotManager::cmdSort_MoveToConveyor()
{
    QString id("A");
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {
        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }
    emit logByRobot(id, QString("[RM] cmdSort_MoveToConveyor triggered for %1").arg(id), Common::LogLevel::Info);
}
// 5. 플레이스 수행
void RobotManager::cmdSort_DoPlace(bool flip, int offset, int thick)
{
    QString id("A");
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {
        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }
    if (flip) {
        // flip 처리
    } else {
        // non-flip 처리
        it->orch->publishFlip_Offset(false, offset, m_yawOffset, thick);
    }
    m_yawOffset = 0;
    triggerByKey(id, "DI6", 500);
    emit logByRobot(id, QString("[RM] cmdSort_DoPlace triggered for %1").arg(id), Common::LogLevel::Info);
}

void RobotManager::cmdSort_GentryTool(bool toggle)
{
    QString id("A");
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {
        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }

    QPointer<ModbusClient> busPtr = it->bus;
    if (!busPtr) return;

    QTimer::singleShot(10, this, [busPtr]{
        busPtr->writeCoil(303, false);  // Tool true
    });

    QTimer::singleShot(100, this, [busPtr]{
        busPtr->writeCoil(303, true);   // Tool false
    });

    QTimer::singleShot(1000, this, [busPtr]{
        busPtr->writeCoil(303, false);  // Tool true
    });
}

void RobotManager::cmdSort_Arrange(const Pose6D &origin, const Pose6D &dest)
{
    QString id("A");
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {
        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }

    qDebug()<<"[RM] cmdSort_Arrange origin:"<<origin.x<<origin.y<<origin.z<<origin.rx<<origin.ry<<origin.rz;
    qDebug()<<"[RM] cmdSort_Arrange dest:"<<dest.x<<dest.y<<dest.z<<dest.rx<<dest.ry<<dest.rz;


    const QVector<double> v_1{ origin.x,origin.y,origin.z,origin.rx,origin.ry,origin.rz };
    const QVector<double> v_2{ dest.x,dest.y,dest.z,dest.rx,dest.ry,dest.rz };
    /////////////////////////////////////////////////////////////////////
    // ✔ 테스트 체크박스(비전 모드)가 있다면: 켜짐=즉시 발행, 꺼짐=큐 적재 (선택)
    if (visionMode(id)) {        // ← 이미 있는 함수면 그대로 사용
        triggerByKey(id, "DI11", 500);
        it->orch->publishArrangePoses(v_1,v_2);

        if (it->model) it->model->add(origin); // 필요 시 큐에 쌓고 나중에 실행
        if (it->model) it->model->add(dest); // 필요 시 큐에 쌓고 나중에 실행
    } else {
        if (it->model) it->model->add(origin); // 필요 시 큐에 쌓고 나중에 실행
        if (it->model) it->model->add(dest); // 필요 시 큐에 쌓고 나중에 실행
    }
    /// ///////////////////////////////////////////////////////////////////

}

/* Aligin */
// 6. 얼라인 초기화
void RobotManager::cmdAlign_Initialize()
{
    QString id("B");
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {
        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }
    triggerByKey(id, "DI3", 500);
    emit logByRobot(id, QString("[RM] cmdAlign_Initialize triggered for %1").arg(id), Common::LogLevel::Info);
}
// 7. ASSY 촬상위치로 이동
void RobotManager::cmdAlign_MoveToAssyReady()
{
    QString id("B");
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {
        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }
    triggerByKey(id, "DI5", 500);
    emit logByRobot(id, QString("[RM] cmdAlign_MoveToAssyReady triggered for %1").arg(id), Common::LogLevel::Info);
}
// 8. 피킹 촬상위치로 이동
void RobotManager::cmdAlign_MoveToPickupReady()
{
    QString id("B");
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {
        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }
    triggerByKey(id, "DI4", 500);
    emit logByRobot(id, QString("[RM] cmdAlign_MoveToPickupReady triggered for %1").arg(id), Common::LogLevel::Info);
}
// 9. 피킹 동작 수행
void RobotManager::cmdAlign_DoPickup(const Pose6D& pose)
{
    QString id("B");
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {
        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }
    /////////////////////////////////////////////////////////////////////
    const QVector<double> v{ pose.x,pose.y,pose.z,pose.rx,pose.ry,pose.rz };
    // ✔ 테스트 체크박스(비전 모드)가 있다면: 켜짐=즉시 발행, 꺼짐=큐 적재 (선택)
    if (visionMode(id)) {        // ← 이미 있는 함수면 그대로 사용
//        it->orch->publishPoseWithKind(v, 50, "pick");
        triggerByKey(id, "DI6", 500);
        it->orch->publishAlignPick(v);
        if (it->model) it->model->add(pose); // 필요 시 큐에 쌓고 나중에 실행

        emit logByRobot(id, QString("[RM] cmdAlign_DoPickup triggered for %1").arg(id), Common::LogLevel::Info);
    } else {
        if (it->model) it->model->add(pose); // 필요 시 큐에 쌓고 나중에 실행
    }
    /// ///////////////////////////////////////////////////////////////////
}
// 10. 플레이스 동작 수행
void RobotManager::cmdAlign_DoPlace(const Pose6D& pose)
{
    QString id("B");
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {
        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }
    /////////////////////////////////////////////////////////////////////
    const QVector<double> v{ pose.x,pose.y,pose.z,pose.rx,pose.ry,pose.rz };
    // ✔ 테스트 체크박스(비전 모드)가 있다면: 켜짐=즉시 발행, 꺼짐=큐 적재 (선택)
    if (visionMode(id)) {        // ← 이미 있는 함수면 그대로 사용
//        it->orch->publishPoseWithKind(v, 50, "place");
        triggerByKey(id, "DI7", 500);
        it->orch->publishAlignPlace(v);
        if (it->model) it->model->add(pose); // 필요 시 큐에 쌓고 나중에 실행

        emit logByRobot(id, QString("[RM] cmdAlign_DoPlace triggered for %1").arg(id), Common::LogLevel::Info);
    } else {
        if (it->model) it->model->add(pose); // 필요 시 큐에 쌓고 나중에 실행
    }
    /// ///////////////////////////////////////////////////////////////////


}
// 11. 클램프 동작 수행
void RobotManager::cmdAlign_Clamp(bool open)
{
    QString id("B");
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {
        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }
    it->bus->writeCoil(110, open); // 110번 코일을 여닫기);
    triggerByKey(id, "DI8", 500);
    emit logByRobot(id, QString("[RM] Clamp %1 command sent").arg(open?"Open":"Close"), Common::LogLevel::Info);
}

void RobotManager::cmdAlign_Scrap()
{
    QString id("B");
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {
        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }
    triggerByKey(id, "DI9", 500);
    emit logByRobot(id, QString("[RM] Screap command sent"), Common::LogLevel::Info);
}

void RobotManager::setAutoMode(const QString& id, bool on)
{
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {
        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }
    QPointer<ModbusClient> busPtr = it->bus;
    busPtr->writeCoil(505, true);
    QTimer::singleShot(500, this, [busPtr]{
        if (!busPtr) return;
            busPtr->writeCoil(505, false);
    });
}

void RobotManager::startMainProgram(const QString& id)
{
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {
        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }
    QPointer<ModbusClient> busPtr = it->bus;
    busPtr->writeCoil(506, false);
    busPtr->writeCoil(506, true);
    QTimer::singleShot(500, this, [busPtr]{
        if (!busPtr) return;
        busPtr->writeCoil(506, false);
    });
}

void RobotManager::stopMainProgram(const QString& id)
{
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {
        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }
    QPointer<ModbusClient> busPtr = it->bus;
    busPtr->writeCoil(503, false);
    busPtr->writeCoil(503, true);
    QTimer::singleShot(500, this, [busPtr]{
        if (!busPtr) return;
        busPtr->writeCoil(503, false);
    });
}

void RobotManager::pauseMainProgram(const QString& id)
{
    /*
    auto it = m_ctx.find(id);
    if (it == m_ctx.end() || !it->bus) {
        emit log(QString("[RM] trigger: no bus for %1").arg(id));
        return;
    }
    QPointer<ModbusClient> busPtr = it->bus;
    busPtr->writeCoil(505, true);
    QTimer::singleShot(500, this, [busPtr]{
        if (!busPtr) return;
        busPtr->writeCoil(505, false);
    });
    */
}
