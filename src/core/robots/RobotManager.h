#ifndef ROBOTMANAGER_H
#define ROBOTMANAGER_H

#include "Orchestrator.h"
#include "Pose6D.h"
#include <QObject>
#include <QMap>
#include <QPointer>
#include <QVariantMap>
#include <Pose6D.h>

class QAbstractItemModel;
class PickListModel;
class ModbusClient;
class Orchestrator;

struct RobotContext {
    QString id;
    QPointer<PickListModel> model;
    QPointer<ModbusClient>  bus;
    QPointer<Orchestrator>  orch;
    QVariantMap addr; // AddressMap
};

class RobotManager : public QObject {
    Q_OBJECT
public:
    explicit RobotManager(QObject* parent=nullptr);

    // 로봇 컨텍스트 생성/등록
    void addRobot(const QString& id, const QString& host, int port,
                  const QVariantMap& addr, QObject* owner);

    // 비전에서 온 포즈를 해당 로봇 큐로 적재
    void enqueuePose(const QString& id, const Pose6D& p);

    // 옵션 파라미터 실시간 반영 (예: 속도)
    void applyExtras(const QString& id, const QVariantMap& extras);

    // 제어
    void start(const QString& id);
    void stop (const QString& id);
    void startAll();
    void stopAll();

    // UI 바인딩용
    QAbstractItemModel *model(const QString& id) const;

    void disconnect(const QString& id);
    void setRepeat(const QString&id, bool on);

signals:
    void heartbeat(bool ok);
//    void heartbeat(const QString& id, bool ok);
    void stateChanged(const QString& id, int state, const QString& name);
    void currentRowChanged(const QString& id, int row);

    void log(const QString& line, int level); // 0=Debug,1=Info,2=Warn,3=Error

private:
    QMap<QString, RobotContext> m_ctx; // "A","B","C" → 컨텍스트
};

#endif // ROBOTMANAGER_H
