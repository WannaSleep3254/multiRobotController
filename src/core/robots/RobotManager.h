#ifndef ROBOTMANAGER_H
#define ROBOTMANAGER_H

#include <QObject>
#include <QMap>
#include <QPointer>
#include <QVariantMap>

#include "Pose6D.h"
#include "LogLevel.h"

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
    PickListModel* pickModel(const QString& id) const;

    void connectTo(const QString& id, const QString& host, int port);
    void disconnect(const QString& id);
    void setRepeat(const QString&id, bool on);

    bool hasRobot(const QString& id) const;
    void addOrConnect(const QString& id, const QString& host, int port,
                      const QVariantMap& addr, QObject* owner); // 패널에서 호출
    // 선택: 이미 추가된 로봇의 호스트/포트만 바꿔 재연결
    void reconnect(const QString& id, const QString& host, int port);
    bool isConnected(const QString& id) const;

    // ★ 좌표 리스트 주입/관리
    void setPoseList(const QString& id, const QVector<Pose6D>& list);
    void clearPoseList(const QString& id);
    bool loadCsvToModel(const QString& id, const QString& filePath, QString* errMsg=nullptr, QObject* owner=nullptr);

signals:
    void heartbeat(const QString& id, bool ok);
    void connectionChanged(const QString& id, bool connected);
    void stateChanged(const QString& id, int state, const QString& name);
    void currentRowChanged(const QString& id, int row);

    void log(const QString& line,
             Common::LogLevel level = Common::LogLevel::Info);

    void logByRobot(const QString& id,
                    const QString& line,
                    Common::LogLevel level = Common::LogLevel::Info);

private slots:
    void onBusHeartbeat(bool ok);
    void onBusConnected();
    void onBusDisconnected();

private:
    QMap<QString, RobotContext> m_ctx; // "A","B","C" → 컨텍스트
    QHash<QObject*, QString> m_busToId; // ModbusClient* → id("A","B" 등)

    void hookSignals(const QString& id, ModbusClient* bus, Orchestrator* orch);
};

#endif // ROBOTMANAGER_H
