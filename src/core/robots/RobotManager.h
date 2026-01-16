#ifndef ROBOTMANAGER_H
#define ROBOTMANAGER_H

#include <QObject>
#include <QMap>
#include <QPointer>
#include <QVariantMap>

#include "Pose6D.h"
#include "LogLevel.h"
//#include "RobotCommand.h"
#include "RobotCommandQueue.h"

class QAbstractItemModel;
class PickListModel;
class ModbusClient;
class Orchestrator;
//class VisionServer; // for friend declaration
class VisionClient;

struct RobotContext {
    QString id;
    QPointer<PickListModel> model;
    QPointer<ModbusClient>  bus;
    QPointer<Orchestrator>  orch;
    QVariantMap addr_; // AddressMap

    QPointer<RobotCommandQueue> cmdq; // ✅ 추가
};

class RobotManager : public QObject {
    Q_OBJECT
public:
    explicit RobotManager(QObject* parent=nullptr);

    void setVisionClient(VisionClient* srv)  { m_vsrv = srv; }

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

    // ✅ 비전 모드 (true면 enqueue→즉시 전송→삭제)
    void setVisionMode(const QString& id, bool on);
    bool visionMode(const QString& id) const;

    // ✅ VisionServer → MainWindow 경유로 호출할 처리 API
//    void processVisionPose(const QString& id, const QString& kind, const Pose6D& p, const QVariantMap& extras);
    // 벌크 픽&플레이스 좌표 처리
//    void processVisionPoseBulk(const QString& id, const Pose6D& pick, const Pose6D& place, const QVariantMap& extras);
    // 코일 트리거
    void triggerByKey(const QString& id, const QString& coilKey, int pulseMs = 100);

    ////////////////////////////////////////////////////////////////////////////////////////
    /// 2025-11-21: VisionClient의 제어 API
    /// /////////////////////////////////////////////////////////////////////////////////////
    /* Tool */
    // 1. 소팅 툴 장착
    void cmdBulk_AttachTool();
    void cmdBulk_DettachTool();
    void cmdBulk_ChangeTool();

    /* Bulk */
    void cmdBulk_DoPickup(const Pose6D& pose, const int &mode);
    void cmdBulk_DoPlace(const Pose6D& pose, const int &mode);

    /* Sorting */
    // 1. 소팅 툴 장착
    void cmdSort_AttachTool();
    void cmdSort_DettachTool();
    void cmdSort_ChangeTool();
    // 2. 피킹 촬상위치로 이동
    void cmdSort_MoveToPickupReady();
    // 3. 피킹 동작 수행
    void cmdSort_DoPickup(const Pose6D &pose, bool flip, int offset, int thick);
    // 4. 컨베이어 이동
    void cmdSort_MoveToConveyor();
    // 5. 플레이스 수행
    void cmdSort_DoPlace(bool flip, int offset, int thick=0);
    // gentry 툴 토글
    void cmdSort_GentryTool(bool toggle);

    void cmdSort_Arrange(const Pose6D &origin, const Pose6D &dest);

    /* Aligin */
    // 6. 얼라인 초기화
    void cmdAlign_Initialize();
    // 7. ASSY 촬상위치로 이동
    void cmdAlign_MoveToAssyReady();
    // 8. 피킹 촬상위치로 이동
    void cmdAlign_MoveToPickupReady();
    // 9. 피킹 동작 수행
    void cmdAlign_DoPickup(const Pose6D& pose);
    // 10. 플레이스 동작 수행
    void cmdAlign_DoPlace(const Pose6D& pose, int clampSequenceMode =0 );
    // 11. 클램프 동작 수행
    void cmdAlign_Clamp(bool open);

    void cmdAlign_Scrap();

    // Interface
    void setAutoMode(const QString& id, bool on);
    void startMainProgram(const QString& id);
    void stopMainProgram(const QString& id);
    void pauseMainProgram(const QString& id);
    ////////////////////////////////////////////////////////////////////////////////////////
signals:
    void heartbeat(const QString& id, bool ok);
    void connectionChanged(const QString& id, bool connected);
    void stateChanged(const QString& id, int state, const QString& name);
    void currentRowChanged(const QString& id, int row);

//    void bulkProcessStarted(const QString& id);
//    void bulkProcessFinished(const QString& id);

    void log(const QString& line,
             Common::LogLevel level = Common::LogLevel::Info);

    void logByRobot(const QString& id,
                    const QString& line,
                    Common::LogLevel level = Common::LogLevel::Info);

    void sortProcessFinished(const QString& id);
    void reqGentryPalce();
    void reqGentryReady();

private slots:
    void onBusHeartbeat(bool ok);
    void onBusConnected();
    void onBusDisconnected();

private:
    QMap<QString, RobotContext> m_ctx; // "A","B","C" → 컨텍스트
    QHash<QObject*, QString> m_busToId; // ModbusClient* → id("A","B" 등)

    void hookSignals(const QString& id, ModbusClient* bus, Orchestrator* orch);

    QHash<QString, bool> m_visionMode;  // ✅ 로봇별 비전 모드
//    VisionServer* m_vsrv{nullptr};  // ✅ 보관용
    VisionClient* m_vsrv{nullptr};  // ✅ 보관용
    float m_yawOffset{0.0f}; // vision pose yaw offset

    QMap<QString, bool> m_workCompleteSent;

    int old_Run_[2]{0,0};
    bool old_Limit_[2]{false,false};
    bool old_Collision_[2]{false,false};
    int old_MainError_[2]{0,0};
    int old_SubError_[2]{0,0};
};

#endif // ROBOTMANAGER_H
