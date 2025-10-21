#ifndef ORCHESTRATOR_H
#define ORCHESTRATOR_H

#include <QObject>
#include <QVariant>
#include <QElapsedTimer>

#include "LogLevel.h"

class ModbusClient;
class PickListModel;
class QTimer;

class Orchestrator : public QObject
{
    Q_OBJECT
public:
    explicit Orchestrator(ModbusClient* bus, PickListModel* model, QObject* parent=nullptr);
//    enum class LogLevel { Debug, Info, Warn, Error };

public slots:
    void start();
    void stop();

    void applyAddressMap(const QVariantMap& m);
    void publishPoseToRobot1(const QVector<double>& pose, int speedPct = 50);

signals:
//    void log(const QString& line, Orchestrator::LogLevel level = Orchestrator::LogLevel::Info);
    void log(const QString& line, Common::LogLevel level = Common::LogLevel::Info);
    void stateChanged(int state, QString name);
    void currentRowChanged(int row);
    void finishedCurrentCycle();

private slots:
    void cycle();

private:
    // FSM: Ready↑ → PublishTarget → WaitPickStart(BUSY↑)
    //    → WaitPickDone(BUSY↓ & DONE↑) → WaitDoneClear(DONE↓) → PublishTarget …
    enum class State {
        Idle,
        WaitRobotReady,
        PublishTarget,
        WaitPickStart,
        WaitPickDone,
        WaitDoneClear
    };
    Q_ENUM(State)
    State m_state{State::Idle};

    ModbusClient* m_bus{nullptr};
    PickListModel* m_model{nullptr};
    QTimer* m_timer{nullptr};
    int m_currentRow{-1};

    bool m_repeat{false};
    QElapsedTimer m_stateTick;

    // AddressMap.json 기반 주소
    int A_PUBLISH_REQ = 100;      // coils

    int A_ROBOT_READY = 100;      // discrete_inputs
    int A_PICK_DONE   = 101;      // discrete_inputs
    int A_ROBOT_BUSY  = 102;      // discrete_inputs

    int A_TARGET_BASE = 132;      // holding: TARGET_POSE_STAGING_BASE (132..143)

    // 선택 파라미터
    int A_SEQ_ID        = -1;
    int A_PAYLOAD_CKSUM = -1;
    int A_SPEED_PCT     = -1;
    int A_CMD_TIMEOUT_MS    = 2000;
    int A_READY_TIMEOUT_MS  = 2000;

    // 에지 검출용 래치
    bool m_lastReady{false};
    bool m_lastBusy{false};
    bool m_lastDone{false};
    quint16 m_seq{0};
private:
    // 상태 전용 헬퍼: 여기서만 상태를 바꾸고, 시그널/로그를 함께 처리
    void setState(State s);
    static QString stateName(State s);

public:
    void setRepeat(bool on) { m_repeat = on;}
    bool isAddressMapValid() const {
        return (A_ROBOT_READY >= 0 && A_ROBOT_BUSY >= 0 && A_PICK_DONE >= 0 &&
                A_PUBLISH_REQ >= 0 && A_TARGET_BASE >= 0);
    }
};

#endif // ORCHESTRATOR_H
