#ifndef ORCHESTRATOR_H
#define ORCHESTRATOR_H

#include <QObject>
#include <QVariant>
#include <QElapsedTimer>
#include <QDateTime>

#include "LogLevel.h"
#include "Pose6D.h"

class ModbusClient;
class PickListModel;
class QTimer;

class Orchestrator : public QObject
{
    Q_OBJECT
public:
    explicit Orchestrator(ModbusClient* bus, PickListModel* model, QObject* parent=nullptr);
//    enum class LogLevel { Debug, Info, Warn, Error };
    // 로봇 상태 구조체 (Orchestrator 내부에 선언)
    struct RobotState {
        Pose6D tcp;            // X,Y,Z,Rx,Ry,Rz
        Pose6D joints;         // J1..J6 (deg)
        qint64 tsMs = 0;       // timestamp
        bool   hasTcp = false;
        bool   hasJoints = false;
    };

signals:
    // 로봇상태를 비전서버에 전달
    void kinematicsUpdated(const QString& robotId, const Orchestrator::RobotState& st);

public slots:
    void setRobotId(const QString& id) { m_robotId = id; }

private:
    // 멤버 캐시 (Orchestrator 클래스 내부)
    RobotState m_kinState;  //kinematics state
    QString m_robotId;

public slots:
    void start();
    void stop();

    void applyAddressMap(const QVariantMap& m);

    void publishPoseWithKind(const QVector<double>& pose, int speedPct, const QString& kind);
    void publishPoseToRobot1(const QVector<double>& pose, int speedPct = 50);

signals:
//    void log(const QString& line, Orchestrator::LogLevel level = Orchestrator::LogLevel::Info);
    void log(const QString& line, Common::LogLevel level = Common::LogLevel::Info);
    void stateChanged(int state, QString name);
    void currentRowChanged(int row);
    void finishedCurrentCycle();
    void processPulse(const QString& robotId, int idx); // idx: 0→DO3, 1→DO4, 2→DO5

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
    int A_PUBLISH_PICK  {100};      // coils
    int A_PUBLISH_PLACE {101};      // coils
    int A_DI2           {102};      // coils
    int A_DI3           {103};      // coils
    int A_DI4           {104};      // coils

    int A_ROBOT_READY   {100};      // discrete_inputs
    int A_PICK_DONE     {101};      // discrete_inputs
    int A_ROBOT_BUSY    {102};      // discrete_inputs
    int A_DO3_PULSE     {103};
    int A_DO4_PULSE     {104};
    int A_DO5_PULSE     {105};

    int A_TARGET_BASE   {132};      // holding: TARGET_POSE_STAGING_BASE (132..143)
    int A_TARGET_BASE_PICK   {132}; // holding: TARGET_POSE_STAGING_BASE (132..143)
    int A_TARGET_BASE_PLACE  {144}; // holding: TARGET_POSE_STAGING_BASE (144..155)

    int IR_JOINT_BASE   {340};        // input_registers: JOINT_BASE (340..351)
    int IR_TCP_BASE     {388};        // input_registers: TCP_BASE (388..399)
    int IR_WORD_PER_POSE   {12};         // 6개 실수값 × 2워드

    QString IR_WORD_ORDER {"HI_LO"}; // "HI_LO" 또는 "LO_HI"
/*
    // 선택 파라미터
    int A_SEQ_ID        = -1;
    int A_PAYLOAD_CKSUM = -1;
    int A_SPEED_PCT     = -1;
    int A_CMD_TIMEOUT_MS    = 2000;
    int A_READY_TIMEOUT_MS  = 2000;
*/
    // 에지 검출용 래치
    bool m_lastReady{false};
    bool m_lastBusy{false};
    bool m_lastDone{false};

    bool m_lastDI2{false}, m_lastDI3{false}, m_lastDI4{false};
    bool m_lastDO3{false}, m_lastDO4{false}, m_lastDO5{false};

    quint16 m_seq{0};
private:
    // 상태 전용 헬퍼: 여기서만 상태를 바꾸고, 시그널/로그를 함께 처리
    void setState(State s);
    static QString stateName(State s);

public:
    void setRepeat(bool on) { m_repeat = on;}
    bool isAddressMapValid() const {
        return (A_ROBOT_READY >= 0 && A_ROBOT_BUSY >= 0 && A_PICK_DONE >= 0 &&
                A_PUBLISH_PICK >= 0 && A_TARGET_BASE >= 0);
    }
};

#endif // ORCHESTRATOR_H
