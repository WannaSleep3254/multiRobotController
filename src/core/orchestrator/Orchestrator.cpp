#include "Orchestrator.h"
#include "ModbusClient.h"
#include "PickListModel.h"

#include <QTimer>
#include <QDebug>
#include <cstring>   // for memcpy

#include "tf/EulerAngleConverter.h"

#define DEG2RAD(deg) ((deg) * M_PI / 180.0)  // 도 → 라디안 변환
#define RAD2DEG(rad) ((rad) * 180.0 / M_PI)  // 라디안 → 도 변환

struct EulerZYX {
    double roll;
    double pitch;
    double yaw; // 최종 YAW 값
};

/////////////////////////////
// ✅ 각도를 -180° ~ 180° 범위로 정규화
double normalizeAngle(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
}
// ✅ EulerZYX 값을 정규화 (Pitch 범위 조정 포함)
EulerZYX normalizeEulerZYX(const EulerZYX& euler) {
    double roll = normalizeAngle(euler.roll);
    double pitch = euler.pitch;
    double yaw = normalizeAngle(euler.yaw);

    // ✅ Pitch 범위 조정 (-90° ~ 90° 유지)
    if (pitch > 90.0) {
        pitch = 180.0 - pitch;
        roll += 180.0;
        yaw += 180.0;
    } else if (pitch < -90.0) {
        pitch = -180.0 - pitch;
        roll += 180.0;
        yaw += 180.0;
    }

    // Roll과 Yaw도 다시 정규화
    roll = normalizeAngle(roll);
    yaw = normalizeAngle(yaw);

    return { roll, pitch, yaw };
}
/////////////////////////////

EulerZYX toolRotate(float rx, float ry, float rz, float target_yaw)
{
    // 입력 오일러 각 (단위: degree)
    double roll_degrees = static_cast<double>(rx);
    double pitch_degrees = static_cast<double>(ry);
    double yaw_degrees = static_cast<double>(rz);
    qDebug()<<"Input Euler Angles (ZYX) in degrees:"
             <<roll_degrees
             <<pitch_degrees
             <<yaw_degrees;
    // Degree to Radian 변환
    double roll_radians = EulerAngleConverter::ConvertDegreesToRadians(roll_degrees);
    double pitch_radians = EulerAngleConverter::ConvertDegreesToRadians(pitch_degrees);
    double yaw_radians = EulerAngleConverter::ConvertDegreesToRadians(yaw_degrees);

    Matrix3d tool_rotation_matrix = EulerAngleConverter::ConvertEulerZYXToRotationMatrix(
        roll_radians, pitch_radians, yaw_radians);

    // 추가할 YAW 회전
    double additional_yaw_degrees = target_yaw;//rz - target_yaw;
    double additional_yaw_radians = EulerAngleConverter::ConvertDegreesToRadians(additional_yaw_degrees);

    // 새로운 회전 행렬 계산
    Matrix3d updated_tool_rotation_matrix = EulerAngleConverter::ApplyYawRotationToToolFrame(
        tool_rotation_matrix, additional_yaw_radians);

    // 새로운 오일러 각(ZYX) 변환
    Vector3d updated_euler_radians = EulerAngleConverter::ConvertRotationMatrixToEulerZYX(updated_tool_rotation_matrix);

    // 정규화 적용
    double normalized_yaw = EulerAngleConverter::NormalizeAngleRadians(updated_euler_radians[0]);
    double normalized_pitch = EulerAngleConverter::NormalizeAngleRadians(updated_euler_radians[1]);
    double normalized_roll = EulerAngleConverter::NormalizeAngleRadians(updated_euler_radians[2]);

    // Radian to Degree 변환
    double normalized_yaw_degrees = EulerAngleConverter::ConvertRadiansToDegrees(normalized_yaw);
    double normalized_pitch_degrees = EulerAngleConverter::ConvertRadiansToDegrees(normalized_pitch);
    double normalized_roll_degrees = EulerAngleConverter::ConvertRadiansToDegrees(normalized_roll);

    qDebug() << "Updated Euler Angles (ZYX) in degrees:"
             << normalized_yaw_degrees
             << normalized_pitch_degrees
             << normalized_roll_degrees;

    EulerZYX normalized = normalizeEulerZYX({normalized_roll_degrees, normalized_pitch_degrees, normalized_yaw_degrees});
    qDebug() << "Normalized Euler Angles (ZYX) in degrees:"
             << normalized.roll
             << normalized.pitch
             << normalized.yaw;

    return normalized;
}
//////////////////////////////
// ---- 유틸: float -> 2word 변환
static void floatToRegs(float value, quint16 &hi, quint16 &lo) {
    quint32 raw;
    memcpy(&raw, &value, sizeof(raw));
    hi = static_cast<quint16>(raw >> 16);
    lo = static_cast<quint16>(raw & 0xFFFF);
}
// ---- 유틸: 2word(HI,LO) -> float
static inline float regsToFloat(quint16 hi, quint16 lo) {
    quint32 u = (quint32(hi) << 16) | quint32(lo);
    float f;
    std::memcpy(&f, &u, sizeof(float));
    return f;
}

Orchestrator::Orchestrator(ModbusClient* bus, PickListModel* model, QObject* parent)
    : QObject(parent), m_bus(bus), m_model(model), m_timer(new QTimer(this))
{
    m_timer->setInterval(10);
    connect(m_timer, &QTimer::timeout, this, &Orchestrator::cycle);

    // READY/DONE/BUSY 읽기 및 에지 처리
    connect(m_bus, &ModbusClient::discreteInputsRead, this, [this](int start, QVector<bool> data){
        if (data.isEmpty()) return;

        auto get = [&](int addr, bool& out){
            int idx = addr - start;
            if (idx >= 0 && idx < data.size())
                out = data[idx];
        };

        bool ready = m_lastReady, busy = m_lastBusy, done = m_lastDone;
        get(A_ROBOT_READY, ready);
        get(A_ROBOT_BUSY,  busy);
        get(A_PICK_DONE,   done);

        // DO3/DO4/DO5 펄스 감지
        bool do3=m_lastDO3, do4=m_lastDO4,   do5=m_lastDO5;
        bool do6=m_lastDO6, do7=m_lastDO7,   do8=m_lastDO8;
        bool do9=m_lastDO9, do10=m_lastDO10, do11=m_lastDO11;
        bool do12=m_lastDO12;

        get(A_DO3_PULSE , do3);
        get(A_DO4_PULSE , do4);
        get(A_DO5_PULSE , do5);
        get(A_DO6_PULSE , do6);
        get(A_DO7_PULSE , do7);
        get(A_DO8_PULSE , do8);
        get(A_DO9_PULSE , do9);
        get(A_DO10_PULSE , do10);
        get(A_DO11_PULSE , do11);
        get(A_DO12_PULSE , do12);

#if true
        // === 상승에지 → 공정 인덱스 배출 ===
        if (!m_lastDO3 && do3) {
            emit processPulse(m_robotId, 3);//, 0);
        }
        if (!m_lastDO4 && do4) {
            emit processPulse(m_robotId, 4);//, 1);
        }
        if (!m_lastDO5 && do5) {
            emit processPulse(m_robotId, 5);//, 2);
        }
        if (!m_lastDO6 && do6) {
            emit processPulse(m_robotId, 6);//, 3);
        }
        if (!m_lastDO7 && do7) {
            emit processPulse(m_robotId, 7);//, 4);
        }
        if (!m_lastDO8 && do8) {
            emit processPulse(m_robotId, 8);//, 5);
        }
        if (!m_lastDO9 && do9) {
            emit processPulse(m_robotId, 9);//, 6);
        }
        if (!m_lastDO10 && do10) {
            emit processPulse(m_robotId, 10);//, 7);
        }
        if (!m_lastDO11 && do11) {
            emit processPulse(m_robotId, 11);//, 8);
        }
        if (!m_lastDO12 && do12) {
            emit processPulse(m_robotId, 12);//, 9);
        }
#else
        // === 하강에지 → 공정 인덱스 배출 ===
        if (m_lastDO3 && !do3) {
            emit processPulse(m_robotId, 0);
        }
        if (m_lastDO4 && !do4) {
            emit processPulse(m_robotId, 1);
        }
        if (m_lastDO5 && !do5) {
            emit processPulse(m_robotId, 2);
        }
        if (m_lastDO6 && !do6) {
            emit processPulse(m_robotId, 3);
        }
        if (m_lastDO7 && !do7) {
            emit processPulse(m_robotId, 4);
        }
        if (m_lastDO8 && !do8) {
            emit processPulse(m_robotId, 5);
        }
        if (m_lastDO9 && !do9) {
            emit processPulse(m_robotId, 6);
        }
        if (m_lastDO10 && !do10) {
            emit processPulse(m_robotId, 7);
        }
        if (m_lastDO11 && !do11) {
            emit processPulse(m_robotId, 8);
        }
        if (m_lastDO12 && !do12) {
            emit processPulse(m_robotId, 9);
        }
#endif
        m_lastDO3 = do3;    m_lastDO4 = do4;    m_lastDO5 = do5;
        m_lastDO6 = do6;    m_lastDO7 = do7;    m_lastDO8 = do8;
        m_lastDO9 = do9;    m_lastDO10 = do10;  m_lastDO11 = do11;
        m_lastDO12 = do12;
    });

    connect(m_bus, &ModbusClient::inputRead, this, [this](int start, const QVector<quint16>& data){
        // 디버그용: 입력 레지스터 읽기 결과 처리
        if (data.isEmpty()) return;

        if(start==310 && data.size()>=13) {
            RobotStateFeedback st;

            st.enabled          = (data[0] == 1);
            st.mode             = data[1];
            st.runningState     = data[2];
            st.toolNumber       = data[3];
            st.workpieceNumber  = data[4];
            st.emergencyStop    = (data[5] == 1);
            st.softLimitExceeded= (data[6] == 1);
            st.mainError        = data[7];
            st.subError         = data[8];
            st.collision        = (data[9] == 1);
            st.motionArrive     = (data[10] == 1);
            st.safetyStopSI0    = (data[11] == 1);
            st.safetyStopSI1    = (data[12] == 1);
/*
            qDebug()<<"[RobotStateFeedback]"
                    <<"enabled:"<<st.enabled
                    <<"mode:"<<st.mode
                    <<"runningState:"<<st.runningState
                    <<"toolNumber:"<<st.toolNumber
                    <<"workpieceNumber:"<<st.workpieceNumber
                    <<"emergencyStop:"<<st.emergencyStop
                    <<"softLimitExceeded:"<<st.softLimitExceeded
                    <<"mainError:"<<st.mainError;
*/
        }
        // 예: 주소 340부터 12개 레지스터 읽기
        else if (start == IR_JOINT_BASE && data.size() >= IR_WORD_PER_POSE) {
            Pose6D joint;
            joint.x  = regsToFloat(data[0],  data[1]);
            joint.y  = regsToFloat(data[2],  data[3]);
            joint.z  = regsToFloat(data[4],  data[5]);
            joint.rx = regsToFloat(data[6],  data[7]);
            joint.ry = regsToFloat(data[8],  data[9]);
            joint.rz = regsToFloat(data[10], data[11]);
            m_kinState.joints = joint;
            m_kinState.hasJoints = true;
        }
        // 예: 주소 388부터 12개 레지스터 읽기
        else if (start == IR_TCP_BASE && data.size() >= IR_WORD_PER_POSE) {
            Pose6D tcp;
            tcp.x  = regsToFloat(data[0],  data[1]);
            tcp.y  = regsToFloat(data[2],  data[3]);
            tcp.z  = regsToFloat(data[4],  data[5]);
            tcp.rx = regsToFloat(data[6],  data[7]);
            tcp.ry = regsToFloat(data[8],  data[9]);
            tcp.rz = regsToFloat(data[10], data[11]);
            m_kinState.tcp = tcp;
            m_kinState.hasTcp = true;
        }

        if (m_kinState.hasTcp && m_kinState.hasJoints) {
            m_kinState.tsMs = QDateTime::currentMSecsSinceEpoch();
            emit kinematicsUpdated(m_robotId, m_kinState);
            // 계속 최신값 유지 (플래그 유지)
        }
    });
}

void Orchestrator::start()
{
    if (m_state != State::Idle)
        return;

    m_currentRow = -1;
//    m_state = State::WaitRobotReady;
    emit log("[RUN] Orchestrator started", Common::LogLevel::Info);
    setState(State::WaitRobotReady);
    m_timer->start();
    m_stateTick.restart();
}

void Orchestrator::stop()
{
    m_timer->stop();
    m_state = State::Idle;

    m_bus->writeCoil(A_PUBLISH_PICK, false);
    m_currentRow = -1;
    if(m_model)
        m_model->setActiveRow(-1);

    m_lastReady = false;
    m_lastBusy  = false;
    m_lastDone  = false;
    m_seq = 0;
    emit log("[RUN] Orchestrator stopped", Common::LogLevel::Info);
}

void Orchestrator::applyAddressMap(const QVariantMap& m)
{
    // 주소 맵 적용
    const auto coils   = m.value("coils").toMap();
    const auto di      = m.value("discrete_inputs").toMap();
    const auto holding = m.value("holding").toMap();
    const auto ir      = m.value("input_registers").toMap();

    auto getAddr = [](const QVariantMap& map, const QString& key, int& out){
        if (map.contains(key)) {
            out = map.value(key).toInt();
        }
    };

    getAddr(di, "ROBOT_READY", A_ROBOT_READY);
    getAddr(di, "ROBOT_BUSY", A_ROBOT_BUSY);
    getAddr(di, "PICK_DONE", A_PICK_DONE);
    getAddr(di, "DO3_PULSE", A_DO3_PULSE);
    getAddr(di, "DO4_PULSE", A_DO4_PULSE);
    getAddr(di, "DO5_PULSE", A_DO5_PULSE);
    getAddr(di, "DO6_PULSE", A_DO6_PULSE);
    getAddr(di, "DO7_PULSE", A_DO7_PULSE);
    getAddr(di, "DO8_PULSE", A_DO8_PULSE);
    getAddr(di, "DO9_PULSE", A_DO9_PULSE);
    getAddr(di, "DO10_PULSE", A_DO10_PULSE);
    getAddr(di, "DO11_PULSE", A_DO11_PULSE);

    getAddr(coils, "PUBLISH_PICK", A_PUBLISH_PICK);
    getAddr(coils, "PUBLISH_PLACE", A_PUBLISH_PLACE);
    getAddr(coils, "DI2",  A_DI2);
    getAddr(coils, "DI3",  A_DI3);
    getAddr(coils, "DI4",  A_DI4);
    getAddr(coils, "DI5",  A_DI5);
    getAddr(coils, "DI6",  A_DI6);
    getAddr(coils, "DI7",  A_DI7);
    getAddr(coils, "DI8",  A_DI8);
    getAddr(coils, "DI9",  A_DI9);
    getAddr(coils, "DI10", A_DI10);
    getAddr(coils, "DI11", A_DI11);

    getAddr(holding, "TARGET_POSE_BASE", A_TARGET_BASE);
    getAddr(holding, "TARGET_POSE_PICK", A_TARGET_BASE_PICK);
    getAddr(holding, "TARGET_POSE_PLACE", A_TARGET_BASE_PLACE);

    getAddr(ir, "CUR_JOINT_BASE", IR_JOINT_BASE);
    getAddr(ir, "CUR_TCP_BASE", IR_TCP_BASE);

    emit log(QString("[ADDR] READY=%1 BUSY=%2 DONE=%3 COIL_PUBLISH=%4 TARGET_BASE=%5")
                 .arg(A_ROBOT_READY).arg(A_ROBOT_BUSY).arg(A_PICK_DONE).arg(A_PUBLISH_PICK).arg(A_TARGET_BASE)
             , Common::LogLevel::Info);
}

void Orchestrator::cycle()
{
    // 상태 레지스터 주기적 읽기
//    m_bus->readInputs(310, 13);
#if false
    // 조인트 값 읽기
    if(IR_JOINT_BASE > 0)
        m_bus->readInputs(IR_JOINT_BASE, IR_WORD_PER_POSE);
    // TCP 값 읽기
    if(IR_TCP_BASE > 0)
        m_bus->readInputs(IR_TCP_BASE, IR_WORD_PER_POSE);
#endif
    m_bus->readDiscreteInputs(A_ROBOT_READY, 13);
}

QString Orchestrator::stateName(Orchestrator::State s)
{
    switch (s) {
    case State::Idle:            return "Idle";
    case State::WaitRobotReady:  return "WaitRobotReady (READY↑)";
    case State::PublishTarget:   return "PublishTarget";
    case State::WaitPickStart:   return "WaitPickStart (BUSY↑)";
    case State::WaitPickDone:    return "WaitPickDone (BUSY↓ & DONE↑)";
    case State::WaitDoneClear:   return "WaitDoneClear (DONE↓)";
    }
    return "Unknown";
}

void Orchestrator::setState(Orchestrator::State s)
{
    if (m_state == s) return;

    const auto prev = stateName(m_state);
    const auto next = stateName(s);
    m_state = s;

    emit log(QString("[FSM] %1 → %2").arg(prev, next), Common::LogLevel::Info);
    emit stateChanged(static_cast<int>(s), next);
}

void Orchestrator::publishPickPlacePoses(const QVector<double>& pick, const QVector<double>& place, int speedPct)
{
    qDebug() << "[ORCH] publishPickPlacePoses:"
             << "pick=" <<pick
             << "place=" <<place
             << "speedPct=" <<speedPct;

    int pick_base = A_TARGET_BASE_PICK;
    int place_base = A_TARGET_BASE_PLACE;

    // 포즈를 12워드(6float)로 인코딩해서 쓰기
    QVector<quint16> pick_regs;
    pick_regs.reserve(12);
    for (int i=0;i<6;i++) {
        quint16 hi, lo;
        floatToRegs(float(pick[i]), hi, lo);
        pick_regs << hi << lo;
    }
    m_bus->writeHoldingBlock(pick_base, pick_regs);
    QTimer::singleShot(50, this, [this]{
        m_bus->writeCoil(A_PUBLISH_PICK, true);
    });
    QTimer::singleShot(200, this, [this]{
        m_bus->writeCoil(A_PUBLISH_PICK, false);
    });

    QVector<quint16> place_regs;
    place_regs.reserve(12);
    for (int i=0;i<6;i++) {
        quint16 hi, lo;
        floatToRegs(float(place[i]), hi, lo);
        place_regs << hi << lo;
    }
    m_bus->writeHoldingBlock(place_base, place_regs);
    QTimer::singleShot(50, this, [this]{
        m_bus->writeCoil(A_PUBLISH_PLACE, true);
    });
    QTimer::singleShot(200, this, [this]{
        m_bus->writeCoil(A_PUBLISH_PLACE, false);
    });
}

void Orchestrator::publishToolComnad(const QVector<quint16>& cmds)
{
    m_bus->writeHoldingBlock(100, cmds);
}
// KJW 2025-11-24: 포즈 발행 함수 개선:
void Orchestrator::publishSortPick(const QVector<double>& pose, bool flip, int offset, float yaw, int thick)
{
    int base = A_TARGET_BASE_PICK;

    if( 60 < pose[5] && pose[5]<= 90 )
    {
        yaw = +30;
        m_yawOffset=+30;
    }
    else if(90 <= pose[5] && pose[5] <120)
    {
        yaw = -30;
        m_yawOffset=-30;
    }

    EulerZYX rpy = toolRotate(pose[3], pose[4], pose[5], yaw);
    QVector<double> rotate_pose{pose};
    rotate_pose[3] = rpy.roll;
    rotate_pose[4] = rpy.pitch;
    rotate_pose[5] = rpy.yaw;


    QVector<quint16> regs;
    regs.reserve(20);
    for (int i=0;i<6;i++) {
        quint16 hi, lo;
        floatToRegs(float(rotate_pose[i]), hi, lo);
        regs << hi << lo;
    }

    float val[4];
    val[0] = flip ? 1.0f : 0.0f;
    val[1] = static_cast<double>(offset);
    val[2] = yaw;
    val[3] = static_cast<double>(thick);

    for (int i=0;i<4;i++) {
        quint16 hi, lo;
        floatToRegs(val[i], hi, lo);
        regs << hi << lo;
    }

    m_bus->writeHoldingBlock(base, regs);

    QTimer::singleShot(10, this, [this]{
        m_bus->writeCoil(A_PUBLISH_PICK, true);
    });
    QTimer::singleShot(500, this, [this]{
        m_bus->writeCoil(A_PUBLISH_PICK, false);
    });
}

void Orchestrator::publishAlignPick(const QVector<double>& pose)
{
    int base = A_TARGET_BASE_PICK;

    EulerZYX rpy = toolRotate(pose[3], pose[4], pose[5], 0);
    QVector<double> rotate_pose{pose};
    rotate_pose[3] = rpy.roll;
    rotate_pose[4] = rpy.pitch;
    rotate_pose[5] = rpy.yaw;

    QVector<quint16> regs; regs.reserve(12);
    for (int i=0;i<6;i++) {
        quint16 hi, lo;
        floatToRegs(float(rotate_pose[i]), hi, lo);
        regs << hi << lo;
    }
    m_bus->writeHoldingBlock(base, regs);

    QTimer::singleShot(10, this, [this]{
        m_bus->writeCoil(A_PUBLISH_PICK, true);
    });
    QTimer::singleShot(500, this, [this]{
        m_bus->writeCoil(A_PUBLISH_PICK, false);
    });
}

void Orchestrator::publishAlignPlace(const QVector<double>& pose)
{
    int base = A_TARGET_BASE_PLACE;

    EulerZYX rpy = toolRotate(pose[3], pose[4], pose[5], 0);
    QVector<double> rotate_pose{pose};
    rotate_pose[3] = rpy.roll;
    rotate_pose[4] = rpy.pitch;
    rotate_pose[5] = rpy.yaw;

    QVector<quint16> regs; regs.reserve(12);
    for (int i=0;i<6;i++) {
        quint16 hi, lo;
        floatToRegs(float(rotate_pose[i]), hi, lo);
        regs << hi << lo;
    }
    m_bus->writeHoldingBlock(base, regs);

    QTimer::singleShot(10, this, [this]{
        m_bus->writeCoil(A_PUBLISH_PLACE, true);
    });
    QTimer::singleShot(500, this, [this]{
        m_bus->writeCoil(A_PUBLISH_PLACE, false);
    });
}

void Orchestrator::publishPoseWithKind(const QVector<double>& pose, int speedPct, const QString& kind)
{
    qDebug() <<"[ORCH] publishPoseWithKind:"
             <<"pose="<<pose
             <<"speedPct="<<speedPct
             <<"kind="<<kind;

    int base = A_TARGET_BASE;   // 기본 TARGET_BASE
    float yaw = 0;              // 기본 YAW 오프셋
    if (!kind.compare("place", Qt::CaseInsensitive) && A_TARGET_BASE_PLACE >= 0)
    {   // kind가 "place"이고 A_TARGET_BASE_PLACE >= 0이면 해당 주소 사용
        base = A_TARGET_BASE_PLACE;
        qDebug()<<"[ORCH] Using TARGET_BASE_PLACE ="<<base;
        yaw = 0;
    }
    else if (!kind.compare("pick", Qt::CaseInsensitive) && A_TARGET_BASE_PICK >= 0)
    {   // kind가 "pick"이고 A_TARGET_BASE_PICK >= 0이면 해당 주소 사용
        base = A_TARGET_BASE_PICK;
        qDebug()<<"[ORCH] Using TARGET_BASE_PICK ="<<base;
        yaw = 0;
        // KJW 2025-12-02: 로봇 ID "A"의 픽 포즈의 YAW 제한값 적용
        // 툴의 기본자세는 (180,0,-90), rz가 90도 근처일때 +30/-30 오프셋 적용.
#if true
        if(m_robotId == "A")
        {
            if( 60 < pose[5] && pose[5]<= 90 )
            {
                yaw = +30;
                m_yawOffset=+30;
            }
            else if(90 <= pose[5] && pose[5] <120)
            {
                yaw = -30;
                m_yawOffset=-30;
            }
        }
#endif
    }
    else
    {   // 그 외에는 기본 A_TARGET_BASE 사용
        qDebug()<<"[ORCH] Using default TARGET_BASE ="<<base;
    }

    // 3) 포즈를 12워드(6float)로 인코딩해서 쓰기
    EulerZYX rpy = toolRotate(pose[3], pose[4], pose[5], yaw);
    QVector<double> rotate_pose{pose};
    rotate_pose[3] = rpy.roll;
    rotate_pose[4] = rpy.pitch;
    rotate_pose[5] = rpy.yaw;

    QVector<quint16> regs; regs.reserve(12);
    for (int i=0;i<6;i++) {
        quint16 hi, lo;
        floatToRegs(float(rotate_pose[i]), hi, lo);
        regs << hi << lo;
    }
    m_bus->writeHoldingBlock(base, regs);

    // 4) FSM 규칙대로 PUBLISH_REQ 올리고 시작
    // FSM 시작: PUBLISH_REQ=1 → BUSY↑ → … → DONE↑ → ACK 후 DONE
    // kind에 따라 coil 분기
    if (!kind.compare("place", Qt::CaseInsensitive) && A_PUBLISH_PLACE > 0)
    {   // kind가 "place"이고 A_PUBLISH_PLACE >= 0이면 해당 주소 사용
        QTimer::singleShot(50, this, [this]{
            m_bus->writeCoil(A_PUBLISH_PLACE, true);
        });
        QTimer::singleShot(200, this, [this]{
            m_bus->writeCoil(A_PUBLISH_PLACE, false);
        });
    }
    else if (!kind.compare("pick", Qt::CaseInsensitive) && A_PUBLISH_PICK >= 0)
    {   // kind가 "pick"이고 A_PUBLISH_PICK >= 0이면 해당 주소 사용
        QTimer::singleShot(50, this, [this]{
            m_bus->writeCoil(A_PUBLISH_PICK, true);
        });
        QTimer::singleShot(200, this, [this]{
            m_bus->writeCoil(A_PUBLISH_PICK, false);
        });
    }
}

void Orchestrator::publishBulkPoseWithKind(const QVector<double>& pose, const QString& kind)
{
    if (pose.size() < 6) {
        qWarning() << "[ORCH] pose needs 6 elements, got" << pose.size();
//        return false;
    }
    int base = A_TARGET_BASE;   // 기본 TARGET_BASE
    if (!kind.compare("place", Qt::CaseInsensitive) && A_TARGET_BASE_PLACE >= 0)
    {   // kind가 "place"이고 A_TARGET_BASE_PLACE >= 0이면 해당 주소 사용
        base = A_TARGET_BASE_PLACE;
    }
    else if (!kind.compare("pick", Qt::CaseInsensitive) && A_TARGET_BASE_PICK >= 0)
    {   // kind가 "pick"이고 A_TARGET_BASE_PICK >= 0이면 해당 주소 사용
        base = A_TARGET_BASE_PICK;
    }
    else
    {   // 그 외에는 기본 A_TARGET_BASE 사용
        qDebug()<<"[ORCH] Using default TARGET_BASE ="<<base;
    }

    QVector<quint16> regs;
    regs.reserve(12);

    for (int i=0;i<6;i++) {
        quint16 hi{0}, lo{0};
        floatToRegs(static_cast<float>(pose[i]), hi, lo);
        regs << hi << lo;
    }
    m_bus->writeHoldingBlock(base, regs);

    if (!kind.compare("place", Qt::CaseInsensitive) && A_PUBLISH_PLACE > 0)
    {   // kind가 "place"이고 A_PUBLISH_PLACE >= 0이면 해당 주소 사용
        QTimer::singleShot(50, this, [this]{
            m_bus->writeCoil(A_PUBLISH_PLACE, true);
        });
        QTimer::singleShot(200, this, [this]{
            m_bus->writeCoil(A_PUBLISH_PLACE, false);
        });
    }
    else if (!kind.compare("pick", Qt::CaseInsensitive) && A_PUBLISH_PICK >= 0)
    {   // kind가 "pick"이고 A_PUBLISH_PICK >= 0이면 해당 주소 사용
        QTimer::singleShot(50, this, [this]{
            m_bus->writeCoil(A_PUBLISH_PICK, true);
        });
        QTimer::singleShot(200, this, [this]{
            m_bus->writeCoil(A_PUBLISH_PICK, false);
        });
    }
}

void Orchestrator::publishFlip_Offset(bool flip, int offset, float yaw, int thick)
{
    int base = A_TARGET_BASE_PLACE;   // 기본 TARGET_BASE_PLACE

    float val[4];
    val[0] = flip ? 1.0f : 0.0f;
    val[1] = static_cast<double>(offset);
    val[2] = yaw;
    val[3] = static_cast<double>(thick);

    QVector<quint16> regs; regs.reserve(8);//(6);
    for (int i=0;i<4;i++) {
        quint16 hi, lo;
        floatToRegs(val[i], hi, lo);
        regs << hi << lo;
    }

    m_bus->writeHoldingBlock(base, regs);
}

void Orchestrator::publishPoseToRobot1(const QVector<double>& pose, int speedPct)
{
    if (pose.size() < 6) {
        emit log("[FSM] pose size < 6");
        return;
    }
    // 6축 float → 12워드(HI,LO)로 인코딩
    QVector<quint16> regs; regs.reserve(12);
    for (int i=0;i<6;++i) {
        float f = float(pose[i]);
        quint16 hi=0, lo=0;
        floatToRegs(f, hi, lo);     // Orchestrator.cpp 상단에 이미 있는 함수
        regs.push_back(hi);
        regs.push_back(lo);
    }

    // HOLDING에 좌표 블록 쓰기 (TARGET_POSE_BASE = 132..143)
    m_bus->writeHoldingBlock(A_TARGET_BASE, regs);
    // FSM: Publish 시작 (PUBLISH_REQ = 1)
    m_bus->writeCoil(A_PUBLISH_PICK, true);
    emit log("[FSM] PUBLISH_REQ=1 (Robot1)");
    // 이후 FSM은 기존 cycle() 로직: BUSY↑ → DONE↑ → PUBLISH_REQ=0 → DONE↓ 반환
}

void Orchestrator::publishArrangePoses(const QVector<double>& pick, const QVector<double>& place)
{
    int pick_base = A_TARGET_BASE_PICK;
    int place_base = A_TARGET_BASE_PLACE;

    qDebug()<<"[ORCH] publishArrangePoses:"
            <<"pick="<<pick
            <<"place="<<place;
    // 포즈를 12워드(6float)로 인코딩해서 쓰기
    QVector<quint16> pick_regs;
    pick_regs.reserve(12);
    for (int i=0;i<6;i++) {
        quint16 hi, lo;
        floatToRegs(float(pick[i]), hi, lo);
        pick_regs << hi << lo;
    }
    m_bus->writeHoldingBlock(pick_base, pick_regs);

    QVector<quint16> place_regs;
    place_regs.reserve(12);
    for (int i=0;i<6;i++) {
        quint16 hi, lo;
        floatToRegs(float(place[i]), hi, lo);
        place_regs << hi << lo;
    }
    m_bus->writeHoldingBlock(place_base, place_regs);
    QTimer::singleShot(50, this, [this]{
        m_bus->writeCoil(A_PUBLISH_PICK, true);
    });
    QTimer::singleShot(200, this, [this]{
        m_bus->writeCoil(A_PUBLISH_PICK, false);
    });
}

void Orchestrator::publishBulkMode(const int &mode)
{
    // bulk 모드 설정 (HOLDING 102번지에 모드 값 기록)
    QVector<quint16> regs;
    regs.reserve(1);
    regs << static_cast<quint16>(mode);
    m_bus->writeHoldingBlock(102, regs);
}
