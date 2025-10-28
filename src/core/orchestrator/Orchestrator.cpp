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
    double additional_yaw_degrees = 270;//rz - target_yaw;
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

        //qDebug()<<"[ORCH] discreteInputsRead start="<<start<<"data="<<data<<data.size();
        auto get = [&](int addr, bool& out){
            int idx = addr - start;
//            qDebug()<<"  get addr="<<addr<<"idx="<<idx;
            if (idx >= 0 && idx < data.size())
                out = data[idx];
        };

        bool ready = m_lastReady, busy = m_lastBusy, done = m_lastDone;
        get(A_ROBOT_READY, ready);
        get(A_ROBOT_BUSY,  busy);
        get(A_PICK_DONE,   done);

        // DO3/DO4/DO5 펄스 감지
        bool do3=m_lastDO3, do4=m_lastDO4, do5=m_lastDO5;
        get(A_DO3_PULSE , do3);
        get(A_DO4_PULSE , do4);
        get(A_DO5_PULSE , do5);
/*
        qDebug()<<"[ORCH] DI read:"
                 <<"READY="<<ready
                 <<"BUSY="<<busy
                 <<"DONE="<<done
                 <<"DO3="<<do3
                 <<"DO4="<<do4
                 <<"DO5="<<do5;
*/
        // 에지 감지
        bool rRise = ( ready && !m_lastReady);
        bool bRise = ( busy && !m_lastBusy );
        bool bFall = (!busy &&  m_lastBusy );
        bool dRise = ( done && !m_lastDone);
        bool dFall = (!done &&  m_lastDone);

        switch (m_state) {
        case State::WaitRobotReady:
            //if (rRise && !busy) {
            if (ready && !busy) {
                setState(State::PublishTarget);
            }
            break;
        case State::WaitPickStart:
            if (bRise) {
                setState(State::WaitPickDone);
                m_stateTick.restart();
            }
            break;
        case State::WaitPickDone:
            if (bFall && done) {
                //m_bus->writeCoil(A_PUBLISH_REQ, false);
                if (A_PUBLISH_PICK >= 0) m_bus->writeCoil(A_PUBLISH_PICK, false);
                setState(State::WaitDoneClear);
                m_stateTick.restart();
            }
            break;
        case State::WaitDoneClear:
            if (dFall) {
                //setState(State::PublishTarget);
                setState(State::WaitRobotReady);
                emit finishedCurrentCycle();
            }
            break;
        default: break;
        }
        // 상태 업데이트
        m_lastReady = ready;
        m_lastBusy  = busy;
        m_lastDone  = done;

        // === 상승에지 → 공정 인덱스 배출 ===
        if (!m_lastDO3 && do3) emit processPulse(m_robotId, 0); // 공정 1
        if (!m_lastDO4 && do4) emit processPulse(m_robotId, 1); // 공정 2
        if (!m_lastDO5 && do5) emit processPulse(m_robotId, 2); // 공정 3
        m_lastDO3 = do3; m_lastDO4 = do4; m_lastDO5 = do5;
    });

    connect(m_bus, &ModbusClient::inputRead, this, [this](int start, const QVector<quint16>& data){
        // 디버그용: 입력 레지스터 읽기 결과 처리
        if (data.isEmpty()) return;

        // 예: 주소 340부터 12개 레지스터 읽기
        if (start == 340 && data.size() >= 12) {
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
        // 예: 주소 388부터 12개 레지스터 읽기
        else if (start == 388 && data.size() >= 12) {
            Pose6D j;
            j.x  = regsToFloat(data[0],  data[1]);  // J1
            j.y  = regsToFloat(data[2],  data[3]);  // J2
            j.z  = regsToFloat(data[4],  data[5]);  // J3
            j.rx = regsToFloat(data[6],  data[7]);  // J4
            j.ry = regsToFloat(data[8],  data[9]);  // J5
            j.rz = regsToFloat(data[10], data[11]); // J6
            m_kinState.joints = j;
            m_kinState.hasJoints = true;
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
    const auto coils   = m.value("coils").toMap();
    const auto di      = m.value("discrete_inputs").toMap();
    const auto holding = m.value("holding").toMap();

    if (di.contains("ROBOT_READY")) A_ROBOT_READY = di.value("ROBOT_READY").toInt();
    if (di.contains("ROBOT_BUSY"))  A_ROBOT_BUSY  = di.value("ROBOT_BUSY").toInt();
    if (di.contains("PICK_DONE"))   A_PICK_DONE   = di.value("PICK_DONE").toInt();
    if (di.contains("DO3_PULSE"))   A_DO3_PULSE = di.value("DO3_PULSE").toInt();
    if (di.contains("DO4_PULSE"))   A_DO4_PULSE = di.value("DO4_PULSE").toInt();
    if (di.contains("DO5_PULSE"))   A_DO5_PULSE = di.value("DO5_PULSE").toInt();

    if (coils.contains("PUBLISH_PICK")) A_PUBLISH_PICK = coils.value("PUBLISH_PICK").toInt();
    if (coils.contains("PUBLISH_PLACE")) A_PUBLISH_PLACE = coils.value("PUBLISH_PLACE").toInt();
    if (coils.contains("DI2")) A_DI2 = coils.value("DI2").toInt();
    if (coils.contains("DI3")) A_DI3 = coils.value("DI3").toInt();
    if (coils.contains("DI4")) A_DI4 = coils.value("DI4").toInt();

    if (holding.contains("TARGET_POSE_BASE"))
        A_TARGET_BASE = holding.value("TARGET_POSE_BASE").toInt();

    if (holding.contains("TARGET_POSE_PICK"))
        A_TARGET_BASE_PICK = holding.value("TARGET_POSE_PICK").toInt();

    if (holding.contains("TARGET_POSE_PLACE"))
        A_TARGET_BASE_PLACE = holding.value("TARGET_POSE_PLACE").toInt();
/*
    if (holding.contains("SEQ_ID"))        A_SEQ_ID        = holding.value("SEQ_ID").toInt();
    if (holding.contains("PAYLOAD_CKSUM")) A_PAYLOAD_CKSUM = holding.value("PAYLOAD_CKSUM").toInt();
    if (holding.contains("SPEED_PCT"))     A_SPEED_PCT     = holding.value("SPEED_PCT").toInt();
    if (holding.contains("CMD_TIMEOUT_MS"))   A_CMD_TIMEOUT_MS   = holding.value("CMD_TIMEOUT_MS").toInt();
    if (holding.contains("READY_TIMEOUT_MS")) A_READY_TIMEOUT_MS = holding.value("READY_TIMEOUT_MS").toInt();
*/
    emit log(QString("[ADDR] READY=%1 BUSY=%2 DONE=%3 COIL_PUBLISH=%4 TARGET_BASE=%5")
                 .arg(A_ROBOT_READY).arg(A_ROBOT_BUSY).arg(A_PICK_DONE).arg(A_PUBLISH_PICK).arg(A_TARGET_BASE)
             , Common::LogLevel::Info);
}

void Orchestrator::cycle()
{
    m_bus->readInputs(340, 12);
    m_bus->readInputs(388, 12);

    m_bus->readDiscreteInputs(100, 6);
#if false
    switch(m_state) {
    case State::Idle:
        return;

    case State::WaitRobotReady: {

        const int diStart = std::min({A_ROBOT_READY, A_PICK_DONE, A_ROBOT_BUSY, A_DO3_PULSE, A_DO4_PULSE, A_DO5_PULSE});
        const int diEnd   = std::max({A_ROBOT_READY, A_PICK_DONE, A_ROBOT_BUSY, A_DO3_PULSE, A_DO4_PULSE, A_DO5_PULSE});
        //m_bus->readDiscreteInputs(diStart, diEnd - diStart + 1);
        m_bus->readDiscreteInputs(100, 6);

/*
        if (m_stateTick.elapsed() > A_READY_TIMEOUT_MS) {
            emit log("[WARN] WaitRobotReady timeout", Common::LogLevel::Debug);
            m_stateTick.restart();
        }
*/
        break;
    }
    case State::PublishTarget: {
        const int total = m_model ? m_model->rowCount() : 0;
        if (total <= 0) {
            emit log("[FSM] No targets. Waiting...", Common::LogLevel::Debug);
            return;
        }
        if (m_currentRow+1 >= total) {
            if (m_repeat) {
                m_currentRow = 0;
            } else {
                emit log("[FSM] Reached end of list. Waiting...", Common::LogLevel::Info);
                return;
            }
        } else {
            ++m_currentRow;
        }
        if (m_currentRow < 0 || m_currentRow >= total) {
            emit log("[FSM] Invalid row index, skip publish", Common::LogLevel::Warn);
            return;
        }
        if (m_model)
            m_model->setActiveRow(m_currentRow);

        const auto pt = m_model->getRow(m_currentRow);

        emit currentRowChanged(m_currentRow);

        QVector<quint16> regs;
        regs.reserve(12);

//KJW -2025-10-22
        EulerZYX rpy = toolRotate(pt.rx, pt.ry, pt.rz, -90.0);
        const float target[6] = {
            static_cast<float>(pt.x), static_cast<float>(pt.y), static_cast<float>(pt.z),
            static_cast<float>(rpy.roll), static_cast<float>(rpy.pitch), static_cast<float>(rpy.yaw)
        };
//

        for (int i=0;i<6;++i) {
            quint16 hi, lo; floatToRegs(target[i], hi, lo);
            regs.push_back(hi);
            regs.push_back(lo);
        }

        m_bus->writeHoldingBlock(A_TARGET_BASE, regs);
        m_bus->writeCoil(A_PUBLISH_REQ, true);
        emit log(QString("[FSM] Published #%1 @%2..%3 (X=%4 Y=%5 Z=%6 | Rx=%7 Ry=%8 Rz=%9)")
                     .arg(m_currentRow)
                     .arg(A_TARGET_BASE).arg(A_TARGET_BASE + regs.size() - 1)
                     .arg(pt.x,0,'f',3).arg(pt.y,0,'f',3).arg(pt.z,0,'f',3)
                     .arg(pt.rx,0,'f',3).arg(pt.ry,0,'f',3).arg(pt.rz,0,'f',3)
                 , Common::LogLevel::Info);

        setState(State::WaitPickStart);     // BUSY 상승 대기
        m_stateTick.restart();
        break; }

    case State::WaitPickStart:
    case State::WaitPickDone:
    case State::WaitDoneClear:
#if false
        m_bus->readDiscreteInputs(qMin(A_ROBOT_READY, A_ROBOT_BUSY),
                                  qAbs(A_ROBOT_BUSY - A_ROBOT_READY) + 1);
#else
    {
        const int diStart = std::min({A_ROBOT_READY, A_PICK_DONE, A_ROBOT_BUSY});
        const int diEnd   = std::max({A_ROBOT_READY, A_PICK_DONE, A_ROBOT_BUSY});
        m_bus->readDiscreteInputs(diStart, diEnd - diStart + 1);
    }
#endif
    /*
        if (m_stateTick.elapsed() > A_CMD_TIMEOUT_MS) {
            emit log("[ERR] Command timeout");
            m_bus->writeCoil(A_PUBLISH_REQ, false);
            m_state = State::WaitRobotReady;
            m_stateTick.restart();
        }
    */
        break;
    }
#endif
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

void Orchestrator::publishPoseWithKind(const QVector<double>& pose, int speedPct, const QString& kind)
{
    // 1) 속도/시퀀스 등 부가 파라미터
//    if (A_SPEED_PCT >= 0) m_bus->writeHolding(A_SPEED_PCT, quint16(speedPct));
//    if (A_SEQ_ID   >= 0) m_bus->writeHolding(A_SEQ_ID,   ++m_seq);

    // 2) 주소 선택: kind=place면 A_TARGET_BASE_PLACE, pick이면 A_TARGET_BASE_PICK, 없으면 기본 A_TARGET_BASE
    int base = A_TARGET_BASE;
    if (!kind.compare("place", Qt::CaseInsensitive) && A_TARGET_BASE_PLACE >= 0)
        base = A_TARGET_BASE_PLACE;
    else if (!kind.compare("pick", Qt::CaseInsensitive) && A_TARGET_BASE_PICK >= 0)
        base = A_TARGET_BASE_PICK;

    // 3) 포즈를 12워드(6float)로 인코딩해서 쓰기
    EulerZYX rpy = toolRotate(pose[3], pose[4], pose[5], -90.0);
    QVector<double> rotate_pose{pose};
    rotate_pose[3] = rpy.roll;
    rotate_pose[4] = rpy.pitch;
    rotate_pose[5] = rpy.yaw;

    QVector<quint16> regs; regs.reserve(12);
    for (int i=0;i<6;i++) {
        quint16 hi, lo; floatToRegs(float(rotate_pose[i]), hi, lo);
        regs << hi << lo;
    }
    m_bus->writeHoldingBlock(base, regs);

    // 4) FSM 규칙대로 PUBLISH_REQ 올리고 시작
    // FSM 시작: PUBLISH_REQ=1 → BUSY↑ → … → DONE↑ → ACK 후 DONE
   // kind에 따라 coil 분기
    if (!kind.compare("place", Qt::CaseInsensitive) && A_PUBLISH_PLACE > 0)
        m_bus->writeCoil(A_PUBLISH_PLACE, true);
    else
        m_bus->writeCoil(A_PUBLISH_PICK, true);

    // FSM은 동일하게 WaitPickStart로 진입
    setState(State::WaitPickStart);
    m_stateTick.restart();
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
/*
    // 속도 등 파라미터
    if (A_SPEED_PCT > 0)
        m_bus->writeHolding(A_SPEED_PCT, quint16(std::clamp(speedPct,0,100)));

    // 간단한 페이로드 체크섬/SEQ (옵션: 사용 중일 때만)
    if (A_SEQ_ID > 0) {
        m_seq++;
        m_bus->writeHolding(A_SEQ_ID, m_seq);
    }
    if (A_PAYLOAD_CKSUM > 0) {
        // 아주 단순 합계 체크섬(데모)
        quint32 sum=0; for (auto v: regs) sum += v;
        m_bus->writeHolding(A_PAYLOAD_CKSUM, quint16(sum & 0xFFFF));
    }
*/
    // FSM: Publish 시작 (PUBLISH_REQ = 1)
    m_bus->writeCoil(A_PUBLISH_PICK, true);
    emit log("[FSM] PUBLISH_REQ=1 (Robot1)");
    // 이후 FSM은 기존 cycle() 로직: BUSY↑ → DONE↑ → PUBLISH_REQ=0 → DONE↓ 반환
}
