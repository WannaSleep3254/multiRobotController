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

Orchestrator::Orchestrator(ModbusClient* bus, PickListModel* model, QObject* parent)
    : QObject(parent), m_bus(bus), m_model(model), m_timer(new QTimer(this))
{
    m_timer->setInterval(50);
    connect(m_timer, &QTimer::timeout, this, &Orchestrator::cycle);

    // READY/DONE/BUSY 읽기 및 에지 처리
    connect(m_bus, &ModbusClient::discreteInputsRead, this, [this](int start, QVector<bool> data){
        if (data.isEmpty()) return;

        auto get = [&](int addr, bool& out){
            int idx = addr - start;
            if (idx >= 0 && idx < data.size()) out = data[idx];
        };

        bool ready = m_lastReady, busy = m_lastBusy, done = m_lastDone;
        get(A_ROBOT_READY, ready);
        get(A_ROBOT_BUSY,  busy);
        get(A_PICK_DONE,   done);

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
                m_bus->writeCoil(A_PUBLISH_REQ, false);
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

    m_bus->writeCoil(A_PUBLISH_REQ, false);
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
    if (coils.contains("PUBLISH_REQ")) A_PUBLISH_REQ = coils.value("PUBLISH_REQ").toInt();
    if (holding.contains("TARGET_POSE_BASE"))
        A_TARGET_BASE = holding.value("TARGET_POSE_BASE").toInt();
    if (holding.contains("SEQ_ID"))        A_SEQ_ID        = holding.value("SEQ_ID").toInt();
    if (holding.contains("PAYLOAD_CKSUM")) A_PAYLOAD_CKSUM = holding.value("PAYLOAD_CKSUM").toInt();
    if (holding.contains("SPEED_PCT"))     A_SPEED_PCT     = holding.value("SPEED_PCT").toInt();
    if (holding.contains("CMD_TIMEOUT_MS"))   A_CMD_TIMEOUT_MS   = holding.value("CMD_TIMEOUT_MS").toInt();
    if (holding.contains("READY_TIMEOUT_MS")) A_READY_TIMEOUT_MS = holding.value("READY_TIMEOUT_MS").toInt();

    emit log(QString("[ADDR] READY=%1 BUSY=%2 DONE=%3 COIL_PUBLISH=%4 TARGET_BASE=%5")
                 .arg(A_ROBOT_READY).arg(A_ROBOT_BUSY).arg(A_PICK_DONE).arg(A_PUBLISH_REQ).arg(A_TARGET_BASE)
             , Common::LogLevel::Info);
}

void Orchestrator::cycle()
{
//    qDebug()<<"[FSM] cycle, state="<<int(m_state);
    m_bus->readInputs(340, 12);
    m_bus->readInputs(388, 12);

    switch(m_state) {
    case State::Idle:
        return;

    case State::WaitRobotReady: {
        m_bus->readDiscreteInputs(qMin(A_ROBOT_READY, A_ROBOT_BUSY),
                                  qAbs(A_ROBOT_BUSY - A_ROBOT_READY) + 1);
        if (m_stateTick.elapsed() > A_READY_TIMEOUT_MS) {
            emit log("[WARN] WaitRobotReady timeout", Common::LogLevel::Debug);
            m_stateTick.restart();
        }
        break; }

    case State::PublishTarget: {
#if false
        const int total = m_model->rowCount();
        if (total == 0) {
            emit log("[FSM] No targets. Waiting...", Common::LogLevel::Debug);
            return;
        }
        if (m_currentRow+1 >= total) {
            if (m_repeat) {
                    // wrap to start
                        m_currentRow = 0;
                } else {
                    emit log("[FSM] Reached end of list. Waiting...", Common::LogLevel::Info);
                    return;
                }
        } else {
           ++m_currentRow;
        }
        if(m_model)
            m_model->setActiveRow(m_currentRow);

        const auto pt = m_model->getRow(m_currentRow);
#else
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
#endif
        emit currentRowChanged(m_currentRow);

        QVector<quint16> regs;
        regs.reserve(12);
/*
        const float vals[6] = {
            static_cast<float>(pt.x), static_cast<float>(pt.y), static_cast<float>(pt.z),
            static_cast<float>(pt.rx), static_cast<float>(pt.ry), static_cast<float>(pt.rz)
        };
*/
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

        // 선택 파라미터 기록
        if (A_SPEED_PCT >= 0)   m_bus->writeHolding(A_SPEED_PCT, 50);
        if (A_SEQ_ID   >= 0)    m_bus->writeHolding(A_SEQ_ID, ++m_seq);
        if (A_PAYLOAD_CKSUM >= 0) {
            quint32 sum = 0; for (auto v : regs) sum += v;
            m_bus->writeHolding(A_PAYLOAD_CKSUM, quint16(sum & 0xFFFF));
        }

        m_bus->writeHoldingBlock(A_TARGET_BASE, regs);
        m_bus->writeCoil(A_PUBLISH_REQ, true);
        emit log(QString("[FSM] Published #%1 @%2..%3 (X=%4 Y=%5 Z=%6 | Rx=%7 Ry=%8 Rz=%9)")
                     .arg(m_currentRow)
                     .arg(A_TARGET_BASE).arg(A_TARGET_BASE + regs.size() - 1)
                     .arg(pt.x,0,'f',3).arg(pt.y,0,'f',3).arg(pt.z,0,'f',3)
                     .arg(pt.rx,0,'f',3).arg(pt.ry,0,'f',3).arg(pt.rz,0,'f',3)
                 , Common::LogLevel::Info);

        //m_state = State::WaitPickStart;
        setState(State::WaitPickStart);     // BUSY 상승 대기
        m_stateTick.restart();
        break; }

    case State::WaitPickStart:
    case State::WaitPickDone:
    case State::WaitDoneClear:
        m_bus->readDiscreteInputs(qMin(A_ROBOT_READY, A_ROBOT_BUSY),
                                  qAbs(A_ROBOT_BUSY - A_ROBOT_READY) + 1);
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
    m_bus->writeCoil(A_PUBLISH_REQ, true);
    emit log("[FSM] PUBLISH_REQ=1 (Robot1)");
    // 이후 FSM은 기존 cycle() 로직: BUSY↑ → DONE↑ → PUBLISH_REQ=0 → DONE↓ 반환
}
