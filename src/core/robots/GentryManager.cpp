#include "GentryManager.h"
#include <qdebug.h>
#include <QModbusDevice>

#define Flip_False 1
#define Flip_True 2
#define gty_x_home_pos 3
#define gty_x_docking_pos 4
#define gty_place_pos 5
#define gty_z_place_pos 6
#define gty_home_pos 7
#define picker_90 8
#define picker_0 9
#define conveyor_step_pos 10
#define msg_rdy_1 11
#define msg_rdy_2 12
#define motion_done 13
#define gentry_rdy 100

QString gantryPoseToString(GantryPose pose)
{
    switch (pose) {
    case GantryPose::None:     return "none";
    case GantryPose::Standby:  return "standby";
    case GantryPose::Docking:  return "docking";
    case GantryPose::Place:    return "place";
    }
    return "unknown";
}

GantryPose stringToGantryPose(const QString& s)
{
    const QString v = s.trimmed().toLower();

    if (v == "standby")  return GantryPose::Standby;
    if (v == "docking")  return GantryPose::Docking;
    if (v == "place")    return GantryPose::Place;

    return GantryPose::None;
}

GentryManager::GentryManager(QObject *parent)
    : QObject{parent}
{
    motorController = new Leadshine::ELD2();

    setup_motorStateMonitoring();

//    motorController->setPort("COM10", "115200");
//    motorController->doConnect();
}

void GentryManager::setPort()
{
    motorController->setPort("COM10", "115200");
}

void GentryManager::doConnect()
{
    motorController->doConnect();
}

void GentryManager::doDisconnect()
{
    motorController->doDisConnect();
}

void GentryManager::doServoOn()
{
    motorController-> reqWriteServo(1, true);
    motorController-> reqWriteServo(2, true);
    motorController-> reqWriteServo(3, true);
    motorController-> reqWriteServo(4, true);
}

void GentryManager::doServoOff()
{
    motorController-> reqWriteServo(1, false);
    motorController-> reqWriteServo(2, false);
    motorController-> reqWriteServo(3, false);
    motorController-> reqWriteServo(4, false);
}

void GentryManager::setup_motorStateMonitoring()
{
    QStringList stateMotorCom;
    stateMotorCom<<"Unconnected"<<"Connecting"<<"Connected"<<"Closing";
    QObject::connect(motorController, &Leadshine::ELD2::comState, this, [=](int state){
        // 통신 상태
        /**
            QModbusDevice::UnconnectedState	0	The device is disconnected.
            QModbusDevice::ConnectingState	1	The device is being connected.
            QModbusDevice::ConnectedState	2	The device is connected to the Modbus network.
            QModbusDevice::ClosingState	    3	The device is being closed.
        **/
        emit log("모터 연결: "+stateMotorCom[state]);

        switch (state) {
        case QModbusDevice::UnconnectedState:
        {
            emit motorComStatus(1, false);
            emit motorComStatus(2, false);
            emit motorComStatus(3, false);
            emit motorComStatus(4, false);
            break;
        }
        case QModbusDevice::ConnectedState:
        {
            break;
        }
        }
    });
    QStringList stateMotorError;
    stateMotorError<<"NoError"<<"ReadError"<<"WriteError"<<"ConnectionError"<<"ConfigurationError"
                    <<"TimeoutError"<<"ProtocolError"<<"ReplyAbortedError"<<"UnknownError";
    //QObject::connect(motorController, &Leadshine::ELD2::comState, this, [=](int state){
    QObject::connect(motorController, &Leadshine::ELD2::errorState, this, [=](int state){
        // 에러 상태
        /**
        QModbusDevice::NoError              0	No errors have occurred.
        QModbusDevice::ReadError            1	An error occurred during a read operation.
        QModbusDevice::WriteError           2	An error occurred during a write operation.
        QModbusDevice::ConnectionError      3	An error occurred when attempting to open the backend.
        QModbusDevice::ConfigurationError	4	An error occurred when attempting to set a configuration parameter.
        QModbusDevice::TimeoutError         5	A timeout occurred during I/O. An I/O operation did not finish within a given time frame.
        QModbusDevice::ProtocolError        6	A Modbus specific protocol error occurred.
        QModbusDevice::ReplyAbortedError	7	The reply was aborted due to a disconnection of the device.
        QModbusDevice::UnknownError         8	An unknown error occurred.
        **/
        emit log("모터 에러: "+stateMotorError[state]);
    });
    // 축별 통신/버전 정보
    QObject::connect(motorController, &Leadshine::ELD2::readVersion,this,[=](int id, QString version){
//        emit log(QString("%1번 모터 SW_Version: %2").arg(id).arg(version));
        emit motorComStatus(id, true);
    });
    // 축별 서보 On/Off
    QObject::connect(motorController, &Leadshine::ELD2::readServo,this,[=](int id, bool servo){
        emit motorServoStatus(id, servo);
//        emit log(QString("%1번 모터 서보: %2").arg(id).arg(servo?"On":"Off"));
    });
    // 축별 상태
    QObject::connect(motorController, &Leadshine::ELD2::readState,this,[=](int id, uint16_t state){
        // 상태 코드: bit_0: RDY, bit_1: RUN, bit_2: ERR, bit_3: HOME_OK, bit_4: INPOS, bit_5: AT-SPEED
        bool rdy = state & (1 << 0);       // Bit 0
        bool run = state & (1 << 1);      // Bit 1
        bool err = state & (1 << 2);      // Bit 2
        bool homeOk = state & (1 << 3);   // Bit 3
        bool inPosition = state & (1 << 4);   // Bit 4
        bool AtSpeed = state & (1 << 5);      // Bit 5

        if(id==4)
        {
            qDebug()<<QString("%1번 모터 상태: RDY=%2, RUN=%3, ERR=%4, HOME_OK=%5, INPOS=%6, AT-SPEED=%7")
                          .arg(id).arg(rdy).arg(run).arg(err).arg(homeOk).arg(inPosition).arg(AtSpeed);
        }
        //emit motorState(id, state);
    });
    // 축별 엔코더
    QObject::connect(motorController, &Leadshine::ELD2::readEncoder,this,[=](int id, int32_t pulse, float pos){
        //qDebug()<<QString("%1번 모터 엔코더: %2 pulse, %3 mm").arg(id).arg(pulse).arg(pos);
        // emit log(QString("%1번 모터 엔코더: %2 pulse, %3 mm").arg(id).arg(pulse).arg(pos));
    });
    // 축별 속도
    QObject::connect(motorController, &Leadshine::ELD2::readVelocity,this,[=](int id, int16_t velocity){
        // emit log(QString("%1번 모터 속도: %2 rpm").arg(id).arg(velocity));
    });
    // 축별 에러
    QObject::connect(motorController, &Leadshine::ELD2::readError,this,[=](int id, uint16_t error_code){
        if(error_code != 0)
            emit log(QString("%1번 모터 에러 코드: %2").arg(id).arg(error_code));
    });
    QObject::connect(motorController, &Leadshine::ELD2::motionFinished,this,[=](int id, float targetPos){
        emit log(QString("%1번 모터 이동 완료: %2").arg(id).arg(targetPos));
        m_targetPos[id] = targetPos;
        onAxisFinished(id, 0, true);
    });
}

//void GentryManager::doGentryPlace(int offset_x, int offset_z)
void GentryManager::doGentryPlace(int offset_z)
{
    m_targetGentryX = 0+2000*37; // TK-1: 74000;
    motorController->reqWritePos(1, m_targetGentryX); //gentry x move 37mm
    logMessage2("gentry move to X place position");

    m_targetGentryZ = -250000+2000*offset_z;
    motorController->reqWritePos(2, m_targetGentryZ); //gentry z move -125mm: 1mm per 2000 pulse
    logMessage2("gentry move to Z place position");
    motorController->reqWritePos(3, -25020); //picker rotate -90 degree
    logMessage2("picker rotate -90 degree");
}

void GentryManager::doGentryReady()
{
    motorController->reqWritePos(1, -260000); //gentry x move -130mm
    logMessage2("gentry move to X home position");
    motorController->reqWritePos(2, 0); //gentry z move 0mm
    logMessage2("gentry move to Z home position");
    motorController->reqWritePos(3, -25020); //picker rotate -90 degree
    logMessage2("picker rotate -90 degree");
}

void GentryManager::startGantryMove()   // 1-2-3축 동시
{
    /*
    const int seq = m_nextSeq++;

    PendingGroup g;
    g.pendingAxes = QSet<int>({1, 2, 3});
    g.ok = true;
    m_gantryMap.insert(seq, g);
    */
    m_gantryOk = true;
    m_gantryPendingAxes = QSet<int>({1,2,3});
}

void GentryManager::startConveyorMove() // 4축
{
/*
    const int seq = m_nextSeq++;

    PendingGroup g;
    g.pendingAxes = QSet<int>({4});
    g.ok = true;
    m_conveyorMap.insert(seq, g);
*/
    m_convOk = true;
    m_conveyorPendingAxes = QSet<int>({4});
}

void GentryManager::onAxisFinished(int axis, int seq, bool ok)
{
    auto near = [](int a, int b, int tol){
        return std::abs(a - b) <= tol;
    };

    if (axis >= 1 && axis <= 3) {
        m_gantryOk = m_gantryOk && ok;
        m_gantryPendingAxes.remove(axis);

        if (m_gantryPendingAxes.isEmpty()) {
            const float xPos        = m_targetPos.value(1, 0.0f);
            const float zPos        = m_targetPos.value(2, 0.0f);
            const float pickerAngle = m_targetPos.value(3, 0.0f);
/////////////////////////////////////////////////////////////////////
            if( near(xPos,0,10) &&
                near(zPos,0,10) &&
                near(pickerAngle,0,10))
            {   // docking
                currentPose = GantryPose::Docking;
            }
            else if(near(xPos,74000,10) && // near(xPos,m_targetGentryX,10)
                    near(zPos, m_targetGentryZ,10) && //near(zPos,-250000,10) &&
                    near(pickerAngle,-25020,10))
            {   // place
                currentPose = GantryPose::Place;
            }
            else if(near(xPos,-260000,10) &&
                    near(zPos,0,10) &&
                    near(pickerAngle, -25020,10))
            {   // stand-by
                currentPose = GantryPose::Standby;
            }
            else
            {
                qDebug()<<"Gentry position unknown:"
                       <<QString("X:%1, Z:%2, P:%3, place_cmd: 74000, %4 , -25020")
                                .arg(xPos).arg(zPos).arg(pickerAngle).arg(m_targetGentryZ);
                currentPose = GantryPose::None;
            }
/////////////////////////////////////////////////////////////////////
            qDebug()<<QString("Gentry currentPose=%1").arg(gantryPoseToString(currentPose));
            emit gantryCommandFinished(m_gantryOk, currentPose);
            m_gantryOk = true;
        }
    }
    else if (axis == 4) {
        m_convOk = m_convOk && ok;
        m_conveyorPendingAxes.remove(axis);

        if (m_conveyorPendingAxes.isEmpty()) {
            emit conveyorCommandFinished(m_convOk);
            m_convOk = true;
        }
    }
}

void GentryManager::logMessage2(const QString& msg)
{
    emit log(msg);
}

void GentryManager::setFalgs(bool conveyor_fwd, bool flip_fwd, bool flip_rev, bool gty_place, bool gty_home_place) {
    flag_conveyor_fwd = conveyor_fwd;
    flag_flip_fwd = flip_fwd;
    flag_flip_rev = flip_rev;
    flag_gty_place = gty_place;
    flag_gty_home_place = gty_home_place;
}

void GentryManager::gentry_motion()
{
    logMessage2("gentry motion start");

    if(flag_conveyor_fwd){
        motion_seq = conveyor_step_pos;
        logMessage2("conveyor received msg");
        flag_conveyor_fwd = false;
    }
    else if(flag_flip_fwd){
        //motion_seq = Flip_False;
        //motion_seq = gty_x_home_pos;
        motion_seq = gty_home_pos;
        logMessage2("flip fwd");
        flag_flip_fwd = false;
    }
    else if(flag_flip_rev){
        //motion_seq = Flip_True;
        motion_seq = gty_x_docking_pos;
        logMessage2("flip rev");
        flag_flip_rev = false;
    }
    else if(flag_gty_place){
        motion_seq = gty_place_pos;
        logMessage2("gentry move to place position");
        flag_gty_place = false;
    }

    switch(motion_seq){

    case conveyor_step_pos: //conveyor next 1 step move
        //qDebug()<<QString("conveyor_step_pos");
        //motorController->reqWriteShift(4, -464100);// conveyor 300mm shift
        motorController->reqWriteShift(4, -402220);// conveyor 260mm shift
        logMessage2("conveyor move fwd one step");
#if false
        motorController->reqWritePos(1, 260000); //gentry x move 130mm
        logMessage2("gentry move to X place position");
        motorController->reqWritePos(2, -250000); //gentry z move -125mm
        logMessage2("gentry move to Z place position");
        motorController->reqWritePos(3, -25020); //picker rotate -90 degree
        logMessage2("picker rotate -90 degree");
#endif
        //magnet Off
        //send motion done msg to Server
        motion_seq = motion_done;
        break;

    case Flip_False: //braket fwd
        motion_seq = gty_x_home_pos; //gentry ready position
        logMessage2("gentry move to Home(Ready) position\r\n");
        break;

    case Flip_True: //braket rev
        motion_seq = gty_x_docking_pos;//gentry & robot docking position
        logMessage2("gentry move to Docking position\r\n");
        break;

    case gty_x_home_pos: //gentry return to home position
        motorController->reqWritePos(1, -260000); //gentry x move -130mm
        logMessage2("gentry move to home position\r\n");
        //send motion done msg to M.C
        motion_seq = motion_done;
        break;

    case gty_x_docking_pos: //gentry & robot x docking position
        motorController->reqWritePos(1, 0); //gentry x move 0mm
        logMessage2("gentry move to X docking position");
        motorController->reqWritePos(2, 0); //gentry z move 0mm
        logMessage2("gentry move to Z docking position");
        motorController->reqWritePos(3, 0); //picker rotate 0 degree
        logMessage2("picker rotate 0 degree");
        //magnet On
        //send motion done msg to M.C
        motion_seq = motion_done;
        break;

    case gty_place_pos: //gentry move to braket place
        //motorController->reqWritePos(1, 260000); //gentry x move 130mm
        motorController->reqWritePos(1, 0); //gentry x move 0mm
        logMessage2("gentry move to X place position");
        motorController->reqWritePos(2, -130000); //gentry z move -65mm
        logMessage2("gentry move to Z place position");
        motorController->reqWritePos(3, -25020); //picker rotate -90 degree
        logMessage2("picker rotate -90 degree");
        //magnet Off
        //send motion done msg to M.C
        motion_seq = motion_done;
        break;

    case gty_home_pos: //gentry z axis move to home
        motorController->reqWritePos(1, -260000); //gentry x move -130mm
        logMessage2("gentry move to X home position");
        motorController->reqWritePos(2, 0); //gentry z move 0mm
        logMessage2("gentry move to Z home position");
        motorController->reqWritePos(3, -25020); //picker rotate -90 degree
        logMessage2("picker rotate 0 degree");
        //send motion done msg to M.C
        motion_seq = motion_done;
        break;

    case gty_z_place_pos: //gentry z axis move to braket place
        motorController->reqWritePos(2, -130000); //gentry z move -65mm
        logMessage2("gentry move to X place position");
        motorController->reqWritePos(3, -25020); //picker rotate -90 degree
        logMessage2("gentry move to Z place position");
        break;

    case picker_90: //picker -90 degree rotate
        qDebug()<<QString("picker_90");
        motorController->reqWritePos(3, -25020); //picker rotate -90 degree
        motion_seq = gty_z_place_pos;
        //magnet Off msg to robot
        motion_seq = msg_rdy_2;
        break;

    case picker_0: //picker 0 degree rotate
        qDebug()<<QString("picker_0");
        motorController->reqWritePos(3, 0); //picker rotate 0 degree
        // QThread::sleep(1000);
        //msg send to robot for magnet On
        motion_seq = msg_rdy_1;
        break;

    case motion_done:

        if(motorController->isMotionDone(4)){
            logMessage2("conveyor move done");
        }
        else if(motorController->isMotionDone(1)){
            logMessage2("X move done\r\n");
        }
        else if(motorController->isMotionDone(2)){
            logMessage2("Z move done\r\n");
        }
        else if(motorController->isMotionDone(3)){
            logMessage2("Picker move done\r\n");
        }

        break;

    case msg_rdy_1:
        qDebug()<<QString("msg_rdy_1");
        /*  if(receive from robot){ //magnet On Msg
                motion_seq = gty_place_pos;
            }
            else{

            }
        */
        break;

    case msg_rdy_2:
        qDebug()<<QString("msg_rdy_2");
        /*  if(receive from robot){ //magnet Off Msg
                motion_seq = gty_x_home_pos;
            }
            else{

            }
        */
        break;

    case gentry_rdy:
        logMessage2("gentry ready");
        if(motorController->isMotionDone(4)){
            logMessage2("conveyor move done");

            logMessage2("send motion done msg to server");
        }
        else{
            logMessage2("conveyor not move done");
        }
        break;

    }
}

