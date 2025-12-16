#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include "RobotManager.h"
#include "GentryManager.h"

#include <QFile>
#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

#include <QMessageBox>
#include <QTimer>
#include <QStatusBar>
#include <QLabel>
#include <QFrame>
#include <QCheckBox>
#include <QDebug>
#include <QDateTime>

#include "widgets/RobotPanel.h"
#include "widgets/MotorPanel.h"

#include <QSplitter>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    setWindowTitle("BinPicking Modbus Controller");
    initVisionClient();

    m_mgr = new RobotManager(this);
    m_mgr->setVisionClient(m_visionClient);
    connect(m_mgr, &RobotManager::log, this,
            [this](const QString& line, Common::LogLevel level){
                QString prefix;
                switch(level) {
                case Common::LogLevel::Debug: prefix = "[DBG] "; break;
                case Common::LogLevel::Info:  prefix = ""; break;
                case Common::LogLevel::Warn:  prefix = "[WARN] "; break;
                case Common::LogLevel::Error: prefix = "[ERR] "; break;
                }
                onLog(prefix + line, level);
            });

    connect(m_mgr, &RobotManager::reqGentryPalce, this, [this]{
        // TODO Gentry Place 동작 시작
        qDebug()<<QDateTime::currentDateTime()<<"MainWindow::reqGentryPlace";
        m_gentryMgr->startGantryMove();
        int offset_mm = (m_sortingOffset>30)? (m_sortingOffset-30+10) : 0;
        m_gentryMgr->doGentryPlace(37, offset_mm);
    });

    connect(m_mgr, &RobotManager::reqGentryReady, this, [this]{
        // TODO Gentry Place 동작 시작
        qDebug()<<QDateTime::currentDateTime()<<"MainWindow::reqGentryReady";
        m_sortingPlacePorcessActive=false;
        m_gentryMgr->doGentryReady();
    });

    connect(m_mgr, &RobotManager::sortProcessFinished, this, [this](const QString& id){
        onLog(QString("[RM] sortProcessFinished received for %1").arg(id));
        m_sortingPlacePorcessActive = false;
    });

    m_gentryMgr = new GentryManager(this);
    connect(m_gentryMgr, &GentryManager::log, this,
            [this](const QString& line){
        onLog("[GENTRY] " + line);
    });

    connect(m_gentryMgr, &GentryManager::gantryCommandFinished, this,
            [this](bool ok, GantryPose pose){
                Q_UNUSED(ok)
                qDebug()<<QString("Gentry gantry command finished: %1").arg(ok?"OK":"FAIL");
                bool flip   = m_sortingFlip;
                int  offset = m_sortingOffset;
                int  thick  = m_sortingThick;

                onLog(QString("m_flip: %1, offset: %2").arg(m_sortingFlip?"OK":"FAIL").arg(offset));
                onLog(QString("flip: %1, offset; %2").arg(flip?"OK":"FAIL").arg(m_sortingOffset));

                if(m_sortingPlacePorcessActive){
                    if(!flip&& pose==GantryPose::Standby)
                    {   //TODO robot->place
                        qDebug()<<"Robot place without flip"<<gantryPoseToString(pose)<<ok;
                        m_mgr->cmdSort_DoPlace(flip, offset, thick);
                    }
                    else if(flip&& pose==GantryPose::Docking)
                    {   //TODO robot->dock -> ready
                        qDebug()<<"Robot place with flip - dock first"<<gantryPoseToString(pose)<<ok;
                    }
                    else if(flip&& pose==GantryPose::Place){
                        // 겐트리 Tool Off
                        qDebug()<<"Gentry place pose"<<gantryPoseToString(pose)<<ok;
                        m_mgr->cmdSort_GentryTool(false);

                        m_sortingPlacePorcessActive=false;
                        m_gantryPickupState = false;

                        QTimer::singleShot(100, this, [this]() {
                            m_visionClient->sendWorkComplete("A", "sorting", "place", 0);
                            m_gentryMgr->doGentryReady();
                        });

                    }
                }

            });


    connect(m_gentryMgr, &GentryManager::conveyorCommandFinished, this,
            [this](bool ok){
                Q_UNUSED(ok)

                onLog(QString("Conveyor command finished: %1").arg(ok?"OK":"FAIL"));
                qInfo()<<QDateTime::currentDateTime()<<"KJW"<<QString("Conveyor command finished: %1").arg(ok?"OK":"FAIL");
        /*
                QTimer::singleShot(100, this, [this]() {
                    m_visionClient->sendWorkComplete("a", "conveyor", "forward", 0);
                });
        */
            });

    m_split  = new QSplitter(Qt::Horizontal, this);
    m_panelA = new RobotPanel(this);
    m_panelB = new RobotPanel(this);
    m_motorPanel = new MotorPanel(this);

    m_panelA->setManager(m_mgr);
    m_panelB->setManager(m_mgr);
    m_motorPanel->setManager(m_gentryMgr);

    m_panelA->setRobotId("A");
    m_panelB->setRobotId("B");

    m_split->addWidget(m_panelA);
    m_split->addWidget(m_panelB);
    m_split->addWidget(m_motorPanel);

    ui->horizontalLayout->addWidget(m_split);
    QTimer::singleShot(0, this, [this]{ loadRobotsFromConfig(); });

    m_motorPanel->onConnect();
    m_motorPanel->onServoOn();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::initVisionClient()
{
    m_visionClient = new VisionClient(this);

    connect(m_visionClient, &VisionClient::connected, this, [this]{
        onLog("[VC] VisionClient connected");
    });
    connect(m_visionClient, &VisionClient::disconnected, this, [this]{
        onLog("[VC] VisionClient disconnected");
    });
    connect(m_visionClient, &VisionClient::ackReceived, this,
            [this](quint32 seq, const QString& status, const QString& msg){
        const QString text = QStringLiteral("[VC] Ack received: seq=%1 status=%2 msg=%3")
                            .arg(QString::number(seq), status, msg);  // multi-arg 사용
        onLog(text);
    });

    // 로그 메시지 처리
    connect(m_visionClient, &VisionClient::lineReceived, this, [this](const QString& line){
        onLog(QString("[VC] Line received: %1").arg(line));
        qDebug()<<QDateTime::currentDateTime()<<"VisionClient line received:"<<line;
    });
    connect(m_visionClient, &VisionClient::commandReceived, this, &MainWindow::onRobotCommand);

}

void MainWindow::onLog(const QString& line)
{
    QString html = QString("<span style='color:blue;font-weight:bold;'>[MODBUS] %1</span>").arg(line);
    ui->plainLog->appendHtml(html);
}

void MainWindow::onLog(const QString& line, Common::LogLevel level)
{
    // Debug 레벨 필터링
    if (level == Common::LogLevel::Debug && !m_showDebugLogs)
        return;

    // QPlainTextEdit은 색상을 못 넣으니, 단순 접두어로 레벨 표기
    QString html;
    switch (level) {
    case Common::LogLevel::Debug:
        html = QString("<span style='color:gray;font-style:italic;'>[DBG] %1</span>").arg(line);
        break;
    case Common::LogLevel::Info:
        html = QString("<span style='color:black;'>%1</span>").arg(line);
        break;
    case Common::LogLevel::Warn:
        html = QString("<span style='color:orange;font-weight:bold;'>[WARN] %1</span>").arg(line);
        break;
    case Common::LogLevel::Error:
        html = QString("<span style='color:red;font-weight:bold;text-decoration:underline;'>[ERR] %1</span>").arg(line);
        break;
    }

    ui->plainLog->appendHtml(html);
}

void MainWindow::setFsmLedColor(const QString& name)
{
    // 색상 매핑
    // Idle / Ready / PublishTarget / WaitPickStart / WaitPickDone / WaitDoneClear
    QString color = "#9ca3af"; // 기본 회색 (Idle)
    if (name.contains("Ready",              Qt::CaseInsensitive))   color = "#3b82f6"; // 파랑
    else if (name.contains("Publish",       Qt::CaseInsensitive))   color = "#8b5cf6"; // 보라
    else if (name.contains("WaitPickStart", Qt::CaseInsensitive))   color = "#f59e0b"; // 주황
    else if (name.contains("WaitPickDone",  Qt::CaseInsensitive))   color = "#10b981"; // 초록(진행 완료 대기)
    else if (name.contains("WaitDoneClear", Qt::CaseInsensitive))   color = "#22c55e"; // 초록(클리어 대기)
    m_fsmLed->setStyleSheet(QString("background:%1;border-radius:6px;").arg(color));
}

void MainWindow::loadRobotsFromConfig()
{
    QFile f(":/config/robots.json");    // 리소스/파일 경로에 맞게 조정
    if (!f.open(QIODevice::ReadOnly))
    {
        //onLog("[ERR] robots.json open failed");
        qDebug()<<"[ERR] robots.json open failed";
        return;
    }
    qDebug()<<"robots.json open";
    const auto doc = QJsonDocument::fromJson(f.readAll());

    if (!doc.isObject())
        return;

    qDebug()<<"robots.json parsed";
    const auto arr = doc.object().value("robots").toArray();
    for (const auto& v : arr) {
        const auto o = v.toObject();
        const QString id   = o.value("id").toString();
        const QString host = o.value("host").toString();
        const int     port = o.value("port").toInt();
        const QString addr_map  = o.value("addr_map").toString();

        QVariantMap addr;
        QFile mf(addr_map);
        if (mf.open(QIODevice::ReadOnly)) {
            auto d = QJsonDocument::fromJson(mf.readAll());
            if (d.isObject())
            {
                addr = d.object().toVariantMap();
                qDebug()<<"Address map loaded from"<<addr_map;
            }
            else
            {
                qDebug()<<"Address map format error";
            }
        }
        if (id == "A") { m_panelA->setEndpoint(host, port, addr); m_panelA->setRobotId("A"); }
        if (id == "B") { m_panelB->setEndpoint(host, port, addr); m_panelB->setRobotId("B"); }

        onLog(QString("[OK] Robot %1 added (%2:%3)").arg(id, host).arg(port));
    }
}

void MainWindow::onRobotCommand(const RobotCommand& cmd)
{
    // dir 필터가 필요하면 여기서
    if (cmd.dir != 1)
        return;

    switch (cmd.robot) {
    case RobotId::A: handleRobotA(cmd); break;
    case RobotId::B: handleRobotB(cmd); break;
    default:
        onLog("Unknown robot command");
        break;
    }
}

void MainWindow::handleRobotA(const RobotCommand& cmd)
{
    if(cmd.type == CmdType::Tool) {
        if(cmd.kind == CmdKind::Tool_Mount) {
            if(cmd.toolCmd.toolName=="bulk") {
                onLog("로봇 A 벌크 툴 마운트 처리 필요\r\n");
                m_visionClient->sendAck(cmd.seq, "ok", "Tool Mount Bulk command received");
                m_mgr->cmdBulk_AttachTool();
            }
            else if(cmd.toolCmd.toolName=="sorting") {
                onLog("로봇 A 소팅 툴 마운트 처리 필요\r\n");
                m_visionClient->sendAck(cmd.seq, "ok", "Tool Mount Sorting command received");
                m_mgr->cmdSort_AttachTool();
            }
        }
        else if(cmd.kind == CmdKind::Tool_UnMount) {
            if(cmd.toolCmd.toolName=="bulk") {
                onLog("로봇 A 벌크 툴 언마운트 처리 필요\r\n");
                m_visionClient->sendAck(cmd.seq, "ok", "Tool UnMount Bulk command received");
                m_mgr->cmdBulk_DettachTool();
            }
            else if(cmd.toolCmd.toolName=="sorting") {
                onLog("로봇 A 소팅 툴 언마운트 처리 필요\r\n");
                m_visionClient->sendAck(cmd.seq, "ok", "Tool UnMount Sorting command received");
                m_mgr->cmdSort_DettachTool();
            }
        }
        else if(cmd.kind == CmdKind::Tool_Change) {
            if(cmd.toolCmd.toolFrom=="bulk" && cmd.toolCmd.toolTo=="sorting") {
                onLog("로봇 A 벌크→소팅 툴 체인지 처리 필요\r\n");
                m_visionClient->sendAck(cmd.seq, "ok", "Tool Change Bulk to Sorting command received");
                QTimer::singleShot(100, this, [this]() {
                    m_mgr->cmdBulk_ChangeTool();
                });

            }
            else if(cmd.toolCmd.toolFrom=="sorting" && cmd.toolCmd.toolTo=="bulk") {
                onLog("로봇 A 소팅→벌크 툴 체인지 처리 필요\r\n");
                m_visionClient->sendAck(cmd.seq, "ok", "Tool Change Sorting to Bulk command received");
                QTimer::singleShot(100, this, [this]() {
                    m_mgr->cmdSort_ChangeTool();
                });
            }
        }
    }
    else if(cmd.type == CmdType::Bulk) {
        switch (cmd.kind) {
        case CmdKind::Pick:
            if(cmd.hasPick)
            {
                onLog("Robot A 벌크 픽 처리 필요, pose:"
                       + QString("[%1,%2,%3] [%4,%5,%6]")
                         .arg(cmd.pick.x).arg(cmd.pick.y).arg(cmd.pick.z)
                         .arg(cmd.pick.rx).arg(cmd.pick.ry).arg(cmd.pick.rz));
                m_visionClient->sendAck(cmd.seq, "ok", "bulk Pick command received");
                const int mode = (cmd.mode=="single")? 2 : 0;

                m_mgr->cmdBulk_DoPickup(cmd.pick, mode);
            }

            break;
        case CmdKind::Place:
            if(cmd.hasPlace)
            {
                onLog("Robot A 벌크 플레이스 처리 필요, pose:"
                       + QString("[%1,%2,%3] [%4,%5,%6]")
                         .arg(cmd.place.x).arg(cmd.place.y).arg(cmd.place.z)
                         .arg(cmd.place.rx).arg(cmd.place.ry).arg(cmd.place.rz));
                m_visionClient->sendAck(cmd.seq, "ok", "bulk Place command received");
                const int mode = (cmd.mode=="single")? 2 : 0;

                m_mgr->cmdBulk_DoPlace(cmd.place, mode);
            }

            break;
        default:
            break;
        }
    }
    else if (cmd.type == CmdType::Sorting) {
        switch (cmd.kind) {

        case CmdKind::Ready:
            onLog("로봇 A 소팅 레디 처리 필요\r\n");
            m_visionClient->sendAck(cmd.seq, "ok", "sorting Ready command received");
            m_mgr->cmdSort_MoveToPickupReady();
            break;
        case CmdKind::Pick:
            if (cmd.hasPick && cmd.isOffset) {
                // cmd.pick 사용
                onLog(QString("로봇 A 소팅 픽 처리 필요, pose: [%1,%2,%3] [%4,%5,%6]\r\n")
                               .arg(cmd.pick.x).arg(cmd.pick.y).arg(cmd.pick.z).arg(cmd.pick.rx).arg(cmd.pick.ry).arg(cmd.pick.rz));

                if(cmd.flip) {
                    m_gentryMgr->startGantryMove();
                    m_gentryMgr->setFalgs(false, false, true, false, false);
                    m_gentryMgr->gentry_motion();
                    m_gantryPickupState = true;
                }
                else {
                    if(!m_gantryPickupState)
                    {
                        m_gentryMgr->startGantryMove();
                        m_gentryMgr->setFalgs(false, true, false, false, false);
                        m_gentryMgr->gentry_motion();
                    }
                }
                m_visionClient->sendAck(cmd.seq, "ok", "sorting Pick command received");
                m_sortingFlip = cmd.flip;
                m_sortingOffset = cmd.sortOffset.height;
                m_sortingThick= cmd.sortOffset.thickness;
//                m_sortingShfit = cmd.sortOffset.shift;

//                Pose6D pick_pose =  cmd.pick;
                m_mgr->cmdSort_DoPickup(cmd.pick,m_sortingFlip, m_sortingOffset, m_sortingThick);
            }
            break;
        case CmdKind::Place:
            if (cmd.flip) {
                // offset: 로봇 후퇴거리/미사용
                onLog("로봇 A 소팅 플립 처리 필요\r\n");
                m_visionClient->sendAck(cmd.seq, "ok", "sorting Place Flip command received");
////////////////////////////////////////////////////////////////
                m_sortingPlacePorcessActive = true;
                m_sortingFlip = true;
                m_sortingOffset=cmd.sortOffset.height; //cmd.offset;
                m_sortingThick=cmd.sortOffset.thickness;
//                m_sortingShfit = cmd.sortOffset.shift;

                m_gentryMgr->startGantryMove();
                int offset_mm = (m_sortingOffset>30)? (m_sortingOffset-30+10) : 0;
                m_sortingShfit=m_sortingThick;
                m_gentryMgr->doGentryPlace(m_sortingShfit, offset_mm);
////////////////////////////////////////////////////////////////
            } else {
                // offset: 겐트리와 컨베어간의 높이차이 -> Z축 환산필요
                onLog(QString("로봇 A 소팅 논플립 처리 필요, offset: %1\r\n")
                               .arg(cmd.offset));
                m_visionClient->sendAck(cmd.seq, "ok", "sorting Place Non-Flip command received");
//                m_mgr->cmdSort_DoPlace(cmd.flip, cmd.offset);
////////////////////////////////////////////////////////////////
                m_sortingPlacePorcessActive = true;
                m_sortingFlip =false;
                m_sortingOffset=cmd.sortOffset.height;//cmd.offset;
                m_sortingThick=cmd.sortOffset.thickness;
//                m_sortingShfit = cmd.sortOffset.shift;

                m_gentryMgr->startGantryMove();
                m_gentryMgr->setFalgs(false, true, false, false, false);
                m_gentryMgr->gentry_motion();
////////////////////////////////////////////////////////////////
            }
            break;

        case CmdKind::Arrange: {
            if(cmd.isArrange)
            {
/*
                onLog("로봇 A 소팅 어레인지 처리 필요, poseOrig:"
                       + QString("[%1,%2,%3] [%4,%5,%6]")
                         .arg(cmd.arrangeCmd.poseOrig.x).arg(cmd.arrangeCmd.poseOrig.y).arg(cmd.arrangeCmd.poseOrig.z)
                         .arg(cmd.arrangeCmd.poseOrig.rx).arg(cmd.arrangeCmd.poseOrig.ry).arg(cmd.arrangeCmd.poseOrig.rz)
                       + QString(" -> poseDest:")
                       + QString("[%1,%2,%3] [%4,%5,%6]")
                         .arg(cmd.arrangeCmd.poseDest.x).arg(cmd.arrangeCmd.poseDest.y).arg(cmd.arrangeCmd.poseDest.z)
                         .arg(cmd.arrangeCmd.poseDest.rx).arg(cmd.arrangeCmd.poseDest.ry).arg(cmd.arrangeCmd.poseDest.rz));
                m_visionClient->sendAck(cmd.seq, "ok", "sorting Arrange command received");
*/
                m_mgr->cmdSort_Arrange(cmd.arrangeCmd.poseOrig, cmd.arrangeCmd.poseDest);

            }
        } break;
        default:
            break;
        }
    } else if (cmd.type == CmdType::Conveyor) {
        if (cmd.kind == CmdKind::Forward) {
            onLog("로봇 A 컨베이어 포워드 처리 필요\r\n");
            qInfo()<<QDateTime::currentDateTime()<<"KJW"<<"Robot A Conveyor Forward command received";

            m_visionClient->sendAck(cmd.seq, "ok", "conveyor Forward command received");
            QTimer::singleShot(1000, this, [this]{
                m_gentryMgr->startConveyorMove();
                m_gentryMgr->doConveyorForwardOneStep();
//                m_gentryMgr->setFalgs(true, false, false, false, false);
//                m_gentryMgr->gentry_motion();
            });
        }
    }
}

void MainWindow::handleRobotB(const RobotCommand& cmd)
{
    if (cmd.type != CmdType::Align)
        return;

    switch (cmd.kind) {
    case CmdKind::Init:  // TODO: 초기화 명령 처리
        onLog("로봇 B 얼라인 이닛 처리 필요\r\n");
        m_visionClient->sendAck(cmd.seq, "ok", "align Init command received");
        m_mgr->cmdAlign_Initialize();

        break;
    case CmdKind::Assy:  // TODO: 어셈블리 명령 처리
        onLog("로봇 B 얼라인 어셈블리 처리 필요\r\n");
        m_visionClient->sendAck(cmd.seq, "ok", "align Assy command received");
        m_mgr->cmdAlign_MoveToAssyReady();
        break;
    case CmdKind::Ready: // TODO: 레디 명령 처리
        onLog("로봇 B 얼라인 레디 처리 필요\r\n");
        m_visionClient->sendAck(cmd.seq, "ok", "align Ready command received");
        m_mgr->cmdAlign_MoveToPickupReady();
        break;
    case CmdKind::Pick:  // TODO: 픽 명령 처리
        if (cmd.hasPick) {
            // cmd.pick 사용
            onLog(QString("로봇 B 얼라인 픽 처리 필요, pose: [%1,%2,%3] [%4,%5,%6]\r\n")
                           .arg(cmd.pick.x).arg(cmd.pick.y).arg(cmd.pick.z).arg(cmd.pick.rx).arg(cmd.pick.ry).arg(cmd.pick.rz));
            m_visionClient->sendAck(cmd.seq, "ok", "align Pick command received");
            m_mgr->cmdAlign_DoPickup(cmd.pick);
        }
        break;
    case CmdKind::Place: // TODO: 플레이스 명령 처리
        if (cmd.hasPlace) {
            // cmd.place 사용
            onLog(QString("로봇 B 얼라인 플레이스 처리 필요, pose: [%1,%2,%3] [%4,%5,%6]\r\n")
                           .arg(cmd.place.x).arg(cmd.place.y).arg(cmd.place.z).arg(cmd.place.rx).arg(cmd.place.ry).arg(cmd.place.rz));
            m_visionClient->sendAck(cmd.seq, "ok", "align Place command received");
            m_mgr->cmdAlign_DoPlace(cmd.place);
        }
        break;
    case CmdKind::Clamp: // 2025-12-11: desperate!
        if (cmd.clamp == "open") {
            onLog("로봇 B 클램프 오픈 처리 필요\r\n");
            qDebug()<<"25-11-24: Robot B Clamp Open command received";
            m_visionClient->sendAck(cmd.seq, "ok", "align Clamp Open command received");
            m_mgr->cmdAlign_Clamp(false);
        } else if (cmd.clamp == "close") {
            onLog("로봇 B 클램프 클로즈 처리 필요\r\n");
            m_visionClient->sendAck(cmd.seq, "ok", "align Clamp Close command received");
            m_mgr->cmdAlign_Clamp(true);
        }
        break;
    case CmdKind::Scrap: // 2025-12-11: desperate!
        onLog("로봇 B 스크랩 처리 필요\r\n");
        m_visionClient->sendAck(cmd.seq, "ok", "align Scrap command received");
        m_mgr->cmdAlign_Scrap();
        break;
    default:
        break;
    }
}

void MainWindow::on_btnConnect_clicked()
{
    QString IP = ui->lineEdit_IP->text();
    quint16 port = static_cast<quint16>(ui->spinBox_Port->value());

    m_visionClient->connectTo(IP, port);
}


void MainWindow::on_btnDisconnect_clicked()
{
    m_visionClient->disconnectFrom();
}


void MainWindow::on_pushButton_Atuo_clicked()
{
    m_mgr->setAutoMode("A", true);
}


void MainWindow::on_pushButton_Manual_clicked()
{
    m_mgr->setAutoMode("A", false);
}


void MainWindow::on_pushButton_Start_clicked()
{
    m_mgr->startMainProgram("A");
}


void MainWindow::on_pushButton_Stop_clicked()
{
    m_mgr->stopMainProgram("A");
}


void MainWindow::on_pushButton_Pause_clicked()
{

}

