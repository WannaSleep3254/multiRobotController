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

    connect(m_mgr, &RobotManager::bulkProcessFinished, this,
            [this](const QString& robotId){
//                if (robotId == "A")
//                    bulkReady();
            });

    connect(m_mgr, &RobotManager::reqGentryPalce, this, [this]{
            // TODO Gentry Place 동작 시작
        qDebug()<<QDateTime::currentDateTime()<<"MainWindow::reqGentryPlace";
        m_gentryMgr->startGantryMove();
        m_gentryMgr->doGentryPlace();

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
            [this](bool ok){
                Q_UNUSED(ok)
                qDebug()<<QString("Gentry gantry command finished: %1").arg(ok?"OK":"FAIL");
                bool flip = m_sortingFlip;
                int  offset = m_sortingOffset;
                bool isDock = m_sortingDock;

                onLog(QString("m_flip: %1, offset: %2").arg(m_sortingFlip?"OK":"FAIL", offset));
                onLog(QString("flip: %1, offset; %2").arg(flip?"OK":"FAIL", m_sortingOffset));

//                if(ok && m_sortingPlacePorcessActive&& !m_sortingDock){
                if(m_sortingPlacePorcessActive&& !m_sortingDock){
                    if(!flip)
                    {   //TODO robot->place
                        qDebug()<<"Robot place without flip";
                        m_mgr->cmdSort_DoPlace(flip, offset);
                    }
                    else if(flip)
                    {   //TODO robot->dock -> ready
                        qDebug()<<"Robot place with flip - dock first";
                        m_sortingDock = true;
                        m_mgr->cmdSort_DoPlace(flip, offset);
                    }
                }
//                else if(ok && m_sortingPlacePorcessActive&& m_sortingDock){
                else if(m_sortingPlacePorcessActive&& m_sortingDock){
                    // 겐트리 Tool Off
                    m_sortingDock = false;
                    m_mgr->cmdSort_GentryTool(false);
                }
            });


    connect(m_gentryMgr, &GentryManager::conveyorCommandFinished, this,
            [this](bool ok){
                Q_UNUSED(ok)
                qDebug()<<QString("Conveyor command finished: %1").arg(ok?"OK":"FAIL");
                m_visionClient->sendWorkComplete("a", "conveyor", "forward", 0);
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
    if (cmd.type == CmdType::Sorting) {
        switch (cmd.kind) {
        case CmdKind::Tool:
            onLog("로봇 A 소팅 툴 처리 필요\r\n");
            m_visionClient->sendAck(0, "ok", "sorting Tool command received");
            m_mgr->cmdSort_AttachTool();
            break;
        case CmdKind::Ready:
            onLog("로봇 A 소팅 레디 처리 필요\r\n");
            m_visionClient->sendAck(0, "ok", "sorting Ready command received");
            m_mgr->cmdSort_MoveToPickupReady();
            break;
        case CmdKind::Pick:
            if (cmd.hasPick) {
                // cmd.pick 사용
                onLog(QString("로봇 A 소팅 픽 처리 필요, pose: [%1,%2,%3] [%4,%5,%6]\r\n")
                               .arg(cmd.pick.x).arg(cmd.pick.y).arg(cmd.pick.z).arg(cmd.pick.rx).arg(cmd.pick.ry).arg(cmd.pick.rz));
                m_visionClient->sendAck(0, "ok", "sorting Pick command received");
                m_mgr->cmdSort_DoPickup(cmd.pick);
            }
            break;
        case CmdKind::Place:
            if (cmd.flip) {
                // offset: 로봇 후퇴거리/미사용
                onLog("로봇 A 소팅 플립 처리 필요\r\n");
                m_visionClient->sendAck(0, "ok", "sorting Place Flip command received");
//                m_mgr->cmdSort_DoPlace(cmd.flip, cmd.offset);
////////////////////////////////////////////////////////////////
                m_sortingPlacePorcessActive = true;
                m_sortingFlip = true;
                m_sortingOffset=cmd.offset;
                m_gentryMgr->startGantryMove();
                m_gentryMgr->setFalgs(false, false, true, false, false);
                m_mgr->cmdSort_GentryTool(true);
                m_gentryMgr->gentry_motion();
////////////////////////////////////////////////////////////////
            } else {
                // offset: 겐트리와 컨베어간의 높이차이 -> Z축 환산필요
                onLog(QString("로봇 A 소팅 논플립 처리 필요, offset: %1\r\n")
                               .arg(cmd.offset));
                m_visionClient->sendAck(0, "ok", "sorting Place Non-Flip command received");
//                m_mgr->cmdSort_DoPlace(cmd.flip, cmd.offset);
////////////////////////////////////////////////////////////////
                m_sortingPlacePorcessActive = true;
                m_sortingFlip =false;
                m_sortingOffset=cmd.offset;
                m_gentryMgr->startGantryMove();
                m_gentryMgr->setFalgs(false, true, false, false, false);
                m_gentryMgr->gentry_motion();
////////////////////////////////////////////////////////////////
            }
            break;
        default:
            break;
        }
    } else if (cmd.type == CmdType::Conveyor) {
        if (cmd.kind == CmdKind::Forward) {
            onLog("로봇 A 컨베이어 포워드 처리 필요\r\n");
            m_visionClient->sendAck(0, "ok", "conveyor Forward command received");
            //m_mgr->cmdSort_MoveToConveyor();
            m_gentryMgr->startConveyorMove();
            m_gentryMgr->setFalgs(true, false, false, false, false);
            m_gentryMgr->gentry_motion();
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
        m_visionClient->sendAck(0, "ok", "align Init command received");
        m_mgr->cmdAlign_Initialize();

        break;
    case CmdKind::Assy:  // TODO: 어셈블리 명령 처리
        onLog("로봇 B 얼라인 어셈블리 처리 필요\r\n");
        m_visionClient->sendAck(0, "ok", "align Assy command received");
        m_mgr->cmdAlign_MoveToAssyReady();
        break;
    case CmdKind::Ready: // TODO: 레디 명령 처리
        onLog("로봇 B 얼라인 레디 처리 필요\r\n");
        m_visionClient->sendAck(0, "ok", "align Ready command received");
        m_mgr->cmdAlign_MoveToPickupReady();
        break;
    case CmdKind::Pick:  // TODO: 픽 명령 처리
        if (cmd.hasPick) {
            // cmd.pick 사용
            onLog(QString("로봇 B 얼라인 픽 처리 필요, pose: [%1,%2,%3] [%4,%5,%6]\r\n")
                           .arg(cmd.pick.x).arg(cmd.pick.y).arg(cmd.pick.z).arg(cmd.pick.rx).arg(cmd.pick.ry).arg(cmd.pick.rz));
            m_visionClient->sendAck(0, "ok", "align Pick command received");
            m_mgr->cmdAlign_DoPickup(cmd.pick);
        }
        break;
    case CmdKind::Place: // TODO: 플레이스 명령 처리
        if (cmd.hasPlace) {
            // cmd.place 사용
            onLog(QString("로봇 B 얼라인 플레이스 처리 필요, pose: [%1,%2,%3] [%4,%5,%6]\r\n")
                           .arg(cmd.place.x).arg(cmd.place.y).arg(cmd.place.z).arg(cmd.place.rx).arg(cmd.place.ry).arg(cmd.place.rz));
            m_visionClient->sendAck(0, "ok", "align Place command received");
            m_mgr->cmdAlign_DoPlace(cmd.place);
        }
        break;
    case CmdKind::Clamp:
        if (cmd.clamp == "open") {
            onLog("로봇 B 클램프 오픈 처리 필요\r\n");
            qDebug()<<"25-11-24: Robot B Clamp Open command received";
            m_visionClient->sendAck(0, "ok", "align Clamp Open command received");
            m_mgr->cmdAlign_Clamp(false);
        } else if (cmd.clamp == "close") {
            onLog("로봇 B 클램프 클로즈 처리 필요\r\n");
            m_visionClient->sendAck(0, "ok", "align Clamp Close command received");
            m_mgr->cmdAlign_Clamp(true);
        }
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

