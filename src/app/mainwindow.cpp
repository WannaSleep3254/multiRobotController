#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include "RobotManager.h"

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

#include "widgets/RobotPanel.h"
#include <QSplitter>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    setWindowTitle("BinPicking Modbus Controller");

    m_mgr = new RobotManager(this);
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

    m_split  = new QSplitter(Qt::Horizontal, this);
    m_panelA = new RobotPanel(this);
    m_panelB = new RobotPanel(this);
    m_panelC = new RobotPanel(this);

    m_panelA->setManager(m_mgr);
    m_panelB->setManager(m_mgr);
    m_panelC->setManager(m_mgr);

    m_panelA->setRobotId("A");
    m_panelB->setRobotId("B");
    m_panelC->setRobotId("C");

    m_split->addWidget(m_panelA);
    m_split->addWidget(m_panelB);
    m_split->addWidget(m_panelC);

    ui->horizontalLayout->addWidget(m_split);

    QTimer::singleShot(0, this, [this]{ loadRobotsFromConfig(); });
    initVisionServer();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::initVisionServer()
{
    // 1) VisionServer 생성
    m_visionServer = new VisionServer(this);

    // 2) 서버 옵션 설정
    VisionServer::Options opt;
    opt.enforceWhitelist = true;
    opt.whitelistIPs = {"192.168.57.100", "127.0.0.1"};
    opt.maxConnections = 8;
    opt.ratePerSec = 50;   // 초당 50 라인 제한
    opt.rateBurst = 100;   // 최대 버스트 100라인
    m_visionServer->setOptions(opt);

    // 3) 토큰 (선택사항)
    m_visionServer->setAuthToken("ccNC2025");

    // 4) 서버 시작
    if (m_visionServer->start(54615))
        onLog("[VS] VisionServer started on port 5555");
    else
        onLog("[VS] VisionServer failed to start");


    // 7) 좌표 수신 시 로봇으로 발행
    connect(m_visionServer, &VisionServer::poseReceived, this,
            [this](const QString& robot, const Pose6D& p, quint32 seq, const QVariantMap& ex){
                Q_UNUSED(robot); Q_UNUSED(seq);
                const int speed = ex.value("speed_pct", 50).toInt();

                Pose6D pose{p.x, p.y, p.z, p.rx, p.ry, p.rz};
                qDebug()<<"[VS] Pose received for robot"<<robot
                       <<"("<<pose.x<<pose.y<<pose.z<<pose.rx<<pose.ry<<pose.rz<<")"
                      <<"speed_pct="<<speed;
                // 로봇 매니저에 발행
                m_mgr->enqueuePose(robot, pose);
            });

    // 8) 다건 좌표 수신 (예: Vision이 여러 픽 포인트를 한번에 전달)
}

void MainWindow::onHeartbeat(bool ok)
{
    ui->ledConnection->setStyleSheet(ok ? "background:#22c55e;border-radius:6px;" : "background:#ef4444;border-radius:6px;");
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
        const QString poseCsv = o.value("pose_csv").toString();
        if (!poseCsv.isEmpty()) {
            QString err;
            m_mgr->loadCsvToModel(id, poseCsv, &err, this);  // ★ 컨텍스트/모델 자동 준비 + 로드
            if (!err.isEmpty()) qDebug() << "[WARN] CSV load:" << err;
        }
        onLog(QString("[OK] Robot %1 added (%2:%3)").arg(id, host).arg(port));
    }
}

