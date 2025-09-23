#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include "RobotManager.h"

#include <QFile>
#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMessageBox>
#include <QTimer>
#include <QDebug>
#include <QStatusBar>
#include <QLabel>
#include <QFrame>
#include <QCheckBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    setWindowTitle("BinPicking Modbus Controller");

    // --- 상태 표시줄에 FSM 상태 표시 ---
    m_fsmLed = new QFrame(this);
    m_fsmLed->setFixedSize(12,12);
    m_fsmLed->setFrameShape(QFrame::NoFrame);
    m_fsmLed->setStyleSheet("background:#9ca3af;border-radius:6px;");

    m_fsmLabel = new QLabel("FSM: Idle", this);
    m_fsmLabel->setMinimumWidth(220);
    m_fsmLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);

    m_chkShowDebug = new QCheckBox("Debug logs", this);
    m_chkShowDebug->setChecked(false);

    statusBar()->addPermanentWidget(m_fsmLed);
    statusBar()->addPermanentWidget(m_fsmLabel);
    statusBar()->addPermanentWidget(m_chkShowDebug);

    connect(m_chkShowDebug, &QCheckBox::toggled, this, [this](bool on){
        m_showDebugLogs = on;
        if (on) ui->plainLog->appendPlainText("[UI] Debug logs: ON");
        else    ui->plainLog->appendPlainText("[UI] Debug logs: OFF");
    });
    m_showDebugLogs = m_chkShowDebug->isChecked();

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
    connect(ui->btnConnect, &QPushButton::clicked, this, &MainWindow::onConnect);
    connect(ui->btnDisconnect, &QPushButton::clicked, this, &MainWindow::onDisconnect);
    connect(ui->btnStart, &QPushButton::clicked, this, &MainWindow::onStart);
    connect(ui->btnStop, &QPushButton::clicked, this, &MainWindow::onStop);

    // --- ModbusClient 및 Orchestrator 설정 ---
    loadAddressMap();

    connect(m_mgr, &RobotManager::heartbeat, this, &MainWindow::onHeartbeat);
    connect(m_mgr, &RobotManager::stateChanged, this,
            [this](const QString& /*id*/, int /*state*/, const QString& name){
                m_fsmLabel->setText(QString("FSM: %1").arg(name));
                setFsmLedColor(name);
                if(ui->plainLog)
                    ui->plainLog->appendPlainText(QString("[FSM] %1").arg(name));
            });
    connect(m_mgr, &RobotManager::currentRowChanged, this,
            [this](const QString& /*id*/, int row){
                if(row > 0)
                    ui->tableView->scrollTo(m_mgr->model("A")->index(row,0), QAbstractItemView::PositionAtCenter);
            });

    // UI에 체크박스 추가(디자이너에서 추가해도 됨)
    QCheckBox* chkRepeat = new QCheckBox("Repeat targets", this);
    statusBar()->addPermanentWidget(chkRepeat);
    connect(chkRepeat, &QCheckBox::toggled, this, [this](bool on){
        if (m_mgr) {
            m_mgr->setRepeat("A", on);
        }
        ui->plainLog->appendHtml(QString("<span style='color:blue;'>[UI] Repeat: %1</span>").arg(on ? "ON":"OFF"));
    });
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::loadAddressMap()
{
    QFile f(":map/AddressMap.json");
    if(!f.open(QIODevice::ReadOnly)) {
        onLog("AddressMap.json not found, using defaults.");
        return;
    }
    else {
        onLog("AddressMap.json loaded.");
    }

    const auto doc = QJsonDocument::fromJson(f.readAll());
    m_addr = doc.object().toVariantMap();
    if(!m_addr.contains("meta")) {
        onLog("AddressMap.json format error, using defaults.");
    }
    else {
        int schema = m_addr.value("meta").toMap().value("schema").toInt();
        if (schema!=3) {
            onLog(QString("AddressMap.json schema version mismatch(%1), using defaults.").arg(schema));
        }
    }
}


void MainWindow::onConnect()
{
    const QString host = ui->editHost->text();
    const int port = ui->spinPort->value();

    m_mgr->addRobot("A", host, port, m_addr, this);
    ui->tableView->setModel(m_mgr->model("A"));
    onLog(QString("Connected to %1:%2").arg(host).arg(port));
}


void MainWindow::onDisconnect()
{
    if(m_mgr) {
        m_mgr->disconnect("A");
    }
}

void MainWindow::onStart()
{
    if(m_mgr) {
        m_mgr->start("A");
    }
}

void MainWindow::onStop()
{
    if(m_mgr) {
        m_mgr->stop("A");
    }
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
    if (name.contains("Ready", Qt::CaseInsensitive))        color = "#3b82f6"; // 파랑
    else if (name.contains("Publish", Qt::CaseInsensitive))  color = "#8b5cf6"; // 보라
    else if (name.contains("WaitPickStart", Qt::CaseInsensitive)) color = "#f59e0b"; // 주황
    else if (name.contains("WaitPickDone", Qt::CaseInsensitive))  color = "#10b981"; // 초록(진행 완료 대기)
    else if (name.contains("WaitDoneClear", Qt::CaseInsensitive)) color = "#22c55e"; // 초록(클리어 대기)
    m_fsmLed->setStyleSheet(QString("background:%1;border-radius:6px;").arg(color));
}
