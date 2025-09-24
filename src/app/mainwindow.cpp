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
    m_panelA->setManager(m_mgr);
    m_panelB->setManager(m_mgr);
    m_panelA->setRobotId("A");
    m_panelB->setRobotId("B");

    m_split->addWidget(m_panelA);
    m_split->addWidget(m_panelB);

    ui->horizontalLayout->addWidget(m_split);

    QTimer::singleShot(0, this, [this]{ loadRobotsFromConfig(); });
}

MainWindow::~MainWindow()
{
    delete ui;
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
        onLog(QString("[OK] Robot %1 added (%2:%3)").arg(id, host).arg(port));
    }
}

