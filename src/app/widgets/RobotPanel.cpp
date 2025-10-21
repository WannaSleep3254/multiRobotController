#include "RobotPanel.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTableView>
#include <QPushButton>
#include <QCheckBox>
#include <QLabel>
#include <QAbstractItemView>
#include <QHeaderView>
#include <QPlainTextEdit>
#include <QFileDialog>

#include "RobotManager.h"

static QLabel* makeLed(QWidget* parent) {
    auto* led = new QLabel(parent);
    led->setFixedSize(12,12);
    led->setStyleSheet("background:#ef4444;border-radius:6px;"); // red
    return led;
}

RobotPanel::RobotPanel(QWidget* parent) : QWidget(parent)
{
    m_table = new QTableView(this);
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setSelectionMode(QAbstractItemView::SingleSelection);
    m_table->horizontalHeader()->setStretchLastSection(true);

    m_btnConnect    = new QPushButton("Connect", this);
    m_btnDisconnect = new QPushButton("Disconnect", this);
    m_btnStart      = new QPushButton("Start", this);
    m_btnStop       = new QPushButton("Stop", this);
    m_btnLoadCsv    = new QPushButton("Load CSV", this);
    m_btnClear      = new QPushButton("Clear", this);
    m_chkRepeat     = new QCheckBox("Repeat targets", this);
    m_led           = makeLed(this);
    m_chkVisionMode = new QCheckBox("Vision", this);

    m_logView = new QPlainTextEdit(this);   // ★
    m_logView->setReadOnly(true);
    m_logView->setMaximumBlockCount(2000);  // 메모리 제한 (필요시)

    auto* row1 = new QHBoxLayout;
    row1->addWidget(m_btnConnect);
    row1->addWidget(m_btnDisconnect);
    row1->addStretch();
    row1->addWidget(new QLabel("Conn:", this));
    row1->addWidget(m_led);

    auto* row2 = new QHBoxLayout;
    row2->addWidget(m_btnStart);
    row2->addWidget(m_btnStop);
    row2->addSpacing(12);
    row2->addWidget(m_btnLoadCsv);   // ★
    row2->addWidget(m_btnClear);     // ★
    row2->addStretch();
    row2->addWidget(m_chkRepeat);
    row2->addSpacing(12);
    row2->addWidget(m_chkVisionMode); // ✅

    auto* lay = new QVBoxLayout(this);
    lay->addLayout(row1);
    lay->addLayout(row2);
    lay->addWidget(m_table);
    lay->addWidget(m_logView);              // ★ 테이블 아래에 로그창
    setLayout(lay);

    connect(m_btnConnect,    &QPushButton::clicked, this, &RobotPanel::onConnect);
    connect(m_btnDisconnect, &QPushButton::clicked, this, &RobotPanel::onDisconnect);
    connect(m_btnStart,      &QPushButton::clicked, this, &RobotPanel::onStart);
    connect(m_btnStop,       &QPushButton::clicked, this, &RobotPanel::onStop);
    connect(m_chkRepeat,     &QCheckBox::toggled,   this, &RobotPanel::onRepeatToggled);

    connect(m_btnLoadCsv,    &QPushButton::clicked, this, &RobotPanel::onLoadCsv); // ★
    connect(m_btnClear,      &QPushButton::clicked, this, &RobotPanel::onClear);   // ★

    // ✅ 비전 모드 토글 → 매니저로 반영
    connect(m_chkVisionMode, &QCheckBox::toggled, this, [this](bool on){
        if (m_mgr && !m_id.isEmpty()) m_mgr->setVisionMode(m_id, on);
    });
}

void RobotPanel::setManager(RobotManager* mgr)
{
    if (m_mgr == mgr) return;
    m_mgr = mgr;

    if (!m_mgr)
        return;

    // RobotManager가 heartbeat 신호를 리레이하도록 만들어뒀다면 여기에 연결
    connect(m_mgr, &RobotManager::heartbeat, this,
            [this](const QString& rid, bool ok){
                if (rid == m_id) onHeartbeat(ok);
            });
#if false
    connect(m_mgr, &RobotManager::currentRowChanged, this,
            [this](const QString& /*id*/, int row){
                if(row > 0)
                    m_table->scrollTo(m_mgr->model(m_id)->index(row,0), QAbstractItemView::PositionAtCenter);
            });
#else
    connect(m_mgr, &RobotManager::currentRowChanged, this,
            [this](const QString& rid, int row){
                if (rid != m_id) return;                 // ★ 내 패널만
                if (row < 0) return;
                auto* mdl = m_mgr->model(m_id);
                if (!mdl) return;                        // ★ 널 가드
                m_table->scrollTo(mdl->index(row,0), QAbstractItemView::PositionAtCenter);
            });
#endif
    connect(m_mgr, &RobotManager::connectionChanged, this,
            [this](const QString& rid, bool up){
                if (rid == m_id) onHeartbeat(up);
            });

    connect(m_mgr, &RobotManager::logByRobot, this,
            [this](const QString& rid, const QString& line, Common::LogLevel lv){
                if (rid == m_id) appendLog(line, lv);
            });

    if (!m_id.isEmpty())
        onHeartbeat(m_mgr->isConnected(m_id));

    bindModel();
}

void RobotPanel::setRobotId(const QString& id)
{
    if (m_id == id) return;
    m_id = id;
    bindModel();
    if (m_mgr)
        onHeartbeat(m_mgr->isConnected(m_id));  // 즉시 LED 동기화
}

void RobotPanel::setEndpoint(const QString& host, int port, const QVariantMap& addr) {
    m_host = host; m_port = port; m_addr = addr;
}

void RobotPanel::bindModel()
{
    if (!m_mgr || m_id.isEmpty())
    {
        qDebug()<<"No manager or ID";
        return;
    }
    qDebug()<<"Binding model for robot"<<m_id;

    auto* model = m_mgr->model(m_id);
    m_table->setModel(static_cast<QAbstractItemModel*>(model));
}

void RobotPanel::appendLog(const QString& line, Common::LogLevel lv)
{
    QString prefix;
    switch (lv) {
    case Common::LogLevel::Warn:  prefix = "[WARN] ";
        break;
    case Common::LogLevel::Error: prefix = "[ERR] ";
        break;
    case Common::LogLevel::Debug: prefix = "[DBG] ";
        break;
    default:
        break;
    }
    m_logView->appendPlainText(prefix + line);
}

void RobotPanel::onConnect()
{
    if (!m_mgr || m_id.isEmpty() || m_host.isEmpty() || m_port <= 0)
        return;

    m_mgr->addOrConnect(m_id, m_host, m_port, m_addr, this);
    bindModel(); // 보장용
}

void RobotPanel::onDisconnect()
{
    if (m_mgr && !m_id.isEmpty())
        m_mgr->disconnect(m_id);
}

void RobotPanel::onStart()
{
    qDebug()<<"RobotPanel::onStart() for robot"<<m_id;
    if (m_mgr && !m_id.isEmpty())
        m_mgr->start(m_id);
}

void RobotPanel::onStop()
{
    if (m_mgr && !m_id.isEmpty())
        m_mgr->stop(m_id);
}

void RobotPanel::onRepeatToggled(bool on)
{
    if (m_mgr && !m_id.isEmpty())
        m_mgr->setRepeat(m_id, on);
}

void RobotPanel::onHeartbeat(bool ok)
{
    m_led->setStyleSheet(ok ? "background:#22c55e;border-radius:6px;"
                            : "background:#ef4444;border-radius:6px;");
}

void RobotPanel::onLoadCsv()
{
    if (!m_mgr || m_id.isEmpty()) return;
    QString path = QFileDialog::getOpenFileName(this, "Load pose CSV", QString(),
                                                "CSV/TXT (*.csv *.txt);;All (*.*)");
    if (path.isEmpty()) return;
    QString err;
    if (!m_mgr->loadCsvToModel(m_id, path, &err)) {
        appendLog(QString("CSV load failed: %1").arg(err), Common::LogLevel::Error);
        return;
    }
    bindModel(); // 새 모델 데이터 반영
}

void RobotPanel::onClear()
{
    if (!m_mgr || m_id.isEmpty()) return;
    m_mgr->clearPoseList(m_id);
}
