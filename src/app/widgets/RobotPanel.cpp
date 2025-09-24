#include "RobotPanel.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTableView>
#include <QPushButton>
#include <QCheckBox>
#include <QLabel>
#include <QAbstractItemView>
#include <QHeaderView>
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
    m_chkRepeat     = new QCheckBox("Repeat targets", this);
    m_led           = makeLed(this);

    auto* row1 = new QHBoxLayout;
    row1->addWidget(m_btnConnect);
    row1->addWidget(m_btnDisconnect);
    row1->addStretch();
    row1->addWidget(new QLabel("Conn:", this));
    row1->addWidget(m_led);

    auto* row2 = new QHBoxLayout;
    row2->addWidget(m_btnStart);
    row2->addWidget(m_btnStop);
    row2->addStretch();
    row2->addWidget(m_chkRepeat);

    auto* lay = new QVBoxLayout(this);
    lay->addLayout(row1);
    lay->addLayout(row2);
    lay->addWidget(m_table);
    setLayout(lay);

    connect(m_btnConnect,    &QPushButton::clicked, this, &RobotPanel::onConnect);
    connect(m_btnDisconnect, &QPushButton::clicked, this, &RobotPanel::onDisconnect);
    connect(m_btnStart,      &QPushButton::clicked, this, &RobotPanel::onStart);
    connect(m_btnStop,       &QPushButton::clicked, this, &RobotPanel::onStop);
    connect(m_chkRepeat,     &QCheckBox::toggled,   this, &RobotPanel::onRepeatToggled);
}

void RobotPanel::setManager(RobotManager* mgr)
{
    if (m_mgr == mgr) return;
    m_mgr = mgr;
    if (m_mgr) {
        // RobotManager가 heartbeat 신호를 리레이하도록 만들어뒀다면 여기에 연결
        connect(m_mgr, SIGNAL(heartbeat(bool)), this, SLOT(onHeartbeat(bool)));
        connect(m_mgr, &RobotManager::currentRowChanged, this,
                [this](const QString& /*id*/, int row){
                    if(row > 0)
                        m_table->scrollTo(m_mgr->model(m_id)->index(row,0), QAbstractItemView::PositionAtCenter);
                });
    }
    bindModel();
}

void RobotPanel::setRobotId(const QString& id)
{
    if (m_id == id) return;
    m_id = id;
    bindModel();
}

void RobotPanel::setEndpoint(const QString& host, int port, const QVariantMap& addr) {
    m_host = host; m_port = port; m_addr = addr;
}

void RobotPanel::bindModel()
{
    if (!m_mgr || m_id.isEmpty()) return;
    // RobotManager::model(...)이 QAbstractItemModel* 또는 PickListModel*를 돌려준다고 가정
    auto* model = m_mgr->model(m_id);
    m_table->setModel(static_cast<QAbstractItemModel*>(model));
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
