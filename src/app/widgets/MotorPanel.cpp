#include "MotorPanel.h"

#include <QPushButton>
#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>

#include "robots/GentryManager.h"

MotorPanel::MotorPanel(QWidget *parent)
    : QWidget{parent}
{
    m_btn_Connect       = new QPushButton("Connect",this);
    m_btn_Disconnect    = new QPushButton("Disconnect",this);
    m_lbl_comStatus[0]     = new QLabel("1축",this);
    m_lbl_comStatus[1]     = new QLabel("2축",this);
    m_lbl_comStatus[2]     = new QLabel("3축",this);
    m_lbl_comStatus[3]     = new QLabel("4축",this);

    m_btn_ServoOn       = new QPushButton("서보 On", this);
    m_btn_ServoOff      = new QPushButton("서보 Off", this);
    m_lbl_servoStatus[0]   = new QLabel("1축",this);
    m_lbl_servoStatus[1]   = new QLabel("2축",this);
    m_lbl_servoStatus[2]   = new QLabel("3축",this);
    m_lbl_servoStatus[3]   = new QLabel("4축",this);
    m_logView              = new QPlainTextEdit(this);

    auto* row1 = new QHBoxLayout;
    row1->addWidget(m_btn_Connect);
    row1->addWidget(m_btn_Disconnect);

    auto* row2 = new QHBoxLayout;
    row2->addWidget(m_lbl_comStatus[0]);
    row2->addWidget(m_lbl_comStatus[1]);
    row2->addWidget(m_lbl_comStatus[2]);
    row2->addWidget(m_lbl_comStatus[3]);

    auto* row3 = new QHBoxLayout;
    row3->addWidget(m_btn_ServoOn);
    row3->addWidget(m_btn_ServoOff);

    auto* row4 = new QHBoxLayout;
    row4->addWidget(m_lbl_servoStatus[0]);
    row4->addWidget(m_lbl_servoStatus[1]);
    row4->addWidget(m_lbl_servoStatus[2]);
    row4->addWidget(m_lbl_servoStatus[3]);

    auto* row5 = new QHBoxLayout;
    row5->addWidget(m_logView);

    auto* lay = new QVBoxLayout(this);
    lay->addLayout(row1);
    lay->addLayout(row2);
    lay->addLayout(row3);
    lay->addLayout(row4);
    lay->addLayout(row5);

    setLayout(lay);

    connect(m_btn_Connect,    &QPushButton::clicked, this, &MotorPanel::onConnect);
    connect(m_btn_Disconnect, &QPushButton::clicked, this, &MotorPanel::onDisconnect);
    connect(m_btn_ServoOn,    &QPushButton::clicked, this, &MotorPanel::onServoOn);
    connect(m_btn_ServoOff,   &QPushButton::clicked, this, &MotorPanel::onServoOff);
}

void MotorPanel::setManager(GentryManager* mgr)
{
    if (m_mgr == mgr)
        return;

    m_mgr = mgr;

    if (!m_mgr)
        return;

    connect(m_mgr, &GentryManager::motorComStatus, this, [this](int id, bool on){
        // 로그 처리
        setComStatus(id-1, on);
    });

    connect(m_mgr, &GentryManager::motorServoStatus, this, [this](int id, bool on){
        // 로그 처리
        setServoStatus(id-1, on);
    });
}

void MotorPanel::onConnect()
{
    m_mgr->setPort();
    m_mgr->doConnect();
}

void MotorPanel::onDisconnect()
{
    m_mgr->doDisconnect();
}

void MotorPanel::onServoOn()
{
    m_mgr->doServoOn();
}

void MotorPanel::onServoOff()
{
    m_mgr->doServoOff();
}

void MotorPanel::setServoStatus(int id, bool on)
{
    if(!on){
        m_lbl_servoStatus[id]->setStyleSheet("background-color:red");
        return;
    }
    m_lbl_servoStatus[id]->setStyleSheet("background-color:green");
}

void MotorPanel::setComStatus(int id, bool connected)
{
    if(!connected){
        m_lbl_comStatus[id]->setStyleSheet("background-color:red");
        return;
    }
    m_lbl_comStatus[id]->setStyleSheet("background-color:green");
}

