#include "motorconnect.h"
#include "ui_motorconnect.h"

#include <QDebug>
#include <QComboBox>

motorConnect::motorConnect(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::motorConnect)
{
    ui->setupUi(this);

    setWindowTitle("모터 연결");
    setWindowIcon(QIcon(":/Icon/motorConnect.svg"));

    serialPort_ = new Serial::SerialPort(this);

    QStringList baud;
    baud<<"9600"<<"19200"<<"38400"<<"57600"<<"115200";
    ui->comboBox_BaudRate->addItems(baud);
}

motorConnect::~motorConnect()
{
    delete ui;
}

void motorConnect::closeEvent(QCloseEvent *event)
{
    Q_UNUSED(event)
}

void motorConnect::showEvent(QShowEvent *event)
{
    Q_UNUSED(event)

    on_portUpdate();
}

void motorConnect::on_portUpdate()
{
    serialPort_->onUpdate();
    serialPortInfos_ = serialPort_->getPortLists();

    ui->comboBox_PortNo->clear();
    for (const QSerialPortInfo &portInfo : std::as_const(serialPortInfos_))
    {
        ui->comboBox_PortNo->addItem(portInfo.portName());
    }

}

void motorConnect::on_comboBox_PortNo_activated(int index)
{
    auto portInfo = serialPortInfos_.at(index);

    QString VID = (portInfo.hasVendorIdentifier()? QByteArray::number(portInfo.vendorIdentifier(), 16):QByteArray());
    QString PID = (portInfo.hasProductIdentifier()? QByteArray::number(portInfo.productIdentifier(), 16): QByteArray());

    ui->lineEdit_VID->setText(VID);
    ui->lineEdit_PID->setText(PID);
    ui->lineEdit_SerialNo->setText(portInfo.serialNumber());
}


void motorConnect::on_pushButton_motorRefresh_clicked()
{
    on_portUpdate();
}


void motorConnect::on_pushButton_motorConnect_clicked()
{
    on_comboBox_PortNo_activated(ui->comboBox_PortNo->currentIndex());

    emit req_motorPort(ui->comboBox_PortNo->currentText() , ui->comboBox_BaudRate->currentText());
    emit req_motorConnect(true);
}


void motorConnect::on_pushButton_motorDisconnect_clicked()
{
    emit req_motorConnect(false);
}

