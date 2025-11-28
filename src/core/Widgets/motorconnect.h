#ifndef MOTORCONNECT_H
#define MOTORCONNECT_H

#include <QDialog>
#include "serialport.h"
QT_BEGIN_NAMESPACE

QT_END_NAMESPACE

namespace Ui {
class motorConnect;
}

class motorConnect : public QDialog
{
    Q_OBJECT

public:
    explicit motorConnect(QWidget *parent = nullptr);
    ~motorConnect();

protected:
    virtual void closeEvent(QCloseEvent *event) override;
    virtual void showEvent(QShowEvent *event) override;

signals:
    void req_motorConnect(const bool&);
    void req_motorPort(const QString&, const QString&);

private slots:
    void on_portUpdate();

    void on_comboBox_PortNo_activated(int index);
    void on_pushButton_motorRefresh_clicked();
    void on_pushButton_motorConnect_clicked();
    void on_pushButton_motorDisconnect_clicked();

private:
    Ui::motorConnect *ui;

    //QTimer *updateTimer_;
    Serial::SerialPort *serialPort_;
    QList<QSerialPortInfo> serialPortInfos_;
};

#endif // MOTORCONNECT_H
