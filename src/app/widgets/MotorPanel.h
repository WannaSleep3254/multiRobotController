#ifndef MOTORPANEL_H
#define MOTORPANEL_H

#include <QWidget>
#include <QPointer>

class QPushButton;
class QLabel;
class GentryManager;
class QPlainTextEdit;

class MotorPanel : public QWidget
{
    Q_OBJECT
public:
    explicit MotorPanel(QWidget *parent = nullptr);

    void setManager(GentryManager* mgr);

    void setServoStatus(int id, bool on);
    void setComStatus(int id, bool connected);

signals:
    void servoOnRequested();
    void servoOffRequested();
    void comConnectRequested();
    void comDisconnectRequested();

public slots:
    void onConnect();
    void onDisconnect();
    void onServoOn();
    void onServoOff();

private:
    QPointer<GentryManager> m_mgr;

    QPushButton* m_btn_Connect = nullptr;
    QPushButton* m_btn_Disconnect = nullptr;
    QLabel*      m_lbl_comStatus[4];

    QPushButton* m_btn_ServoOn = nullptr;
    QPushButton* m_btn_ServoOff = nullptr;
    QLabel*      m_lbl_servoStatus[4];;

    QPlainTextEdit* m_logView = nullptr;   // ★ 패널용 로그창

};

#endif // MOTORPANEL_H
