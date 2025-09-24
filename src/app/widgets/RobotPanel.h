#pragma once
#include <QWidget>
#include <QPointer>

class QTableView;
class QPushButton;
class QCheckBox;
class QLabel;
class RobotManager;

class RobotPanel : public QWidget {
    Q_OBJECT
public:
    explicit RobotPanel(QWidget* parent=nullptr);

    void setManager(RobotManager* mgr);
    void setRobotId(const QString& id);   // "A" / "B" 등
    void setEndpoint(const QString& host, int port, const QVariantMap& addr);

private slots:
    void onConnect();
    void onDisconnect();
    void onStart();
    void onStop();
    void onRepeatToggled(bool on);
    void onHeartbeat(bool ok);

private:
    void bindModel(); // RobotManager의 모델을 TableView에 바인딩

private:
    QString m_id;
    QString m_host;
    int     m_port = 0;
    QVariantMap m_addr;

    QPointer<RobotManager> m_mgr;

    QTableView*  m_table = nullptr;
    QPushButton* m_btnConnect = nullptr;
    QPushButton* m_btnDisconnect = nullptr;
    QPushButton* m_btnStart = nullptr;
    QPushButton* m_btnStop = nullptr;
    QCheckBox*   m_chkRepeat = nullptr;
    QLabel*      m_led = nullptr;   // 연결 상태 LED
};
