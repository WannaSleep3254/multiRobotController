#pragma once
#include <QWidget>
#include <QPointer>
#include "LogLevel.h"

class QTableView;
class QPushButton;
class QCheckBox;
class QLabel;
class QPlainTextEdit;
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

    void onLoadCsv();      // ★ CSV 로드
    void onClear();        // ★ 리스트 지우기

private:
    void bindModel(); // RobotManager의 모델을 TableView에 바인딩
    void appendLog(const QString& line, Common::LogLevel lv);

private:
    QString m_id;
    QString m_host;
    int     m_port = 0;
    QVariantMap m_addr;

    QPointer<RobotManager> m_mgr;

    QTableView*  m_table = nullptr;
    QPushButton* m_btnConnect = nullptr;

    QPushButton* m_btnLoadCsv = nullptr;
    QPushButton* m_btnClear   = nullptr;

    QPushButton* m_btnDisconnect = nullptr;
    QPushButton* m_btnStart = nullptr;
    QPushButton* m_btnStop = nullptr;
    QCheckBox*   m_chkRepeat = nullptr;
    QLabel*      m_led = nullptr;   // 연결 상태 LED
    QPlainTextEdit* m_logView = nullptr;   // ★ 패널용 로그창

    QCheckBox*   m_chkVisionMode = nullptr;
};
