#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPointer>
#include "LogLevel.h"

class RobotManager;
class QLabel;
class QFrame;
class QCheckBox;
class RobotPanel;
class QSplitter;

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
#if false
    void onConnect();
    void onDisconnect();
    void onStart();
    void onStop();
#endif
    void onHeartbeat(bool ok);
    void onLog(const QString& line);
    void onLog(const QString& line, Common::LogLevel level);
#if false
    void onConnect_A();
    void onDisconnect_A();
    void onStart_A();
    void onStop_A();

    void onConnect_B();
    void onDisconnect_B();
    void onStart_B();
    void onStop_B();
#endif
#if false
private:
    void loadAddressMap();
#endif
private:
    Ui::MainWindow *ui;

    QPointer<RobotManager> m_mgr;
    QVariantMap m_addr; // parsed AddressMap.json

    QSplitter*   m_split = nullptr;
    RobotPanel*  m_panelA = nullptr;
    RobotPanel*  m_panelB = nullptr;

    void loadRobotsFromConfig();

private:
    QLabel* m_fsmLabel{nullptr};
    QFrame* m_fsmLed{nullptr};

    QCheckBox* m_chkShowDebug{nullptr};
    bool m_showDebugLogs{false};

    void setFsmLedColor(const QString& name);
};
#endif // MAINWINDOW_H
