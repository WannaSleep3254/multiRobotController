#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPointer>
#include "LogLevel.h"
#include "vision/VisionServer.h"

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

    void initVisionServer();

private slots:
    void onHeartbeat(bool ok);
    void onLog(const QString& line);
    void onLog(const QString& line, Common::LogLevel level);

    void on_btnStart_clicked();

    void on_btnStop_clicked();

private:
    Ui::MainWindow *ui;

    QPointer<RobotManager> m_mgr;
    QVariantMap m_addr; // parsed AddressMap.json

    QSplitter*   m_split = nullptr;
    RobotPanel*  m_panelA = nullptr;
    RobotPanel*  m_panelB = nullptr;
    RobotPanel*  m_panelC = nullptr;

    void loadRobotsFromConfig();

private:
    QLabel* m_fsmLabel{nullptr};
    QFrame* m_fsmLed{nullptr};

    QCheckBox* m_chkShowDebug{nullptr};
    bool m_showDebugLogs{false};

    void setFsmLedColor(const QString& name);

    VisionServer* m_visionServer{nullptr};
};
#endif // MAINWINDOW_H
