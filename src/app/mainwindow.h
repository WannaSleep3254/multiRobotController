#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPointer>
#include "LogLevel.h"
#include "vision/VisionClient.h"

class RobotManager;
class GentryManager;
class QLabel;
class QFrame;
class QCheckBox;
class RobotPanel;
class MotorPanel;
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

    void initVisionClient();

private:
    void handleRobotA(const RobotCommand& cmd);
    void handleRobotB(const RobotCommand& cmd);


private slots:
    void onLog(const QString& line);
    void onLog(const QString& line, Common::LogLevel level);

    void onRobotCommand(const RobotCommand& cmd);

    void on_btnConnect_clicked();
    void on_btnDisconnect_clicked();

    void on_pushButton_Atuo_clicked();

    void on_pushButton_Manual_clicked();

    void on_pushButton_Start_clicked();

    void on_pushButton_Stop_clicked();

    void on_pushButton_Pause_clicked();

private:
    Ui::MainWindow *ui;

    QPointer<RobotManager> m_mgr;
    QPointer<GentryManager> m_gentryMgr;
    QVariantMap m_addr; // parsed AddressMap.json

    QSplitter*   m_split = nullptr;
    RobotPanel*  m_panelA = nullptr;
    RobotPanel*  m_panelB = nullptr;
    MotorPanel*  m_motorPanel = nullptr;
//    RobotPanel*  m_panelC = nullptr;

    void loadRobotsFromConfig();

private:
    QLabel* m_fsmLabel{nullptr};
    QFrame* m_fsmLed{nullptr};

    QCheckBox* m_chkShowDebug{nullptr};
    bool m_showDebugLogs{false};

    void setFsmLedColor(const QString& name);

    VisionClient* m_visionClient{nullptr};

    bool m_sortingPlacePorcessActive{false};
    bool m_sortingFlip{false};
    bool m_sortingDock{false};
    int  m_sortingOffset{0};
    int  m_sortingThick{0};
    int  m_sortingShfit{0};
    bool m_gantryPickupState{false};
};
#endif // MAINWINDOW_H
