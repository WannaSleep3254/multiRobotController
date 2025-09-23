#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPointer>
#include "LogLevel.h"

class RobotManager;
class QLabel;
class QFrame;
class QCheckBox;

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
    void onConnect();
    void onDisconnect();
    void onStart();
    void onStop();
    void onHeartbeat(bool ok);
    void onLog(const QString& line);
    void onLog(const QString& line, Common::LogLevel level);

private:
    void loadAddressMap();

private:
    Ui::MainWindow *ui;

    QPointer<RobotManager> m_mgr;
    QVariantMap m_addr; // parsed AddressMap.json

private:
    QLabel* m_fsmLabel{nullptr};
    QFrame* m_fsmLed{nullptr};

    QCheckBox* m_chkShowDebug{nullptr};
    bool m_showDebugLogs{false};

    void setFsmLedColor(const QString& name);
};
#endif // MAINWINDOW_H
