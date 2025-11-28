#ifndef GENTRYMANAGER_H
#define GENTRYMANAGER_H

#include <QObject>
#include <QSet>

#include "eld2.h"

class GentryManager : public QObject
{
    Q_OBJECT
public:
    explicit GentryManager(QObject *parent = nullptr);
    void setup_motorStateMonitoring();

    void setPort();

    void doConnect();
    void doDisconnect();
    void doServoOn();
    void doServoOff();

//    void setServoOn(int axis);

    bool flag_conveyor_fwd = false;
    bool flag_flip_fwd = false;
    bool flag_flip_rev = false;
    bool flag_gty_place = false;
    bool flag_gty_home_place = false;

    void setFalgs(bool conveyor_fwd, bool flip_fwd, bool flip_rev, bool gty_place, bool gty_home_place);
    void gentry_motion();

    void doGentryPlace();
    void doGentryReady();

    void startGantryMove();   // 1-2-3축 동시
    void startConveyorMove(); // 4축

    void onAxisFinished(int axis, int seq, bool ok);

        void logMessage2(const QString& msg);
signals:
    void log(const QString& msg);

    void gantryCommandFinished(bool ok);
    void conveyorCommandFinished(bool ok);

    void motorServoStatus(int id, bool on);
    void motorComStatus(int id, bool connected);

private:
    qint8 motion_seq;
    Leadshine::ELD2 *motorController = nullptr;

    struct PendingGroup {
        QSet<int> pendingAxes;  // 아직 안 끝난 축들
        bool ok = true;         // 전체 성공 여부
    };

    int m_nextSeq = 1;
    QHash<int, PendingGroup> m_gantryMap;   // seq → group (1-2-3)
    QHash<int, PendingGroup> m_conveyorMap; // seq → group (4)

    bool m_gantryOk{true}, m_convOk{true};
    QSet<int> m_gantryPendingAxes;
    QSet<int> m_conveyorPendingAxes;
};

#endif // GENTRYMANAGER_H
