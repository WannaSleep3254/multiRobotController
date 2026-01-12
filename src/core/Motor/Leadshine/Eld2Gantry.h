#ifndef ELD2_GANTRY_H
#define ELD2_GANTRY_H

#include <QObject>

QT_BEGIN_NAMESPACE
class QTimer;
namespace Com {
class Modbus;
}
namespace Leadshine{
struct Properties;
class Parameter;
}
QT_END_NAMESPACE

namespace Leadshine
{
    class Eld2Gantry : public QObject
    {
        Q_OBJECT

        enum Axis
        {
            None  = 0,
            AxisX = 1,
            AxisY = 2,
            AxisZ = 3,
            AxisC = 4,

            // 별칭(읽을 때 의미 분명하게)
            GantryX       = AxisX,
            GantryZ       = AxisY,
            GantryPicker  = AxisZ,
            ConveyorAxis  = AxisC,

             AxisCount     = 5  // 0~4 사용
        };

        enum Dir
        {
            Stop = 0,
            Backward   = 1,
            Forward  = 2,
        };

        struct AxisRuntime
        {
            bool  moving       = false;
            float targetPos    = 0.0f;
            float lastPos      = 0.0f;
            qint16 lastVel     = 0;
            bool targetInposition = false;
            bool lastInposition = false;
        };

        struct Config
        {
            int Gear = 1;
            int Lead = 10;

            uint16_t velJog = 100;
            uint16_t accelJog = 100;
            uint16_t decelJog = 100;

            uint16_t velMove = 100;
            uint16_t accelMove = 100;
            uint16_t decelMove = 100;
            uint16_t pauselMove = 100;
            uint16_t positveLimit = 0;
            uint16_t negativeLimit = 0;

            bool isConnect = false;

            Config(int gear, int lead, uint16_t vel_1, uint16_t vel_2) : Gear(gear), Lead(lead), velJog(vel_1), velMove(vel_2) {}
        };

        enum class PollKind { PrPos, Status };
    public:
        explicit Eld2Gantry(QObject *parent = nullptr);
        ~Eld2Gantry();

        void setPort(const QString& name, const QString& baud);

        void doConnect();
        void doDisConnect();
        void doConnect(bool &);

        // read
        void reqReadEncoder(const int&);
        void reqReadError(const int &);
        void reqReadVersion(const int&);
        // write
        void reqWriteJog(const int &, const int &);
        void reqWriteLimit(const int &, const int &);
        void reqWritePos(const int &, const int32_t &);
        void reqWriteServo(const int &, const bool &);
        void reqWriteShift(const int &, const int32_t &);

//        void reqMovePosition(const float &, const float &);
        void reqMoveStop(const int &);
        void reqMoveStop();

        float convertPulseToPos(const int &, const int32_t &);
        qint32 convertPosToPulse(const int &, const float &);

        bool isMotionDone(int id) const;

    signals:
        void comState(const int&);
        void errorState(const int&);

        void readVersion(const int&, const QString&);
        void readReady(const int& , const bool&);
        void readServo(const int& , const bool&);
        void readState(const int&, const uint16_t&);
        void readEncoder(const int& , qint32& , const float&);
        void readVelocity(const int&, const qint16&);
        void readError(const int&, const quint16&);
        void motionFinished(const int& id, float targetPos);

    private slots:
        void readData(const int&, const int&, const QVector<quint16>&);

    private:
        enum class Phase { Status, Encoder };

        struct PollCtx {
            Com::Modbus* bus = nullptr;
            QTimer*      timer = nullptr;
            QVector<int> axes;
            int          axisIdx = 0;
            Phase        phase = Phase::Status;
            int          tickMs = 10;
        };

        PollCtx gantry_;

    private:
//        Com::Modbus *driver_;
//        QTimer *timer_;

        QVector<Config> config_;
        int pollAxis_ = Axis::AxisX;
        PollKind pollKind_ = PollKind::PrPos;
        QVector<AxisRuntime> runtime_;
    };
}
#endif // ELD2_H
