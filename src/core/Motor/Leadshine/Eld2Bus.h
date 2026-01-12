#ifndef ELD2BUS_H
#define ELD2BUS_H

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
    class Eld2Bus : public QObject
    {
        Q_OBJECT
    public:
        enum class Phase {Status, Encoder};
        struct PollCtx
        {
            Com::Modbus  *bus = nullptr;
            QTimer       *timer = nullptr;
            QVector<int> axes;  // axis index list
            int          axisIdx = 0; // current axis index in axes
            Phase        phase = Phase::Status;
            int          tickMs = 20;
        };

    public:
        explicit Eld2Bus(QObject *parent = nullptr);
        ~Eld2Bus();

        void setPortSettings(const Serial::Settings &settings);)
    };
}
#endif // ELD2BUS_H
