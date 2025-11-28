#ifndef PARAMETER_H
#define PARAMETER_H

#include <QObject>
namespace Leadshine
{
    struct Properties
    {
        QString  Name;
        uint16_t X;
        uint16_t Y;
        uint16_t Min;
        uint16_t Max;
        uint16_t Default;
        QString  Unit;

        Properties(QString name="", uint16_t x_val=100, uint16_t y_val=100, uint16_t min_val=0, uint16_t max_val=65535, uint16_t default_val=100, QString unit="")
            : Name(name), X(x_val), Y(y_val), Min(min_val), Max(max_val),Default(default_val),Unit(unit){}
    };

    class Parameter : public QObject
    {
        Q_OBJECT

    public:
        explicit Parameter(QObject *parent = nullptr);

        void initParameter();

        void setJogParmeter(const int &index, const Properties &parameter);
        void setPosParmeter(const int &index, const Properties &parameter);
        void setLimitParmeter(const int &index, const Properties &parameter);

        QVector<Properties> getJogParmeter();
        QVector<Properties> getPosParmeter();
        QVector<Properties> getLimitParmeter();

    signals:

    private:
        QVector<Properties> jogParameter_;
        QVector<Properties> posParameter_;
        QVector<Properties> limitParameter_;
    };
}
#endif // PARAMETER_H
