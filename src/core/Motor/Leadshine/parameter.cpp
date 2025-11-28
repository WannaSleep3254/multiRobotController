#include "parameter.h"

#include <QDebug>

namespace Leadshine
{
    Parameter::Parameter(QObject *parent)
        : QObject{parent}
    {
        initParameter();
    }

    void Parameter::initParameter()
    {
        jogParameter_.resize(3);
        setJogParmeter(0, Properties("Speed", 100, 100, -20000, 20000, 100, "rpm"));
        setJogParmeter(1, Properties("Accel", 100, 100, 1, 3267, 100, "ms/Krpm"));
        setJogParmeter(2, Properties("Decel", 100, 100, 1, 3267, 100, "ms/Krpm"));

        posParameter_.resize(4);
        setPosParmeter(0,Properties("Speed", 100, 100, -20000, 20000, 100, "rpm"));
        setPosParmeter(1,Properties("Accel", 100, 100, 1, 3267, 100, "ms/Krpm"));
        setPosParmeter(2,Properties("Decel", 100, 100, 1, 3267, 100, "ms/Krpm"));
        setPosParmeter(3,Properties("Pause", 100, 100, 0, 3267, 100, "ms"));

        limitParameter_.resize(2);
        setLimitParmeter(0,Properties("Positive", 0, 0, 0, 65535, 0, "mm"));
        setLimitParmeter(1,Properties("Negative", 0, 0, 0, 65535, 0, "mm"));
    }

    void Parameter::setJogParmeter(const int &index, const Properties &parameter)
    {
        jogParameter_[index] = parameter;
    }

    void Parameter::setPosParmeter(const int &index, const Properties &parameter)
    {
        posParameter_[index] = parameter;
    }

    void Parameter::setLimitParmeter(const int &index, const Properties &parameter)
    {
        limitParameter_[index] = parameter;
    }

    QVector<Properties> Parameter::getJogParmeter()
    {
        QVector<Properties> returnParameter;
        if(jogParameter_.size() != 3)
            return returnParameter;

        for(const auto &parameter : std::as_const(jogParameter_))
        {
            returnParameter<<parameter;
        }
        return returnParameter;
    }

    QVector<Properties> Parameter::getPosParmeter()
    {
        QVector<Properties> returnParameter;
        if(posParameter_.size() != 4)
            return returnParameter;

        for(const auto &parameter : std::as_const(posParameter_))
        {
            returnParameter<<parameter;
        }
        return returnParameter;
    }

    QVector<Properties> Parameter::getLimitParmeter()
    {
        QVector<Properties> returnParameter;
        if(limitParameter_.size() != 2)
            return returnParameter;

        for(const auto &parameter : std::as_const(limitParameter_))
        {
            returnParameter<<parameter;
        }
        return returnParameter;
    }
}
