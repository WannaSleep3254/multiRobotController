#ifndef MOTORCONFIG_H
#define MOTORCONFIG_H

#include <QDialog>

QT_BEGIN_NAMESPACE
namespace Ui {
class motorConfig;
}
class QTableWidget;

namespace Leadshine {
struct Properties;
class Parameter;
}
QT_END_NAMESPACE

class motorConfig : public QDialog
{
    Q_OBJECT

public:
    explicit motorConfig(QWidget *parent = nullptr);
    ~motorConfig();

    void setToolbar();
    void initTableWidget(QTableWidget *);
    void setTableWidget_RowSpan(QTableWidget &, const int &row, const QString &text, const QFont &font);

    void printTableWidget();
    void printParameter(const int &index, const QVector<Leadshine::Properties> &parameters);

    void initParameter();
    void resetParameter();

signals:
    void requestApply(QVector<QPoint>, QVector<QPoint>, QVector<QPoint>);

private slots:
    void loadConfig();
    void saveConfig();
    void resetConfig();

    void readSettings();
    void writeSettings();

    void convertConfigToParameter();
    void applyParameter();

private:
    Ui::motorConfig *ui;
    QTableWidget *tableConfig_;
    Leadshine::Parameter *parameters_;

    QVector<Leadshine::Properties> jogParameter_;
    QVector<Leadshine::Properties> posParameter_;
    QVector<Leadshine::Properties> limitParameter_;

    QVector<QPoint> jogConfig_;
    QVector<QPoint> posConfig_;
    QVector<QPoint> limitConfig_;
};

#endif // MOTORCONFIG_H

