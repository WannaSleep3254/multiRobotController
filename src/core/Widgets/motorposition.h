#ifndef MOTORPOSITION_H
#define MOTORPOSITION_H

#include <QDialog>

namespace Ui {
class motorPosition;
}

class motorPosition : public QDialog
{
    Q_OBJECT

public:
    explicit motorPosition(QWidget *parent = nullptr);
    ~motorPosition();

private slots:
    void on_pushButton_ServoOn_clicked();
    void on_pushButton_ServoOff_clicked();

    void on_pushButton_X_1_clicked();
    void on_pushButton_X_2_clicked();
    void on_pushButton_X_3_clicked();

    void on_pushButton_Z_1_clicked();
    void on_pushButton_Z_2_clicked();

    void on_pushButton_P_1_clicked();
    void on_pushButton_P_2_clicked();
    void on_pushButton_P_3_clicked();

    void on_pushButton_C_1_clicked();

signals:
    void reqServo(const bool&);
//    void reqMove(const float&, const float&);
    void reqMove(int id, const int32_t& pos);
private:
    Ui::motorPosition *ui;
};

#endif // MOTORPOSITION_H
