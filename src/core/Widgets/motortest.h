#ifndef MOTORTEST_H
#define MOTORTEST_H

#include <QDialog>

namespace Ui {
class motorTest;
}

class motorTest : public QDialog
{
    Q_OBJECT

public:
    explicit motorTest(QWidget *parent = nullptr);
    ~motorTest();

signals:
    void reqServo(const int&, const bool&);
    void reqJog(const int&, const int&);
    void reqPos(const int&, const int32_t&);
    void reqShift(const int&, const int32_t&);
    void reqLimit(const int&, const int&);

private slots:
    void on_pushButton_ServoOn_clicked();
    void on_pushButton_ServoOff_clicked();

    void on_pushButton_JogLeft_pressed();
    void on_pushButton_JogRight_pressed();

    void on_pushButton_JogLeft_released();
    void on_pushButton_JogRight_released();

    void on_pushButton_Move_clicked();
    void on_pushButton_Shift_clicked();

    void on_pushButton_LimitA_clicked();
    void on_pushButton_LimitB_clicked();

    void on_pushButton_ErrorCheck_clicked();
    void on_pushButton_ErrorClear_clicked();

    void on_checkBox_SelectX_toggled(bool checked);
    void on_checkBox_SelectZ_toggled(bool checked);
    void on_checkBox_SelectP_toggled(bool checked);
    void on_checkBox_SelectC_toggled(bool checked);

private:
    Ui::motorTest *ui;

    int motorID_=1;
};

#endif // MOTORTEST_H
