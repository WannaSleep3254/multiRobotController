#include "motortest.h"
#include "ui_motortest.h"

#include <QDebug>

motorTest::motorTest(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::motorTest)
{
    ui->setupUi(this);

    setWindowTitle("모터-단축 테스트");
    setWindowIcon(QIcon(":/Icon/motorManual.svg"));
}

motorTest::~motorTest()
{
    delete ui;
}

void motorTest::on_pushButton_ServoOn_clicked()
{
    emit reqServo(motorID_, true);
}

void motorTest::on_pushButton_ServoOff_clicked()
{
    emit reqServo(motorID_, false);
}

void motorTest::on_pushButton_JogLeft_pressed()
{
    emit reqJog(motorID_, 1);
}

void motorTest::on_pushButton_JogRight_pressed()
{
    emit reqJog(motorID_, 2);
}

void motorTest::on_pushButton_JogLeft_released()
{
    emit reqJog(motorID_, 0);
}

void motorTest::on_pushButton_JogRight_released()
{
    emit reqJog(motorID_, 0);
}

void motorTest::on_pushButton_Move_clicked()
{
    emit reqPos(motorID_, ui->lineEdit_posMove->text().toInt());
}

void motorTest::on_pushButton_Shift_clicked()
{
    emit reqShift(motorID_, ui->lineEdit_posShift->text().toInt());
}

void motorTest::on_pushButton_LimitA_clicked()
{
    emit reqLimit(motorID_, 1);
}


void motorTest::on_pushButton_LimitB_clicked()
{
    emit reqLimit(motorID_, 2);
}


void motorTest::on_pushButton_ErrorCheck_clicked()
{

}


void motorTest::on_pushButton_ErrorClear_clicked()
{

}


void motorTest::on_checkBox_SelectX_toggled(bool checked)
{
    if(!checked)
        return;

    motorID_=1;
}


void motorTest::on_checkBox_SelectZ_toggled(bool checked)
{
    if(!checked)
        return;

    motorID_=2;
}

void motorTest::on_checkBox_SelectP_toggled(bool checked)
{
    if(!checked)
        return;

    motorID_=3;
}

void motorTest::on_checkBox_SelectC_toggled(bool checked)
{
    if(!checked)
        return;

    motorID_=4;
}
