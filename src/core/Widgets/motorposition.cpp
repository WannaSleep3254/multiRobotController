#include "motorposition.h"
#include "ui_motorposition.h"

motorPosition::motorPosition(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::motorPosition)
{
    ui->setupUi(this);

    setWindowTitle("모터-위치 이동");
    setWindowIcon(QIcon(":/Icon/motorPosition.svg"));
}

motorPosition::~motorPosition()
{
    delete ui;
}

void motorPosition::on_pushButton_ServoOn_clicked()
{
    emit reqServo(true);
}

void motorPosition::on_pushButton_ServoOff_clicked()
{
    emit reqServo(false);
}

void motorPosition::on_pushButton_X_1_clicked()
{
    float goal_X = ui->lineEdit_X_1->text().toFloat();
    std::int32_t pulse_X = static_cast<std::int32_t>(goal_X * 10000.0f / 5.0f); // mm to pulse

    reqMove(1, pulse_X);
}

void motorPosition::on_pushButton_X_2_clicked()
{
    float goal_X = ui->lineEdit_X_2->text().toFloat();
    std::int32_t pulse_X = static_cast<std::int32_t>(goal_X * 10000.0f / 5.0f); // mm to pulse

    reqMove(1, pulse_X);
}

void motorPosition::on_pushButton_X_3_clicked()
{
    float goal_X = ui->lineEdit_X_3->text().toFloat();
    std::int32_t pulse_X = static_cast<std::int32_t>(goal_X * 10000.0f / 5.0f); // mm to pulse

    reqMove(1, pulse_X);
}

void motorPosition::on_pushButton_Z_1_clicked()
{
    float goal_Z = ui->lineEdit_Z_1->text().toFloat();
    std::int32_t pulse_Z = static_cast<std::int32_t>(goal_Z * 10000.0f / 5.0f); // mm to pulse

    reqMove(2, pulse_Z);
}

void motorPosition::on_pushButton_Z_2_clicked()
{
    float goal_Z = ui->lineEdit_Z_2->text().toFloat();
    std::int32_t pulse_Z = static_cast<std::int32_t>(goal_Z * 10000.0f / 5.0f); // mm to pulse

    reqMove(2, pulse_Z);
}

void motorPosition::on_pushButton_P_1_clicked()
{
    float goal_P = ui->lineEdit_P_1->text().toFloat();
    std::int32_t pulse_P = static_cast<std::int32_t>(goal_P * 10000.0f *10 / 360.0f); // degree to pulse

    reqMove(3, pulse_P);
}

void motorPosition::on_pushButton_P_2_clicked()
{
    float goal_P = ui->lineEdit_P_2->text().toFloat();
    std::int32_t pulse_P = static_cast<std::int32_t>(goal_P * 10000.0f *10 / 360.0f); // degree to pulse

    reqMove(3, pulse_P);
}

void motorPosition::on_pushButton_P_3_clicked()
{
    float goal_P = ui->lineEdit_P_3->text().toFloat();
    std::int32_t pulse_P = static_cast<std::int32_t>(goal_P * 10000.0f *10 / 360.0f); // degree to pulse

    reqMove(3, pulse_P);
}

void motorPosition::on_pushButton_C_1_clicked()
{
    float goal_C = ui->lineEdit_C_1->text().toFloat();
    std::int32_t pulse_C = -340230;

    reqMove(4, pulse_C);
}
