#include "mainwindow.h"

#include <QApplication>
#include <QStyleFactory>
#include <QPalette>
#include <QStyle>
#include <QFile>
#include <QDir>
#include <QDateTime>
#include <QTextStream>


#pragma comment(linker, "/entry::WinMainCRTStartup /subsystem:console")

static void applyDarkPalette() {
    qApp->setStyle(QStyleFactory::create("Fusion"));

    QPalette p;
    p.setColor(QPalette::Window, QColor(37, 37, 38));
    p.setColor(QPalette::WindowText, QColor(220, 220, 220));
    p.setColor(QPalette::Base, QColor(30, 30, 30));
    p.setColor(QPalette::AlternateBase, QColor(45, 45, 48));
    p.setColor(QPalette::ToolTipBase, QColor(255, 255, 220));
    p.setColor(QPalette::ToolTipText, QColor(0, 0, 0));
    p.setColor(QPalette::Text, QColor(220, 220, 220));
    p.setColor(QPalette::Button, QColor(45, 45, 48));
    p.setColor(QPalette::ButtonText, QColor(220, 220, 220));
    p.setColor(QPalette::BrightText, QColor(255, 0, 0));
    p.setColor(QPalette::Highlight, QColor(14, 99, 156));
    p.setColor(QPalette::HighlightedText, QColor(255, 255, 255));
    qApp->setPalette(p);

    // 필요 시 폰트 가독성 개선
    qApp->setStyleSheet(
        "QPlainTextEdit { background:#1e1e1e; color:#e0e0e0; }"
        "QTableView { gridline-color:#505050; }"
        );
}

static void applyLightPalette() {
    qApp->setStyle(QStyleFactory::create("Fusion"));

    QPalette p;                           // 기본 값에서 시작
    // ----- 라이트 팔레트 강제 지정 -----
    p.setColor(QPalette::Window,         QColor(239,239,239));
    p.setColor(QPalette::WindowText,     QColor(0,0,0));
    p.setColor(QPalette::Base,           QColor(255,255,255));
    p.setColor(QPalette::AlternateBase,  QColor(245,245,245));
    p.setColor(QPalette::ToolTipBase,    QColor(255,255,255));
    p.setColor(QPalette::ToolTipText,    QColor(0,0,0));
    p.setColor(QPalette::Text,           QColor(0,0,0));
    p.setColor(QPalette::Button,         QColor(239,239,239));
    p.setColor(QPalette::ButtonText,     QColor(0,0,0));
    p.setColor(QPalette::BrightText,     QColor(255,0,0));
    p.setColor(QPalette::Highlight,      QColor(0,120,215));
    p.setColor(QPalette::HighlightedText,QColor(255,255,255));
    qApp->setPalette(p);

    // 혹시 이전 다크 스타일시트가 남아 있으면 해제
    qApp->setStyleSheet("");
}

void LogToFile(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{

    const QString logDirPath = QStringLiteral("C:/Work/multiRobotController/tmp");
    QDir().mkpath(logDirPath);   // 없으면 생성

    const QString logFilePath = logDirPath + "/log_"+ QDate::currentDate().toString("yyMMdd") + ".txt";

    QFile file(logFilePath);
    if(!file.open(QIODevice::Append | QIODevice::Text)) {
        return;
    }
    QTextStream out(&file);

    QString curtime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    //context.file, context.line, context.function
    switch (type) {
    case QtDebugMsg:
        out << "[Debug]" << curtime <<": "<< msg << "\n";
        break;
    case QtInfoMsg:
        out << "[Info]" << curtime <<": " << msg << "\n";
        break;
    case QtWarningMsg:
        out << "[Warning]" << curtime <<": " << msg << "\n";
        break;
    case QtCriticalMsg:
        out << "[Critical]" << curtime <<": " << msg << context.function << context.line << "\n";
        break;
    case QtFatalMsg:
        out << "[Fatal]" << curtime <<": " << msg << "\n";
        abort();
    }
    file.close();
}


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    applyLightPalette(); // 기본 라이트 모드 적용
    qInstallMessageHandler(LogToFile);

    MainWindow w;
    w.show();
    return a.exec();
}
