//#include "motorconfig.h"
#include "motorconfig.h"
#include "ui_motorconfig.h"

#include <QIcon>
#include <QDebug>
#include <QAction>
#include <QToolBar>
#include <QTableWidget>
#include <QHeaderView>
#include <QSettings>

#include "Leadshine/parameter.h"

motorConfig::motorConfig(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::motorConfig)
    , tableConfig_(new QTableWidget(this))
    , parameters_(new Leadshine::Parameter(this))
{
    ui->setupUi(this);

    setWindowTitle("모터 설정");
    setWindowIcon(QIcon(":/Icon/motorConfig.svg"));

    setToolbar();
    initTableWidget(tableConfig_);

    initParameter();
    loadConfig();
}

motorConfig::~motorConfig()
{
    delete ui;
}

void motorConfig::setToolbar()
{
    QAction *action_Load = new QAction(QIcon(":/Icon/Download.svg"),"Load");
    QAction *action_Save = new QAction(QIcon(":/Icon/Apply.svg"),"Save");
    QAction *action_Reset = new QAction(QIcon(":/Icon/Reset.svg"),"Reset");

    QObject::connect(action_Load,   &QAction::triggered, this, &motorConfig::loadConfig);
    QObject::connect(action_Save,   &QAction::triggered, this, &motorConfig::saveConfig);
    QObject::connect(action_Reset,   &QAction::triggered, this, &motorConfig::resetConfig);

    QList<QAction*> listAction;
    listAction<<action_Load<<action_Save<<action_Reset;//<<action_Read<<action_Write;

    QToolBar *toolbarConfig = new QToolBar(this);
    toolbarConfig->addSeparator();
    toolbarConfig->setIconSize(QSize(45,45));
    toolbarConfig->addActions(listAction);

    ui->gridLayout->addWidget(toolbarConfig);
}

void motorConfig::initTableWidget(QTableWidget* tableWidget)
{
    tableWidget->setColumnCount(7);
    tableWidget->setRowCount(12);

    QStringList horizontalHeader;
    horizontalHeader<<"X축"<<"Y축"<<"최소값"<<"최대값"<<"초기값"<<"단위"<<"비고";
    QStringList verticalHeader;
    verticalHeader <<""<<"속도"<<"가속도"<<"감속도"
                   <<""<<"속도"<<"가속도"<<"감속도"<<"Pause"
                   <<""<<"정방향"<<"역방향";

    QFont fontBold;
    fontBold.setBold(true);

    tableWidget->setHorizontalHeaderLabels(horizontalHeader);
    tableWidget->setVerticalHeaderLabels(verticalHeader);
    tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    tableWidget->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);

    setTableWidget_RowSpan(*tableWidget, 0, QString("조그 이동"), fontBold);
    setTableWidget_RowSpan(*tableWidget, 4, QString("좌표 이동"), fontBold);
    setTableWidget_RowSpan(*tableWidget, 9, QString("위치 제한"), fontBold);

    ui->gridLayout->addWidget(tableWidget);

    for(int row=1; row<4; row++)
    {
        for(int col=0; col<tableWidget->columnCount(); col++)
        {
            tableWidget->setItem(row,col,new QTableWidgetItem(QString("null")));
            tableWidget->item(row,col)->setTextAlignment(Qt::AlignCenter);
            if( col > 1 )
                tableWidget->item(row,col)->setFlags(tableWidget->item(row,col)->flags() ^ Qt::ItemIsEditable);
        }
    }

    for(int row=5; row<9; row++)
    {
        for(int col=0; col<tableWidget->columnCount(); col++)
        {
            tableWidget->setItem(row,col,new QTableWidgetItem(QString("null")));
            tableWidget->item(row,col)->setTextAlignment(Qt::AlignCenter);
            if( col > 1 )
                tableWidget->item(row,col)->setFlags(tableWidget->item(row,col)->flags() ^ Qt::ItemIsEditable);
        }
    }

    for(int row=10; row<12; row++)
    {
        for(int col=0; col<tableWidget->columnCount(); col++)
        {
            tableWidget->setItem(row,col,new QTableWidgetItem(QString("null")));
            tableWidget->item(row,col)->setTextAlignment(Qt::AlignCenter);
            if( col > 1 )
                tableWidget->item(row,col)->setFlags(tableWidget->item(row,col)->flags() ^ Qt::ItemIsEditable);
        }
    }

}

void motorConfig::setTableWidget_RowSpan(QTableWidget &tableWidget, const int &row, const QString &text, const QFont &font)
{
    tableWidget.setSpan(row,0,1,tableWidget.columnCount());
    tableWidget.setItem(row,0,new QTableWidgetItem(text));
    tableWidget.item(row,0)->setTextAlignment(Qt::AlignCenter);
    tableWidget.item(row,0)->setBackground(QColor(180, 229, 162));
    tableWidget.item(row,0)->setForeground(QColor(0,0,0));
    tableWidget.item(row,0)->setFont(font);
    tableWidget.item(row,0)->setFlags(tableWidget.item(row,0)->flags() ^ Qt::ItemIsEditable);
}

void motorConfig::printTableWidget()
{
    printParameter(1, jogParameter_);
    printParameter(5, posParameter_);
    printParameter(10, limitParameter_);
}

void motorConfig::printParameter(const int &index, const QVector<Leadshine::Properties> &parameters)
{
    int row = index;
    for(const auto &parameter : std::as_const(parameters))
    {
        tableConfig_->item(row,0)->setText((QString::number(static_cast<int16_t>(parameter.X))));
        tableConfig_->item(row,1)->setText((QString::number(static_cast<int16_t>(parameter.Y))));
        tableConfig_->item(row,2)->setText((QString::number(static_cast<int16_t>(parameter.Min))));
        tableConfig_->item(row,3)->setText((QString::number(static_cast<int16_t>(parameter.Max))));
        tableConfig_->item(row,4)->setText((QString::number(static_cast<int16_t>(parameter.Default))));
        tableConfig_->item(row,5)->setText((QString(parameter.Unit)));
        tableConfig_->item(row,6)->setText((QString("")));
        row++;
    }
}

void motorConfig::initParameter()
{
    jogParameter_ = parameters_->getJogParmeter();
    posParameter_ = parameters_->getPosParmeter();
    limitParameter_ = parameters_->getLimitParmeter();
}

void motorConfig::resetParameter()
{
    for(int i=0; i<jogParameter_.size(); i++)
    {
        jogParameter_[i].X = tableConfig_->item(1+i, 4)->text().toInt();
        jogParameter_[i].Y = tableConfig_->item(1+i, 4)->text().toInt();

        jogConfig_[i].setX(jogParameter_[i].X);
        jogConfig_[i].setY(jogParameter_[i].Y);
    }
    for(int i=0; i<posParameter_.size(); i++)
    {
        posParameter_[i].X = tableConfig_->item(5+i, 4)->text().toInt();
        posParameter_[i].Y = tableConfig_->item(5+i, 4)->text().toInt();

        posConfig_[i].setX(posParameter_[i].X);
        posConfig_[i].setY(posParameter_[i].Y);
    }
    for(int i=0; i<limitParameter_.size(); i++)
    {
        limitParameter_[i].X = tableConfig_->item(10+i, 4)->text().toInt();
        limitParameter_[i].Y = tableConfig_->item(10+i, 4)->text().toInt();

        limitConfig_[i].setX(limitParameter_[i].X);
        limitConfig_[i].setY(limitParameter_[i].Y);
    }
}

void motorConfig::loadConfig()
{
    readSettings();
    convertConfigToParameter();
    printTableWidget();
}

void motorConfig::saveConfig()
{
    writeSettings();
    applyParameter();
}

void motorConfig::resetConfig()
{
    resetParameter();
    loadConfig();
    saveConfig();
}

void motorConfig::readSettings()
{
    QSettings settings("gachi" , "test");

    jogConfig_.clear();
    for(const auto &parameter : std::as_const(jogParameter_))
    {
        const QPoint config = settings.value(QString("Jog/%1").arg(parameter.Name), QPoint(parameter.X, parameter.Y)).toPoint();
        jogConfig_<<config;
    }

    posConfig_.clear();
    for(const auto &parameter : std::as_const(posParameter_))
    {
        QPoint config = settings.value(QString("Pos/%1").arg(parameter.Name), QPoint(parameter.X, parameter.Y)).toPoint();
        posConfig_<<config;
    }

    limitConfig_.clear();
    for(const auto &parameter : std::as_const(limitParameter_))
    {
        QPoint config = settings.value(QString("Limit/%1").arg(parameter.Name), QPoint(parameter.X, parameter.Y)).toPoint();
        limitConfig_<<config;
    }
}

void motorConfig::writeSettings()
{
    QSettings settings("gachi" , "test");

    for(int i=0; i<jogParameter_.size(); i++)
    {
        jogParameter_[i].X = tableConfig_->item(1+i, 0)->text().toInt();
        jogParameter_[i].Y = tableConfig_->item(1+i, 1)->text().toInt();

        jogConfig_[i].setX(jogParameter_[i].X);
        jogConfig_[i].setY(jogParameter_[i].Y);
        settings.setValue(QString("Jog/%1").arg(jogParameter_.at(i).Name), jogConfig_.at(i));
    }
    for(int i=0; i<posParameter_.size(); i++)
    {
        posParameter_[i].X = tableConfig_->item(5+i, 0)->text().toInt();
        posParameter_[i].Y = tableConfig_->item(5+i, 1)->text().toInt();

        posConfig_[i].setX(posParameter_[i].X);
        posConfig_[i].setY(posParameter_[i].Y);
        settings.setValue(QString("Pos/%1").arg(posParameter_.at(i).Name), posConfig_.at(i));
    }
    for(int i=0; i<limitParameter_.size(); i++)
    {
        limitParameter_[i].X = tableConfig_->item(10+i, 0)->text().toInt();
        limitParameter_[i].Y = tableConfig_->item(10+i, 1)->text().toInt();

        limitConfig_[i].setX(limitParameter_[i].X);
        limitConfig_[i].setY(limitParameter_[i].Y);
        settings.setValue(QString("Limit/%1").arg(limitParameter_.at(i).Name), limitConfig_.at(i));
    }
}

void motorConfig::convertConfigToParameter()
{
    for(int i=0; i<jogConfig_.size(); i++)
    {
        jogParameter_[i].X=jogConfig_[i].x();
        jogParameter_[i].Y=jogConfig_[i].y();
    }
    for(int i=0; i<posConfig_.size(); i++)
    {
        posParameter_[i].X=posConfig_[i].x();
        posParameter_[i].Y=posConfig_[i].y();
    }
    for(int i=0; i<limitConfig_.size(); i++)
    {
        limitParameter_[i].X=limitConfig_[i].x();
        limitParameter_[i].Y=limitConfig_[i].y();
    }
}
void motorConfig::applyParameter()
{
    emit requestApply(jogConfig_, posConfig_, limitConfig_);
}
