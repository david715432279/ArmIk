#include "robotinitform.h"
#include "ui_robotinitform.h"

RobotInitForm::RobotInitForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::RobotInitForm)
{
    ui->setupUi(this);
}

RobotInitForm::~RobotInitForm()
{
    delete ui;
}
