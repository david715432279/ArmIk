#include "robotinitform.h"
#include "ui_robotinitform.h"

RobotInitForm::RobotInitForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::RobotInitForm)
{
    ui->setupUi(this);
    this->hide();

}

void RobotInitForm::CheckRobotState(){

    //step 0 初始化接口设备
    //step 1 检查机器人的状态

    //step 2 读取机器人状态，设置相应的按钮参数


    //step 3 打开界面
    this->show();
}

RobotInitForm::~RobotInitForm()
{
    delete ui;
}
