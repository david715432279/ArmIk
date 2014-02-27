#include "widget.h"
#include "ui_widget.h"


Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);

    robot_init_form = new RobotInitForm(this);

    ButtonInit();//初始化按钮


}

Widget::~Widget()
{
    delete ui;
}

void Widget::ButtonInit()
{
    connect(ui->SysTestButton,SIGNAL(clicked()),robot_init_form,SLOT(CheckRobotState()));
}



