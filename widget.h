#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <robotinitform.h>
#include <QPushButton>
//#include <ikfunc.h>
//#include <ikarmdesc.h>

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = 0);
    ~Widget();

private:
    Ui::Widget *ui;
    RobotInitForm *robot_init_form;    
    void ShowRobotInitForm();

    void ButtonInit();  //初始化按钮状态，以后可能用checknetstate代替
public slots:
 //   void test();

};

#endif // WIDGET_H
