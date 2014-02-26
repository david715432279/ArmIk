#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <robotinitform.h>

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

    void ButtonInit();  //初始化按钮状态，以后可能用checknetstate代替
};

#endif // WIDGET_H
